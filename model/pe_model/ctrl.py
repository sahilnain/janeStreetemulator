# SPDX-License-Identifier: Apache-2.0
"""
ctrl: the controller (pe_ctrl + pe_decode, docs/spec/isa.md, docs/spec/blocks.md Section 2).

What this file does
-------------------
One instruction stream. A slot bit alternates every cycle and an instruction may
only issue in slot 0 with the stall counter at 0. comb() computes everything the
instruction does in this cycle from the current state and inputs; clock() applies
it. A blocked instruction has no effect (only a timed `wait` counts rd down).
"""

OP_NOP, OP_JMP, OP_DJNZ, OP_CALL, OP_RET = 0, 1, 2, 3, 4
OP_DELAY, OP_MOV, OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_SHL, OP_SHR, OP_CMP = range(6, 16)
OP_SET, OP_SETOE, OP_WAIT, OP_JPIN, OP_RDPINS, OP_BITIN, OP_BITOUT = range(16, 23)
OP_FGET, OP_FPUT, OP_LD, OP_ST, OP_UCFG, OP_URD, OP_USTART, OP_USTOP = range(23, 31)
OP_UWAIT, OP_EVT, OP_HOST, OP_PAR = range(31, 35)


class ModelError(Exception):
    """The chip would execute an unknown (X) instruction."""


def decode(w):
    return dict(op=w >> 26, dly=(w >> 23) & 7, a=(w >> 20) & 7, i=(w >> 19) & 1, b=(w >> 16) & 7, imm=w & 0xFFFF)


class Ctrl:
    def __init__(self, cfg):
        self.cfg = cfg
        self.nm = (1 << cfg.num_pins) - 1
        self.stack = [None] * cfg.stack_depth
        self.reset()

    def reset(self):
        self.slot = self.pc = self.z = self.c = self.to = self.db = self.sp = self.stall = self.blocked = 0
        self.r = [0] * 8

    def ready(self, run):
        return bool(run and not self.slot and self.stall == 0)

    def _cond(self, code, s):
        table = (1, self.z, not self.z, self.c, not self.c, self.to, not self.to, s["erx"], not s["erx"],
                 s["tx_busy"], not s["tx_busy"], s["rx_busy"], not s["rx_busy"], s["etx"], not s["etx"],
                 s["ferr"], not s["ferr"], s["txf_empty"], not s["txf_empty"], s["txf_full"], s["rxf_empty"],
                 not s["rxf_empty"], s["rxf_full"], self.db, not self.db, s["rxrdy"], not s["rxrdy"])
        return bool(table[code]) if code < len(table) else False

    def comb(self, run, word, s, rd_data, dm_rdata):
        """s: status inputs. rd_data(unit, idx), dm_rdata(addr). Returns the cycle's effects."""
        e = dict(wm=0, wv=0, om=0, ov=0, txf_rd=0, rxf_rd=0, txf_wr=0, rxf_wr=0, fdata=0, dm_we=0, dm_addr=0,
                 dm_wdata=0, cfg_we=0, cfg_unit=0, cfg_idx=0, cfg_data=0, rd_unit=0, rd_idx=0, rd_strobe=0,
                 start_mask=0, start_imm=0, start_data=0, stop_mask=0, det_sel=0, det_clr=0, det_arm=0,
                 det_disarm=0, host_irq=0, ill=0, stk=0, ready=0, db_clr=0, done=1, nxt=None)
        if not self.ready(run):
            return e
        if word is None:
            raise ModelError(f"unknown instruction at pc {self.pc}")
        d = decode(word)
        op, fa, fb, imm = d["op"], d["a"], d["b"], d["imm"]
        r = self.r
        ra = r[fa]
        rb = r[fb]
        op2 = imm if d["i"] else r[imm & 7]
        pin = imm >> 12
        lvl = (imm >> 11) & 1
        pin_in = s["pin_in"]
        pin_bit = (pin_in >> pin) & 1
        one = (1 << pin) & self.nm
        target = (imm & 0x3FF) & ((1 << self.cfg.aw) - 1)
        done, jump, push, pop = 1, 0, 0, 0
        wa, wb = None, None                # register writes
        z, c, to = self.z, self.c, self.to
        wm = wv = om = ov = 0
        e["ready"] = 1
        e["start_imm"] = fb & 1
        e["start_data"] = op2
        e["cfg_unit"], e["cfg_idx"], e["cfg_data"] = fa, fb, op2
        e["rd_unit"], e["rd_idx"] = fb, imm & 7
        e["det_sel"] = fa & 1
        e["fdata"] = ra & 0xFF
        e["dm_addr"], e["dm_wdata"] = rb & 0xFF, ra & 0xFF

        if op in (OP_NOP, OP_DELAY):
            pass
        elif op == OP_JMP:
            jump = self._cond(imm >> 11, s)
        elif op == OP_DJNZ:
            wa = (ra - 1) & 0xFFFF
            jump = wa != 0
        elif op == OP_CALL:
            if self.sp == self.cfg.stack_depth:
                e["stk"] = 1
            else:
                push, jump = 1, 1
        elif op == OP_RET:
            if self.sp == 0:
                e["stk"] = 1
            else:
                pop = 1
        elif op == OP_MOV:
            wa = op2
        elif op == OP_ADD:
            x = ra + op2
            wa, z, c = x & 0xFFFF, int(x & 0xFFFF == 0), x >> 16
        elif op in (OP_SUB, OP_CMP):
            x = (ra - op2) & 0x1FFFF
            z, c = int(x & 0xFFFF == 0), x >> 16
            if op == OP_SUB:
                wa = x & 0xFFFF
        elif op in (OP_AND, OP_OR, OP_XOR):
            wa = ra & op2 if op == OP_AND else (ra | op2 if op == OP_OR else ra ^ op2)
            z = int(wa == 0)
        elif op == OP_SHL:
            n = imm & 15
            wa = (ra << n) & 0xFFFF
            z = int(wa == 0)
            c = ((ra << n) >> 16) & 1 if n else 0
        elif op == OP_SHR:
            n = imm & 15
            wa = ra >> n
            z = int(wa == 0)
            c = (ra >> (n - 1)) & 1 if n else 0
        elif op == OP_SET:
            wm = one
            v = (self.c ^ (imm & 1)) if imm & 2 else imm & 1
            wv = self.nm if v else 0
        elif op == OP_SETOE:
            om = one
            ov = self.nm if imm & 1 else 0
        elif op == OP_WAIT:
            if pin_bit == lvl:
                if d["i"]:
                    to = 0
            elif d["i"] and ra == 0:
                to = 1
            else:
                done = 0
                if d["i"]:
                    wa = (ra - 1) & 0xFFFF
        elif op == OP_JPIN:
            jump = pin_bit == lvl
        elif op == OP_RDPINS:
            wa = pin_in
        elif op == OP_BITIN:
            wa = ((ra << 1) | pin_bit) & 0xFFFF if imm & 1 else (pin_bit << 15) | (ra >> 1)
        elif op == OP_BITOUT:
            wm = one
            wv = self.nm if ((ra >> 15) if imm & 1 else ra & 1) else 0
            wa = (ra << 1) & 0xFFFF if imm & 1 else ra >> 1
        elif op == OP_FGET:
            f_rx = fb & 1
            if not (s["rxf_rd_gnt"] if f_rx else s["txf_rd_gnt"]):
                done = 0
            elif s["rxf_empty"] if f_rx else s["txf_empty"]:
                if imm & 1:
                    c = 1
                else:
                    done = 0
            else:
                e["rxf_rd" if f_rx else "txf_rd"] = 1
                wa = s["rxf_rdata"] if f_rx else s["txf_rdata"]
                if imm & 1:
                    c = 0
        elif op == OP_FPUT:
            f_rx = fb & 1
            if not (s["rxf_wr_gnt"] if f_rx else s["txf_wr_gnt"]):
                done = 0
            elif s["rxf_full"] if f_rx else s["txf_full"]:
                if imm & 1:
                    c = 1
                else:
                    done = 0
            else:
                e["rxf_wr" if f_rx else "txf_wr"] = 1
                if imm & 1:
                    c = 0
        elif op in (OP_LD, OP_ST):
            if not s["dm_gnt"]:
                done = 0
            else:
                if op == OP_ST:
                    e["dm_we"] = 1
                else:
                    wa = dm_rdata(rb & 0xFF)
                if imm & 1:
                    wb = (rb + 1) & 0xFFFF
        elif op == OP_UCFG:
            e["cfg_we"] = 1
        elif op == OP_URD:
            e["rd_strobe"] = 1
            wa = rd_data(fb, imm & 7)
        elif op == OP_USTART:
            m = fa & 3
            busy = ((m & 1) and s["tx_busy"]) or ((m & 2) and s["rx_busy"])
            need = ((m & 1) and s["tx_need"]) or ((m & 2) and s["rx_need"])
            if busy or (need and not fb & 1 and s["txf_empty"]):
                done = 0
            else:
                e["start_mask"] = m
        elif op == OP_USTOP:
            e["stop_mask"] = fa & 3
        elif op == OP_UWAIT:
            m = fa & 3
            done = 0 if ((m & 1) and s["tx_busy"]) or ((m & 2) and s["rx_busy"]) else 1
        elif op == OP_EVT:
            flag = s["erx"] if fa & 1 else s["etx"]
            if fb == 0:
                done = int(bool(flag))
            elif fb == 1:
                done = int(bool(flag))
                e["det_clr"] = int(bool(flag))
            elif fb == 2:
                e["det_clr"] = 1
            elif fb == 3:
                e["det_arm"] = 1
            elif fb == 4:
                e["det_disarm"] = 1
        elif op == OP_HOST:
            e["host_irq"] = int(fb == 0)
            e["db_clr"] = int(fb == 1)
        elif op == OP_PAR:
            c = bin(ra).count("1") & 1
        else:
            e["ill"] = 1

        if done:
            e["wm"], e["wv"], e["om"], e["ov"] = wm, wv, om, ov
            stall = d["dly"] + (op2 if op == OP_DELAY else 0)
            if pop:
                pc = self.stack[self.sp - 1]
            elif jump:
                pc = target
            else:
                pc = (self.pc + 1) & ((1 << self.cfg.aw) - 1)
            ret = (self.pc + 1) & ((1 << self.cfg.aw) - 1)
            e["nxt"] = dict(pc=pc, z=z, c=c, to=to, stall=stall, blocked=0, wa=wa, wb=wb, a=fa, b=fb,
                            push=ret if push else None, pop=pop)
        else:
            e["cfg_we"] = e["cfg_we"]
            e["nxt"] = dict(pc=self.pc, z=self.z, c=self.c, to=self.to, stall=0, blocked=1,
                            wa=wa if op == OP_WAIT else None, wb=None, a=fa, b=fb, push=None, pop=0)
            for k in ("txf_rd", "rxf_rd", "txf_wr", "rxf_wr", "dm_we", "start_mask", "det_clr"):
                e[k] = 0
        e["done"] = done
        return e

    def clock(self, rst, restart, start_pc, db_set, e):
        if rst:
            self.reset()
            return
        self.slot ^= 1
        if db_set:
            self.db = 1
        elif e["db_clr"] and e["ready"]:
            self.db = 0
        if restart:
            self.pc, self.z, self.c, self.to, self.sp, self.stall, self.blocked = start_pc, 0, 0, 0, 0, 1, 0
            self.r = [0] * 8
        elif e["ready"]:
            n = e["nxt"]
            self.pc, self.z, self.c, self.to, self.stall, self.blocked = (
                n["pc"], n["z"], n["c"], n["to"], n["stall"], n["blocked"])
            if n["wb"] is not None:
                self.r[n["b"]] = n["wb"]
            if n["wa"] is not None:
                self.r[n["a"]] = n["wa"]
            if n["push"] is not None:
                self.stack[self.sp] = n["push"]
                self.sp += 1
            if n["pop"]:
                self.sp -= 1
        elif self.stall:
            self.stall -= 1
