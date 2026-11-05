# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_ctrl: unit test for src/core/pe_ctrl.v (the controller).

What is checked
---------------
`Ctrl` is a cycle model of docs/spec/isa.md + blocks.md Section 2: slot
issue every 2 cycles, stall counter, blocking (no side effects), all 35
opcodes, the 27 jump conditions, reserved opcodes (ill_err), stack
overflow / underflow (stk_err), restart, RUN and the doorbell. The
testbench emulates pe_imem's registered read (im_rdata = prog[im_raddr of
the previous cycle]) and compares EVERY output port of pe_ctrl in EVERY
cycle. The operand ports (cfg_data = op2, f_wdata = rA, dm_addr = rB) are
not gated, so the register file is checked continuously as well.
1. random_programs: 12 random programs x 3000 cycles. Opcodes are weighted
   so that every opcode (and reserved ones) occurs; the environment (FIFO
   flags and data, grants, unit busy/need, flags, pins, DMEM data, urd data,
   run, restart, doorbell) is random every cycle.
2. timing: issue spacing is 2 cycles; dly d costs 2*ceil((d+1)/2); delay
   rX loads the stall from a register.
3. restart_run: RESTART loads START_PC and clears state; the stale fetch is
   not issued; RUN = 0 holds the controller.
4. stack: call depth = STACK_DEPTH works, one more -> stk_err (nop);
   ret with an empty stack -> stk_err.
"""

import random

import cocotb
import pe_isa as I
from tbutil import fall, param, settle, start, v

DEPTH = param("IMEM_DEPTH", 64)
NP = param("NUM_PINS", 8)
SD = param("STACK_DEPTH", 4)
AW = max(1, (DEPTH - 1).bit_length())
AM = (1 << AW) - 1
PM = (1 << NP) - 1

INS = ("run", "restart", "start_pc", "db_set", "pin_in", "txf_empty", "txf_full", "txf_rdata",
       "rxf_empty", "rxf_full", "rxf_rdata", "txf_rd_gnt", "txf_wr_gnt", "rxf_rd_gnt",
       "rxf_wr_gnt", "dm_rdata", "dm_gnt", "rd_data", "tx_busy", "rx_busy", "tx_need",
       "rx_need", "etx_flag", "erx_flag", "ferr", "rxrdy")
OUTS = ("im_raddr", "pw_mask", "pw_val", "po_mask", "po_val", "txf_rd", "rxf_rd", "txf_wr",
        "rxf_wr", "f_wdata", "dm_addr", "dm_we", "dm_wdata", "cfg_we", "cfg_unit", "cfg_idx",
        "cfg_data", "rd_unit", "rd_idx", "rd_strobe", "start_mask", "start_imm", "start_data",
        "stop_mask", "det_sel", "det_clr", "det_arm", "det_disarm", "host_irq", "ill_err",
        "stk_err", "pc", "z_o", "c_o", "to_o", "db_o", "blocked")


class Ctrl:
    def __init__(self):
        self.slot = 0
        self.pc = 0
        self.r = [0] * 8
        self.z = self.c = self.to = self.db = 0
        self.stack = []
        self.stall = 0
        self.blocked = 0

    def cond(self, k, e):
        t = {0: 1, 1: self.z, 2: not self.z, 3: self.c, 4: not self.c, 5: self.to, 6: not self.to,
             7: e["erx_flag"], 8: not e["erx_flag"], 9: e["tx_busy"], 10: not e["tx_busy"],
             11: e["rx_busy"], 12: not e["rx_busy"], 13: e["etx_flag"], 14: not e["etx_flag"],
             15: e["ferr"], 16: not e["ferr"], 17: e["txf_empty"], 18: not e["txf_empty"],
             19: e["txf_full"], 20: e["rxf_empty"], 21: not e["rxf_empty"], 22: e["rxf_full"],
             23: self.db, 24: not self.db, 25: e["rxrdy"], 26: not e["rxrdy"]}
        return int(bool(t.get(k, 0)))

    def comb(self, w, e):
        """Outputs and the next-state effects of this cycle."""
        op, dly, a, fi, b, imm = w >> 26, (w >> 23) & 7, (w >> 20) & 7, (w >> 19) & 1, (w >> 16) & 7, w & 0xFFFF
        r = self.r
        op2 = imm if fi else r[imm & 7]
        pin = (imm >> 12) & 0xF
        lvl = (imm >> 11) & 1
        pbit = (e["pin_in"] >> pin) & 1 if pin < NP else 0
        onem = (1 << pin) & PM if pin < NP else 0
        tgt = imm & 0x3FF & AM
        ready = e["run"] and not self.slot and self.stall == 0
        o = {k: 0 for k in OUTS}
        o.update(im_raddr=self.pc, f_wdata=r[a] & 0xFF, dm_addr=r[b] & 0xFF, dm_wdata=r[a] & 0xFF,
                 cfg_unit=a, cfg_idx=b, cfg_data=op2, rd_unit=b, rd_idx=imm & 7, start_imm=b & 1,
                 start_data=op2, det_sel=a & 1, pc=self.pc, z_o=self.z, c_o=self.c,
                 to_o=self.to, db_o=self.db, blocked=self.blocked)
        x = dict(done=1, jump=0, wa=None, wb=None, z=self.z, c=self.c, to=self.to, push=0, pop=0,
                 dbclr=0, ill=0, stk=0)
        fx = {}                                    # side-effect outputs (gated by ready)

        def wr(val):
            x["wa"] = val & 0xFFFF

        def alu_flags(res, carry=None):
            x["z"] = int(res & 0xFFFF == 0)
            if carry is not None:
                x["c"] = carry

        umask = a & 3
        ubusy = (umask & 1 and e["tx_busy"]) or (umask & 2 and e["rx_busy"])
        need = (umask & 1 and e["tx_need"]) or (umask & 2 and e["rx_need"])
        flag = e["erx_flag"] if a & 1 else e["etx_flag"]
        if op == 0 or op == 6:
            pass
        elif op == 1:
            x["jump"] = self.cond((imm >> 11) & 0x1F, e)
        elif op == 2:
            d = (r[a] - 1) & 0xFFFF
            wr(d)
            x["jump"] = int(d != 0)
        elif op == 3:
            if len(self.stack) == SD:
                x["stk"] = 1
            else:
                x["push"], x["jump"] = 1, 1
        elif op == 4:
            if not self.stack:
                x["stk"] = 1
            else:
                x["pop"] = 1
        elif op == 7:
            wr(op2)
        elif op == 8:
            s = r[a] + op2
            wr(s)
            alu_flags(s, s >> 16)
        elif op in (9, 15):
            s = r[a] - op2
            if op == 9:
                wr(s)
            alu_flags(s, int(s < 0))
        elif op in (10, 11, 12):
            s = r[a] & op2 if op == 10 else r[a] | op2 if op == 11 else r[a] ^ op2
            wr(s)
            alu_flags(s)
        elif op == 13:
            n = imm & 0xF
            s = (r[a] << n)
            wr(s)
            alu_flags(s, (s >> 16) & 1 if n else 0)
        elif op == 14:
            n = imm & 0xF
            s = r[a] >> n
            wr(s)
            alu_flags(s, (r[a] >> (n - 1)) & 1 if n else 0)
        elif op == 16:
            val = (self.c ^ (imm & 1)) if imm & 2 else imm & 1
            fx.update(pw_mask=onem, pw_val=onem if val else 0)
        elif op == 17:
            fx.update(po_mask=onem, po_val=onem if imm & 1 else 0)
        elif op == 18:
            if pbit == lvl:
                if fi:
                    x["to"] = 0
            elif fi and r[a] == 0:
                x["to"] = 1
            else:
                x["done"] = 0
                if fi:
                    wr(r[a] - 1)
        elif op == 19:
            x["jump"] = int(pbit == lvl)
        elif op == 20:
            wr(e["pin_in"] & 0xFFFF)
        elif op == 21:
            wr(((r[a] << 1) | pbit) if imm & 1 else ((pbit << 15) | (r[a] >> 1)))
        elif op == 22:
            bitv = (r[a] >> 15) & 1 if imm & 1 else r[a] & 1
            fx.update(pw_mask=onem, pw_val=onem if bitv else 0)
            wr((r[a] << 1) if imm & 1 else (r[a] >> 1))
        elif op in (23, 24):
            rx = b & 1
            gnt = e[("rxf_" if rx else "txf_") + ("rd_gnt" if op == 23 else "wr_gnt")]
            blk = e[("rxf_" if rx else "txf_") + ("empty" if op == 23 else "full")]
            if not gnt:
                x["done"] = 0
            elif blk:
                if imm & 1:
                    x["c"] = 1
                else:
                    x["done"] = 0
            else:
                fx[("rxf_" if rx else "txf_") + ("rd" if op == 23 else "wr")] = 1
                if op == 23:
                    wr(e["rxf_rdata"] if rx else e["txf_rdata"])
                if imm & 1:
                    x["c"] = 0
        elif op in (25, 26):
            if not e["dm_gnt"]:
                x["done"] = 0
            else:
                if op == 26:
                    fx["dm_we"] = 1
                else:
                    wr(e["dm_rdata"])
                if imm & 1:
                    x["wb"] = (b, (r[b] + 1) & 0xFFFF)
        elif op == 27:
            fx["cfg_we"] = 1
        elif op == 28:
            fx["rd_strobe"] = 1
            wr(e["rd_data"])
        elif op == 29:
            if ubusy or (need and not (b & 1) and e["txf_empty"]):
                x["done"] = 0
            else:
                fx["start_mask"] = umask
        elif op == 30:
            fx["stop_mask"] = umask
        elif op == 31:
            x["done"] = int(not ubusy)
        elif op == 32:
            if b == 0:
                x["done"] = flag
            elif b == 1:
                x["done"] = flag
                if flag:
                    fx["det_clr"] = 1
            elif b == 2:
                fx["det_clr"] = 1
            elif b == 3:
                fx["det_arm"] = 1
            elif b == 4:
                fx["det_disarm"] = 1
        elif op == 33:
            if b == 0:
                fx["host_irq"] = 1
            elif b == 1:
                x["dbclr"] = 1
        elif op == 34:
            x["c"] = bin(self.r[a]).count("1") & 1
        else:
            x["ill"] = 1
        if not x["done"]:
            # blocked: no side effects; only a timed-out wait's counter may change
            for k in ("pw_mask", "po_mask", "txf_rd", "rxf_rd", "txf_wr", "rxf_wr", "dm_we",
                      "start_mask", "det_clr"):
                fx.pop(k, None)
            x.update(z=self.z, c=self.c, to=self.to, push=0, pop=0, wb=None)
            if op != 18:
                x["wa"] = None
        if ready:
            o.update(fx)
            o["ill_err"], o["stk_err"] = x["ill"], x["stk"]
        x.update(ready=ready, op=op, a=a, dly=dly, op2=op2, tgt=tgt)
        return o, x

    def step(self, x, e):
        db_set, restart = e["db_set"], e["restart"]
        if db_set:
            self.db = 1
        elif x["ready"] and x["dbclr"]:
            self.db = 0
        self.slot ^= 1
        if restart:
            self.pc = e["start_pc"]
            self.z = self.c = self.to = 0
            self.stack = []
            self.stall = 1
            self.blocked = 0
            self.r = [0] * 8
        elif x["ready"]:
            done = x["done"]
            if not done:
                pc_n = self.pc
            elif x["pop"]:
                pc_n = self.stack[-1]
            elif x["jump"]:
                pc_n = x["tgt"]
            else:
                pc_n = (self.pc + 1) & AM
            if x["push"]:
                self.stack.append((self.pc + 1) & AM)
            if x["pop"]:
                self.stack.pop()
            self.z, self.c, self.to = x["z"], x["c"], x["to"]
            self.stall = (x["dly"] + (x["op2"] if x["op"] == 6 else 0)) if done else 0
            self.blocked = int(not done)
            if x["wb"] is not None:
                self.r[x["wb"][0]] = x["wb"][1]
            if x["wa"] is not None:
                self.r[x["a"]] = x["wa"]
            self.pc = pc_n
        elif self.stall:
            self.stall -= 1


async def init(dut):
    for s in INS + ("im_rdata",):
        getattr(dut, s).value = 0
    await start(dut)


def check(dut, o, t, w):
    for k in OUTS:
        g = v(getattr(dut, k))
        e = o[k]
        if k == "pw_val":
            g &= o["pw_mask"]
        if k == "po_val":
            g &= o["po_mask"]
        assert g == e, f"t={t} word={w:08x} op={w >> 26}: {k} = {g:#x}, expected {e:#x}"


class Harness:
    """Drives pe_ctrl with a program and an environment; compares every cycle."""

    def __init__(self, dut, prog):
        self.dut = dut
        self.prog = prog
        self.m = Ctrl()
        self.raddr = 0
        self.t = 0
        self.ops = set()          # opcodes issued and completed

    async def reset(self):
        """Reset the DUT so it is aligned with a fresh model."""
        self.dut.rst.value = 1
        self.dut.im_rdata.value = self.prog[0]
        for k, val in quiet_env(run=0).items():
            getattr(self.dut, k).value = val
        await fall(self.dut, 2)
        self.dut.rst.value = 0
        self.raddr = 0
        return self

    async def cycle(self, e):
        dut = self.dut
        w = self.prog[self.raddr % len(self.prog)] if self.raddr < len(self.prog) else 0
        dut.im_rdata.value = w
        for k, val in e.items():
            getattr(dut, k).value = val
        await settle()
        o, x = self.m.comb(w, e)
        check(dut, o, self.t, w)
        self.raddr = v(dut.im_raddr)
        if x["ready"] and x["done"]:
            self.ops.add(x["op"])
        self.m.step(x, e)
        await fall(dut)
        self.t += 1
        return o


def quiet_env(**kw):
    e = {k: 0 for k in INS}
    e.update(run=1, txf_empty=1, rxf_empty=1, txf_rd_gnt=1, txf_wr_gnt=1, rxf_rd_gnt=1,
             rxf_wr_gnt=1, dm_gnt=1)
    e.update(kw)
    return e


def rand_env(t):
    e = {k: random.getrandbits(1) for k in INS}
    e.update(run=int(random.random() < 0.95), restart=int(random.random() < 0.002),
             start_pc=random.randrange(DEPTH), db_set=int(random.random() < 0.02),
             pin_in=random.getrandbits(NP), txf_rdata=random.getrandbits(8),
             rxf_rdata=random.getrandbits(8), dm_rdata=random.getrandbits(8),
             rd_data=random.getrandbits(16))
    for g in ("txf_rd_gnt", "txf_wr_gnt", "rxf_rd_gnt", "rxf_wr_gnt", "dm_gnt"):
        e[g] = int(random.random() < 0.8)
    return e


def rand_word():
    r = random.random()
    if r < 0.06:
        op = random.choice(I.RESERVED_OPS)
    else:
        op = random.choice(list(I.OP.values()))
    dly = random.choice([0, 0, 0, 1, 2, 7])
    imm = random.getrandbits(16)
    if op == 6:
        imm &= 0x1F                      # keep delays short
    if op in (1, 2, 3, 19):
        imm = (imm & ~0x3FF) | random.randrange(DEPTH)
    if op == 13 or op == 14:
        imm = random.randrange(17) & 0xF
    w = I.word(op, a=random.randrange(8), i=random.getrandbits(1), b=random.randrange(8), imm=imm, dly=dly)
    if op == 6 and not (w >> 19) & 1:
        pass                             # delay rX: register values are random -> may be long
    return w


@cocotb.test(timeout_time=500, timeout_unit="ms")
async def random_programs(dut):
    await init(dut)
    seen = set()
    for _ in range(12):
        prog = [rand_word() for _ in range(DEPTH)]
        # avoid huge register delays: `delay rX` only with an immediate
        prog = [w | (1 << 19) if (w >> 26) == 6 else w for w in prog]
        h = await Harness(dut, prog).reset()
        e = quiet_env(restart=1, start_pc=0)
        await h.cycle(e)
        for t in range(3000):
            await h.cycle(rand_env(t))
        seen |= h.ops
    missing = set(I.OP.values()) - seen
    assert not missing, f"opcodes never reached: {missing}"
    assert seen & set(I.RESERVED_OPS), "no reserved opcode executed"


async def issue_times(dut, prog, n):
    h = await Harness(dut, prog).reset()
    await h.cycle(quiet_env(restart=1, start_pc=0))
    times = []
    for t in range(n):
        pc_before = h.m.pc
        await h.cycle(quiet_env())
        if h.m.pc != pc_before:
            times.append(t)
    return times, h.m.pc


@cocotb.test()
async def timing(dut):
    await init(dut)
    for d in range(8):
        a = I.Asm()
        for _ in range(4):
            a.nop(dly=d)
        a.L("e").jmp("e")
        times, _ = await issue_times(dut, a.words(), 80)
        gaps = [y - x for x, y in zip(times, times[1:4])]
        exp = 2 * ((d + 1 + 1) // 2)
        assert gaps == [exp] * 3, (d, gaps, exp)
    a = I.Asm().mov(3, 9).delay(I.R(3)).nop().L("e").jmp("e")
    times, _ = await issue_times(dut, a.words(), 60)
    assert times[2] - times[1] == 2 * ((9 + 1 + 1) // 2), times
    # the longest stall: delay 0xFFFF with dly 7 (17-bit stall counter)
    a = I.Asm().delay(0xFFFF, dly=7).nop().L("e").jmp("e")
    times, _ = await issue_times(dut, a.words(), 0xFFFF + 7 + 20)
    assert times[1] - times[0] == 2 * ((0xFFFF + 7 + 2) // 2), times


@cocotb.test()
async def restart_run(dut):
    await init(dut)
    a = I.Asm()
    for k in range(10):
        a.mov(k % 8, 0x100 + k)
    a.L("e").jmp("e")
    h = await Harness(dut, a.words()).reset()
    await h.cycle(quiet_env(restart=1, start_pc=0))
    for _ in range(9):
        await h.cycle(quiet_env())
    pc_mid = h.m.pc
    for _ in range(20):                       # RUN = 0 holds everything
        await h.cycle(quiet_env(run=0))
    assert h.m.pc == pc_mid
    await h.cycle(quiet_env(restart=1, start_pc=5))
    assert h.m.pc == 5 and h.m.r == [0] * 8
    for _ in range(40):
        await h.cycle(quiet_env())
    assert h.m.r[5:8] == [0x105, 0x106, 0x107] and h.m.r[0] == 0x108


@cocotb.test()
async def stack(dut):
    await init(dut)
    a = I.Asm()
    a.call("f0")
    a.ret()                                   # empty stack: stk_err
    a.L("e").jmp("e")
    for n in range(SD + 1):
        a.L(f"f{n}")
        if n < SD:
            a.call(f"f{n + 1}")               # the call at depth SD+1 is a nop
        a.mov(n, 0x40 + n)
        a.ret()
    h = await Harness(dut, a.words()).reset()
    await h.cycle(quiet_env(restart=1, start_pc=0))
    stk = 0
    for _ in range(200):
        o = await h.cycle(quiet_env())
        stk += o["stk_err"]
    assert stk == 2, stk                      # one overflow + one underflow
    assert h.m.r[SD] == 0, "f_SD never ran"
    assert h.m.r[:SD] == [0x40 + n for n in range(SD)]
