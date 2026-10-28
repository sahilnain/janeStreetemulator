# SPDX-License-Identifier: Apache-2.0
"""
isa: instruction set of the Protocol Emulator (reference definition).

What this file does
-------------------
The single Python source of truth for the ISA, used by the reference model,
the tests (test/common/pe_isa.py re-exports it) and, later, the assembler.
  * encoders: one function per instruction (`set_`, `out`, `jmp`, ...);
  * decode(): instruction word -> fields (mirrors src/core/pe_decode.v);
  * execute(): the semantics of one instruction for one context
    (mirrors src/core/pe_exec.v): next context state, pin/FIFO/vote side
    effects, and whether the instruction completes or is held.

Word layout (IW = 24): [23:19] opcode, [18:16] delay, [15:0] operands.
See the pe_decode.v header for the operand table.
"""

IW_DEFAULT = 24

OP = dict(nop=0, set=1, setoe=2, out=3, in_=4, wait=5, jpin=6, jmp=7, djnz=8,
          mov=9, delay=10, pull=11, push=12, vote=13)
OP_NAMES = {v: k for k, v in OP.items()}

# jmp condition codes
ALWAYS, C, NC, TO, NTO, TXF_EMPTY, RXF_FULL, OSR_EMPTY = range(8)

# set source codes
SRC_0, SRC_1, SRC_C, SRC_NC = range(4)


# ----------------------------------------------------------------- encoders
def word(op, operands=0, dly=0, iw=IW_DEFAULT):
    assert 0 <= dly <= 7 and 0 <= operands <= 0xFFFF
    return (op << (iw - 5)) | (dly << (iw - 8)) | operands


def nop(dly=0):
    return word(OP["nop"], 0, dly)


def set_(pin, src, dly=0):
    return word(OP["set"], (pin << 12) | src, dly)


def setoe(pin, v, dly=0):
    return word(OP["setoe"], (pin << 12) | (v & 1), dly)


def out(pin, n, dly=0):
    assert 1 <= n <= 16
    return word(OP["out"], (pin << 12) | ((n - 1) << 8), dly)


def in_(pin, n, dly=0):
    assert 1 <= n <= 16
    return word(OP["in_"], (pin << 12) | ((n - 1) << 8), dly)


def wait(pin, lvl, to_ctr=None, dly=0):
    """wait pin==lvl. With to_ctr=0/1, give up when that counter reaches 0 (sets TO)."""
    ops = (pin << 12) | (lvl << 11)
    if to_ctr is not None:
        ops |= (1 << 10) | (to_ctr << 9)
    return word(OP["wait"], ops, dly)


def jpin(pin, lvl, target, dly=0):
    return word(OP["jpin"], (pin << 12) | (lvl << 11) | (target & 0xFF), dly)


def jmp(target, cond=ALWAYS, dly=0):
    return word(OP["jmp"], (cond << 8) | (target & 0xFF), dly)


def djnz(ctr, target, dly=0):
    return word(OP["djnz"], (ctr << 8) | (target & 0xFF), dly)


def mov(ctr, imm, dly=0):
    assert 0 <= imm < (1 << 15)
    return word(OP["mov"], (ctr << 15) | imm, dly)


def delay(n, dly=0):
    assert 0 <= n <= 0xFFFF
    return word(OP["delay"], n, dly)


def pull(nb=0, dly=0):
    return word(OP["pull"], nb & 1, dly)


def push(nb=0, dly=0):
    return word(OP["push"], nb & 1, dly)


def vote(pin, dly=0):
    return word(OP["vote"], pin << 12, dly)


# ------------------------------------------------------------------ decode
def decode(w, iw=IW_DEFAULT, aw=6):
    """Instruction word -> fields. Same names/values as pe_decode.v outputs,
    plus 'op': the op name ('nop' for opcode 0 and for reserved opcodes)."""
    opc = (w >> (iw - 5)) & 0x1F
    name = OP_NAMES.get(int(opc))  # int(): concrete key, also when w is symbolic
    f = {f"op_{k.rstrip('_')}": int(name == k) for k in OP if k != "nop"}
    f["op_illegal"] = int(opc > OP["vote"])
    f["op"] = (name or "nop").rstrip("_")
    f["dly"] = (w >> (iw - 8)) & 7
    f["pin"] = (w >> 12) & 0xF
    f["n"] = ((w >> 8) & 0xF) + 1
    f["lvl"] = (w >> 11) & 1
    f["to_en"] = (w >> 10) & 1
    f["to_ctr"] = (w >> 9) & 1
    f["cond"] = (w >> 8) & 7
    f["ctr_sel"] = (w >> 15) & 1 if name == "mov" else (w >> 8) & 1
    f["src"] = w & 3
    f["nb"] = w & 1
    f["target"] = w & ((1 << aw) - 1)
    f["imm"] = (w & 0x7FFF) if name == "mov" else (w & 0xFFFF)
    return f


# ----------------------------------------------------------------- execute
def execute(cfg, f, s, e):
    """Semantics of one instruction (see the pe_exec.v header).

    cfg: Config (uses aw, num_pins, sr_w, data_w, ctr_w)
    f:   decoded fields (from decode())
    s:   context state: pc, ctr0, ctr1, osr, osr_cnt, isr, c, to,
         shift_msb, vote_avail
    e:   environment: pin_in (synchronised), txf_rdata, txf_empty, rxf_full
    Returns a dict with the next state (pc_n, ctr0_n, ...), done, stall_n,
    and the side-effect requests (pin_wr_*, pin_oe_*, txf_rd, rxf_wr,
    rxf_wdata, vote_start, vote_pin).
    """
    SRM = (1 << cfg.sr_w) - 1
    PM = (1 << cfg.num_pins) - 1
    DM = (1 << cfg.data_w) - 1
    CM = (1 << cfg.ctr_w) - 1
    o = dict(done=1, ctr0_n=s["ctr0"], ctr1_n=s["ctr1"], osr_n=s["osr"], osr_cnt_n=s["osr_cnt"],
             isr_n=s["isr"], c_n=s["c"], to_n=s["to"], pin_wr_en=0, pin_wr_mask=0, pin_wr_val=0,
             pin_oe_en=0, pin_oe_mask=0, pin_oe_val=0, txf_rd=0, rxf_wr=0, vote_start=0,
             vote_pin=f["pin"])
    msb, pin = s["shift_msb"], f["pin"]
    n = min(f["n"], cfg.sr_w)
    nmask = (1 << n) - 1
    pin_x = e["pin_in"] >> pin
    pin_bit = pin_x & 1
    isr, osr = s["isr"], s["osr"]
    o["rxf_wdata"] = (isr & DM) if msb else ((isr >> (cfg.sr_w - cfg.data_w)) & DM)
    jump = 0
    op = f["op"]

    if op == "set":
        src = f["src"]
        b = (s["c"] ^ (src & 1)) if src & 2 else (src & 1)
        o.update(pin_wr_en=1, pin_wr_mask=(1 << pin) & PM, pin_wr_val=PM if b else 0)
    elif op == "setoe":
        o.update(pin_oe_en=1, pin_oe_mask=(1 << pin) & PM, pin_oe_val=PM if f["src"] & 1 else 0)
    elif op == "out":
        bits = (osr >> (cfg.sr_w - n)) if msb else (osr & nmask)
        o["osr_n"] = ((osr << n) & SRM) if msb else (osr >> n)
        o["osr_cnt_n"] = max(s["osr_cnt"] - n, 0)
        o.update(pin_wr_en=1, pin_wr_mask=(nmask << pin) & PM, pin_wr_val=(bits << pin) & PM)
    elif op == "in":
        ib = pin_x & nmask
        o["isr_n"] = (((isr << n) | ib) & SRM) if msb else ((isr >> n) | (ib << (cfg.sr_w - n))) & SRM
    elif op == "wait":
        ct = s["ctr1"] if f["to_ctr"] else s["ctr0"]
        if pin_bit == f["lvl"]:
            if f["to_en"]:
                o["to_n"] = 0
        elif f["to_en"] and ct == 0:
            o["to_n"] = 1
        else:
            o["done"] = 0
            if f["to_en"]:
                o["ctr1_n" if f["to_ctr"] else "ctr0_n"] = (ct - 1) & CM
    elif op == "jpin":
        jump = int(pin_bit == f["lvl"])
    elif op == "jmp":
        jump = [1, s["c"], 1 - s["c"], s["to"], 1 - s["to"], e["txf_empty"], e["rxf_full"],
                int(s["osr_cnt"] == 0)][f["cond"]]
    elif op == "djnz":
        dec = (s["ctr1" if f["ctr_sel"] else "ctr0"] - 1) & CM
        o["ctr1_n" if f["ctr_sel"] else "ctr0_n"] = dec
        jump = int(dec != 0)
    elif op == "mov":
        o["ctr1_n" if f["ctr_sel"] else "ctr0_n"] = f["imm"] & CM
    elif op == "pull":
        if not e["txf_empty"]:
            o["txf_rd"] = 1
            o["osr_n"] = (e["txf_rdata"] << (cfg.sr_w - cfg.data_w)) if msb else e["txf_rdata"]
            o["osr_cnt_n"] = cfg.data_w
        elif not f["nb"]:
            o["done"] = 0
    elif op == "push":
        if e["rxf_full"] and not f["nb"]:
            o["done"] = 0
        else:
            o["rxf_wr"] = 1
            o["isr_n"] = 0
    elif op == "vote":
        o["vote_start"] = s["vote_avail"]

    if o["done"]:
        o["pc_n"] = f["target"] if jump else (s["pc"] + 1) & ((1 << cfg.aw) - 1)
        o["stall_n"] = f["dly"] + (f["imm"] if op == "delay" else 0)
    else:
        o["pc_n"] = s["pc"]
        o["stall_n"] = 0
    return o
