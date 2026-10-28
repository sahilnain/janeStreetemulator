# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_exec: unit test for src/core/pe_exec.v (shared datapath, combinational).

What is checked
---------------
The reference is the Python model's `execute()` (model/pe_model/isa.py),
written from the semantics in the pe_exec.v header (shift directions,
blocking/non-blocking FIFO ops, wait with timeout, jmp conditions, stall
loading). For each vector the test drives the decoded fields (from the
model's decode()), a random context state and random environment
(pins, FIFO status), then compares EVERY output of pe_exec with the model.
1. random_all_ops: 6000 random vectors, opcode chosen uniformly from all
   32 (illegal ones must behave as nop).
2. corner_cases: directed vectors for the boundary values (counters at 0
   and 1, osr_cnt below n, n > SR_W, pin index beyond NUM_PINS, FIFO
   full/empty with and without nb).
"""

import random

import cocotb
import pe_isa as I
from pe_model.config import Config
from tbutil import param, settle, v

OPS = ["set", "setoe", "out", "in", "wait", "jpin", "jmp", "djnz", "mov", "delay", "pull",
       "push", "vote"]
FIELD_IN = ["dly", "pin", "n", "lvl", "to_en", "to_ctr", "cond", "ctr_sel", "src", "nb",
            "target", "imm"]
STATE_IN = ["pc", "ctr0", "ctr1", "osr", "osr_cnt", "isr", "c", "to", "shift_msb", "vote_avail"]
ENV_IN = ["pin_in", "txf_rdata", "txf_empty", "rxf_full"]
OUTS = ["done", "pc_n", "ctr0_n", "ctr1_n", "osr_n", "osr_cnt_n", "isr_n", "c_n", "to_n",
        "stall_n", "pin_wr_en", "pin_wr_mask", "pin_wr_val", "pin_oe_en", "pin_oe_mask",
        "pin_oe_val", "txf_rd", "rxf_wr", "rxf_wdata", "vote_start", "vote_pin"]


class Cfg:
    """pe_exec parameters for this run, in the reference model's Config terms."""

    def __init__(self):
        self.aw = param("AW", 6)
        self.np = param("NUM_PINS", 8)
        self.sr = param("SR_W", 16)
        self.dw = param("DATA_W", 8)
        self.cw = param("CTR_W", 16)
        self.model = Config(imem_depth=1 << self.aw, num_pins=self.np, sr_w=self.sr,
                            data_w=self.dw, ctr_w=self.cw)


def ref_exec(cfg, f, s, e):
    """The reference: model/pe_model/isa.py execute()."""
    return I.execute(cfg.model, f, s, e)


def random_state(cfg):
    return dict(pc=random.getrandbits(cfg.aw),
                ctr0=random.choice([0, 1, 2, random.getrandbits(cfg.cw)]),
                ctr1=random.choice([0, 1, 2, random.getrandbits(cfg.cw)]),
                osr=random.getrandbits(cfg.sr), osr_cnt=random.randint(0, cfg.sr),
                isr=random.getrandbits(cfg.sr), c=random.getrandbits(1), to=random.getrandbits(1),
                shift_msb=random.getrandbits(1), vote_avail=random.getrandbits(1))


def random_env(cfg):
    return dict(pin_in=random.getrandbits(cfg.np), txf_rdata=random.getrandbits(cfg.dw),
                txf_empty=random.getrandbits(1), rxf_full=random.getrandbits(1))


async def apply(dut, cfg, w, s, e):
    f = I.decode(w, 24, cfg.aw)
    opname = f["op"]
    for k in OPS:
        getattr(dut, f"op_{k}").value = int(opname == k)
    for k in FIELD_IN:
        getattr(dut, k).value = f[k]
    for k in STATE_IN:
        getattr(dut, k).value = s[k]
    for k in ENV_IN:
        getattr(dut, k).value = e[k]
    await settle()
    ref = ref_exec(cfg, f, s, e)
    for k in OUTS:
        got = v(getattr(dut, k))
        assert got == ref[k], f"op={opname} word={w:06x} {k}: got {got:x}, expected {ref[k]:x}\nstate={s}\nenv={e}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_all_ops(dut):
    cfg = Cfg()
    for _ in range(6000):
        w = (random.randrange(32) << 19) | random.getrandbits(19)
        await apply(dut, cfg, w, random_state(cfg), random_env(cfg))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def corner_cases(dut):
    cfg = Cfg()
    s0 = dict(pc=(1 << cfg.aw) - 1, ctr0=0, ctr1=1, osr=(1 << cfg.sr) - 1, osr_cnt=1,
              isr=0x5A5 & ((1 << cfg.sr) - 1), c=1, to=0, shift_msb=0, vote_avail=0)
    words = [
        I.djnz(0, 3), I.djnz(1, 3),               # counter wraps / reaches 0
        I.out(0, 16), I.out(0, 1), I.in_(0, 16),  # n > SR_W possible, osr_cnt saturates
        I.out(15, 4), I.set_(15, I.SRC_NC), I.setoe(15, 1),  # pin beyond NUM_PINS
        I.wait(15, 0, to_ctr=0), I.wait(15, 1, to_ctr=1), I.wait(15, 1),
        I.pull(0), I.pull(1), I.push(0), I.push(1), I.vote(3),
        I.jmp(5, I.OSR_EMPTY), I.delay(0xFFFF, dly=7), I.nop(dly=7),
    ]
    for w in words:
        for msb in (0, 1):
            for empty_full in ((0, 0), (1, 1)):
                s = dict(s0, shift_msb=msb)
                e = dict(pin_in=0, txf_rdata=0xA5 & ((1 << cfg.dw) - 1),
                         txf_empty=empty_full[0], rxf_full=empty_full[1])
                await apply(dut, cfg, w, s, e)
