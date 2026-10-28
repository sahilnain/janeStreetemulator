# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_pins: unit test for src/io/pe_pins.v (pin fabric).

What is checked
---------------
A Python model keeps the out/oe registers and the sticky owner_err.
pad_out/pad_oe are compared every cycle.
1. reset_state: out = all ones (idle high), oe = 0, so no pin drives after reset.
2. random_owner_writes: random owner map, random context, random
   mask/value writes to out and oe, random open-drain map. Only pins owned
   by the writing context change; any non-owned bit in an enabled mask sets
   owner_err; clr_err clears it (a new violation in the same cycle wins).
3. open_drain: with pin_od = 1, pad_out is always 0 and pad_oe = !out;
   the oe register has no effect.
4. input_sync: pin_in follows pad_in with exactly 2 cycles of latency.
"""

import random

import cocotb
from tbutil import fall, param, start, v


class Model:
    def __init__(self, np_, nctx):
        self.np = np_
        self.m = (1 << np_) - 1
        self.ctx_w = max(1, (nctx - 1).bit_length())
        self.out = self.m
        self.oe = 0
        self.err = 0

    def own_mask(self, owner, ctx):
        mw = (1 << self.ctx_w) - 1
        return sum(1 << i for i in range(self.np) if ((owner >> (i * self.ctx_w)) & mw) == ctx)

    def step(self, owner, ctx, wr_en, wmask, wval, oe_en, omask, oval, clr):
        own = self.own_mask(owner, ctx)
        wh = wmask & own if wr_en else 0
        oh = omask & own if oe_en else 0
        denied = (wr_en and (wmask & ~own & self.m)) or (oe_en and (omask & ~own & self.m))
        self.out = (self.out & ~wh) | (wval & wh)
        self.oe = (self.oe & ~oh) | (oval & oh)
        if denied:
            self.err = 1
        elif clr:
            self.err = 0

    def pads(self, od):
        pad_out = self.out & ~od & self.m
        pad_oe = ((od & ~self.out) | (~od & self.oe)) & self.m
        return pad_out, pad_oe


def zero_inputs(dut):
    for s in ("wr_en", "wr_ctx", "wr_mask", "wr_val", "oe_en", "oe_mask", "oe_val",
              "pin_owner", "pin_od", "clr_err", "pad_in"):
        getattr(dut, s).value = 0


async def init(dut):
    zero_inputs(dut)
    await start(dut)
    return Model(param("NUM_PINS", 8), param("NUM_CTX", 2))


def check(dut, m, od):
    po, pe = m.pads(od)
    assert v(dut.pad_out) == po, f"pad_out {v(dut.pad_out):x} != {po:x}"
    assert v(dut.pad_oe) == pe, f"pad_oe {v(dut.pad_oe):x} != {pe:x}"
    assert v(dut.owner_err) == m.err


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def reset_state(dut):
    m = await init(dut)
    check(dut, m, 0)
    assert v(dut.pad_oe) == 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_owner_writes(dut):
    m = await init(dut)
    nctx = param("NUM_CTX", 2)
    for i in range(2000):
        if i % 100 == 0:
            owner = sum(random.randrange(nctx) << (p * m.ctx_w) for p in range(m.np))
            od = random.getrandbits(m.np)
            dut.pin_owner.value, dut.pin_od.value = owner, od
        ctx = random.randrange(nctx)
        args = (int(random.random() < 0.6), random.getrandbits(m.np), random.getrandbits(m.np),
                int(random.random() < 0.6), random.getrandbits(m.np), random.getrandbits(m.np))
        clr = int(random.random() < 0.1)
        dut.wr_ctx.value = ctx
        (dut.wr_en.value, dut.wr_mask.value, dut.wr_val.value,
         dut.oe_en.value, dut.oe_mask.value, dut.oe_val.value) = args
        dut.clr_err.value = clr
        m.step(owner, ctx, *args, clr)
        await fall(dut)
        check(dut, m, od)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def open_drain(dut):
    m = await init(dut)
    od = m.m
    dut.pin_od.value = od
    for _ in range(200):
        val, oe = random.getrandbits(m.np), random.getrandbits(m.np)
        dut.wr_en.value, dut.wr_mask.value, dut.wr_val.value = 1, m.m, val
        dut.oe_en.value, dut.oe_mask.value, dut.oe_val.value = 1, m.m, oe
        m.step(0, 0, 1, m.m, val, 1, m.m, oe, 0)
        await fall(dut)
        check(dut, m, od)
        assert v(dut.pad_out) == 0
        assert v(dut.pad_oe) == (~val & m.m)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def input_sync(dut):
    m = await init(dut)
    hist = []
    for i in range(300):
        x = random.getrandbits(m.np)
        dut.pad_in.value = x
        hist.append(x)
        await fall(dut)
        if i >= 1:
            assert v(dut.pin_in) == hist[i - 1]
