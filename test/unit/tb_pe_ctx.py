# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_ctx: unit test for src/core/pe_ctx.v (one context's state).

What is checked
---------------
1. reset_and_run: after reset all state is 0 and ready follows `run`.
2. writeback: `we` stores every next-state field (random values, masked to
   their widths).
3. stall_countdown: a loaded stall of d keeps ready low for exactly d
   cycles (d = 1..20, plus 0 = no stall).
4. restart: PC <- start_pc, state cleared, ready low for exactly one cycle.
5. vote_result: vote_busy forces ready low. vote_done shifts vote_bit into
   ISR (right shift / top entry for LSB-first, left shift for MSB-first) and
   copies it to C.
"""

import random

import cocotb
from tbutil import fall, param, start, v

STATE = ["pc", "ctr0", "ctr1", "osr", "osr_cnt", "isr", "c", "to"]


def widths():
    sr = param("SR_W", 16)
    return dict(pc=param("AW", 6), ctr0=param("CTR_W", 16), ctr1=param("CTR_W", 16), osr=sr,
                osr_cnt=sr.bit_length(), isr=sr, c=1, to=1)


async def init(dut):
    for s in ("run", "restart", "start_pc", "shift_msb", "we", "vote_busy", "vote_done", "vote_bit"):
        getattr(dut, s).value = 0
    for s in STATE + ["stall"]:
        getattr(dut, s + "_n").value = 0
    await start(dut)


async def write(dut, vals, stall=0):
    for s, x in vals.items():
        getattr(dut, s + "_n").value = x
    dut.stall_n.value = stall
    dut.we.value = 1
    await fall(dut)
    dut.we.value = 0


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def reset_and_run(dut):
    await init(dut)
    for s in STATE:
        assert v(getattr(dut, s)) == 0, s
    assert v(dut.ready) == 0
    dut.run.value = 1
    await fall(dut)
    assert v(dut.ready) == 1


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def writeback(dut):
    await init(dut)
    W = widths()
    for _ in range(100):
        vals = {s: random.getrandbits(W[s]) for s in STATE}
        vals["osr_cnt"] = random.randint(0, param("SR_W", 16))
        await write(dut, vals)
        for s, x in vals.items():
            assert v(getattr(dut, s)) == x, s


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def stall_countdown(dut):
    await init(dut)
    dut.run.value = 1
    for d in [0] + list(range(1, 21)):
        await write(dut, {}, stall=d)
        low = 0
        while v(dut.ready) == 0:
            low += 1
            await fall(dut)
            assert low <= d
        assert low == d, f"stall {d}: ready low for {low}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def restart(dut):
    await init(dut)
    W = widths()
    dut.run.value = 1
    await write(dut, {s: (1 << W[s]) - 1 for s in STATE})
    spc = random.getrandbits(W["pc"])
    dut.start_pc.value = spc
    dut.restart.value = 1
    await fall(dut)
    dut.restart.value = 0
    assert v(dut.pc) == spc
    for s in STATE[1:]:
        assert v(getattr(dut, s)) == 0, s
    assert v(dut.ready) == 0
    await fall(dut)
    assert v(dut.ready) == 1


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def vote_result(dut):
    await init(dut)
    sr = param("SR_W", 16)
    m = (1 << sr) - 1
    dut.run.value = 1
    dut.vote_busy.value = 1
    await fall(dut)
    assert v(dut.ready) == 0
    dut.vote_busy.value = 0
    for msb in (0, 1):
        dut.shift_msb.value = msb
        isr = random.getrandbits(sr)
        await write(dut, {"isr": isr})
        for _ in range(20):
            b = random.getrandbits(1)
            dut.vote_done.value, dut.vote_bit.value = 1, b
            await fall(dut)
            isr = ((isr << 1) | b) & m if msb else (isr >> 1) | (b << (sr - 1))
            assert v(dut.isr) == isr
            assert v(dut.c) == b
        dut.vote_done.value = 0
