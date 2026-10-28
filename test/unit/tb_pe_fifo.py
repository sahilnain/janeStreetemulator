# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_fifo: unit test for src/mem/pe_fifo.v.

What is checked
---------------
A Python model (a deque plus the RTL's accept rules) runs alongside the
FIFO. Every cycle it compares empty, full, count, rdata (first-word
fall-through) and overrun.
  rd_ok = rd and not empty
  wr_ok = wr and (not full or rd_ok)
  overrun is set by a dropped write, cleared by clr_overrun (set wins)
Tests:
1. random_traffic: 3000 cycles of random wr/rd/clr with changing bias, so
   both full and empty are reached often.
2. fill_overrun_clear: fill to DEPTH, one extra write is dropped and sets
   overrun, the data is intact, clr_overrun clears the flag.
3. full_read_write_same_cycle: when full, rd + wr in the same cycle is
   accepted with no overrun.
"""

import random
from collections import deque

import cocotb
from tbutil import fall, param, start, v


class Model:
    def __init__(self, depth, w):
        self.q = deque()
        self.depth = depth
        self.mask = (1 << w) - 1
        self.ovr = 0

    def step(self, wr, wdata, rd, clr):
        empty = len(self.q) == 0
        full = len(self.q) == self.depth
        rd_ok = rd and not empty
        wr_ok = wr and (not full or rd_ok)
        if rd_ok:
            self.q.popleft()
        if wr_ok:
            self.q.append(wdata & self.mask)
        if wr and not wr_ok:
            self.ovr = 1
        elif clr:
            self.ovr = 0


def check(dut, m):
    assert v(dut.empty) == int(len(m.q) == 0)
    assert v(dut.full) == int(len(m.q) == m.depth)
    assert v(dut.count) == len(m.q)
    assert v(dut.overrun) == m.ovr
    if m.q:
        assert v(dut.rdata) == m.q[0]


async def init(dut):
    dut.wr.value = 0
    dut.rd.value = 0
    dut.wdata.value = 0
    dut.clr_overrun.value = 0
    await start(dut)
    return Model(param("DEPTH", 4), param("W", 8))


async def drive(dut, m, wr, wdata, rd, clr=0):
    dut.wr.value, dut.wdata.value, dut.rd.value, dut.clr_overrun.value = wr, wdata, rd, clr
    m.step(wr, wdata, rd, clr)
    await fall(dut)
    check(dut, m)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_traffic(dut):
    m = await init(dut)
    check(dut, m)
    W = param("W", 8)
    for i in range(3000):
        bias = 0.75 if (i // 200) % 2 == 0 else 0.25
        await drive(dut, m, int(random.random() < bias), random.getrandbits(W),
                    int(random.random() < 1 - bias), int(random.random() < 0.05))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def fill_overrun_clear(dut):
    m = await init(dut)
    D, W = param("DEPTH", 4), param("W", 8)
    data = [random.getrandbits(W) for _ in range(D)]
    for x in data:
        await drive(dut, m, 1, x, 0)
    assert v(dut.full) == 1 and v(dut.overrun) == 0
    await drive(dut, m, 1, 0x1F, 0)  # dropped
    assert v(dut.overrun) == 1
    await drive(dut, m, 0, 0, 0, clr=1)
    assert v(dut.overrun) == 0
    for x in data:
        assert v(dut.rdata) == x
        await drive(dut, m, 0, 0, 1)
    assert v(dut.empty) == 1


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def full_read_write_same_cycle(dut):
    m = await init(dut)
    D, W = param("DEPTH", 4), param("W", 8)
    for i in range(D):
        await drive(dut, m, 1, i, 0)
    for i in range(10):
        await drive(dut, m, 1, random.getrandbits(W), 1)
        assert v(dut.full) == 1 and v(dut.overrun) == 0
