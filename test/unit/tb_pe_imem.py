# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_imem: unit test for src/mem/pe_imem.v (flop and latch implementations).

What is checked
---------------
1. write_then_read: fill every word with random data, then read every
   address in random order. rdata must equal the shadow copy one cycle
   after raddr is set.
2. registered_read: rdata does not change in the same cycle that raddr
   changes, only after the next clock edge (the timing pe_core relies on).
3. random_mixed: 1000 cycles of random writes interleaved with random
   reads, checked against a shadow memory. A read returns the memory as it
   was before the write on the same clock edge (for both implementations).
4. edge_cases (every implementation, with and without clock gating, CG):
   - the first and the last word, and every bit of a word on its own
     (walking 1 and walking 0), all-0 / all-1 / alternating patterns
   - a write never disturbs the words next to it
   - back-to-back writes to one word: the last one wins
   - read and write of the same word on the same edge: the old value, then
     the new one
   - we = 0 with a changing address and data writes nothing
   - an address past DEPTH: the write does nothing (no aliasing onto a real
     word) and the read returns 0
   - write enable only during the high half of the clock: no write; set in
     the low half and dropped right after the edge: the write happens. With
     CG this checks that the clock gate cannot glitch.
"""

import random

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge, Timer
from tbutil import fall, param, settle, start, v


async def init(dut):
    dut.we.value = 0
    dut.waddr.value = 0
    dut.wdata.value = 0
    dut.raddr.value = 0
    await start(dut, rst_name=None)
    await fall(dut)
    return param("DEPTH", 64), param("IW", 24)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def write_then_read(dut):
    D, IW = await init(dut)
    shadow = [random.getrandbits(IW) for _ in range(D)]
    for a, x in enumerate(shadow):
        dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, x
        await fall(dut)
    dut.we.value = 0
    await fall(dut, 2)
    order = list(range(D))
    random.shuffle(order)
    for a in order:
        dut.raddr.value = a
        await fall(dut)
        assert v(dut.rdata) == shadow[a], f"addr {a}"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def registered_read(dut):
    D, IW = await init(dut)
    for a, x in ((0, 0x123456 & ((1 << IW) - 1)), (1, 0xABCDEF & ((1 << IW) - 1))):
        dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, x
        await fall(dut)
    dut.we.value = 0
    dut.raddr.value = 0
    await fall(dut, 2)
    before = v(dut.rdata)
    dut.raddr.value = 1
    await settle()
    assert v(dut.rdata) == before, "rdata changed before the clock edge"
    await fall(dut)
    assert v(dut.rdata) == 0xABCDEF & ((1 << IW) - 1)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_mixed(dut):
    D, IW = await init(dut)
    shadow = [0] * D
    for a in range(D):
        dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, 0
        await fall(dut)
    for _ in range(1000):
        we = int(random.random() < 0.4)
        wa, wd, ra = random.randrange(D), random.getrandbits(IW), random.randrange(D)
        dut.we.value, dut.waddr.value, dut.wdata.value, dut.raddr.value = we, wa, wd, ra
        exp = shadow[ra]  # a read sees memory as it was before this edge's write
        await fall(dut)
        assert v(dut.rdata) == exp, f"read {ra}"
        if we:
            shadow[wa] = wd


async def wr(dut, a, x):
    dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, x
    await fall(dut)
    dut.we.value = 0


async def rd(dut, a):
    dut.raddr.value = a
    await fall(dut)
    return v(dut.rdata)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def edge_cases(dut):
    D, IW = await init(dut)
    M = (1 << IW) - 1
    AW = max(1, (D - 1).bit_length())
    shadow = {}
    for a in range(D):                                   # known contents: word a holds a
        await wr(dut, a, a & M)
        shadow[a] = a & M
    await fall(dut, 2)                                   # the latch version writes a cycle late

    async def check_all(why):
        for a in range(D):
            assert await rd(dut, a) == shadow[a], f"{why}: word {a} = {v(dut.rdata):#x}, expected {shadow[a]:#x}"

    # first and last word, every bit on its own, and the usual patterns
    pats = [0, M, 0x5555_5555 & M, 0xAAAA_AAAA & M] + [1 << i for i in range(IW)] + [M ^ (1 << i) for i in range(IW)]
    for a in (0, D - 1):
        for x in pats:
            await wr(dut, a, x)
            await fall(dut)
            got = await rd(dut, a)
            assert got == x, f"word {a}: wrote {x:#x}, read {got:#x}"
        shadow[a] = pats[-1]
    # a write leaves its neighbours alone
    for a in (1, D // 2, D - 2):
        await wr(dut, a, M)
        shadow[a] = M
        await fall(dut)
        for b in (a - 1, a + 1):
            assert await rd(dut, b) == shadow[b], f"writing word {a} changed word {b}"
    # back-to-back writes to one word: the last one wins
    a = D // 3
    for x in (0x1234 & M, 0x5678 & M, 0x0F0F & M):
        dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, x
        await fall(dut)
    dut.we.value = 0
    shadow[a] = 0x0F0F & M
    await fall(dut)
    assert await rd(dut, a) == shadow[a], "back-to-back writes"
    # read and write of one word on the same edge
    a = D // 4
    dut.raddr.value = a
    await fall(dut)
    dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, M & ~shadow[a]
    await fall(dut)
    dut.we.value = 0
    assert v(dut.rdata) == shadow[a], "same-edge read must return the old word"
    shadow[a] = M & ~shadow[a]
    await fall(dut, 2)
    assert await rd(dut, a) == shadow[a], "the new word, after the write"
    # we = 0: nothing is written, whatever the address and data do
    for _ in range(50):
        dut.we.value, dut.waddr.value, dut.wdata.value = 0, random.randrange(D), random.getrandbits(IW)
        await fall(dut)
    # past the end: writes do nothing (no aliasing), reads give 0
    if D < (1 << AW):
        for a in range(D, 1 << AW):
            await wr(dut, a, M)
        await fall(dut, 2)
        for a in (D, (1 << AW) - 1):
            assert await rd(dut, a) == 0, f"address {a} past the end must read 0"
    # write enable only in the high half of the clock: no write
    a = D // 5
    await RisingEdge(dut.clk)
    await Timer(3, unit="ns")
    dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, M & ~shadow[a]
    await Timer(3, unit="ns")
    dut.we.value = 0
    await FallingEdge(dut.clk)
    await fall(dut, 2)
    assert await rd(dut, a) == shadow[a], "a write enable seen only while clk was high wrote the word"
    # set in the low half, dropped just after the edge: the write happens
    dut.we.value, dut.waddr.value, dut.wdata.value = 1, a, M & ~shadow[a]
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.we.value = 0
    shadow[a] = M & ~shadow[a]
    await FallingEdge(dut.clk)
    await fall(dut, 2)
    assert await rd(dut, a) == shadow[a], "a write enable dropped just after the edge was lost"
    await check_all("at the end")
