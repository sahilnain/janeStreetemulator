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
"""

import random

import cocotb
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
