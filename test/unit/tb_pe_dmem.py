# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_dmem: unit test for src/mem/pe_dmem.v.

What is checked
---------------
A shadow memory runs alongside. Every cycle:
  c_gnt == !h_acc; rdata == mem[addr], addr = h_acc ? h_addr : c_addr
  (low log2(DEPTH) bits); a write uses the host port when h_acc, else the
  controller port.
1. random_traffic: 4000 cycles of random host/controller reads and writes,
   including both in the same cycle (host wins, the controller write is
   dropped).
2. fill_and_alias: every address written by the controller and read by the
   host; addresses >= DEPTH alias onto addr % DEPTH.
3. edge_cases (with and without clock gating, CG):
   - the first and the last byte; every bit on its own, all-0 / all-1
   - a write never disturbs the bytes next to it
   - the read is combinational: the old byte before the edge, the new one
     right after it (a `st` and then an `ld` of the same byte, back to back)
   - back-to-back writes to one byte: the last one wins
   - host and controller write in the same cycle: the host's byte is kept
   - write enable only during the high half of the clock: no write; set in
     the low half and dropped right after the edge: the write happens. With
     CG this checks that the clock gate cannot glitch.
"""

import random

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge, Timer
from tbutil import fall, param, settle, start, v

DEPTH = param("DEPTH", 256)


async def init(dut):
    for s in ("h_acc", "h_we", "h_addr", "h_wdata", "c_we", "c_addr", "c_wdata"):
        getattr(dut, s).value = 0
    await start(dut, rst_name=None)
    mem = [0] * DEPTH
    # initialise (flops have no reset)
    for a in range(DEPTH):
        dut.c_we.value, dut.c_addr.value, dut.c_wdata.value = 1, a, 0
        await fall(dut)
    dut.c_we.value = 0
    return mem


async def cycle(dut, mem, h_acc, h_we, h_addr, h_wd, c_we, c_addr, c_wd):
    dut.h_acc.value, dut.h_we.value, dut.h_addr.value, dut.h_wdata.value = h_acc, h_we, h_addr, h_wd
    dut.c_we.value, dut.c_addr.value, dut.c_wdata.value = c_we, c_addr, c_wd
    await settle()
    a = (h_addr if h_acc else c_addr) % DEPTH
    assert v(dut.c_gnt) == int(not h_acc)
    assert v(dut.rdata) == mem[a]
    we, wd = (h_we, h_wd) if h_acc else (c_we, c_wd)
    await fall(dut)
    if we:
        mem[a] = wd


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_traffic(dut):
    mem = await init(dut)
    for _ in range(4000):
        await cycle(dut, mem, random.random() < 0.4, random.getrandbits(1), random.getrandbits(8),
                    random.getrandbits(8), random.getrandbits(1), random.getrandbits(8), random.getrandbits(8))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def fill_and_alias(dut):
    mem = await init(dut)
    for a in range(256):
        await cycle(dut, mem, 0, 0, 0, 0, 1, a, (a * 7 + 3) & 0xFF)
    for a in range(256):
        await cycle(dut, mem, 1, 0, a, 0, 0, 0, 0)
    for a in range(DEPTH):
        assert mem[a] == ((a + (256 - DEPTH)) * 7 + 3) & 0xFF if DEPTH < 256 else mem[a] == (a * 7 + 3) & 0xFF


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def edge_cases(dut):
    mem = await init(dut)
    for a in range(DEPTH):                               # known contents: byte a holds a
        await cycle(dut, mem, 0, 0, 0, 0, 1, a, a & 0xFF)
    last = DEPTH - 1
    # first and last byte, every bit on its own, all-0 / all-1; the host reads each back
    for a in (0, last):
        for x in [0x00, 0xFF, 0x55, 0xAA] + [1 << i for i in range(8)] + [0xFF ^ (1 << i) for i in range(8)]:
            await cycle(dut, mem, 1, 1, a, x, 0, 0, 0)       # host write
            await cycle(dut, mem, 1, 0, a, 0, 0, 0, 0)       # host read (checked in cycle())
            await cycle(dut, mem, 0, 0, 0, 0, 1, a, x ^ 0xFF)   # controller write
            await cycle(dut, mem, 0, 0, 0, 0, 0, a, 0)       # controller read
    # neighbours untouched
    for a in (1, DEPTH // 2, last - 1):
        await cycle(dut, mem, 0, 0, 0, 0, 1, a, 0xC3)
        for b in (a - 1, a + 1):
            await cycle(dut, mem, 1, 0, b, 0, 0, 0, 0)
    # combinational read: old byte before the edge, new byte straight after it
    a = DEPTH // 3
    dut.h_acc.value, dut.c_we.value, dut.c_addr.value, dut.c_wdata.value = 0, 1, a, mem[a] ^ 0x5A
    await settle()
    assert v(dut.rdata) == mem[a], "before the edge the read must show the old byte"
    await fall(dut)
    mem[a] ^= 0x5A
    dut.c_we.value = 0
    await settle()
    assert v(dut.rdata) == mem[a], "st then ld of the same byte: the ld must see the new byte"
    # back-to-back writes, last one wins
    for x in (0x11, 0x22, 0x33):
        await cycle(dut, mem, 0, 0, 0, 0, 1, a, x)
    await cycle(dut, mem, 0, 0, 0, 0, 0, a, 0)
    assert mem[a] == 0x33
    # host and controller write in the same cycle (the controller to another byte): only the host's lands
    await cycle(dut, mem, 1, 1, a, 0x99, 1, a + 1, 0x77)
    await cycle(dut, mem, 1, 0, a + 1, 0, 0, 0, 0)
    await cycle(dut, mem, 1, 0, a, 0, 0, 0, 0)
    # write enable only while clk is high: no write
    a = DEPTH // 5
    dut.h_acc.value, dut.h_we.value = 0, 0               # the controller has the port
    await RisingEdge(dut.clk)
    await Timer(3, unit="ns")
    dut.c_we.value, dut.c_addr.value, dut.c_wdata.value = 1, a, mem[a] ^ 0xFF
    await Timer(3, unit="ns")
    dut.c_we.value = 0
    await FallingEdge(dut.clk)
    await cycle(dut, mem, 0, 0, 0, 0, 0, a, 0)           # checks the byte did not change
    # set in the low half, dropped just after the edge: written
    dut.c_we.value, dut.c_addr.value, dut.c_wdata.value = 1, a, mem[a] ^ 0xFF
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.c_we.value = 0
    mem[a] ^= 0xFF
    await FallingEdge(dut.clk)
    await cycle(dut, mem, 0, 0, 0, 0, 0, a, 0)
    # every byte, at the end
    for a in range(DEPTH):
        await cycle(dut, mem, 1, 0, a, 0, 0, 0, 0)
