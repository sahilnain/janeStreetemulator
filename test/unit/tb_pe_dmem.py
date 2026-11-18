# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_dmem: unit test for src/mem/pe_dmem.v (v3: one port, registered read).

What is checked
---------------
A shadow memory and a shadow output register run alongside. Every cycle:
a read (re) puts mem[addr] on rdata from the next cycle until the next read;
a write (we) changes the byte and leaves rdata alone. re and we are never
both 1 (pe_dmov makes sure). The same bench runs on every implementation:
flip-flops (IMPL = 0), with clock gating (CG = 1), and the IHP SRAM macro
(IMPL = 1, the PDK's behavioural model), so all three behave the same.
1. random_traffic: 4000 cycles of random reads, writes and idle cycles.
2. fill_and_alias: every address written then read; addresses >= DEPTH alias
   onto addr % DEPTH.
3. edge_cases:
   - the first and the last byte; every bit on its own, all-0 / all-1
   - a write never disturbs the bytes next to it, nor rdata
   - read latency: rdata shows the byte one cycle after the read, holds it
     through idle cycles and writes (even a write to the same byte), and
     changes only on the next read
   - a write then a read of the same byte, back to back: the read sees the
     new byte; a read then a write: the read sees the old byte
   - back-to-back writes to one byte: the last one wins
   - write enable only during the high half of the clock: no write; set in
     the low half and dropped right after the edge: the write happens. With
     CG this checks that the clock gate cannot glitch.
"""

import random

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge, Timer
from tbutil import fall, param, settle, start, v

DEPTH = param("DEPTH", 256)


class Shadow:
    def __init__(self):
        self.mem = [0] * DEPTH
        self.dout = None


async def init(dut):
    for s in ("re", "we", "addr", "wdata"):
        getattr(dut, s).value = 0
    await start(dut, rst_name=None)
    sh = Shadow()
    for a in range(DEPTH):                         # no reset: write every byte first
        dut.we.value, dut.addr.value, dut.wdata.value = 1, a, 0
        await fall(dut)
    dut.we.value = 0
    return sh


async def cycle(dut, sh, re=0, we=0, addr=0, wd=0):
    """One cycle: drive, check rdata (the result of the last read), clock, update the shadow."""
    assert not (re and we)
    dut.re.value, dut.we.value, dut.addr.value, dut.wdata.value = re, we, addr, wd
    await settle()
    if sh.dout is not None:
        assert v(dut.rdata) == sh.dout, f"rdata {v(dut.rdata):#x}, expected {sh.dout:#x}"
    a = addr % DEPTH
    await fall(dut)
    if re:
        sh.dout = sh.mem[a]
    if we:
        sh.mem[a] = wd


async def read(dut, sh, addr):
    await cycle(dut, sh, re=1, addr=addr)
    await cycle(dut, sh)                           # checks rdata == the byte


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_traffic(dut):
    sh = await init(dut)
    for _ in range(4000):
        r = random.random()
        if r < 0.35:
            await cycle(dut, sh, re=1, addr=random.getrandbits(8))
        elif r < 0.7:
            await cycle(dut, sh, we=1, addr=random.getrandbits(8), wd=random.getrandbits(8))
        else:
            await cycle(dut, sh, addr=random.getrandbits(8), wd=random.getrandbits(8))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def fill_and_alias(dut):
    sh = await init(dut)
    for a in range(256):
        await cycle(dut, sh, we=1, addr=a, wd=(a * 7 + 3) & 0xFF)
    for a in range(256):
        await read(dut, sh, a)
    for a in range(DEPTH):
        hi = a + (256 - DEPTH) if DEPTH < 256 else a     # the last write to each byte
        assert sh.mem[a] == (hi * 7 + 3) & 0xFF


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def edge_cases(dut):
    sh = await init(dut)
    for a in range(DEPTH):                               # known contents: byte a holds a
        await cycle(dut, sh, we=1, addr=a, wd=a & 0xFF)
    last = DEPTH - 1
    # first and last byte, every bit on its own, all-0 / all-1
    for a in (0, last):
        for x in [0x00, 0xFF, 0x55, 0xAA] + [1 << i for i in range(8)] + [0xFF ^ (1 << i) for i in range(8)]:
            await cycle(dut, sh, we=1, addr=a, wd=x)
            await read(dut, sh, a)
    # neighbours untouched, and a write leaves rdata alone
    for a in (1, DEPTH // 2, last - 1):
        await read(dut, sh, a - 1)
        await cycle(dut, sh, we=1, addr=a, wd=0xC3)     # rdata still holds byte a - 1
        await cycle(dut, sh)
        for b in (a - 1, a + 1):
            await read(dut, sh, b)
    # latency: the byte appears one cycle after the read and holds until the next read
    a = DEPTH // 3
    await cycle(dut, sh, we=1, addr=a, wd=0x3C)
    await cycle(dut, sh, re=1, addr=a)
    for _ in range(3):
        await cycle(dut, sh)                            # idle: rdata holds 0x3C
    await cycle(dut, sh, we=1, addr=a, wd=0xA5)          # a write to the same byte: rdata still 0x3C
    await cycle(dut, sh)
    assert sh.dout == 0x3C
    # write then read of the same byte, back to back: the read sees the new byte
    await cycle(dut, sh, we=1, addr=a, wd=0x5A)
    await read(dut, sh, a)
    assert sh.dout == 0x5A
    # read then write, back to back: the read returns the old byte
    await cycle(dut, sh, re=1, addr=a)
    await cycle(dut, sh, we=1, addr=a, wd=0x77)          # checks rdata == 0x5A here
    await cycle(dut, sh)
    # back-to-back writes, last one wins
    for x in (0x11, 0x22, 0x33):
        await cycle(dut, sh, we=1, addr=a, wd=x)
    await read(dut, sh, a)
    assert sh.dout == 0x33
    # write enable only while clk is high: no write
    a = DEPTH // 5
    await RisingEdge(dut.clk)
    await Timer(3, unit="ns")
    dut.we.value, dut.addr.value, dut.wdata.value = 1, a, sh.mem[a] ^ 0xFF
    await Timer(3, unit="ns")
    dut.we.value = 0
    await FallingEdge(dut.clk)
    await read(dut, sh, a)                               # the byte did not change
    # set in the low half, dropped just after the edge: written
    dut.we.value, dut.addr.value, dut.wdata.value = 1, a, sh.mem[a] ^ 0xFF
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    dut.we.value = 0
    sh.mem[a] ^= 0xFF
    await FallingEdge(dut.clk)
    await read(dut, sh, a)
    # every byte, at the end
    for a in range(DEPTH):
        await read(dut, sh, a)
