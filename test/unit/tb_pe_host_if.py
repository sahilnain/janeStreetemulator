# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_host_if: unit test for src/host/pe_host_if.v (4-bit host bus).

What is checked
---------------
The test plays both sides: a host bus driver (toggle strobe, wait for ack)
and a fake register file behind r_addr/r_wdata/r_we/r_re/r_rdata. A monitor
logs every r_we and r_re pulse.
1. write_read_random: 200 random byte writes then reads at random
   addresses, with random host delays between ops. Each byte write gives
   exactly ONE r_we with the right address/data. Each byte read gives
   exactly ONE r_re and returns the register value (high nibble first).
2. ack_per_op: h_ack toggles exactly once per strobe toggle, even with
   long gaps.
3. sync_recovers_phase: after a stray single write nibble, OP_SYNC resets
   the nibble phase, so the next full byte write is correct.
4. irq_passthrough: h_irq follows irq.
"""

import random

import cocotb
from cocotb.triggers import FallingEdge
from tbutil import fall, settle, start, v

OP_ADDR, OP_WRITE, OP_READ, OP_SYNC = range(4)


class Env:
    def __init__(self, dut):
        self.dut = dut
        self.regs = [random.getrandbits(8) for _ in range(256)]
        self.writes, self.reads = [], []
        self.stb = 0
        self.max_gap = 0

    async def monitor(self):
        d = self.dut
        while True:
            await FallingEdge(d.clk)
            if v(d.r_we):
                self.regs[v(d.r_addr)] = v(d.r_wdata)
                self.writes.append((v(d.r_addr), v(d.r_wdata)))
            if v(d.r_re):
                self.reads.append(v(d.r_addr))
            d.r_rdata.value = self.regs[v(d.r_addr)]

    async def op(self, op, nib=0):
        d = self.dut
        ack = v(d.h_ack)
        for _ in range(random.randint(0, self.max_gap)):
            await fall(d)
        self.stb ^= 1
        d.h_din.value, d.h_op.value, d.h_stb.value = nib, op, self.stb
        for _ in range(20):
            await fall(d)
            if v(d.h_ack) != ack:
                await fall(d, 2)
                assert v(d.h_ack) != ack, "ack toggled twice"
                return v(d.h_dout)
        raise AssertionError("no ack")

    async def write(self, a, x):
        await self.op(OP_ADDR, a >> 4)
        await self.op(OP_ADDR, a & 15)
        await self.op(OP_WRITE, x >> 4)
        await self.op(OP_WRITE, x & 15)

    async def read(self, a):
        await self.op(OP_ADDR, a >> 4)
        await self.op(OP_ADDR, a & 15)
        hi = await self.op(OP_READ)
        lo = await self.op(OP_READ)
        return (hi << 4) | lo


async def setup(dut):
    for s in ("h_din", "h_op", "h_stb", "r_rdata", "irq"):
        getattr(dut, s).value = 0
    await start(dut)
    e = Env(dut)
    cocotb.start_soon(e.monitor())
    return e


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def write_read_random(dut):
    e = await setup(dut)
    e.max_gap = 5
    for _ in range(200):
        a, x = random.getrandbits(8), random.getrandbits(8)
        n = len(e.writes)
        await e.write(a, x)
        assert e.writes[n:] == [(a, x)], e.writes[n:]
        a = random.getrandbits(8)
        n = len(e.reads)
        assert await e.read(a) == e.regs[a]
        assert e.reads[n:] == [a], e.reads[n:]


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def ack_per_op(dut):
    e = await setup(dut)
    e.max_gap = 40
    for _ in range(30):
        await e.op(random.choice([OP_ADDR, OP_SYNC]), random.getrandbits(4))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def sync_recovers_phase(dut):
    e = await setup(dut)
    await e.op(OP_ADDR, 0x3)
    await e.op(OP_ADDR, 0xC)
    await e.op(OP_WRITE, 0x7)  # stray half byte
    await e.op(OP_SYNC)
    n = len(e.writes)
    await e.op(OP_WRITE, 0x9)
    await e.op(OP_WRITE, 0x6)
    assert e.writes[n:] == [(0x3C, 0x96)]
    await e.op(OP_READ)  # stray half read
    await e.op(OP_SYNC)
    assert await e.read(0x3C) == 0x96


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def irq_passthrough(dut):
    await setup(dut)
    for x in (1, 0, 1):
        dut.irq.value = x
        await settle()
        assert v(dut.h_irq) == x
