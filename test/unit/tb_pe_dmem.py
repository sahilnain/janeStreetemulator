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
"""

import random

import cocotb
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
