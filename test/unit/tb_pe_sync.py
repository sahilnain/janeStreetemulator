# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_sync: unit test for src/io/pe_sync.v (2-FF synchroniser).

What is checked
---------------
1. reset_value: while rst is high, q equals RESET_VAL.
2. latency_random: with random input every cycle, q equals the input from
   exactly 2 clock cycles earlier (500 cycles).
"""

import random

import cocotb
from tbutil import fall, param, start, v


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def reset_value(dut):
    W, RV = param("W", 1), param("RESET_VAL", 0)
    dut.d.value = (~RV) & ((1 << W) - 1)
    await start(dut, rst_cycles=0)
    dut.rst.value = 1
    await fall(dut, 3)
    assert v(dut.q) == RV


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def latency_random(dut):
    W = param("W", 1)
    dut.d.value = 0
    await start(dut)
    hist = []
    for i in range(500):
        x = random.getrandbits(W)
        dut.d.value = x
        hist.append(x)
        await fall(dut)
        if i >= 1:
            assert v(dut.q) == hist[i - 1], f"cycle {i}"
