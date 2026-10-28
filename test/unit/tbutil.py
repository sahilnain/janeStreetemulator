# SPDX-License-Identifier: Apache-2.0
"""
tbutil: shared helpers for the unit testbenches.

What this file does
-------------------
* Puts test/common on sys.path, so tests can `import pe_isa` etc.
* `P`: the module's parameters for this run. The pytest runner
  (test_unit.py) passes them as JSON in the PE_PARAMS env var, so a test
  knows which config it is checking.
* `start(dut)`: starts a 50 MHz clock and applies reset.
* Every test is declared @cocotb.test(timeout_time=5, timeout_unit="ms"):
  a hung test (for example, waiting forever for a FIFO byte) FAILS instead of
  blocking the run.
* Convention used by every test: drive inputs and check registered outputs
  on the FALLING edge, so values are stable and the rising edge in between
  is "the clock". For combinational outputs, `settle()` waits 1 ns after
  driving.
"""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

import cocotb  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import FallingEdge, Timer  # noqa: E402

P = json.loads(os.environ.get("PE_PARAMS", "{}"))


def param(name, default):
    return int(P.get(name, default))


async def start(dut, rst_cycles=3, rst_name="rst"):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    if rst_name:
        getattr(dut, rst_name).value = 1
    for _ in range(rst_cycles):
        await FallingEdge(dut.clk)
    if rst_name:
        getattr(dut, rst_name).value = 0


async def fall(dut, n=1):
    for _ in range(n):
        await FallingEdge(dut.clk)


async def settle():
    await Timer(1, unit="ns")


def v(sig):
    return int(sig.value)
