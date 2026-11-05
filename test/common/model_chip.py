# SPDX-License-Identifier: Apache-2.0
"""
model_chip: the Python model as a test peer, with the same interface as chip.Chip.

What this file does
-------------------
`ModelChip(dut)` holds one pe_model.Chip, clocked by the testbench clock: at each rising
edge it applies the current ui_in, uio_in and rst_n. `bus`, `line()`, `sig()`, `transfer()`,
`read_rx()`, `start_uart()`, `samples` and `trace()` work as on an RTL chip, so a test can
put either side in either role (test_cross.py).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "model") not in sys.path:
    sys.path.insert(0, str(ROOT / "model"))

import cocotb  # noqa: E402
import pe_host as R  # noqa: E402
from chip import Chip  # noqa: E402
from cocotb.triggers import ClockCycles, RisingEdge  # noqa: E402
from pe_model import Chip as Model  # noqa: E402


class _Sig:
    """Stands in for a cocotb signal handle."""

    def __init__(self, value=0):
        self.value = value


class ModelBus(R.HostBus):
    def __init__(self, dut, peer):
        self.dut = dut
        self.timeout = 50
        self.stb = 0
        self.extra = 0
        self.peer = peer
        self.ui = peer.sigs["ui_in"]
        self.uo = None

    def _uo_bit(self, i):
        return (self.peer.outs["uo_out"] >> i) & 1

    def _uo_nibble(self):
        o = self.peer.outs
        return o["uo_out"] & 0xF if o["uo_known"] & 0xF == 0xF else None


class ModelChip(Chip):
    def __init__(self, dut):
        self.dut = dut
        self.p = "model_"
        self.model = Model()
        self.sigs = {k: _Sig(v) for k, v in (("ui_in", 0), ("uio_in", 0xFF), ("rst_n", 0), ("ena", 1))}
        self.outs = self.model.outputs()
        self.bus = ModelBus(dut, self)
        self.loop = []
        self.drive = {}
        self.devices = []
        self.ext = 0xFF
        self.samples = []
        self.clock = False
        self.pads = False
        self.ls = None

    def sig(self, name):
        if name in ("uio_out", "uio_oe", "uo_out"):
            return _Sig(self.outs[name])
        return self.sigs[name]

    async def _run(self):
        while True:
            await RisingEdge(self.dut.clk)
            s = self.sigs
            self.model.step(s["ui_in"].value, s["uio_in"].value, s["rst_n"].value)
            self.outs = self.model.outputs()

    async def reset(self):
        self.task = cocotb.start_soon(self._run())
        self.sigs["rst_n"].value = 0
        await ClockCycles(self.dut.clk, 10)
        self.sigs["rst_n"].value = 1
        await ClockCycles(self.dut.clk, 5)

    def stop(self):
        self.task.cancel()
