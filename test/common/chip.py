# SPDX-License-Identifier: Apache-2.0
"""
chip: top-level test harness for one emulator instance.

What this file does
-------------------
* `Chip(dut, prefix)`: reset, host bus (pe_host.HostBus) and the uio pads.
  A coroutine runs every falling edge:
    line = driven pins (uio_out where uio_oe) | `ext` on undriven pins
    then applies `loop` wires (src pin -> dst pin) and `drive` overrides
    (pin -> level set by the test), writes uio_in and records `line` in
    `samples` (one entry per clock cycle).
  Open-drain pins behave as wired-AND with `ext` (pull-up = 1).
  `devices`: protocol models (spi.py, i2c.py); each sees the line every
  cycle and returns it with its own outputs applied (I2C: wired-AND).
* `transfer(data)`: the host sends bytes through TXF while collecting RXF,
  using only the host bus.
* `start_uart(**kw)`: loads pe_fw.uart and the pin owners, then runs it.
* With the environment variable LOCKSTEP set (make LOCKSTEP=1), `reset()` also starts
  a lockstep.Lockstep, so the test compares the RTL with the Python model every cycle.

`prefix` selects one of two instances in tb_dual.v ("e1_" / "e2_").
"""

import os

import cocotb
import pe_fw
import pe_host as R
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge


_RUNNING = {}   # per instance: the pads task; per dut: the clock task (one of each)


class Chip:
    def __init__(self, dut, prefix="", clock=True, pads=True):
        self.dut = dut
        self.p = prefix
        self.bus = R.HostBus(dut, prefix=prefix)
        self.loop = []          # (src_pin, dst_pin) wired outside the chip
        self.drive = {}         # pin -> level forced by the test
        self.devices = []       # objects with step(line, t) -> line (see spi.py, i2c.py)
        self.ext = 0xFF         # level on undriven uio inputs (pull-ups)
        self.samples = []
        self.clock = clock
        self.pads = pads        # False: the test drives uio_in itself (tb_dual)

    def sig(self, name):
        return getattr(self.dut, self.p + name)

    async def reset(self):
        dut = self.dut
        st = _RUNNING.setdefault((id(dut), self.p), {"clock": False, "pads": None})
        if st["pads"] is not None and not st["pads"].done():   # an earlier Chip on this instance
            st["pads"].cancel()
        st["pads"] = None
        ck = _RUNNING.get((id(dut), "clk"))
        if self.clock and (ck is None or ck.done()):   # cocotb ends all tasks after each test
            _RUNNING[(id(dut), "clk")] = cocotb.start_soon(Clock(dut.clk, 20834, unit="ps").start())
        ls = st.get("lockstep")
        if ls is not None and not ls.done():
            ls.cancel()
        self.ls = None
        if os.environ.get("LOCKSTEP"):
            from lockstep import Lockstep

            self.ls = Lockstep(dut, self.p)
            st["lockstep"] = cocotb.start_soon(self.ls.run())
        self.sig("ena").value = 1
        self.sig("ui_in").value = 0
        self.sig("uio_in").value = 0xFF
        self.sig("rst_n").value = 0
        await ClockCycles(dut.clk, 10)
        self.sig("rst_n").value = 1
        await ClockCycles(dut.clk, 5)
        if self.ls is not None:
            self.ls.arm()
        if self.pads:
            st["pads"] = cocotb.start_soon(self._pads())

    def line(self):
        out, oe = int(self.sig("uio_out").value), int(self.sig("uio_oe").value)
        return ((out & oe) | (self.ext & ~oe)) & 0xFF

    async def _pads(self):
        while True:
            await FallingEdge(self.dut.clk)
            line = self.line()
            for s, t in self.loop:
                line = (line & ~(1 << t)) | (((line >> s) & 1) << t)
            for p, lv in self.drive.items():
                line = (line & ~(1 << p)) | ((lv & 1) << p)
            for d in self.devices:
                line = d.step(line, len(self.samples))
            self.samples.append(line & 0xFF)
            self.sig("uio_in").value = line & 0xFF

    def trace(self, pin, start=0):
        return [(s >> pin) & 1 for s in self.samples[start:]]

    async def start_uart(self, txd=0, rxd=1, **kw):
        bus = self.bus
        await bus.write(R.RUN, 0)
        await bus.load_program(pe_fw.uart(txd=txd, rxd=rxd, **kw).words())
        await bus.owners(R.owner_map(**{f"p{txd}": R.OWN_TX}))
        await bus.start()

    async def transfer(self, data, n_rx=None, limit=200000):
        """Send `data` through TXF while collecting n_rx bytes from RXF."""
        bus = self.bus
        n_rx = len(data) if n_rx is None else n_rx
        got, todo = [], list(data)
        for _ in range(limit):
            st = await bus.read(R.STATUS)
            if not st & R.ST_RXF_EMPTY:
                got.append(await bus.read(R.RXF))
                if len(got) == n_rx and not todo:
                    return got
            elif todo and not st & R.ST_TXF_FULL:
                await bus.write(R.TXF, todo.pop(0))
            elif not todo and len(got) >= n_rx:
                return got
        raise AssertionError(f"timeout, got {got}")

    async def read_rx(self, n, limit=20000):
        got = []
        for _ in range(limit):
            if len(got) == n:
                return got
            st = await self.bus.read(R.STATUS)
            if not st & R.ST_RXF_EMPTY:
                got.append(await self.bus.read(R.RXF))
        raise AssertionError(f"timeout, got {got}")
