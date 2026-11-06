# SPDX-License-Identifier: Apache-2.0
"""
bench: runs the Python chip model on a bus, with no simulator and no RTL.

What this file does
-------------------
* `Bench`: one or more chips and any number of devices on the 8 uio pins. Every clock it
  resolves each pin (drivers, then the pin's pull level, then wire delay), records the result
  in `samples` (one byte per cycle, ready for sigrok) and clocks the chips.
* `Node`: one chip with its host bus. Firmware and memory are loaded directly, because
  the host bus is not what these tests are about; registers go over the real bus.
* `Script`: a device that runs a Python generator. The generator yields a dict
  {pin: level or None}: what it drives until its next turn; None releases the pin.
  It may yield an int n to keep driving as it is for n cycles.
* `Edges`: wire delay for one pin, for the I2C rise and fall time limits.

Contention (two drivers at different levels) is recorded in `Bench.contention`.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "model", ROOT / "test" / "common"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from pe_model import Chip  # noqa: E402
from pe_model.blocks import (A_DMADDR, A_DMDATA, A_OWNER, A_PIN_OD, A_RESTART, A_RUN,  # noqa: E402
                             A_RXF, A_START_PC, A_STATUS, A_TXF)

OP_ADDR, OP_WRITE, OP_READ = 0, 1, 2
OWN_CTRL, OWN_TX, OWN_RX = 0, 1, 2
CLOCK_HZ = 48_000_000
NS = 1e9 / CLOCK_HZ          # one cycle, in ns
ST_RXF_EMPTY, ST_TXF_FULL = 0x04, 0x02


def owners(**pins):
    """owners(p0=OWN_TX, p3=OWN_RX) -> the 16-bit PIN_OWNER value."""
    v = 0
    for k, o in pins.items():
        v |= o << (2 * int(k[1:]))
    return v


class Edges:
    """A pin that needs `rise` cycles to reach high and `fall` cycles to reach low."""

    def __init__(self, rise=0, fall=0):
        self.rise, self.fall = rise, fall
        self.level, self.left = 1, 0

    def step(self, want):
        if want == self.level:
            self.left = 0
            return want
        self.left += 1
        if self.left > (self.rise if want else self.fall):
            self.level, self.left = want, 0
        return self.level


class Node:
    def __init__(self, bench):
        self.bench = bench
        self.chip = Chip()
        self.ui = 0
        self.stb = 0
        self.outs = self.chip.outputs()

    def _op(self, op, nib=0):
        ack = (self.outs["uo_out"] >> 4) & 1
        self.stb ^= 1
        self.ui = (self.stb << 6) | (op << 4) | (nib & 0xF)
        for _ in range(50):
            self.bench.tick()
            if ((self.outs["uo_out"] >> 4) & 1) != ack:
                return self.outs["uo_out"] & 0xF
        raise AssertionError("host bus: no ack")

    def write(self, addr, data):
        self._op(OP_ADDR, addr >> 4)
        self._op(OP_ADDR, addr & 0xF)
        self._op(OP_WRITE, data >> 4)
        self._op(OP_WRITE, data & 0xF)

    def write_same(self, data):
        self._op(OP_WRITE, data >> 4)
        self._op(OP_WRITE, data & 0xF)

    def read(self, addr):
        self._op(OP_ADDR, addr >> 4)
        self._op(OP_ADDR, addr & 0xF)
        return self.read_same()

    def read_same(self):
        hi = self._op(OP_READ)
        return (hi << 4) | self._op(OP_READ)

    def load(self, asm, own=0, od=0):
        """Load firmware and pin settings, and run it."""
        words = asm.words()
        assert len(words) <= len(self.chip.imem.mem), f"{len(words)} words do not fit IMEM"
        self.chip.imem.mem[:len(words)] = words
        self.write(A_RUN, 0)
        self.write(A_PIN_OD, od)
        for b in range(4):
            self.write(A_OWNER + b, (own >> (8 * b)) & 0xFF)
        self.write(A_START_PC, 0)
        self.write(A_START_PC + 1, 0)
        self.write(A_RESTART, 1)
        self.write(A_RUN, 1)

    def irq(self):
        return (self.outs["uo_out"] >> 5) & 1

    def dmem(self, addr, data=()):
        for i, d in enumerate(data):
            self.chip.dmem.mem[(addr + i) & 0xFF] = d & 0xFF

    def peek(self, addr, n=1):
        return [self.chip.dmem.mem[(addr + i) & 0xFF] for i in range(n)]

    def doorbell(self):
        self.write(0x0F, 1)

    def clear_irq(self):
        """Clear the latched firmware IRQ (ERR_CLR bit 7, as in the register map)."""
        self.write(0x05, 0xFF)


class Script:
    def __init__(self, gen):
        self.gen = gen
        self.wait = 0
        self.m = self.v = 0
        self.line = 0xFF
        self.done = False

    def step(self, line, t):
        self.line = line
        if self.wait:
            self.wait -= 1
        elif not self.done:
            try:
                y = next(self.gen)
            except StopIteration:
                self.done = True
                y = None
            if isinstance(y, int):
                self.wait = y - 1 if y > 0 else 0
            elif y:
                for p, lv in y.items():
                    if lv is None:
                        self.m &= ~(1 << p)
                    else:
                        self.m |= 1 << p
                        self.v = (self.v & ~(1 << p)) | ((lv & 1) << p)
        return self.m, self.v


class Bench:
    def __init__(self, n=1, pull=0xFF, edges=None):
        self.nodes = [Node(self) for _ in range(n)]
        self.devices = []
        self.pull = pull
        self.edges = edges or {}
        self.samples = bytearray()
        self.line = pull
        self.contention = []
        for nd in self.nodes:
            for _ in range(10):
                nd.chip.step(0, 0xFF, 0)
            nd.outs = nd.chip.outputs()
        self.run(5)

    def tick(self):
        t = len(self.samples)
        hi = lo = 0
        for nd in self.nodes:
            o = nd.outs
            hi |= o["uio_out"] & o["uio_oe"]
            lo |= ~o["uio_out"] & o["uio_oe"]
        for d in self.devices:
            dm, dv = d.step(self.line, t)
            hi |= dv & dm
            lo |= ~dv & dm
        if hi & lo & 0xFF:
            self.contention.append((t, hi & lo & 0xFF))
        line = (self.pull | hi) & ~lo & 0xFF       # low wins, as on a real wire
        for p, e in self.edges.items():
            line = (line & ~(1 << p)) | (e.step((line >> p) & 1) << p)
        self.line = line
        self.samples.append(line)
        for nd in self.nodes:
            nd.chip.step(nd.ui, line, 1)
            nd.outs = nd.chip.outputs()

    def run(self, n):
        for _ in range(n):
            self.tick()

    def until(self, cond, limit=2_000_000, what="condition"):
        for _ in range(limit):
            if cond():
                return
            self.tick()
        raise AssertionError(f"{what}: not met in {limit} cycles")

    def bit(self, pin, start=0):
        return [(s >> pin) & 1 for s in self.samples[start:]]


def rx_all(node, n, limit=2_000_000):
    """Read n bytes from RXF over the host bus."""
    got = []
    for _ in range(limit):
        if len(got) == n:
            return got
        if not node.read(A_STATUS) & ST_RXF_EMPTY:
            got.append(node.read(A_RXF))
    raise AssertionError(f"RX timeout, got {got}")


def backdoor_txf(node, data):
    """Top up TXF from `data` (a list); returns what is left. No bus time spent."""
    f = node.chip.txf
    while data and not f.full:
        node.write(A_TXF, data.pop(0))
    return data


__all__ = ["Bench", "Node", "Script", "Edges", "owners", "rx_all", "NS", "CLOCK_HZ",
           "OWN_CTRL", "OWN_TX", "OWN_RX", "A_DMADDR", "A_DMDATA", "A_TXF", "A_RXF", "A_STATUS"]
