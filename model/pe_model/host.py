# SPDX-License-Identifier: Apache-2.0
"""
host: host-bus driver for the Python chip model (no simulator needed).

What this file does
-------------------
ModelHost plays the RP2040: it drives ui_in like test/common/pe_host.py does
for the RTL (set op + nibble, toggle strobe, wait for the ack toggle) but by
calling Chip.step() directly. `pins_in` is a callback giving the uio input
value for each cycle (for loopbacks or driven lines); every sampled cycle of
the outputs is appended to `trace`.
"""

from .blocks import A_IMADDR, A_IMADDR_H, A_IMDATA, A_OWNER, A_RESTART, A_RUN, A_START_PC

OP_ADDR, OP_WRITE, OP_READ, OP_SYNC = range(4)


class ModelHost:
    def __init__(self, chip, pins_in=None):
        self.chip = chip
        self.ui = 0
        self.stb = 0
        self.pins_in = pins_in or (lambda outs: 0xFF)
        self.trace = []

    def cycle(self, n=1):
        for _ in range(n):
            outs = self.chip.outputs()
            self.trace.append(outs)
            self.chip.step(self.ui, self.pins_in(outs) & 0xFF, 1)

    def reset(self, cycles=10):
        for _ in range(cycles):
            self.chip.step(0, 0xFF, 0)
        self.cycle(5)

    def op(self, op, nib=0, timeout=50):
        ack = (self.chip.outputs()["uo_out"] >> 4) & 1
        self.stb ^= 1
        self.ui = (self.stb << 6) | (op << 4) | (nib & 0xF)
        for _ in range(timeout):
            self.cycle()
            o = self.chip.outputs()
            if ((o["uo_out"] >> 4) & 1) != ack:
                return o["uo_out"] & 0xF
        raise AssertionError("model host bus: no ack")

    def write(self, addr, data):
        self.op(OP_ADDR, addr >> 4)
        self.op(OP_ADDR, addr & 0xF)
        self.op(OP_WRITE, data >> 4)
        self.op(OP_WRITE, data & 0xF)

    def read(self, addr):
        self.op(OP_ADDR, addr >> 4)
        self.op(OP_ADDR, addr & 0xF)
        return (self.op(OP_READ) << 4) | self.op(OP_READ)

    def load_program(self, words, start=0):
        self.write(A_IMADDR, start & 0xFF)
        self.write(A_IMADDR_H, start >> 8)
        for w in words:
            for b in range(4):
                self.write(A_IMDATA + b, (w >> (8 * b)) & 0xFF)

    def owners(self, v):
        for b in range(4):
            self.write(A_OWNER + b, (v >> (8 * b)) & 0xFF)

    def start(self, pc=0):
        self.write(A_RUN, 0)
        self.write(A_START_PC, pc & 0xFF)
        self.write(A_START_PC + 1, pc >> 8)
        self.write(A_RESTART, 1)
        self.write(A_RUN, 1)
