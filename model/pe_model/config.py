# SPDX-License-Identifier: Apache-2.0
"""
config: chip parameters for the reference model.

What this file does
-------------------
One dataclass with the same parameters (and defaults) as the localparams in
src/tt_um_sahil_protoemu.v, plus the widths the RTL derives with $clog2.
"""

from dataclasses import dataclass


def clog2(x):
    return max(0, (x - 1).bit_length())


@dataclass
class Config:
    imem_depth: int = 160
    dmem_depth: int = 256
    fifo_depth: int = 8
    stack_depth: int = 4
    num_pins: int = 8
    vote_n_w: int = 4
    vote_s_w: int = 16
    chip_id: int = 0x50
    version: int = 0x02

    @property
    def aw(self):
        return max(1, clog2(self.imem_depth))
