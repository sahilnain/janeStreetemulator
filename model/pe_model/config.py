# SPDX-License-Identifier: Apache-2.0
"""
config: chip parameters for the reference model.

What this file does
-------------------
One dataclass with the same parameters (and defaults) as the localparams in
src/tt_um_sahil_protoemu.v, plus the derived widths the RTL computes with
$clog2. Every model block takes a Config, so the model can be run in any
configuration the RTL supports.
"""

from dataclasses import dataclass


def clog2(x):
    return max(0, (x - 1).bit_length())


@dataclass
class Config:
    num_ctx: int = 2
    imem_depth: int = 64
    iw: int = 24
    num_pins: int = 8
    data_w: int = 8
    fifo_depth: int = 4
    sr_w: int = 16
    ctr_w: int = 16
    vote_ctx_mask: int = 0b11
    vote_n_w: int = 4
    vote_s_w: int = 12
    vote_n_rst: int = 9
    vote_k_rst: int = 1
    vote_s_rst: int = 48
    chip_id: int = 0x50
    version: int = 0x01

    # derived (same formulas as the RTL)
    @property
    def aw(self):
        return max(1, clog2(self.imem_depth))

    @property
    def ctx_w(self):
        return max(1, clog2(self.num_ctx))

    @property
    def num_slots(self):
        return max(2, self.num_ctx)

    @property
    def cnt_w(self):
        return clog2(self.sr_w + 1)

    @property
    def fifo_cw(self):
        return clog2(self.fifo_depth + 1)

    @property
    def nb(self):
        """bytes per instruction word"""
        return (self.iw + 7) // 8

    stall_w: int = 17
