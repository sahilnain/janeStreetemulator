# SPDX-License-Identifier: Apache-2.0
"""
config: chip parameters for the reference model.

What this file does
-------------------
One dataclass with the same parameters (and defaults) as the localparams in
src/tt_um_sahil_protoemu.v, plus the widths the RTL derives with $clog2.

Protocol support (docs/spec/blocks.md Section 13): `spi`, `i2c` and `usb` match the RTL macros
SUPPORT_SPI, SUPPORT_I2C and SUPPORT_USB. UART is always built. By default they come from the
environment variable PE_PROTOCOLS, a comma-separated list such as "uart,i2c"; without it, all
four are built. The test benches set PE_PROTOCOLS from `make PROTOCOLS=...`, so the model and
the RTL are built alike.
"""

import os
from dataclasses import dataclass, field

PROTOCOLS = ("uart", "spi", "i2c", "usb")


def protocols_from_env():
    """The set of protocols named by PE_PROTOCOLS (all four if unset). UART is always included."""
    s = os.environ.get("PE_PROTOCOLS", "").strip()
    names = {p.strip().lower() for p in s.split(",") if p.strip()} if s else set(PROTOCOLS)
    bad = names - set(PROTOCOLS)
    if bad:
        raise ValueError(f"PE_PROTOCOLS: unknown {sorted(bad)}; known: {', '.join(PROTOCOLS)}")
    return names | {"uart"}


def _env(name):
    return field(default_factory=lambda: name in protocols_from_env())


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
    spi: bool = _env("spi")
    i2c: bool = _env("i2c")
    usb: bool = _env("usb")

    @property
    def protocols(self):
        return {"uart"} | {p for p in ("spi", "i2c", "usb") if getattr(self, p)}

    @property
    def aw(self):
        return max(1, clog2(self.imem_depth))
