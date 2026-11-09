# SPDX-License-Identifier: Apache-2.0
"""
pe_build: which protocols the chip under test was built with (docs/spec/blocks.md Section 13).

What this file does
-------------------
`make PROTOCOLS=uart,i2c` builds the RTL with -DSUPPORT_SPI=0 -DSUPPORT_USB=0 and sets the
environment variable PE_PROTOCOLS, which the Python model reads too (pe_model.config). This module
lets a test say what it needs:

    @pe_build.test("i2c")          # skipped unless I2C is built
    async def test_master_write_read(dut): ...

    if pe_build.has("i2c"): ...    # a generic test that adapts to the build

A test with no protocol needs only what is always built: UART and everything that is not part of a
protocol's features (controller, memories, FIFOs, pins, host bus).
"""

import sys
from pathlib import Path

import cocotb

_MODEL = Path(__file__).resolve().parents[2] / "model"
if str(_MODEL) not in sys.path:
    sys.path.insert(0, str(_MODEL))

from pe_model.config import protocols_from_env  # noqa: E402

PROTOCOLS = protocols_from_env()


def has(*names):
    """True if every protocol named is built."""
    return set(names) <= PROTOCOLS


def test(*needs, **kw):
    """cocotb.test, skipped when a protocol it needs is not built."""
    skip = kw.pop("skip", False) or not has(*needs)
    return cocotb.test(skip=skip, **kw)
