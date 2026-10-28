# SPDX-License-Identifier: Apache-2.0
"""
pe_model: cycle-accurate Python reference model of the Protocol Emulator chip.

What this package is
--------------------
An executable specification written from the module headers / ISA, used to
check the RTL: the lockstep test (test/test_lockstep.py) runs the RTL and
this model side by side and compares the pins and the internal state every
clock cycle. It is also the base for the future assembler and instruction
set simulator (hw_plan.md M2).

  config.py  Config: chip parameters (same defaults as the RTL top)
  isa.py     encoders, decode(), execute()  (single source of truth for the ISA)
  blocks.py  Fifo, Imem, Pins, Vote, Context, HostIf, Regs
  core.py    Core: slot scheduler, contexts, vote units
  chip.py    Chip: the whole chip, step()/outputs() per clock cycle
  host.py    ModelHost: drives the host bus of a Chip model (model-only tests)
"""

from .chip import Chip
from .config import Config
from .core import ModelError

__all__ = ["Chip", "Config", "ModelError"]
