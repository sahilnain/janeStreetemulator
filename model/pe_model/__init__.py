# SPDX-License-Identifier: Apache-2.0
"""
pe_model: cycle-accurate Python reference model of the Protocol Emulator v2 chip.

What this package is
--------------------
An executable specification, written from docs/spec/isa.md and docs/spec/blocks.md,
used to check the RTL. test/test_lockstep.py runs the RTL and this model side by
side and compares the pins and the internal state every clock cycle.
test/test_cross.py connects this model to the RTL on one bus, so one side talks a
protocol to the other, in both directions.

  config.py  Config: chip parameters (same defaults as the RTL top)
  blocks.py  Fifo, Imem, Dmem, Pins, Vote, Crc, HostIf, Regs
  ctrl.py    Ctrl: the controller and instruction decode
  txu.py     Txu: the TX unit
  rxu.py     Rxu: the RX unit
  det.py     Det: the event detector (ETX, ERX)
  chip.py    Chip: the whole chip, step() / outputs() per clock cycle
  host.py    ModelHost: drives the host bus of a Chip model (model-only tests)
"""

from .chip import Chip
from .config import Config
from .ctrl import ModelError

__all__ = ["Chip", "Config", "ModelError"]
