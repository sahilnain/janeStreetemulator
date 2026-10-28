# SPDX-License-Identifier: Apache-2.0
"""
pe_isa: the instruction set, for the tests.

What this file does
-------------------
Re-exports model/pe_model/isa.py, the single source of truth for the ISA
(encoders such as set_/out/jmp, decode(), execute(), opcode and condition
codes). Tests keep using `import pe_isa as I`; the definitions live with the
reference model so the model, the tests and the future assembler cannot
drift apart.
"""

import sys
from pathlib import Path

_MODEL = str(Path(__file__).resolve().parents[2] / "model")
if _MODEL not in sys.path:
    sys.path.insert(0, _MODEL)

from pe_model.isa import *  # noqa: E402,F401,F403
from pe_model.isa import IW_DEFAULT, OP, OP_NAMES, decode, execute, word  # noqa: E402,F401
