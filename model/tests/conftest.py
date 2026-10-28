# SPDX-License-Identifier: Apache-2.0
"""conftest: make `pe_model`, `equiv` (model/) and the shared test helpers (test/common) importable."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT / "model", ROOT / "test" / "common"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
