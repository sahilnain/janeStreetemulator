# SPDX-License-Identifier: Apache-2.0
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "conformance", ROOT / "model", ROOT / "test" / "common"):
    sys.path.insert(0, str(p))
