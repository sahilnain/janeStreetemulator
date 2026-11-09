# SPDX-License-Identifier: Apache-2.0
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "conformance", ROOT / "model", ROOT / "test" / "common"):
    sys.path.insert(0, str(p))

# Protocol support (docs/spec/blocks.md Section 13): PE_PROTOCOLS=uart,i2c builds the model without
# SPI and USB (pe_model.config), and the tests of a protocol that is not built are skipped.
# test_controls.py checks the decoders and checkers only, so it runs in every build.
import pytest  # noqa: E402
from pe_model.config import protocols_from_env  # noqa: E402

NEEDS = {"test_spi.py": "spi", "test_i2c.py": "i2c", "test_usb.py": "usb"}


def pytest_collection_modifyitems(config, items):
    built = protocols_from_env()
    for item in items:
        need = NEEDS.get(item.path.name)
        if need and need not in built:
            item.add_marker(pytest.mark.skip(reason=f"{need} is not built (PE_PROTOCOLS)"))
