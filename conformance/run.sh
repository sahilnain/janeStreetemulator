#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Run the conformance tests in the tools image. Arguments go to pytest.
# Example: conformance/run.sh test_uart.py -k tx_every
R=$(cd "$(dirname "$0")/.." && pwd)
exec docker run --rm --name "pe_conf_$$" -v "$R:/w" -w /w/conformance --user "$(id -u)" \
    -e PYTHONDONTWRITEBYTECODE=1 -e HOME=/tmp pe-conformance \
    python3 -m pytest -p no:cacheprovider -q "$@"
