# SPDX-License-Identifier: Apache-2.0
# Tools for the protocol conformance tests: the sigrok protocol decoders (third party)
# and a Python to run the chip model. Build: docker build -t pe-conformance conformance
FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends \
    sigrok-cli libsigrokdecode4 python3 python3-pytest python3-pytest-xdist python3-crcmod \
    && rm -rf /var/lib/apt/lists/*
