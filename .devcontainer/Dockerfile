ARG VARIANT=ubuntu-24.04
FROM mcr.microsoft.com/vscode/devcontainers/base:${VARIANT}

ENV DEBIAN_FRONTEND=noninteractive
ENV PDK_ROOT=/home/vscode/ttsetup/pdk
ENV PDK=ihp-sg13g2
ARG TT_SUPPORT_TOOLS_BRANCH=main

RUN apt update
RUN apt install -y iverilog gtkwave python3 python3-pip python3-venv python3-tk python-is-python3 libcairo2 verilator libpng-dev libqhull-dev

# Clone tt-support-tools
RUN mkdir -p /ttsetup
RUN git clone -b ${TT_SUPPORT_TOOLS_BRANCH} https://github.com/TinyTapeout/tt-support-tools /ttsetup/tt-support-tools

COPY test/requirements.txt /ttsetup/test_requirements.txt
COPY .devcontainer/copy_tt_support_tools.sh /ttsetup

RUN python -m venv /ttsetup/venv
RUN /ttsetup/venv/bin/pip install -r /ttsetup/test_requirements.txt -r /ttsetup/tt-support-tools/requirements.txt

# Install verible (for formatting)
RUN umask 022 && \
    curl -L https://github.com/chipsalliance/verible/releases/download/v0.0-4023-gc1271a00/verible-v0.0-4023-gc1271a00-linux-static-x86_64.tar.gz | \
    tar zxf - -C /usr/local --strip-components=1 && \
    chmod 755 /usr/local/bin

# Install LibreLane
RUN /ttsetup/venv/bin/pip install librelane==3.0.0.dev44

# Formal verification (Protocol Emulator): YosysHQ OSS CAD Suite, pinned release.
# Provides yosys, SymbiYosys (sby) and the solvers (yices, boolector, bitwuzla, z3).
# Its bin/ goes LAST on PATH, so the apt iverilog/verilator used by the cocotb
# tests stay first; yosys/sby/solvers only exist here.
ARG OSS_CAD_SUITE_DATE=2026-10-01
RUN umask 022 && \
    curl -fL https://github.com/YosysHQ/oss-cad-suite-build/releases/download/${OSS_CAD_SUITE_DATE}/oss-cad-suite-linux-x64-$(echo ${OSS_CAD_SUITE_DATE} | tr -d -).tgz | \
    tar zxf - -C /opt
ENV PATH=${PATH}:/opt/oss-cad-suite/bin

# Model-vs-RTL equivalence proofs (model/equiv): Z3 Python bindings.
RUN /ttsetup/venv/bin/pip install z3-solver==4.15.4.0
