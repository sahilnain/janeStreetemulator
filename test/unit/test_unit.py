# SPDX-License-Identifier: Apache-2.0
"""
test_unit: pytest runner for all module-level (unit) testbenches.

What this file does
-------------------
For every RTL module it compiles the module with Icarus Verilog under one
or more parameter sets, then runs the matching cocotb testbench
`tb_<module>.py`. Running each module under several configs is how we check
that the RTL really is parameterizable.

Run (inside the tt-dev container, from test/unit):
    pytest -v                 # everything (Icarus)
    pytest -v -k pe_fifo      # one module
Code coverage (needs Verilator >= 5.036, image tt-dev-cov):
    SIM=verilator COV=1 pytest -v
    then: python3 coverage_report.py   (merges sim_build/*/coverage.dat)
Build output goes to test/unit/sim_build/<sim>/<module>_<config>/.
"""

import json
import os
import shutil
import sys
from pathlib import Path

import pytest
from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1] / "src"
RTL = sorted(str(p) for p in SRC.rglob("*.v"))
COMMON = str(HERE.parent / "common")

# Shared helpers (pe_isa, pe_fw, uart) must be importable inside the simulator.
# The cocotb runner builds the simulator's PYTHONPATH from sys.path.
sys.path.insert(0, COMMON)

# (module, config name, parameters, extra sources)
CASES = [
    ("pe_sync", "w1", dict(W=1), []),
    ("pe_sync", "w5_rst", dict(W=5, RESET_VAL=0b10110), []),
    ("pe_fifo", "d8", dict(W=8, DEPTH=8), []),
    ("pe_fifo", "d3", dict(W=8, DEPTH=3), []),
    ("pe_fifo", "d1_w5", dict(W=5, DEPTH=1), []),
    ("pe_imem", "flop_w32", dict(IW=32, DEPTH=64, IMPL=0), []),
    ("pe_imem", "latch_w32", dict(IW=32, DEPTH=16, IMPL=1), []),
    ("pe_dmem", "d256", dict(DEPTH=256), []),
    ("pe_dmem", "d64", dict(DEPTH=64), []),
    ("pe_pins", "p8", dict(NUM_PINS=8), []),
    ("pe_pins", "p16", dict(NUM_PINS=16), []),
    ("pe_vote", "default", dict(NUM_PINS=8, N_W=4, S_W=16), []),
    ("pe_vote", "p16_n5", dict(NUM_PINS=16, N_W=5, S_W=6), []),
    ("pe_decode", "aw6", dict(AW=6), []),
    ("pe_decode", "aw10", dict(AW=10), []),
    ("pe_txu", "p8", dict(NUM_PINS=8), []),
    ("pe_txu", "p16", dict(NUM_PINS=16), []),
    ("pe_rxu", "p8", dict(NUM_PINS=8), []),
    ("pe_rxu", "p16", dict(NUM_PINS=16), []),
    ("pe_det", "etx", dict(KIND=0, NUM_PINS=8), []),
    ("pe_det", "erx", dict(KIND=1, NUM_PINS=8), []),
    ("pe_det", "erx_p16", dict(KIND=1, NUM_PINS=16), []),
    ("pe_crc", "default", dict(), []),
    ("pe_ctrl", "default", dict(IMEM_DEPTH=64, NUM_PINS=8, STACK_DEPTH=4), []),
    ("pe_ctrl", "d1024_p16_s2", dict(IMEM_DEPTH=1024, NUM_PINS=16, STACK_DEPTH=2), []),
    ("pe_host_if", "default", dict(), []),
    ("pe_regs", "default", dict(IMEM_DEPTH=64, NUM_PINS=8, FIFO_CW=4), []),
    ("pe_regs", "d1024_p16", dict(IMEM_DEPTH=1024, NUM_PINS=16, FIFO_CW=3), []),
]

SIM = os.environ.get("SIM", "icarus")
COV = os.environ.get("COV", "0") == "1"


@pytest.mark.parametrize("module,cfg,params,extra", CASES, ids=[f"{c[0]}-{c[1]}" for c in CASES])
def test_module(module, cfg, params, extra):
    runner = get_runner(SIM)
    build_dir = HERE / "sim_build" / SIM / f"{module}_{cfg}"
    build_args = []
    if SIM == "verilator":
        build_args = ["-Wno-fatal", "-Wno-lint", "-Wno-style", "--timing"]
        if COV:
            build_args += ["--coverage-line", "--coverage-toggle", "--coverage-user"]
    runner.build(
        sources=RTL + extra,
        hdl_toplevel=module,
        parameters=params,
        build_dir=build_dir,
        build_args=build_args,
        always=True,
        timescale=("1ns", "1ps"),
    )
    tb = "tb_" + module.removesuffix("_tb")
    cov = HERE / "coverage.dat"
    cov.unlink(missing_ok=True)
    results = runner.test(
        hdl_toplevel=module,
        test_module=tb,
        build_dir=build_dir,
        test_dir=HERE,
        extra_env={"PE_PARAMS": json.dumps(params)},
        timescale=("1ns", "1ps"),
    )
    if cov.exists():
        shutil.move(str(cov), str(build_dir / "coverage.dat"))
    total, failed = get_results(results)
    assert total > 0, "no tests ran"
    assert failed == 0, f"{failed}/{total} cocotb tests failed"
