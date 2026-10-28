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
    pytest -v                 # everything
    pytest -v -k pe_fifo      # one module
Build output goes to test/unit/sim_build/<module>_<config>/.
"""

import json
import sys
from pathlib import Path

import pytest
from cocotb_tools.runner import get_results, get_runner

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1] / "src"
RTL = sorted(str(p) for p in SRC.rglob("*.v"))
HARNESS = [str(HERE / "hdl" / "pe_core_tb.v")]
COMMON = str(HERE.parent / "common")

# Shared helpers (pe_isa, pe_fw, uart) must be importable inside the simulator.
# The cocotb runner builds the simulator's PYTHONPATH from sys.path.
sys.path.insert(0, COMMON)

# (module, config name, parameters, extra sources)
CASES = [
    ("pe_sync", "w1", dict(W=1), []),
    ("pe_sync", "w5_rst", dict(W=5, RESET_VAL=0b10110), []),
    ("pe_fifo", "d4", dict(W=8, DEPTH=4), []),
    ("pe_fifo", "d3", dict(W=8, DEPTH=3), []),
    ("pe_fifo", "d1_w5", dict(W=5, DEPTH=1), []),
    ("pe_imem", "flop", dict(IW=24, DEPTH=64, IMPL=0), []),
    ("pe_imem", "latch", dict(IW=24, DEPTH=16, IMPL=1), []),
    ("pe_imem", "flop_w32", dict(IW=32, DEPTH=8, IMPL=0), []),
    ("pe_pins", "p8_c2", dict(NUM_PINS=8, NUM_CTX=2), []),
    ("pe_pins", "p16_c3", dict(NUM_PINS=16, NUM_CTX=3), []),
    ("pe_pins", "p4_c1", dict(NUM_PINS=4, NUM_CTX=1), []),
    ("pe_vote", "default", dict(NUM_PINS=8, N_W=4, S_W=12), []),
    ("pe_vote", "p16_n5", dict(NUM_PINS=16, N_W=5, S_W=6), []),
    ("pe_decode", "iw24", dict(IW=24, AW=6), []),
    ("pe_decode", "iw32_aw8", dict(IW=32, AW=8), []),
    ("pe_ctx", "default", dict(AW=6, SR_W=16, CTR_W=16, STALL_W=17), []),
    ("pe_ctx", "small", dict(AW=4, SR_W=10, CTR_W=12, STALL_W=17), []),
    ("pe_exec", "default", dict(AW=6, NUM_PINS=8, SR_W=16, DATA_W=8, CTR_W=16), []),
    ("pe_exec", "p16_sr12", dict(AW=8, NUM_PINS=16, SR_W=12, DATA_W=6, CTR_W=12), []),
    ("pe_core_tb", "c2", dict(NUM_CTX=2, VOTE_CTX_MASK=3), HARNESS),
    ("pe_core_tb", "c3_p16", dict(NUM_CTX=3, NUM_PINS=16, VOTE_CTX_MASK=6, IMEM_DEPTH=128), HARNESS),
    ("pe_core_tb", "c1", dict(NUM_CTX=1, VOTE_CTX_MASK=1), HARNESS),
    ("pe_host_if", "default", dict(), []),
    ("pe_regs", "default", dict(), []),
    ("pe_regs", "c3_p16_iw32", dict(NUM_CTX=3, NUM_PINS=16, IW=32, IMEM_DEPTH=256), []),
]


@pytest.mark.parametrize("module,cfg,params,extra", CASES, ids=[f"{c[0]}-{c[1]}" for c in CASES])
def test_module(module, cfg, params, extra):
    runner = get_runner("icarus")
    build_dir = HERE / "sim_build" / f"{module}_{cfg}"
    runner.build(
        sources=RTL + extra,
        hdl_toplevel=module,
        parameters=params,
        build_dir=build_dir,
        always=True,
        timescale=("1ns", "1ps"),
    )
    tb = "tb_" + module.removesuffix("_tb")
    results = runner.test(
        hdl_toplevel=module,
        test_module=tb,
        build_dir=build_dir,
        test_dir=HERE,
        extra_env={"PE_PARAMS": json.dumps(params)},
        timescale=("1ns", "1ps"),
    )
    total, failed = get_results(results)
    assert total > 0, "no tests ran"
    assert failed == 0, f"{failed}/{total} cocotb tests failed"
