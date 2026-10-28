# SPDX-License-Identifier: Apache-2.0
"""
test_equiv: PROVE the Python reference model equals the RTL, module by module.

What is checked
---------------
For every module below and every listed configuration: for ALL current
states (allowed by the invariants already proved in formal/) and ALL inputs,
the model's outputs and next state are identical to the RTL's, bit for bit,
on every feasible path through the model code (see model/equiv/).
Composition (pe_core, the chip top) is checked by the lockstep simulation in
test/test_lockstep.py, which compares every block's state every cycle.
Needs yosys and the z3 Python package (both in the tt-dev image).
"""

import shutil

import pytest

z3 = pytest.importorskip("z3")
if not shutil.which("yosys"):
    pytest.skip("yosys not on PATH", allow_module_level=True)

from equiv import harness as H  # noqa: E402

CASES = [
    ("pe_fifo d4", lambda: H.fifo(8, 4)),
    ("pe_fifo d3", lambda: H.fifo(4, 3)),
    ("pe_fifo d1", lambda: H.fifo(5, 1)),
    ("pe_imem 16x24", lambda: H.imem(24, 16)),
    ("pe_imem 8x32", lambda: H.imem(32, 8)),
    ("pe_pins 8/2", lambda: H.pins(8, 2)),
    ("pe_pins 16/3", lambda: H.pins(16, 3)),
    ("pe_vote default", lambda: H.vote(8, 4, 12)),
    ("pe_vote p16", lambda: H.vote(16, 5, 6)),
    ("pe_ctx default", lambda: H.ctx(6, 16, 16)),
    ("pe_ctx small", lambda: H.ctx(4, 10, 12)),
    ("pe_decode iw24", lambda: H.decode_ob(24, 6)),
    ("pe_decode iw32", lambda: H.decode_ob(32, 8)),
    ("pe_host_if", lambda: H.host_if()),
    ("pe_regs default", lambda: H.regs()),
    ("pe_regs c3 p16 iw32", lambda: H.regs(3, 16, 32, 256)),
]
CASES += [(f"pe_exec {op or 'none'}", (lambda op=op: H.exec_ob(op))) for op in [None] + H.OPS]
CASES += [(f"pe_exec {op} p16sr12", (lambda op=op: H.exec_ob(op, 8, 16, 12, 6, 12)))
          for op in ("out", "in", "pull", "push", "set", "wait")]


@pytest.mark.parametrize("name,make", CASES, ids=[c[0] for c in CASES])
def test_model_equals_rtl(name, make):
    ob = make()
    paths = ob.prove()
    print(f"{ob.name}: equivalent on all {paths} model paths "
          f"(RTL: {ob.rtl.ncells} gates, {ob.rtl.nff} flip-flops)")
