# SPDX-License-Identifier: Apache-2.0
"""
test_equiv: PROVE the Python reference model equals the RTL, module by module.

What is checked
---------------
For every module of the v2 chip, in the chip's configuration: for ALL current
states and ALL inputs, the model's outputs and next state are identical to the
RTL's, bit for bit, on every feasible path through the model code (see
model/equiv/). harness.py lists what is compared and what is assumed.
The wiring between the modules (the chip top) is checked by the lockstep
simulation in test/test_lockstep.py, which compares every block's state every
cycle. Needs yosys and the z3 Python package (both in the tt-dev image).
"""

import shutil

import pytest

z3 = pytest.importorskip("z3")
if not shutil.which("yosys"):
    pytest.skip("yosys not on PATH", allow_module_level=True)

from equiv import harness as H  # noqa: E402

CASES = [
    ("pe_decode", H.decode_ob),
    ("pe_fifo d8", lambda: H.fifo(8, 8)),
    ("pe_fifo d3", lambda: H.fifo(8, 3)),
    ("pe_imem", H.imem),
    ("pe_dmem", H.dmem),
    ("pe_pins", H.pins),
    ("pe_vote", H.vote),
    ("pe_crc", H.crc),
    ("pe_host_if", H.host_if),
    ("pe_regs", H.regs),
    ("pe_det etx", lambda: H.det(0)),
    ("pe_det erx", lambda: H.det(1)),
    ("pe_txu", H.txu),
    ("pe_rxu", H.rxu),
]
# Protocol support (docs/spec/blocks.md Section 13): the modules with switches, in reduced builds.
# UART only, and each protocol on its own, between them take every switch both ways.
for _p in (("uart",), ("uart", "spi"), ("uart", "i2c"), ("uart", "usb")):
    _t = ",".join(_p)
    CASES += [(f"pe_txu [{_t}]", (lambda p=_p: H.txu(protos=p))),
              (f"pe_rxu [{_t}]", (lambda p=_p: H.rxu(protos=p))),
              (f"pe_det etx [{_t}]", (lambda p=_p: H.det(0, protos=p))),
              (f"pe_det erx [{_t}]", (lambda p=_p: H.det(1, protos=p)))]
CASES += [("pe_crc [uart]", lambda: H.crc(protos=("uart",))),
          ("pe_regs [uart]", lambda: H.regs(protos=("uart",)))]
CASES += [(f"pe_ctrl op{op}", (lambda op=op: H.ctrl(op))) for op in H.CTRL_OPS]
CASES += [("pe_ctrl illegal", lambda: H.ctrl(None))]


@pytest.mark.parametrize("name,make", CASES, ids=[c[0] for c in CASES])
def test_model_equals_rtl(name, make):
    ob = make()
    paths = ob.prove()
    print(f"{ob.name}: equivalent on all {paths} model paths "
          f"(RTL: {ob.rtl.ncells} gates, {ob.rtl.nff} flip-flops)")
