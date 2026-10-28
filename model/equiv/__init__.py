# SPDX-License-Identifier: Apache-2.0
"""
equiv: formal equivalence of the Python reference model (pe_model) and the RTL.

  rtl.py      real Verilog -> Yosys AND/NOT netlist -> Z3 transition function
  sym.py      symbolic execution of the unchanged model over every path
  harness.py  one obligation per RTL module (state/IO mapping, invariants)
Run: cd model && python -m pytest -v tests/test_equiv.py
"""
