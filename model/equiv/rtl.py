# SPDX-License-Identifier: Apache-2.0
"""
rtl: load a real RTL module as Z3 formulas (no hand translation).

What this file does
-------------------
Runs Yosys on the actual src/ Verilog (FORMAL blocks excluded, chparam for
the requested configuration) and lowers it to a pure AND/NOT netlist plus
plain D flip-flops:
    proc; flatten; memory -nordff; memory_map; techmap; dffunmap; aigmap
(-nordff keeps every register as written: without it Yosys would copy a
read-address register into the memory port, creating a duplicate state bit.)
The JSON netlist is then turned into Z3 Booleans:
  * every input-port bit           -> a free Boolean
  * every flip-flop output (Q) bit -> a free Boolean (the CURRENT state)
  * every other net                -> an AND/NOT expression over those
  * next(reg)                      -> the flip-flop D expressions (NEXT state)
So `Rtl` is the exact one-cycle transition function of the RTL, for ANY
current state and ANY inputs. Undefined bits ('x') become free Booleans.
"""

import json
import subprocess
import tempfile
from pathlib import Path

import z3

SRC = Path(__file__).resolve().parents[2] / "src"
W = 64  # width of the model-side bit-vectors (see sym.py)


def _synth(top, params, sources):
    chp = " ".join(f"-set {k} {v}" for k, v in params.items())
    out = Path(tempfile.mkdtemp(prefix="equiv_")) / f"{top}.json"
    script = (f"read_verilog {' '.join(str(s) for s in sources)}; "
              + (f"chparam {chp} {top}; " if params else "")
              + f"hierarchy -top {top}; proc; flatten; memory -nordff -nomap; memory_map; opt_clean; "
              f"techmap; dffunmap; aigmap; opt_clean; write_json {out}")
    subprocess.run(["yosys", "-q", "-p", script], check=True)
    return json.loads(out.read_text())


class Rtl:
    def __init__(self, top, params=None, sources=None):
        sources = sources or sorted(SRC.rglob("*.v"))
        j = _synth(top, params or {}, sources)
        mod = j["modules"][top]
        self.top = top
        self.ports = {n: p for n, p in mod["ports"].items()}
        self.names = {n: v["bits"] for n, v in mod["netnames"].items()}
        self.var = {}      # bit id -> z3 Bool for inputs and flip-flop outputs
        self.drv = {}      # bit id -> (cell type, [input bit ids])
        self.dff_d = {}    # Q bit id -> D bit id
        for n, p in self.ports.items():
            if p["direction"] == "input":
                for i, b in enumerate(p["bits"]):
                    if isinstance(b, int):
                        self.var[b] = z3.Bool(f"in.{n}[{i}]")
        for c in mod["cells"].values():
            t, con = c["type"], c["connections"]
            if t == "$_DFF_P_":
                q = con["Q"][0]
                self.var[q] = z3.Bool(f"q.{q}")
                self.dff_d[q] = con["D"][0]
            elif t == "$_AND_":
                self.drv[con["Y"][0]] = ("and", [con["A"][0], con["B"][0]])
            elif t == "$_NOT_":
                self.drv[con["Y"][0]] = ("not", [con["A"][0]])
            elif t == "$_BUF_":
                self.drv[con["Y"][0]] = ("buf", [con["A"][0]])
            elif t == "$scopeinfo":  # hierarchy metadata left by flatten, no logic
                continue
            else:
                raise ValueError(f"unexpected cell {t} in {top}")
        self._memo = {}
        self.ncells = len(mod["cells"])
        self.nff = len(self.dff_d)

    # ---------------------------------------------------------- bit level
    def bit(self, b):
        """z3 Bool for net bit `b` (int id or '0'/'1'/'x'/'z')."""
        if b == "0":
            return z3.BoolVal(False)
        if b == "1":
            return z3.BoolVal(True)
        if not isinstance(b, int):
            return z3.FreshBool("undef")
        if b in self._memo:
            return self._memo[b]
        stack = [b]
        while stack:  # iterative post-order, the netlist can be deep
            x = stack[-1]
            if x in self._memo:
                stack.pop()
                continue
            if x in self.var:
                self._memo[x] = self.var[x]
                stack.pop()
                continue
            if x not in self.drv:
                self._memo[x] = z3.FreshBool("undriven")
                stack.pop()
                continue
            t, ins = self.drv[x]
            todo = [i for i in ins if isinstance(i, int) and i not in self._memo]
            if todo:
                stack.extend(todo)
                continue
            a = [self.bit(i) if not isinstance(i, int) else self._memo[i] for i in ins]
            self._memo[x] = z3.And(a[0], a[1]) if t == "and" else (z3.Not(a[0]) if t == "not" else a[0])
            stack.pop()
        return self._memo[b]

    # -------------------------------------------------------- vector level
    @staticmethod
    def bv(bools):
        """List of Booleans (LSB first) -> 64-bit bit-vector (zero-extended)."""
        bits = [z3.If(x, z3.BitVecVal(1, 1), z3.BitVecVal(0, 1)) for x in bools]
        v = bits[0] if len(bits) == 1 else z3.Concat(*reversed(bits))
        return z3.ZeroExt(W - len(bools), v)

    def bits_of(self, name):
        if name in self.ports:
            return self.ports[name]["bits"]
        if name in self.names:
            return self.names[name]
        raise KeyError(f"{self.top}: no net or port named {name!r}")

    def has(self, name):
        return name in self.ports or name in self.names

    def now(self, name, lo=0, width=None, default=None):
        """Current value of a net / port / register (slice [lo, lo+width)).
        `default`: value to use if Yosys removed the net entirely (a register
        that can only ever hold one value, e.g. a DEPTH=1 FIFO pointer)."""
        if default is not None and not self.has(name):
            return z3.BitVecVal(default, W)
        bits = self.bits_of(name)
        bits = bits[lo:lo + width] if width is not None else bits[lo:]
        return self.bv([self.bit(b) for b in bits])

    def next(self, name, lo=0, width=None):
        """Next-state value of a register: the D input of each flip-flop bit.
        Bits that Yosys proved constant keep their constant."""
        bits = self.bits_of(name)
        bits = bits[lo:lo + width] if width is not None else bits[lo:]
        out = []
        for b in bits:
            if isinstance(b, int) and b in self.dff_d:
                out.append(self.bit(self.dff_d[b]))
            elif not isinstance(b, int) and b in "01":
                out.append(self.bit(b))
            else:
                raise KeyError(f"{self.top}: {name} bit {b} is not a flip-flop")
        return self.bv(out)

    def next_masked(self, name, lo=0, width=None):
        """Like next(), but tolerates bits Yosys removed as unobservable
        (e.g. a staging byte that is never read). Returns (value, mask):
        only bits set in `mask` are real flip-flops (or constants)."""
        bits = self.bits_of(name)
        bits = bits[lo:lo + width] if width is not None else bits[lo:]
        vals, mask = [], 0
        for i, b in enumerate(bits):
            if isinstance(b, int) and b in self.dff_d:
                vals.append(self.bit(self.dff_d[b]))
                mask |= 1 << i
            elif not isinstance(b, int) and b in "01":
                vals.append(self.bit(b))
                mask |= 1 << i
            else:
                vals.append(z3.BoolVal(False))
        return self.bv(vals), mask, len(bits)

