# SPDX-License-Identifier: Apache-2.0
"""
sym: run the UNCHANGED Python model symbolically, over every path.

What this file does
-------------------
* `Sym` wraps a 64-bit Z3 bit-vector and overloads Python's integer
  operators (+ - * & | ^ ~ << >> and comparisons), so model code such as
  `(osr >> n) | (bits << (16 - n))` builds a Z3 expression instead of a
  number. 64-bit two's complement with SIGNED comparisons and arithmetic >>
  reproduces Python int behaviour for every value the model can produce
  (all well inside +/-2^63), including temporary negatives like
  `max(osr_cnt - n, 0)`.
* When the model branches on a symbolic value (`if`, `and`/`or`, `max`,
  `min`, `int()`, list indexing), `Explorer.decide()` asks the solver which
  outcomes are possible on the current path and follows each feasible one
  in turn (depth-first, by re-running the model with a recorded decision
  prefix). So EVERY feasible path through the model is executed once.
* For each path, `explore()` asks Z3 whether ANY state/input on that path
  makes the model differ from the RTL. "unsat" on every path = equivalent.
* `SymList` stands in for a model list such as a register file. Indexing it with
  a symbolic value builds a mux instead of splitting the path once per index,
  which keeps the path count small, and still fails if the index can be out of
  range.
"""

import os

import z3

W = 64
PROGRESS = int(os.environ.get("EQUIV_PROGRESS", 0))


class EquivError(AssertionError):
    pass


class Explorer:
    cur = None  # explorer in charge of the running model code

    def __init__(self, assumptions=(), timeout_s=None):
        self.s = z3.SolverFor("QF_BV")
        if timeout_s:
            self.s.set("timeout", int(timeout_s * 1000))  # per query, so a slow case fails and cannot hang
        for a in assumptions:
            self.s.add(a)
        self.paths = 0
        self.describe = None  # gives the state and inputs of a counterexample

    def _sat(self, cond):
        self.s.push()
        self.s.add(cond)
        r = self.s.check()
        self.s.pop()
        return r != z3.unsat

    def decide(self, cond):
        cond = z3.simplify(cond)
        if z3.is_true(cond):
            return True
        if z3.is_false(cond):
            return False
        if self.i < len(self.prefix):
            d = self.prefix[self.i]
        elif not self._sat(cond):
            d = False
        else:
            d = True
            if self._sat(z3.Not(cond)):
                self.pending.append(self.decisions + [False])
        self.decisions.append(d)
        self.i += 1
        self.s.add(cond if d else z3.Not(cond))
        return d

    def explore(self, run, check):
        """run(): executes the model, returns its results.
        check(results): list of (name, z3 Bool that is True when model != RTL)."""
        if self.s.check() != z3.sat:      # assumptions that contradict would make any proof pass
            raise EquivError("the assumptions cannot all hold")
        self.pending = [[]]
        while self.pending:
            self.prefix, self.i, self.decisions = self.pending.pop(), 0, []
            self.s.push()
            Explorer.cur = self
            try:
                res = run()
                diffs = check(res)
            finally:
                Explorer.cur = None
            self.s.push()
            self.s.add(z3.Or([d for _, d in diffs]) if diffs else z3.BoolVal(False))
            r = self.s.check()
            if r == z3.sat:
                m = self.s.model()
                bad = [n for n, d in diffs if z3.is_true(m.eval(d, model_completion=True))]
                info = self.describe(m) if self.describe else ""
                raise EquivError(f"model != RTL: {bad}\n{info}")
            if r != z3.unsat:
                raise EquivError(f"solver returned {r} on path {self.decisions}")
            self.s.pop()
            self.s.pop()
            self.paths += 1
            if PROGRESS and self.paths % PROGRESS == 0:
                print(f"paths {self.paths}, open {len(self.pending)}", flush=True)
        return self.paths


def V(x):
    """Z3 bit-vector for a Sym, Python int or bool."""
    if isinstance(x, Sym):
        return x.e
    if isinstance(x, z3.BitVecRef):
        return x
    return z3.BitVecVal(int(x), W)


class Sym:
    __slots__ = ("e",)
    __hash__ = None

    def __init__(self, e):
        self.e = e

    # arithmetic / bitwise
    def __add__(self, o): return Sym(self.e + V(o))
    def __radd__(self, o): return Sym(V(o) + self.e)
    def __sub__(self, o): return Sym(self.e - V(o))
    def __rsub__(self, o): return Sym(V(o) - self.e)
    def __mul__(self, o): return Sym(self.e * V(o))
    def __rmul__(self, o): return Sym(V(o) * self.e)
    def __and__(self, o): return Sym(self.e & V(o))
    def __rand__(self, o): return Sym(V(o) & self.e)
    def __or__(self, o): return Sym(self.e | V(o))
    def __ror__(self, o): return Sym(V(o) | self.e)
    def __xor__(self, o): return Sym(self.e ^ V(o))
    def __rxor__(self, o): return Sym(V(o) ^ self.e)
    def __lshift__(self, o): return Sym(self.e << V(o))
    def __rlshift__(self, o): return Sym(V(o) << self.e)
    def __rshift__(self, o): return Sym(self.e >> V(o))      # arithmetic, like Python
    def __rrshift__(self, o): return Sym(V(o) >> self.e)
    def __mod__(self, o): return Sym(z3.URem(self.e, V(o)))  # only used on values >= 0
    def __neg__(self): return Sym(-self.e)
    def __invert__(self): return Sym(~self.e)

    # comparisons (signed, like Python ints)
    def __eq__(self, o): return SymBool(self.e == V(o))
    def __ne__(self, o): return SymBool(self.e != V(o))
    def __lt__(self, o): return SymBool(self.e < V(o))
    def __le__(self, o): return SymBool(self.e <= V(o))
    def __gt__(self, o): return SymBool(self.e > V(o))
    def __ge__(self, o): return SymBool(self.e >= V(o))

    def mux(self, a, b):
        return Sym(z3.If(self.e != 0, V(a), V(b)))

    # control flow on a symbolic value -> path decision
    def __bool__(self):
        return Explorer.cur.decide(self.e != 0)

    def __index__(self):
        """Concrete value needed (list index, range, dict key): enumerate the
        feasible values one decision at a time."""
        ex = Explorer.cur
        while True:
            ex.s.check()
            v = ex.s.model().eval(self.e, model_completion=True).as_signed_long()
            if ex.decide(self.e == v):
                return v

    __int__ = __index__


class SymBool(Sym):
    """Result of a comparison: behaves as 0/1 in arithmetic, branches as a condition."""
    __slots__ = ("b",)

    def __init__(self, b):
        super().__init__(z3.If(b, z3.BitVecVal(1, W), z3.BitVecVal(0, W)))
        self.b = b

    def __bool__(self):
        return Explorer.cur.decide(self.b)

    def __index__(self):
        return 1 if bool(self) else 0

    __int__ = __index__


class SymList:
    """A list whose index may be symbolic: a read is a mux, a write a conditional update."""

    def __init__(self, items):
        self.items = [V(x) for x in items]

    def __len__(self):
        return len(self.items)

    def _check(self, i):
        ex = Explorer.cur
        if ex._sat(z3.Or(i.e < 0, i.e >= len(self.items))):
            raise EquivError(f"list index can be out of range: {z3.simplify(i.e)}")

    def __getitem__(self, i):
        if not isinstance(i, Sym):
            return Sym(self.items[i])
        self._check(i)
        out = self.items[-1]
        for k in range(len(self.items) - 2, -1, -1):
            out = z3.If(i.e == k, self.items[k], out)
        return Sym(out)

    def __setitem__(self, i, val):
        if not isinstance(i, Sym):
            self.items[i] = V(val)
            return
        self._check(i)
        for k in range(len(self.items)):
            self.items[k] = z3.If(i.e == k, V(val), self.items[k])
