# SPDX-License-Identifier: Apache-2.0
"""
harness: model-vs-RTL equivalence obligations, one per RTL module.

What this file does
-------------------
For each module, build:
  * the RTL transition function (rtl.Rtl, from the real Verilog), and
  * a `run()` that puts the RTL's CURRENT state and inputs into the matching
    pe_model block (as symbolic values), evaluates its combinational outputs
    and calls its clock() method;
then `check()` lists every output and every next-state register that must
be equal. Explorer.explore() proves "equal on every path, for every state
and input" (or returns a counterexample).

Assumptions are only reachable-state invariants already PROVED by the
SymbiYosys proofs in formal/ (cited per module), or the input contract of
pe_exec that pe_decode is proved to satisfy.
"""

import z3

from pe_model.blocks import STATE, Context, Fifo, HostIf, Imem, Pins, Regs, Vote
from pe_model.config import Config
from pe_model.isa import decode, execute

from .rtl import Rtl
from .sym import Explorer, Sym, V

W64 = 64


def S(e):
    return Sym(e)


def ulist(r, name, n, w):
    """Packed vector -> list of n Sym fields of width w."""
    return [S(r.now(name, k * w, w)) for k in range(n)]


def pack(vals, w):
    out = 0
    for k, x in enumerate(vals):
        out = out | (x << (k * w))
    return out


def eq_out(r, name, model_val):
    return (f"out {name}", V(model_val) != r.now(name))


def eq_next(r, name, model_val):
    """Next-state check; also fails if the model value does not fit the register."""
    val, mask, width = r.next_masked(name)
    mv = V(model_val)
    return (f"next {name}", z3.Or(mv & mask != val, z3.LShR(mv, width) != 0))


class Obligation:
    def __init__(self, name, rtl, run, check, assumptions=()):
        self.name, self.rtl, self.run, self.check = name, rtl, run, check
        self.assumptions = list(assumptions)

    def prove(self):
        return Explorer(self.assumptions).explore(self.run, self.check)


# ------------------------------------------------------------------ pe_fifo
def fifo(W=8, DEPTH=4):
    r = Rtl("pe_fifo", dict(W=W, DEPTH=DEPTH))
    # DEPTH=1: the pointers are always 0 and Yosys removes them
    cnt, rp, wp = r.now("cnt"), r.now("rptr", default=0), r.now("wptr", default=0)
    s = rp + cnt
    # invariants proved in formal/pe_fifo.sby
    inv = [z3.ULE(cnt, DEPTH), z3.ULT(rp, DEPTH), wp == z3.If(z3.UGE(s, DEPTH), s - DEPTH, s)]

    def run():
        m = Fifo(DEPTH, W)
        m.mem = [S(r.now(f"mem[{i}]")) for i in range(DEPTH)]
        m.wptr, m.rptr, m.cnt, m.overrun = S(wp), S(rp), S(cnt), S(r.now("overrun"))
        out = dict(empty=m.empty, full=m.full, count=m.cnt, rdata=m.rdata, overrun=m.overrun)
        m.clock(S(r.now("rst")), S(r.now("wr")), S(r.now("wdata")), S(r.now("rd")), S(r.now("clr_overrun")))
        nxt = dict(cnt=m.cnt, overrun=m.overrun)
        if r.has("rptr"):
            nxt.update(wptr=m.wptr, rptr=m.rptr)
        else:  # removed by Yosys: the model must keep them at 0 too
            nxt.update(_ptrs_zero=(m.wptr | m.rptr))
        nxt.update({f"mem[{i}]": m.mem[i] for i in range(DEPTH)})
        return out, nxt

    def check(res):
        out, nxt = res
        d = [eq_out(r, k, v) for k, v in out.items()]
        for k, v in nxt.items():
            d.append(("next pointers stay 0", V(v) != 0) if k == "_ptrs_zero" else eq_next(r, k, v))
        return d

    return Obligation(f"pe_fifo W={W} DEPTH={DEPTH}", r, run, check, inv)


# ------------------------------------------------------------------ pe_imem
def imem(IW=24, DEPTH=16):
    r = Rtl("pe_imem", dict(IW=IW, DEPTH=DEPTH, IMPL=0))

    def run():
        m = Imem(DEPTH, IW)
        m.mem = [S(r.now(f"mem[{i}]")) for i in range(DEPTH)]
        m.rdata = S(r.now("rdata"))
        out = dict(rdata=m.rdata)
        m.clock(S(r.now("we")), S(r.now("waddr")), S(r.now("wdata")), S(r.now("raddr")))
        nxt = {f"mem[{i}]": m.mem[i] for i in range(DEPTH)}
        nxt["rdata"] = m.rdata
        return out, nxt

    def check(res):
        out, nxt = res
        return [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]

    return Obligation(f"pe_imem IW={IW} DEPTH={DEPTH} (flop)", r, run, check)


# ------------------------------------------------------------------ pe_pins
def pins(NUM_PINS=8, NUM_CTX=2):
    r = Rtl("pe_pins", dict(NUM_PINS=NUM_PINS, NUM_CTX=NUM_CTX))
    cfg = Config(num_pins=NUM_PINS, num_ctx=NUM_CTX)

    def run():
        m = Pins(cfg)
        m.out_r, m.oe_r, m.owner_err = S(r.now("out_r")), S(r.now("oe_r")), S(r.now("owner_err"))
        m.s1, m.s2 = S(r.now("u_sync.s1")), S(r.now("u_sync.s2"))
        od = S(r.now("pin_od"))
        out = dict(pad_out=m.pad_out(od), pad_oe=m.pad_oe(od), pin_in=m.pin_in, owner_err=m.owner_err)
        g = lambda n: S(r.now(n))  # noqa: E731
        m.clock(g("rst"), g("wr_en"), g("wr_ctx"), g("wr_mask"), g("wr_val"), g("oe_en"), g("oe_mask"),
                g("oe_val"), g("pin_owner"), g("clr_err"), g("pad_in"))
        nxt = {"out_r": m.out_r, "oe_r": m.oe_r, "owner_err": m.owner_err, "u_sync.s1": m.s1, "u_sync.s2": m.s2}
        return out, nxt

    def check(res):
        out, nxt = res
        return [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]

    return Obligation(f"pe_pins NUM_PINS={NUM_PINS} NUM_CTX={NUM_CTX}", r, run, check)


# ------------------------------------------------------------------ pe_vote
VOTE_FIELDS = {"running": "running", "done": "done", "bit_out": "bit", "sel": "sel", "n_l": "n_l",
               "k_l": "k_l", "s_l": "s_l", "scnt": "scnt", "idx": "idx", "ones": "ones"}


def vote(NUM_PINS=8, N_W=4, S_W=12):
    r = Rtl("pe_vote", dict(NUM_PINS=NUM_PINS, N_W=N_W, S_W=S_W))
    cfg = Config(num_pins=NUM_PINS, vote_n_w=N_W, vote_s_w=S_W)

    def run():
        v = Vote(cfg)
        for rn, mn in VOTE_FIELDS.items():
            setattr(v, mn, S(r.now(rn)))
        out = dict(busy=v.busy, done=v.done, bit_out=v.bit)
        g = lambda n: S(r.now(n))  # noqa: E731
        v.clock(g("rst"), g("start"), g("pin_sel"), g("pin_in"), g("n_cfg"), g("k_cfg"), g("s_cfg"))
        return out, {rn: getattr(v, mn) for rn, mn in VOTE_FIELDS.items()}

    def check(res):
        out, nxt = res
        return [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]

    return Obligation(f"pe_vote NUM_PINS={NUM_PINS} N_W={N_W} S_W={S_W}", r, run, check)


# ------------------------------------------------------------------- pe_ctx
def ctx(AW=6, SR_W=16, CTR_W=16):
    r = Rtl("pe_ctx", dict(AW=AW, SR_W=SR_W, CTR_W=CTR_W))
    cfg = Config(imem_depth=1 << AW, sr_w=SR_W, ctr_w=CTR_W)

    def run():
        c = Context(cfg)
        for k in STATE + ("stall",):
            setattr(c, k, S(r.now(k)))
        g = lambda n: S(r.now(n))  # noqa: E731
        out = dict(ready=c.ready(g("run"), g("vote_busy")))
        out.update({k: getattr(c, k) for k in STATE})
        nxt_in = {k + "_n": g(k + "_n") for k in STATE + ("stall",)}
        c.clock(g("rst"), g("restart"), g("start_pc"), g("shift_msb"), g("we"), nxt_in, g("vote_done"),
                g("vote_bit"))
        return out, {k: getattr(c, k) for k in STATE + ("stall",)}

    def check(res):
        out, nxt = res
        return [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]

    return Obligation(f"pe_ctx AW={AW} SR_W={SR_W} CTR_W={CTR_W}", r, run, check)


# ---------------------------------------------------------------- pe_decode
DECODE_OUTS = ["op_set", "op_setoe", "op_out", "op_in", "op_wait", "op_jpin", "op_jmp", "op_djnz",
               "op_mov", "op_delay", "op_pull", "op_push", "op_vote", "op_illegal", "dly", "pin", "n",
               "lvl", "to_en", "to_ctr", "cond", "ctr_sel", "src", "nb", "target", "imm"]


def decode_ob(IW=24, AW=6):
    r = Rtl("pe_decode", dict(IW=IW, AW=AW))

    def run():
        return decode(S(r.now("ir")), IW, AW)

    def check(f):
        return [eq_out(r, k, f[k]) for k in DECODE_OUTS]

    return Obligation(f"pe_decode IW={IW} AW={AW}", r, run, check)


# ------------------------------------------------------------------ pe_exec
OPS = ["set", "setoe", "out", "in", "wait", "jpin", "jmp", "djnz", "mov", "delay", "pull", "push", "vote"]
EXEC_FIELDS = ["dly", "pin", "n", "lvl", "to_en", "to_ctr", "cond", "ctr_sel", "src", "nb", "target", "imm"]
EXEC_OUTS = ["done", "pc_n", "ctr0_n", "ctr1_n", "osr_n", "osr_cnt_n", "isr_n", "c_n", "to_n", "stall_n",
             "pin_wr_en", "pin_wr_mask", "pin_wr_val", "pin_oe_en", "pin_oe_mask", "pin_oe_val", "txf_rd",
             "rxf_wr", "rxf_wdata", "vote_start", "vote_pin"]


def exec_ob(op, AW=6, NUM_PINS=8, SR_W=16, DATA_W=8, CTR_W=16):
    """One obligation per op (op=None: no op flag set = nop/illegal)."""
    r = Rtl("pe_exec", dict(AW=AW, NUM_PINS=NUM_PINS, SR_W=SR_W, DATA_W=DATA_W, CTR_W=CTR_W))
    cfg = Config(imem_depth=1 << AW, num_pins=NUM_PINS, sr_w=SR_W, data_w=DATA_W, ctr_w=CTR_W)
    # input contract (pe_decode is proved to produce it): one op flag, n in 1..16
    asm = [r.now(f"op_{k}") == (1 if k == op else 0) for k in OPS]
    n = r.now("n")
    asm += [z3.UGE(n, 1), z3.ULE(n, 16)]

    def run():
        f = {k: S(r.now(k)) for k in EXEC_FIELDS}
        f["op"] = op or "nop"
        g = lambda k: S(r.now(k))  # noqa: E731
        s = {k: g(k) for k in ("pc", "ctr0", "ctr1", "osr", "osr_cnt", "isr", "c", "to", "shift_msb",
                               "vote_avail")}
        e = {k: g(k) for k in ("pin_in", "txf_rdata", "txf_empty", "rxf_full")}
        return execute(cfg, f, s, e)

    def check(o):
        return [eq_out(r, k, o[k]) for k in EXEC_OUTS]

    return Obligation(f"pe_exec op={op or 'none'} (AW={AW} NUM_PINS={NUM_PINS} SR_W={SR_W})", r, run, check, asm)


# --------------------------------------------------------------- pe_host_if
HOST_FIELDS = {"u_sync_stb.s1": "s1", "u_sync_stb.s2": "s2", "stb_q": "stb_q", "h_ack": "ack",
               "h_dout": "dout", "r_addr": "addr", "wbuf": "wbuf", "wphase": "wphase", "rlow": "rlow",
               "rphase": "rphase"}


def host_if():
    r = Rtl("pe_host_if")

    def run():
        h = HostIf()
        for rn, mn in HOST_FIELDS.items():
            setattr(h, mn, S(r.now(rn)))
        din, op = S(r.now("h_din")), S(r.now("h_op"))
        c = h.comb(din, op)
        out = {k: c[k] for k in ("r_addr", "r_wdata", "r_we", "r_re")}
        out.update(h_dout=h.dout, h_ack=h.ack)
        h.clock(S(r.now("rst")), din, op, S(r.now("h_stb")), S(r.now("r_rdata")))
        return out, {rn: getattr(h, mn) for rn, mn in HOST_FIELDS.items()}

    def check(res):
        out, nxt = res
        d = [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]
        return d + [("out h_irq", r.now("h_irq") != r.now("irq"))]

    return Obligation("pe_host_if", r, run, check)


# ------------------------------------------------------------------ pe_regs
def regs(NUM_CTX=2, NUM_PINS=8, IW=24, IMEM_DEPTH=64):
    r = Rtl("pe_regs", dict(NUM_CTX=NUM_CTX, NUM_PINS=NUM_PINS, IW=IW, IMEM_DEPTH=IMEM_DEPTH))
    cfg = Config(num_ctx=NUM_CTX, num_pins=NUM_PINS, iw=IW, imem_depth=IMEM_DEPTH, fifo_depth=4)
    aw, nw, sw = cfg.aw, cfg.vote_n_w, cfg.vote_s_w

    def run():
        g = Regs(cfg)
        g.run, g.msb = S(r.now("ctx_run")), S(r.now("ctx_shift_msb"))
        g.start_pc = ulist(r, "ctx_start_pc", NUM_CTX, aw)
        g.vote_n, g.vote_k = ulist(r, "vote_n", NUM_CTX, nw), ulist(r, "vote_k", NUM_CTX, nw)
        g.vote_s = ulist(r, "vote_s", NUM_CTX, sw)
        for rn, mn in (("pin_owner", "owner"), ("pin_od", "od"), ("imaddr", "imaddr"), ("stage", "stage"),
                       ("irq_en", "irq_en"), ("imem_err", "imem_err")):
            setattr(g, mn, S(r.now(rn)))
        h = {k: S(r.now(k)) for k in ("r_addr", "r_wdata", "r_we", "r_re")}
        st = {k: S(r.now(k)) for k in ("txf_empty", "txf_full", "txf_count", "txf_overrun", "rxf_empty",
                                         "rxf_full", "rxf_count", "rxf_overrun", "rxf_rdata", "owner_err",
                                         "pin_in")}
        st["ctx_pc"] = ulist(r, "ctx_pc", NUM_CTX, aw)
        for k, rn in (("ready", "ctx_ready"), ("vote_busy", "ctx_vote_busy"), ("c", "ctx_c"), ("to", "ctx_to")):
            st[k] = ulist(r, rn, NUM_CTX, 1)
        rc = g.comb(h, st)
        out = {"r_rdata": rc["r_rdata"], "irq": rc["irq"], "im_we": rc["im_we"], "im_waddr": rc["im_waddr"],
               "im_wdata": rc["im_wdata"], "txf_wr": rc["txf_wr"], "txf_wdata": rc["txf_wdata"],
               "rxf_rd": rc["rxf_rd"], "txf_clr_overrun": rc["txf_clr"], "rxf_clr_overrun": rc["rxf_clr"],
               "owner_clr_err": rc["owner_clr"], "ctx_restart": rc["restart"]}
        g.clock(S(r.now("rst")), h, rc)
        nxt = {"ctx_run": g.run, "ctx_shift_msb": g.msb, "ctx_start_pc": pack(g.start_pc, aw),
               "vote_n": pack(g.vote_n, nw), "vote_k": pack(g.vote_k, nw), "vote_s": pack(g.vote_s, sw),
               "pin_owner": g.owner, "pin_od": g.od, "imaddr": g.imaddr, "stage": g.stage,
               "irq_en": g.irq_en, "imem_err": g.imem_err}
        return out, nxt

    def check(res):
        out, nxt = res
        return [eq_out(r, k, v) for k, v in out.items()] + [eq_next(r, k, v) for k, v in nxt.items()]

    return Obligation(f"pe_regs NUM_CTX={NUM_CTX} NUM_PINS={NUM_PINS} IW={IW} IMEM_DEPTH={IMEM_DEPTH}",
                      r, run, check)
