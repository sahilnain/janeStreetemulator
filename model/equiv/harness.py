# SPDX-License-Identifier: Apache-2.0
"""
harness: model-vs-RTL equivalence obligations, one per RTL module of the v2 chip.

What this file does
-------------------
For each module, build:
  * the RTL transition function (rtl.Rtl, from the real Verilog), and
  * a `run()` that puts the RTL's CURRENT state and inputs into the matching
    pe_model block (as symbolic values), evaluates its combinational outputs
    and calls its clock() method.
Explorer.explore() then proves, on every path through the model and for every
state and input, that each listed output and next-state register is equal.

What "equal" means
------------------
The proof is an induction step: if the model and the RTL agree now, they agree
after one clock. Three things make it precise:
  * A register that only means something in some states (the bit timer `o` while
    TX is idle) is compared under a condition. While the condition is false the
    model holds a free value, not the RTL's, so the proof fails if the model
    reads it there. See `Ob.loose`.
  * An output that only means something when its strobe is set (cfg_data with
    cfg_we) is compared under that strobe.
  * Assumptions are facts about reachable states. Each one is checked to hold
    again after the clock (`Ob.inv`), so it holds by induction from reset.
    Assumptions on inputs are contracts on the module's neighbours: they name
    the formal/ proof that shows the neighbour keeps them.
Memories have no reset, so they are assumed equal at the start, as the lockstep
test does when it copies them. An unwritten word is `None` in the model and X in
the RTL.
"""

import os

import z3

from pe_model.blocks import Crc, Dmem, Fifo, HostIf, Imem, Pins, Regs, Vote
from pe_model.config import Config
from pe_model.ctrl import Ctrl, decode
from pe_model.det import Det
from pe_model.rxu import Rxu
from pe_model.txu import Txu

from .rtl import Rtl
from .sym import EquivError, Explorer, Sym, SymList, V

TIMEOUT_S = float(os.environ.get("EQUIV_TIMEOUT", 1800))


def S(e):
    return Sym(e)


def B(x):
    """z3 Bool: a model or RTL value is non-zero."""
    return z3.BoolVal(bool(x)) if isinstance(x, (int, bool)) else V(x) != 0


class Ob:
    """One proof obligation. `build(ob)` returns (outputs, next state, extra checks).

    outputs and next state: {RTL name: model value} or {RTL name: (model value, condition)}.
    extra checks: [(label, z3 Bool that is True on a failure)].
    """

    def __init__(self, name, rtl, build, assumptions=()):
        self.name, self.rtl, self.build = name, rtl, build
        self.assumptions = list(assumptions)
        self._free = {}

    def free(self, name):
        if name not in self._free:
            self._free[name] = z3.BitVec(f"free.{name}", 64)
        return self._free[name]

    def loose(self, name, cond, val=None):
        """The model's value of a register that only means something while `cond` holds."""
        val = self.rtl.now(name) if val is None else V(val)
        return S(z3.If(cond, val, self.free(name)))

    def opt(self, name):
        """A register Yosys may remove because nothing reads it: free in the model if so."""
        return S(self.rtl.now(name) if self.rtl.has(name) else self.free(name))

    def inv(self, label, cond_next):
        """An assumption must hold again after the clock."""
        return (f"invariant {label}", z3.Not(cond_next))

    def _diffs(self, res):
        r = self.rtl
        outs, nxt, extra = res
        d = []
        for k, v in outs.items():
            v, c = v if isinstance(v, tuple) else (v, None)
            ne = V(v) != r.now(k)
            d.append((f"out {k}", ne if c is None else z3.And(B(c), ne)))
        for k, v in nxt.items():
            v, c = v if isinstance(v, tuple) else (v, None)
            val, mask, width = r.next_masked(k)
            mv = V(v)
            ne = z3.Or(mv & mask != val, z3.LShR(mv, width) != 0)
            d.append((f"next {k}", ne if c is None else z3.And(B(c), ne)))
        return d + list(extra)

    def _describe(self, m):
        """Counterexample: the inputs, the registers and their next values, as hex."""
        r = self.rtl
        val = lambda e: f"{m.eval(e, model_completion=True).as_long():x}"  # noqa: E731
        ins = [f"{n}={val(r.now(n))}" for n, p in r.ports.items() if p["direction"] == "input" and n != "clk"]
        regs = []
        for n, bits in sorted(r.names.items()):
            if "$" not in n and all(isinstance(b, int) and b in r.dff_d for b in bits):
                regs.append(f"{n}={val(r.now(n))}>{val(r.next(n))}")
        return "inputs: " + " ".join(ins) + "\nregs now>next: " + " ".join(regs)

    def prove(self):
        ex = Explorer(self.assumptions, TIMEOUT_S)
        ex.describe = self._describe
        paths = ex.explore(lambda: self.build(self), self._diffs)
        missed = set(self.rtl.dff_d) - self.rtl.compared
        if missed:
            names = sorted({n for n, bits in self.rtl.names.items() if missed & set(bits) and "$" not in n})
            raise EquivError(f"{self.name}: {len(missed)} flip-flops are not compared: {names}")
        return paths


def fields_now(r, fields):
    """Packed model config word from RTL registers: [(reg, lo, width, model lo)]."""
    out = z3.BitVecVal(0, 64)
    for reg, lo, w, mlo in fields:
        out = out | (r.now(reg, lo, w) << mlo)
    return S(out)


def fields_next(r, m_val, fields):
    """Checks that a packed model config word matches the RTL's next registers."""
    d = []
    mv = V(m_val)
    for reg, lo, w, mlo in fields:
        val, mask, _ = r.next_masked(reg, lo, w)
        d.append((f"next {reg}[{lo}+{w}]", (z3.LShR(mv, mlo) & ((1 << w) - 1)) & mask != val))
    return d


# ------------------------------------------------------------------ pe_fifo
def fifo(W=8, DEPTH=8):
    r = Rtl("pe_fifo", dict(W=W, DEPTH=DEPTH))
    cnt, rp, wp = r.now("cnt"), r.now("rptr", default=0), r.now("wptr", default=0)
    # ring: wptr is rptr + cnt, wrapped (formal/pe_fifo.sby)
    ring = lambda c, a, b: z3.And(z3.ULE(c, DEPTH), z3.ULT(a, DEPTH), z3.ULT(b, DEPTH),  # noqa: E731
                                  b == z3.If(z3.UGE(a + c, DEPTH), a + c - DEPTH, a + c))
    inv = [ring(cnt, rp, wp)]

    def build(ob):
        m = Fifo(DEPTH, W)
        m.mem = SymList([r.now(f"mem[{i}]") for i in range(DEPTH)])
        m.wptr, m.rptr, m.cnt, m.overrun = S(wp), S(rp), S(cnt), S(r.now("overrun"))
        out = dict(empty=m.empty, full=m.full, count=m.cnt, rdata=m.rdata, overrun=m.overrun)
        m.clock(S(r.now("rst")), S(r.now("wr")), S(r.now("wdata")), S(r.now("rd")), S(r.now("clr_overrun")))
        nxt = dict(cnt=m.cnt, overrun=m.overrun, wptr=m.wptr, rptr=m.rptr)
        nxt.update({f"mem[{i}]": m.mem[i] for i in range(DEPTH)})
        extra = [ob.inv("fifo ring", ring(V(m.cnt), V(m.rptr), V(m.wptr)))]
        return out, nxt, extra

    return Ob(f"pe_fifo W={W} DEPTH={DEPTH}", r, build, inv)


# ------------------------------------------------------------------ pe_imem
def imem(IW=32, DEPTH=160):
    r = Rtl("pe_imem", dict(IW=IW, DEPTH=DEPTH, IMPL=0))

    def build(ob):
        m = Imem(DEPTH, IW)
        m.mem = SymList([r.now(f"mem[{i}]") for i in range(DEPTH)])
        m.rdata = S(r.now("rdata"))
        out = dict(rdata=m.rdata)
        m.clock(S(r.now("we")), S(r.now("waddr")), S(r.now("wdata")), S(r.now("raddr")))
        nxt = {f"mem[{i}]": m.mem[i] for i in range(DEPTH)}
        nxt["rdata"] = m.rdata
        return out, nxt, []

    return Ob(f"pe_imem IW={IW} DEPTH={DEPTH} (flops)", r, build)


# ------------------------------------------------------------------ pe_dmem
def dmem(DEPTH=256):
    r = Rtl("pe_dmem", dict(DEPTH=DEPTH))

    def build(ob):
        m = Dmem(DEPTH)
        m.mem = SymList([r.now(f"mem[{i}]") for i in range(DEPTH)])
        g = lambda n: S(r.now(n))  # noqa: E731
        # The host-first mux is in chip.py: the host's access goes first and blocks the controller.
        acc = g("h_acc")
        addr = g("h_addr") if acc else g("c_addr")
        out = dict(rdata=m.rdata(addr), c_gnt=int(not acc))
        if acc:
            m.clock(g("h_we"), g("h_addr"), g("h_wdata"))
        else:
            m.clock(g("c_we"), g("c_addr"), g("c_wdata"))
        return out, {f"mem[{i}]": m.mem[i] for i in range(DEPTH)}, []

    return Ob(f"pe_dmem DEPTH={DEPTH}", r, build)


# ------------------------------------------------------------------ pe_pins
def pins(NUM_PINS=8):
    r = Rtl("pe_pins", dict(NUM_PINS=NUM_PINS))
    cfg = Config(num_pins=NUM_PINS)

    def build(ob):
        m = Pins(cfg)
        m.out_r, m.oe_r, m.owner_err = S(r.now("out_r")), S(r.now("oe_r")), S(r.now("owner_err"))
        m.s1, m.s2 = S(r.now("u_sync.s1")), S(r.now("u_sync.s2"))
        m.f_out, m.f_c0, m.f_c1 = S(r.now("filt_out")), S(r.now("filt_c0")), S(r.now("filt_c1"))
        g = lambda n: S(r.now(n))  # noqa: E731
        od, filt = g("pin_od"), g("pin_filt")
        out = dict(pad_out=m.pad_out(od), pad_oe=m.pad_oe(od), pin_in=m.pin_in(filt), owner_err=m.owner_err)
        m.clock(g("rst"), (g("c_wm"), g("c_wv"), g("c_om"), g("c_ov")), (g("t_wm"), g("t_wv"), g("t_om"), g("t_ov")),
                (g("r_wm"), g("r_wv"), 0, 0), g("tx_busy"), g("rx_busy"), g("pin_owner"), g("clr_err"), g("pad_in"),
                filt)
        nxt = {"out_r": m.out_r, "oe_r": m.oe_r, "owner_err": m.owner_err, "u_sync.s1": m.s1, "u_sync.s2": m.s2,
               "filt_out": m.f_out, "filt_c0": m.f_c0, "filt_c1": m.f_c1}
        return out, nxt, []

    return Ob(f"pe_pins NUM_PINS={NUM_PINS}", r, build)


# ------------------------------------------------------------------ pe_vote
VOTE_FIELDS = {"running": "running", "done": "done", "bit_out": "bit", "sel": "sel", "n_l": "n_l",
               "k_l": "k_l", "s_l": "s_l", "scnt": "scnt", "idx": "idx", "ones": "ones"}


def vote(NUM_PINS=8, N_W=4, S_W=16):
    r = Rtl("pe_vote", dict(NUM_PINS=NUM_PINS, N_W=N_W, S_W=S_W))
    cfg = Config(num_pins=NUM_PINS, vote_n_w=N_W, vote_s_w=S_W)

    def build(ob):
        v = Vote(cfg)
        for rn, mn in VOTE_FIELDS.items():
            setattr(v, mn, S(r.now(rn)))
        out = dict(busy=v.busy, done=v.done, bit_out=v.bit)
        g = lambda n: S(r.now(n))  # noqa: E731
        v.clock(g("rst"), g("start"), g("pin_sel"), g("pin_in"), g("n_cfg"), g("k_cfg"), g("s_cfg"))
        return out, {rn: getattr(v, mn) for rn, mn in VOTE_FIELDS.items()}, []

    return Ob(f"pe_vote NUM_PINS={NUM_PINS} N_W={N_W} S_W={S_W}", r, build)


# ------------------------------------------------------------------- pe_crc
def crc():
    r = Rtl("pe_crc")

    def build(ob):
        m = Crc()
        m.src, m.app, m.c_skip, m.poly = [S(r.now(n)) for n in ("c_src", "c_app", "c_skip", "c_poly")]
        m.crc, m.skip, m.sent = [S(r.now(n)) for n in ("crc", "skip", "sent")]
        g = lambda n: S(r.now(n))  # noqa: E731
        out = dict(app_ok=m.app_ok, value=m.value, rd_data=m.read(g("rd_idx")))
        m.clock(g("rst"), g("cfg_we"), g("cfg_idx"), g("cfg_data"), g("tx_v"), g("tx_bit"), g("rx_v"), g("rx_bit"),
                g("sent_set"))
        nxt = dict(c_src=m.src, c_app=m.app, c_skip=m.c_skip, c_poly=m.poly, crc=m.crc, skip=m.skip, sent=m.sent)
        return out, nxt, []

    return Ob("pe_crc", r, build)


# --------------------------------------------------------------- pe_host_if
HOST_FIELDS = {"u_sync_stb.s1": "s1", "u_sync_stb.s2": "s2", "stb_q": "stb_q", "h_ack": "ack",
               "h_dout": "dout", "r_addr": "addr", "wbuf": "wbuf", "wphase": "wphase", "rlow": "rlow",
               "rphase": "rphase"}


def host_if():
    r = Rtl("pe_host_if")

    def build(ob):
        h = HostIf()
        for rn, mn in HOST_FIELDS.items():
            setattr(h, mn, S(r.now(rn)))
        din, op = S(r.now("h_din")), S(r.now("h_op"))
        c = h.comb(din, op)
        out = {k: c[k] for k in ("r_addr", "r_wdata", "r_we", "r_re")}
        out.update(h_dout=h.dout, h_ack=h.ack)
        h.clock(S(r.now("rst")), din, op, S(r.now("h_stb")), S(r.now("r_rdata")))
        extra = [("out h_irq", r.now("h_irq") != r.now("irq"))]
        return out, {rn: getattr(h, mn) for rn, mn in HOST_FIELDS.items()}, extra

    return Ob("pe_host_if", r, build)


# ------------------------------------------------------------------ pe_regs
def regs(IMEM_DEPTH=160, NUM_PINS=8):
    r = Rtl("pe_regs", dict(IMEM_DEPTH=IMEM_DEPTH, NUM_PINS=NUM_PINS, FIFO_CW=4))
    cfg = Config(imem_depth=IMEM_DEPTH, num_pins=NUM_PINS)

    def build(ob):
        g = lambda n: S(r.now(n))  # noqa: E731
        m = Regs(cfg)
        m.run, m.start_pc, m.owner, m.od = g("run"), g("start_pc"), g("pin_owner"), g("pin_od")
        m.filt = g("pin_filt")
        m.imaddr, m.stage, m.irq_en, m.imem_err = g("imaddr"), g("stage"), g("irq_en"), g("imem_err")
        m.dm_addr = g("dm_addr")
        # st2[2] is always 0 and never read: rx_overrun takes its place in STATUS2
        m.st2 = S(r.now("st2") & 0x1B)
        h = dict(r_addr=g("r_addr"), r_wdata=g("r_wdata"), r_we=g("r_we"), r_re=g("r_re"))
        cs = lambda k: S(r.now("cstat_in", k, 1))  # noqa: E731
        st = dict(txf_empty=g("txf_empty"), txf_full=g("txf_full"), txf_count=g("txf_count"),
                  txf_overrun=g("txf_overrun"), rxf_empty=g("rxf_empty"), rxf_full=g("rxf_full"),
                  rxf_count=g("rxf_count"), rxf_overrun=g("rxf_overrun"), rxf_rdata=g("rxf_rdata"),
                  owner_err=g("owner_err"), rx_overrun=g("rx_overrun"), pin_in=g("pin_in"), pc=g("pc"),
                  blocked=cs(1), z=cs(2), c=cs(3), to=cs(4), etx=cs(5), erx=cs(6), db=cs(7), dm_rdata=g("dm_rdata"))
        rc = m.comb(h)
        out = dict(r_rdata=m.read(h["r_addr"], st), irq=m.irq(st), im_we=rc["im_we"], im_waddr=rc["im_waddr"],
                   im_wdata=rc["im_wdata"], txf_wr=rc["txf_wr"], txf_wdata=rc["txf_wdata"], rxf_rd=rc["rxf_rd"],
                   txf_clr_overrun=rc["txf_clr"], rxf_clr_overrun=rc["rxf_clr"], owner_clr_err=rc["owner_clr"],
                   restart=rc["restart"], db_set=rc["db_set"], rx_ovr_clr=rc["rx_ovr_clr"], dm_acc=rc["dm_acc"],
                   dm_we=rc["dm_we"], dm_addr=m.dm_addr, dm_wdata=rc["dm_wdata"], run=m.run, start_pc=m.start_pc,
                   pin_owner=m.owner, pin_od=m.od, pin_filt=m.filt)
        m.clock(g("rst"), h, rc, g("ill_err"), g("stk_err"), g("tx_underrun"), g("ctrl_irq"))
        nxt = dict(run=m.run, start_pc=m.start_pc, pin_owner=m.owner, pin_od=m.od, pin_filt=m.filt, imaddr=m.imaddr, stage=m.stage,
                   irq_en=m.irq_en, imem_err=m.imem_err, st2=m.st2, dm_addr=m.dm_addr)
        return out, nxt, []

    return Ob(f"pe_regs IMEM_DEPTH={IMEM_DEPTH} NUM_PINS={NUM_PINS}", r, build)


# ---------------------------------------------------------------- pe_decode
def decode_ob():
    r = Rtl("pe_decode", dict(AW=8))

    def build(ob):
        d = decode(S(r.now("ir")))
        return dict(opcode=d["op"], dly=d["dly"], fa=d["a"], fi=d["i"], fb=d["b"], imm=d["imm"]), {}, []

    return Ob("pe_decode", r, build)


# ------------------------------------------------------------------ pe_ctrl
CTRL_OPS = list(range(5)) + list(range(6, 35))  # every legal opcode; None = the illegal ones


def ctrl(op, IMEM_DEPTH=160, NUM_PINS=8, STACK_DEPTH=4):
    """One obligation per opcode. op=None covers the illegal opcodes (5 and above 34)."""
    r = Rtl("pe_ctrl", dict(IMEM_DEPTH=IMEM_DEPTH, NUM_PINS=NUM_PINS, STACK_DEPTH=STACK_DEPTH))
    cfg = Config(imem_depth=IMEM_DEPTH, num_pins=NUM_PINS, stack_depth=STACK_DEPTH)
    code = r.now("im_rdata", 26, 6)
    sp = r.now("sp")
    asm = [code == op if op is not None else z3.Or(code == 5, z3.UGT(code, 34)), z3.ULE(sp, STACK_DEPTH)]

    def build(ob):
        g = lambda n: S(r.now(n))  # noqa: E731
        m = Ctrl(cfg)
        m.slot, m.pc, m.z, m.c, m.to, m.db = g("slot"), g("pc"), g("z"), g("c"), g("to"), g("db")
        m.sp, m.stall, m.blocked = g("sp"), g("stall"), g("blocked")
        m.r = SymList([r.now(f"r[{i}]") for i in range(8)])
        m.stack = SymList([r.now(f"stack[{i}]") for i in range(STACK_DEPTH)])
        s = dict(pin_in=g("pin_in"), tx_busy=g("tx_busy"), rx_busy=g("rx_busy"), tx_need=g("tx_need"),
                 rx_need=g("rx_need"), etx=g("etx_flag"), erx=g("erx_flag"), ferr=g("ferr"), rxrdy=g("rxrdy"),
                 txf_empty=g("txf_empty"), txf_full=g("txf_full"), txf_rdata=g("txf_rdata"),
                 rxf_empty=g("rxf_empty"), rxf_full=g("rxf_full"), rxf_rdata=g("rxf_rdata"),
                 txf_rd_gnt=g("txf_rd_gnt"), txf_wr_gnt=g("txf_wr_gnt"), rxf_rd_gnt=g("rxf_rd_gnt"),
                 rxf_wr_gnt=g("rxf_wr_gnt"), dm_gnt=g("dm_gnt"))
        read_args = []

        def rd(unit, idx):
            read_args.append(("rd_unit", unit))
            read_args.append(("rd_idx", idx))
            return g("rd_data")

        def dm(addr):
            read_args.append(("dm_addr", addr))
            return g("dm_rdata")

        out = dict(im_raddr=m.pc, pc=m.pc, z_o=m.z, c_o=m.c, to_o=m.to, db_o=m.db, blocked=m.blocked)
        e = m.comb(g("run"), g("im_rdata"), s, rd, dm)
        for k in ("txf_rd", "rxf_rd", "txf_wr", "rxf_wr", "dm_we", "cfg_we", "rd_strobe", "start_mask",
                  "stop_mask", "det_clr", "det_arm", "det_disarm", "host_irq"):
            out[k] = e[k]
        out.update(ill_err=e["ill"], stk_err=e["stk"], pw_mask=e["wm"], po_mask=e["om"])
        fx = e["txf_wr"] | e["rxf_wr"]
        out["f_wdata"] = (e["fdata"], fx)
        out["dm_wdata"] = (e["dm_wdata"], e["dm_we"])
        out["dm_addr"] = (e["dm_addr"], e["dm_we"])
        for k in ("cfg_unit", "cfg_idx", "cfg_data"):
            out[k] = (e[k], e["cfg_we"])
        out["start_imm"] = (e["start_imm"], e["start_mask"])
        out["start_data"] = (e["start_data"], e["start_mask"])
        out["det_sel"] = (e["det_sel"], e["det_clr"] | e["det_arm"] | e["det_disarm"])
        # the values only matter on the masked pins, and pe_pins is proved to use them that way
        extra = [("out pw_val", V(e["wv"]) & V(e["wm"]) != r.now("pw_val") & r.now("pw_mask")),
                 ("out po_val", V(e["ov"]) & V(e["om"]) != r.now("po_val") & r.now("po_mask"))]
        # what the model read from data memory or a unit: the same address as the RTL drives
        extra += [(f"read {k}", V(v) != r.now(k)) for k, v in read_args]
        m.clock(g("rst"), g("restart"), g("start_pc"), g("db_set"), e)
        nxt = dict(slot=m.slot, pc=m.pc, z=m.z, c=m.c, to=m.to, db=m.db, sp=m.sp, stall=m.stall, blocked=m.blocked)
        nxt.update({f"r[{i}]": m.r[i] for i in range(8)})
        nxt.update({f"stack[{i}]": m.stack[i] for i in range(STACK_DEPTH)})
        extra.append(ob.inv("sp <= depth", z3.And(V(m.sp) >= 0, V(m.sp) <= STACK_DEPTH)))
        return out, nxt, extra

    return Ob(f"pe_ctrl op={'illegal' if op is None else op}", r, build, asm)


# ------------------------------------------------------------------- pe_det
def det(KIND, NUM_PINS=8):
    r = Rtl("pe_det", dict(KIND=KIND, NUM_PINS=NUM_PINS))
    cfg = Config(num_pins=NUM_PINS)
    apend, acnt, ferr = r.now("apend"), r.now("acnt"), r.now("ferr")
    # A running after-step always has a count left. ETX never has a check, so never an error.
    asm = [z3.Or(apend == 0, acnt != 0)]
    if KIND == 0:
        asm.append(ferr == 0)

    def build(ob):
        g = lambda n: S(r.now(n))  # noqa: E731
        m = Det(KIND, cfg)
        m.cond, m.dur, m.aft, m.adly = g("c_cond"), g("c_dur"), g("c_after"), ob.opt("c_adly")
        m.armed, m.pend, m.mine, m.prev, m.hold = g("armed"), g("pend"), g("mine"), g("prev"), g("hold")
        m.cnt, m.flag, m.ferr, m.apend = g("cnt"), g("flag"), g("ferr"), g("apend")
        m.hlvl = ob.loose("hlvl", r.now("hold") != 0)       # only read while hold is set
        m.acnt = ob.loose("acnt", apend != 0)               # only read while apend is set
        pin_in = g("pin_in")
        act_pin = g("act_pin")
        t = m.trigger(pin_in)
        a = m.after(g("unit_done"), g("unit_chain"), (1 << act_pin) & m.nm, g("act2_mask"), (pin_in >> act_pin) & 1)
        out = dict(start_req=t["start_req"], stop_req=t["stop_req"], after_end=a["after_end"], after_pend=m.apend,
                   flag=m.flag, ferr=m.ferr, pw_mask=a["pw_mask"], pw_val=a["pw_val"], po_mask=a["po_mask"],
                   po_val=a["po_val"], rd_data=m.stat)
        m.clock(g("rst"), g("restart"), t, a, g("unit_busy"), g("unit_done"), g("unit_stop"), g("start_ack"),
                g("clr"), g("arm"), g("disarm"), g("cfg_we"), g("cfg_idx"), g("cfg_data"))
        nxt = dict(c_cond=m.cond, c_dur=m.dur, c_after=m.aft, c_adly=m.adly, armed=m.armed, pend=m.pend, mine=m.mine,
                   prev=m.prev, hold=m.hold, cnt=m.cnt, flag=m.flag, ferr=m.ferr, apend=m.apend,
                   hlvl=(m.hlvl, m.hold), acnt=(m.acnt, m.apend))
        nxt = {k: v for k, v in nxt.items() if r.has(k)}
        extra = [ob.inv("apend -> acnt != 0", z3.Or(V(m.apend) == 0, V(m.acnt) != 0))]
        if KIND == 0:
            extra.append(ob.inv("ETX ferr = 0", V(m.ferr) == 0))
        return out, nxt, extra

    return Ob(f"pe_det KIND={KIND}", r, build, asm)


# ------------------------------------------------------------------- pe_txu
TX_PINS = [("c_pins", 0, 5, 0), ("c_pins", 8, 5, 8)]
TX_FRAME = [("c_msb", 0, 1, 0), ("c_inv", 0, 1, 2), ("c_nbits", 0, 5, 4), ("c_chain", 0, 1, 9)]
TX_CLK = [("c_cpol", 0, 1, 0), ("c_cpha", 0, 1, 1), ("c_stretch", 0, 1, 2)]
TX_TICK = [("c_ext", 0, 1, 0), ("c_epin", 0, 4, 4), ("c_eedge", 0, 1, 8), ("c_lead", 0, 1, 9)]
TX_LINE = [("c_nrzi", 0, 1, 0), ("c_eop", 0, 1, 1), ("c_diff", 0, 1, 2), ("c_d2pin", 0, 4, 4), ("c_stuffn", 0, 4, 8)]
TX_STATE = ["busy", "sh", "idx", "lf", "sw", "swc", "sp", "crc_pkt", "ones", "eph", "eprev", "lvl"]


def txu(NUM_PINS=8):
    r = Rtl("pe_txu", dict(NUM_PINS=NUM_PINS))
    cfg = Config(num_pins=NUM_PINS)
    busy, start = r.now("busy"), r.now("start")
    # A start only comes while TX is idle: formal/pe_ctrl (start_mask) and the ETX start in the top level.
    asm = [z3.Or(start == 0, busy == 0)]

    def build(ob):
        g = lambda n: S(r.now(n))  # noqa: E731
        m = Txu(cfg)
        m.pins, m.frame = fields_now(r, TX_PINS), fields_now(r, TX_FRAME)
        m.clk, m.tick, m.line = fields_now(r, TX_CLK), fields_now(r, TX_TICK), fields_now(r, TX_LINE)
        m.bit = g("c_bit")
        for k in TX_STATE:
            setattr(m, k, g(k))
        m.o = ob.loose("o", busy != 0)                      # the bit timer is set by a start
        t = m.comb(g("start"), g("start_data"), g("stop"), g("next_ok"), g("next_crc"), g("next_data"), g("pin_in"))
        out = dict(busy=m.busy, done=t["done"], chain=t["chain"], stick=t["stick"], dtick=t["dtick"],
                   bit_v=t["bit_v"], bit_o=(t["bit_o"], t["bit_v"]), pw_mask=t["pw_mask"], den=m.den, dpin=m.dpin,
                   d2_mask=m.d2_mask, rd_data=m.read(g("rd_idx")))
        extra = [("out pw_val", V(t["pw_val"]) & V(t["pw_mask"]) != r.now("pw_val") & r.now("pw_mask"))]
        m.clock(g("rst"), g("cfg_we"), g("cfg_idx"), g("cfg_data"), t)
        nxt = {k: getattr(m, k) for k in TX_STATE}
        nxt["c_bit"] = m.bit
        nxt["o"] = (m.o, m.busy)
        for val, fields in ((m.pins, TX_PINS), (m.frame, TX_FRAME), (m.clk, TX_CLK), (m.tick, TX_TICK),
                            (m.line, TX_LINE)):
            extra += fields_next(r, val, fields)
        return out, nxt, extra

    return Ob("pe_txu", r, build, asm)


# ------------------------------------------------------------------- pe_rxu
RX_PINS = [("c_pins", 0, 14, 0)]
RX_FRAME = [("c_msb", 0, 1, 0), ("c_hold", 0, 1, 1), ("c_inv", 0, 1, 2), ("c_chain", 0, 1, 3), ("c_nbits", 0, 5, 4)]
RX_VCFG = [("c_n", 0, 4, 0), ("c_k", 0, 4, 4)]
RX_TICK = [("c_src", 0, 2, 0), ("c_epin", 0, 4, 4), ("c_eedge", 0, 1, 8), ("c_sdly", 0, 4, 12)]
RX_ODLY = [("c_odly", 0, 4, 0), ("c_nrzi", 0, 1, 4), ("c_hunt", 0, 1, 5), ("c_stuffn", 0, 4, 8)]
RX_REGS = {"busy": "busy", "sh": "word", "od": "od", "rbyte": "rbyte", "data_q": "data_q", "nbits": "nb",
           "oidx": "oidx", "wcnt": "wcnt", "wins": "wins", "eprev": "eprev", "iprev": "iprev", "tq": "tq",
           "oq": "oq", "rxrdy": "rxrdy", "overrun": "overrun", "lp": "sprev", "hunting": "hunting", "hz": "hz",
           "rones": "rones", "serr": "serr"}
RX_VOTE = {f"u_vote.{k}": v for k, v in VOTE_FIELDS.items()}


def rxu(NUM_PINS=8):
    r = Rtl("pe_rxu", dict(NUM_PINS=NUM_PINS))
    cfg = Config(num_pins=NUM_PINS)
    busy, start, hz = r.now("busy"), r.now("start"), r.now("hz")
    # A start only comes while RX is idle (as for TX). The HUNT zero counter saturates at 2.
    asm = [z3.Or(start == 0, busy == 0), z3.ULE(hz, 2)]

    def build(ob):
        g = lambda n: S(r.now(n))  # noqa: E731
        m = Rxu(cfg)
        m.pins, m.frame = fields_now(r, RX_PINS), fields_now(r, RX_FRAME)
        m.vcfg, m.tick, m.odly = fields_now(r, RX_VCFG), fields_now(r, RX_TICK), fields_now(r, RX_ODLY)
        m.bit, m.step, m.first = g("c_bit"), g("c_step"), g("c_first")
        for rn, mn in RX_REGS.items():
            setattr(m, mn, g(rn))
        for rn, mn in RX_VOTE.items():
            setattr(m.vote, mn, g(rn))
        c = m.comb(g("start"), g("start_data"), g("stop"), g("pin_in"), g("tx_stick"), g("tx_dtick"))
        out = dict(busy=m.busy, done=c["done"], push=c["push"], push_data=(c["push_data"], c["push"]), rxrdy=m.rxrdy,
                   overrun=m.overrun, pw_mask=c["wm"], need=m.need, chain_on=m.chain_on, ipin=m.ipin,
                   bit_v=c["bit_v"], bit_o=(c["bit_o"], c["bit_v"]), rd_data=m.read(g("rd_idx"), g("ferr")))
        extra = [("out pw_val", V(c["wv"]) & V(c["wm"]) != r.now("pw_val") & r.now("pw_mask"))]
        rd_now = g("rd_strobe") & (g("rd_idx") == 0)
        m.clock(g("rst"), g("restart"), g("cfg_we"), g("cfg_idx"), g("cfg_data"), c, g("after_end"), rd_now,
                g("clr_ovr"))
        nxt = {rn: getattr(m, mn) for rn, mn in RX_REGS.items()}
        nxt.update({rn: getattr(m.vote, mn) for rn, mn in RX_VOTE.items()})
        nxt.update(c_bit=m.bit, c_step=m.step, c_first=m.first)
        for val, fields in ((m.pins, RX_PINS), (m.frame, RX_FRAME), (m.vcfg, RX_VCFG), (m.tick, RX_TICK),
                            (m.odly, RX_ODLY)):
            extra += fields_next(r, val, fields)
        extra.append(ob.inv("hz <= 2", z3.And(V(m.hz) >= 0, V(m.hz) <= 2)))
        return out, nxt, extra

    return Ob("pe_rxu", r, build, asm)
