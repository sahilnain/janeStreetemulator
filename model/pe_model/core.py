# SPDX-License-Identifier: Apache-2.0
"""
core: cycle-accurate model of pe_core (slot scheduler + contexts + vote units).

What this file does
-------------------
Fixed time-slot round-robin over NUM_SLOTS = max(2, NUM_CTX) slots. In each
cycle:
  * fetch: im_raddr = PC of the context owning the NEXT slot (the IMEM model
    returns it on the next cycle, like the registered RTL read);
  * execute: if the context owning THIS slot is ready, it issues the word in
    the IMEM read register: decode() + execute() from isa.py give its next
    state and side effects.
Contexts and vote units are clocked with exactly the RTL priority
(reset > restart > writeback/stall countdown, then vote result).
"""

from .blocks import Context, Vote
from .config import Config
from .isa import decode, execute


class ModelError(Exception):
    pass


class Core:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.slot = 0
        self.ctxs = [Context(cfg) for _ in range(cfg.num_ctx)]
        self.votes = [Vote(cfg) if (cfg.vote_ctx_mask >> g) & 1 else None for g in range(cfg.num_ctx)]

    def comb(self, run, shift_msb, im_rdata, pin_in, txf_rdata, txf_empty, rxf_full):
        cfg = self.cfg
        slot_nx = 0 if self.slot == cfg.num_slots - 1 else self.slot + 1
        slot_valid = self.slot < cfg.num_ctx
        cur = self.slot if slot_valid else 0
        nxt = slot_nx if slot_nx < cfg.num_ctx else 0
        vbusy = [v.busy if v else 0 for v in self.votes]
        ready = [c.ready((run >> g) & 1, vbusy[g]) for g, c in enumerate(self.ctxs)]
        issue = int(slot_valid and ready[cur])
        out = dict(slot_nx=slot_nx, cur=cur, issue=issue, im_raddr=self.ctxs[nxt].pc, ready=ready,
                   vote_busy=vbusy, pin_wr_en=0, pin_wr_mask=0, pin_wr_val=0, pin_oe_en=0,
                   pin_oe_mask=0, pin_oe_val=0, txf_rd=0, rxf_wr=0, rxf_wdata=0, x=None)
        if issue:
            if im_rdata is None:
                raise ModelError(f"context {cur} executed an unwritten IMEM word (pc={self.ctxs[cur].pc})")
            f = decode(im_rdata, cfg.iw, cfg.aw)
            s = self.ctxs[cur].state()
            s["shift_msb"] = (shift_msb >> cur) & 1
            s["vote_avail"] = int(self.votes[cur] is not None)
            x = execute(cfg, f, s, dict(pin_in=pin_in, txf_rdata=txf_rdata, txf_empty=txf_empty,
                                        rxf_full=rxf_full))
            for k in ("pin_wr_en", "pin_wr_mask", "pin_wr_val", "pin_oe_en", "pin_oe_mask",
                      "pin_oe_val", "txf_rd", "rxf_wr", "rxf_wdata"):
                out[k] = x[k]
            out["x"] = x
        return out

    def clock(self, rst, restart, start_pc, shift_msb, vote_n, vote_k, vote_s, cc, pin_in):
        self.slot = 0 if rst else cc["slot_nx"]
        for g, ctx in enumerate(self.ctxs):
            we = cc["issue"] and cc["cur"] == g
            v = self.votes[g]
            vdone, vbit = (v.done, v.bit) if v else (0, 0)
            ctx.clock(rst, (restart >> g) & 1, start_pc[g], (shift_msb >> g) & 1, we,
                      cc["x"] if we else None, vdone, vbit)
            if v:
                x = cc["x"] if we else None
                v.clock(rst, bool(x and x["vote_start"]), x["vote_pin"] if x else 0, pin_in,
                        vote_n[g], vote_k[g], vote_s[g])
