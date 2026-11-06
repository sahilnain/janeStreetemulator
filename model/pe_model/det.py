# SPDX-License-Identifier: Apache-2.0
"""
det: the event detector (ETX and ERX), written from docs/spec/blocks.md Section 5.

What this file does
-------------------
trigger() gives the fire decision, which depends only on state and pin_in, so the
chip model can use it before the controller. after() gives the after-packet step.
clock() updates the state.
"""

from .blocks import mux


class Det:
    def __init__(self, kind, cfg):
        self.kind = kind                     # 0 ETX, 1 ERX
        self.nm = (1 << cfg.num_pins) - 1
        self.reset()

    def reset(self):
        self.cond = self.dur = self.aft = self.adly = 0
        self.armed = self.pend = self.mine = self.prev = self.hold = self.hlvl = 0
        self.cnt = self.flag = self.ferr = self.apend = self.acnt = 0

    @property
    def stat(self):
        return (self.armed | self.flag << 1 | self.pend << 2 | (self.apend | self.mine) << 3
                | (self.ferr << 4 if self.kind else 0))

    def trigger(self, pin_in):
        c = self.cond
        cur = (pin_in >> (c & 15)) & 1
        mode = (c >> 4) & 3
        qual = (((c >> 13) & 1) ^ 1) | (((pin_in >> ((c >> 9) & 15)) & 1) == (c >> 14) & 1)
        is_lvl = mode == 0
        dir_ok = (mode == 3) | ((mode == 1) & cur) | ((mode == 2) & (cur ^ 1))
        edge_ev = (is_lvl ^ 1) & (cur != self.prev) & dir_ok & qual
        # QUAL applies to the edge only
        cont = mux(is_lvl, (cur == (c >> 6) & 1) & qual, edge_ev | (self.hold & (cur == self.hlvl)))
        cnt_nx = mux(edge_ev, 1, self.cnt + 1)
        fire = self.armed & cont & (cnt_nx == mux(self.dur != 0, self.dur, 1))
        return dict(cur=cur, edge_ev=edge_ev, cont=cont, cnt_nx=cnt_nx, fire=fire,
                    start_req=fire & ((c >> 7) & 1), stop_req=fire & ((c >> 15) & 1))

    def after(self, unit_done, unit_chain, act_mask, act2_mask, act_in):
        aft = self.aft & 3
        chk_en = (self.kind == 1) & (self.aft & 1) & (unit_chain ^ 1)
        last = self.apend & (self.acnt == 1)
        chk_now = chk_en & ((unit_done & (self.adly == 0)) | last)
        after_end = (unit_done & ((chk_en & (self.adly != 0)) ^ 1)) | last
        drv = (self.kind == 0) & unit_done & (aft != 0)
        drv_out = drv & (aft != 3)
        om = mux(drv, act_mask | act2_mask, 0)
        return dict(chk_en=chk_en, chk_now=chk_now, after_end=after_end, pw_mask=mux(drv_out, act_mask, 0),
                    pw_val=mux(drv_out & (aft == 2), act_mask, 0), po_mask=om, po_val=mux(drv_out, om, 0),
                    act_in=act_in)

    def clock(self, rst, restart, t, a, unit_busy, unit_done, unit_stop, start_ack, clr, arm, disarm,
              cfg_we, cfg_idx, cfg_data):
        if rst:
            self.reset()
            return
        trig = cfg_we & ((cfg_idx == 0) | (cfg_idx == 1))
        self.prev = t["cur"]
        if restart:
            self.armed = self.pend = self.mine = self.hold = self.cnt = 0
            self.flag = self.ferr = self.apend = self.acnt = 0
        else:
            fire, cont, edge_ev, after_end = t["fire"], t["cont"], t["edge_ev"], a["after_end"]
            active = unit_busy | self.apend
            rearm = after_end & (self.pend | (((self.cond >> 8) & 1) & self.mine))
            idle_pend = self.pend & (active ^ 1) & (unit_done ^ 1)
            # counter
            clear = (self.armed ^ 1) | fire | trig | (cont ^ 1)
            self.cnt = mux(clear, 0, t["cnt_nx"] & 0xFFFF)
            self.hlvl = mux(clear | (edge_ev ^ 1), self.hlvl, t["cur"])
            self.hold = mux(clear, 0, self.hold | edge_ev)
            # flag
            self.flag = mux(fire, 1, mux(clr, 0, self.flag))
            # armed / pending
            self.armed = mux(fire | disarm, 0, mux(rearm | (arm & (active ^ 1)) | idle_pend, 1, self.armed))
            self.pend = mux(disarm | rearm, 0, mux(arm & active, 1, mux(idle_pend, 0, self.pend)))
            # mine
            self.mine = mux(start_ack, 1, mux(after_end | unit_stop, 0, self.mine))
            # after step
            self.ferr = mux(a["chk_now"], a["act_in"] ^ ((self.aft >> 1) & 1), self.ferr)
            load = unit_done & a["chk_en"] & (self.adly != 0)
            last = self.apend & (self.acnt == 1)
            self.acnt = mux(unit_stop, self.acnt, mux(load, self.adly, mux(self.apend, self.acnt - 1, self.acnt)))
            self.apend = mux(unit_stop, 0, mux(load, 1, mux(last, 0, self.apend)))
        if cfg_we:
            if cfg_idx == 0:
                self.cond = cfg_data
            elif cfg_idx == 1:
                self.dur = cfg_data
            elif cfg_idx == 2:
                self.aft = cfg_data & 3
            elif cfg_idx == 3:
                self.adly = cfg_data
