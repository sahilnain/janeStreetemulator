# SPDX-License-Identifier: Apache-2.0
"""
det: the event detector (ETX and ERX), written from docs/spec/blocks.md Section 5.

What this file does
-------------------
trigger() gives the fire decision, which depends only on state and pin_in, so the
chip model can use it before the controller. after() gives the after-packet step.
clock() updates the state.
"""


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
        qual = not (c >> 13) & 1 or ((pin_in >> ((c >> 9) & 15)) & 1) == (c >> 14) & 1
        is_lvl = mode == 0
        edge = cur != self.prev and (mode == 3 or (mode == 1 and cur) or (mode == 2 and not cur))
        edge_ev = bool(edge and qual and not is_lvl)
        if is_lvl:
            cont = cur == (c >> 6) & 1 and qual
        else:
            cont = edge_ev or (self.hold and cur == self.hlvl)   # QUAL applies to the edge only
        cnt_nx = 1 if edge_ev else self.cnt + 1
        fire = bool(self.armed and cont and cnt_nx == (self.dur or 1))
        return dict(cur=cur, edge_ev=edge_ev, cont=cont, cnt_nx=cnt_nx, fire=fire,
                    start_req=int(fire and (c >> 7) & 1), stop_req=int(fire and (c >> 15) & 1))

    def after(self, unit_done, unit_chain, act_mask, act2_mask, act_in):
        chk_en = self.kind == 1 and self.aft & 1 and not unit_chain
        last = self.apend and self.acnt == 1
        chk_now = chk_en and ((unit_done and self.adly == 0) or last)
        after_end = int(bool((unit_done and not (chk_en and self.adly != 0)) or last))
        wm = wv = om = ov = 0
        if self.kind == 0 and unit_done and self.aft & 3:
            om = act_mask | act2_mask
            if self.aft & 3 == 3:
                ov = 0
            else:
                wm = act_mask
                wv = act_mask if self.aft & 3 == 2 else 0
                ov = om
        return dict(chk_en=chk_en, chk_now=chk_now, after_end=after_end, pw_mask=wm, pw_val=wv, po_mask=om,
                    po_val=ov, act_in=act_in)

    def clock(self, rst, restart, t, a, unit_busy, unit_done, unit_stop, start_ack, clr, arm, disarm,
              cfg_we, cfg_idx, cfg_data):
        if rst:
            self.reset()
            return
        trig = cfg_we and cfg_idx in (0, 1)
        self.prev = t["cur"]
        if restart:
            self.armed = self.pend = self.mine = self.hold = self.cnt = 0
            self.flag = self.ferr = self.apend = self.acnt = 0
        else:
            active = unit_busy or self.apend
            rearm = a["after_end"] and (self.pend or ((self.cond >> 8) & 1 and self.mine))
            # counter
            if not self.armed or t["fire"] or trig or not t["cont"]:
                self.cnt = self.hold = 0
            else:
                self.cnt = t["cnt_nx"] & 0xFFFF
                if t["edge_ev"]:
                    self.hold, self.hlvl = 1, t["cur"]
            # flag
            if t["fire"]:
                self.flag = 1
            elif clr:
                self.flag = 0
            # armed / pending
            idle_pend = self.pend and not active and not unit_done
            if t["fire"] or disarm:
                self.armed = 0
            elif rearm or (arm and not active) or idle_pend:
                self.armed = 1
            if disarm or rearm:
                self.pend = 0
            elif arm and active:
                self.pend = 1
            elif idle_pend:
                self.pend = 0
            # mine
            if start_ack:
                self.mine = 1
            elif a["after_end"] or unit_stop:
                self.mine = 0
            # after step
            if a["chk_now"]:
                self.ferr = int(a["act_in"] != (self.aft >> 1) & 1)
            if unit_stop:
                self.apend = 0
            elif unit_done and a["chk_en"] and self.adly != 0:
                self.apend, self.acnt = 1, self.adly
            elif self.apend:
                self.acnt -= 1
                if self.acnt == 0:
                    self.apend = 0
        if cfg_we:
            if cfg_idx == 0:
                self.cond = cfg_data
            elif cfg_idx == 1:
                self.dur = cfg_data
            elif cfg_idx == 2:
                self.aft = cfg_data & 3
            elif cfg_idx == 3:
                self.adly = cfg_data
