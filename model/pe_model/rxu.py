# SPDX-License-Identifier: Apache-2.0
"""
rxu: the RX unit, written from docs/spec/blocks.md Section 4.

What this file does
-------------------
comb() finds the raw sample of this cycle from the selected source (vote, TX tick,
external edge or the resynchronised timer), decodes it as Section 4.5 lists, and
gives the pin write, done and push. clock() updates the state.
"""

from .blocks import Vote
from .txu import field, nbits_of


class Rxu:
    def __init__(self, cfg):
        self.cfg = cfg
        self.nm = (1 << cfg.num_pins) - 1
        self.vote = Vote(cfg)
        self.reset()

    def reset(self):
        self.pins = self.frame = self.first = self.tick = self.odly = 0
        self.bit, self.vcfg, self.step = 1, 1, 1      # reset values: B = 1, N = 1, STEP = 1
        self.busy = self.word = self.nb = self.rbyte = self.data_q = self.rxrdy = self.overrun = self.serr = 0
        self.wins = self.wcnt = self.od = self.oidx = 0
        self.tq = self.oq = self.eprev = self.iprev = self.sprev = 0
        self.hunting = self.hz = self.rones = 0

    @property
    def need(self):
        return (self.pins >> 12) & 1

    @property
    def ipin(self):
        return self.pins & 15

    @property
    def chain_on(self):
        return (self.frame >> 3) & 1 and (self.tick & 3) != 0 and not self.need

    def cfg_write(self, idx, data):
        if idx == 0:
            self.pins = data & 0x3FFF
        elif idx == 1:
            self.bit = data
        elif idx == 2:
            self.frame = data & 0x01FF
        elif idx == 3:
            self.vcfg = data & 0xFF
        elif idx == 4:
            self.step = data
        elif idx == 5:
            self.first = data
        elif idx == 6:
            self.tick = data & 0xF1F3
        elif idx == 7:
            self.odly = data & 0x0F3F

    def read(self, idx, ferr):
        if idx == 0:
            return self.data_q
        if idx == 1:
            return (self.busy | self.rxrdy << 1 | ferr << 2 | self.overrun << 3 | self.serr << 4 | self.nb << 8)
        return 0

    def comb(self, start, start_data, stop, pin_in, stick, dtick):
        ipin = self.ipin
        ibit = (pin_in >> ipin) & 1
        i2 = (pin_in >> field(self.pins, 7, 4)) & 1
        src = self.tick & 3
        epin = field(self.tick, 7, 4)
        eedge = (self.tick >> 8) & 1
        ein = (pin_in >> epin) & 1
        sdly = self.tick >> 12
        msb, hold, inv = self.frame & 1, (self.frame >> 1) & 1, (self.frame >> 2) & 1
        n = nbits_of(self.frame)
        B = self.bit or 1
        live = self.busy
        diff = (self.pins >> 13) & 1
        se0 = diff and not ibit and not i2
        odly, nrzi, huntcfg, stuffn = self.odly & 15, (self.odly >> 4) & 1, (self.odly >> 5) & 1, field(self.odly, 11, 8)

        # sample and drive events
        edge_i = ibit != self.iprev
        sel_edge = ein != self.eprev and ein == (eedge ^ 1)
        opp_edge = ein != self.eprev and ein == eedge
        raw_t = live and (stick if src == 1 else (sel_edge if src == 2 else 0))
        tq_n = ((self.tq << 1) | int(bool(raw_t))) & 0xFFFF
        take_t = bool(raw_t) if sdly == 0 else bool(live and (self.tq >> (sdly - 1)) & 1)
        raw_o = live and (dtick if src == 1 else (opp_edge if src == 2 else 0))
        oq_n = ((self.oq << 1) | int(bool(raw_o))) & 0xFFFF
        drv_d = bool(raw_o) if odly == 0 else bool(live and (self.oq >> (odly - 1)) & 1)

        win_now = src == 0 and ((start and self.first == 0) or (live and self.wins < n and self.wcnt == 0))
        take = False
        sample = 0
        if live:
            if src == 0:
                take = bool(self.vote.done)
                sample = self.vote.bit
            elif src == 3:
                take = self.wcnt == 0 and not edge_i
                sample = ibit
            else:
                take = take_t
                sample = ibit
        if se0:
            take = False

        # decoding, in the order of Section 4.5
        v = sample ^ inv
        data = False
        d_bit = 0
        hunting, hz, rones, serr = self.hunting, self.hz, self.rones, self.serr
        sprev = self.sprev
        if take:
            d = int(v == sprev) if nrzi else v
            sprev = v
            if hunting:
                if d and hz == 2:
                    hunting, rones = 0, 1
                hz = 0 if d else min(hz + 1, 2)
            elif stuffn and rones == stuffn:
                if d:
                    serr = 1
                rones = 0
            else:
                data, d_bit = True, d
                rones = (rones + 1) & 15 if d else 0

        done = data and self.nb == n - 1
        word = self.word
        if data:
            word = ((word << 1) | d_bit) & 0xFFFF if msb else (word | (d_bit << self.nb))

        # drive
        oen = self.need
        src_own = src == 0
        drv_ev = live and self.oidx != n - 1 and ((win_now and self.wins != 0) if src_own else drv_d)
        wr_o = oen and (start or drv_ev)
        wm = wv = 0
        if wr_o:
            obit = 0 if start else self.oidx + 1
            dsrc = start_data if start else self.od
            b = (dsrc >> (n - 1 - obit if msb else obit)) & 1
            wm = (1 << field(self.pins, 11, 8)) & self.nm
            wv = wm if b ^ inv else 0

        # A stop in cycle D is too late: the word is complete (the FIFO grant to the controller
        # depends on push, so push cannot depend on the controller's ustop).
        push = int(done and not hold)
        return dict(wm=wm, wv=wv, done=int(done), push=push, push_data=word & 0xFF, bit_v=int(data),
                    bit_o=d_bit, _=dict(start=start, start_data=start_data, stop=stop, ibit=ibit, ein=ein, tq=tq_n,
                                        oq=oq_n, win_now=win_now, take=take, data=data, word=word, hunting=hunting,
                                        hz=hz, rones=rones, serr=serr, sprev=sprev, done=done, hold=hold,
                                        edge_i=edge_i, drv_ev=drv_ev, n=n, B=B, src=src, huntcfg=huntcfg,
                                        inv=inv, pin_in=pin_in))

    def clock(self, rst, restart, cfg_we, cfg_idx, cfg_data, r, after_end, rd_data_now, clr_ovr):
        if rst:
            self.reset()
            self.vote = Vote(self.cfg)
            return
        x = r["_"]
        start, stop, n, B, src = x["start"], x["stop"], x["n"], x["B"], x["src"]
        hold = x["hold"]
        v = self.vcfg
        self.vote.clock(bool(stop or restart), x["win_now"], self.ipin, x["pin_in"], (v & 15) or 1, v >> 4, self.step)
        self.eprev, self.iprev = x["ein"], x["ibit"]
        self.tq, self.oq = (0, 0) if (stop or restart or start) else (x["tq"], x["oq"])

        # results
        if x["done"]:
            self.rbyte = x["word"]
        if restart:
            self.rxrdy = self.overrun = 0
        else:
            if after_end and hold and self.rxrdy and not rd_data_now:
                self.overrun = 1                    # a set wins over a clear in the same cycle
            elif clr_ovr:
                self.overrun = 0
            if after_end and hold:
                self.data_q = x["word"] if x["done"] else self.rbyte
                self.rxrdy = 1
            elif rd_data_now:
                self.rxrdy = 0

        if start:
            self.busy = 1
            self.word = self.nb = self.oidx = self.serr = 0
            self.od = x["start_data"]
            if self.first == 0:
                self.wins, self.wcnt = 1, B - 1
            else:
                self.wins, self.wcnt = 0, self.first - 1
            self.hunting, self.hz, self.rones = x["huntcfg"], 0, 0
            self.sprev = x["ibit"] ^ x["inv"]
        elif self.busy:
            if stop or restart or (x["done"] and not self.chain_on):
                self.busy = 0
            self.hunting, self.hz, self.rones, self.serr = x["hunting"], x["hz"], x["rones"], x["serr"]
            self.sprev = x["sprev"]
            if x["data"]:
                restart_word = x["done"] and self.chain_on
                self.word = 0 if restart_word else x["word"]
                self.nb = 0 if restart_word else self.nb + 1
            if x["drv_ev"]:
                self.oidx = (self.oidx + 1) & 15
            if src == 0:
                if self.wcnt == 0:
                    if self.wins < n:
                        self.wins += 1
                        self.wcnt = B - 1
                else:
                    self.wcnt -= 1
            elif src == 3:
                half = B >> 1
                if x["edge_i"]:
                    self.wcnt = max(half - 1, 0)
                elif self.wcnt == 0:
                    self.wcnt = B - 1
                else:
                    self.wcnt -= 1
        if cfg_we:
            self.cfg_write(cfg_idx, cfg_data)
