# SPDX-License-Identifier: Apache-2.0
"""
txu: the TX unit, written from docs/spec/blocks.md Section 3.

What this file does
-------------------
comb() returns what the unit does in this cycle (pin writes, ticks, done, chain),
clock() updates its state. `o` counts the cycles of the current bit: 1 in the
cycle after the bit was written, B in the cycle the next write is due.
"""


from .blocks import mux


def field(v, hi, lo):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)


def nbits_of(frame):
    n = field(frame, 8, 4)
    return mux(n == 0, 8, mux(n >= 16, 16, n))


class Txu:
    def __init__(self, cfg):
        self.nm = (1 << cfg.num_pins) - 1
        self.reset()

    def reset(self):
        self.pins = self.frame = self.clk = self.tick = self.line = 0
        self.bit = 1
        self.busy = self.sh = self.idx = 0
        self.o = 1
        self.lf = self.sw = self.swc = self.sp = 0
        self.crc_pkt = self.ones = self.eph = self.eprev = 0
        self.lvl = 1

    # ------------------------------------------------------- config fields
    @property
    def den(self):
        return (self.pins >> 4) & 1

    @property
    def dpin(self):
        return self.pins & 15

    @property
    def d2_mask(self):
        return mux(self.diff, self._mask(field(self.line, 7, 4)), 0)

    @property
    def diff(self):
        return ((self.line >> 2) & 1) & self.den

    def _mask(self, pin):
        return (1 << pin) & self.nm

    def read(self, idx):
        return mux(idx == 0, (self.idx << 4) | self.busy, 0)

    def _bit_of(self, word, i, crc):
        # 4-bit position, as in the RTL: after a mid-word change of NBITS it wraps
        last = mux(crc, 15, nbits_of(self.frame) - 1)
        return (word >> mux(crc | (self.frame & 1), (last - i) & 15, i & 15)) & 1

    # --------------------------------------------------------------- comb
    def comb(self, start, start_data, stop, next_ok, next_crc, next_data, pin_in):
        ext = self.tick & 1
        epin = field(self.tick, 7, 4)
        eedge = (self.tick >> 8) & 1
        lead = ((self.tick >> 9) & 1) & ext
        ein = (pin_in >> epin) & 1
        edge_seen = ext & (ein != self.eprev) & (ein == (eedge ^ 1))
        edge_opp = ext & (ein != self.eprev) & (ein == eedge)
        cpin = field(self.pins, 11, 8)
        cen = (self.pins >> 12) & 1
        cpin_in = (pin_in >> cpin) & 1
        cpol, cpha, stretch = self.clk & 1, (self.clk >> 1) & 1, (self.clk >> 2) & 1
        B = mux(self.bit != 0, self.bit, 1)
        H = B >> 1
        last = mux(self.crc_pkt, 15, nbits_of(self.frame) - 1)
        stuffn = field(self.line, 11, 8)
        nrzi, eop = self.line & 1, (self.line >> 1) & 1
        inv = (self.frame >> 2) & 1
        c_chain = (self.frame >> 9) & 1

        vh = mux(cpha, cpol, cpol ^ 1)                      # the SCK level written at offset H
        rel_h = stretch & cen & vh                          # a release: the tick comes later
        go_sck = mux(cpha, cpol ^ 1, cpol)                  # the SCK level written when a word begins
        release = self.sw & (self.swc == 0) & cpin_in
        hold = self.sw & (release ^ 1)
        at_b = self.o == B
        at_h = (H != 0) & (self.o == H)
        stuff_now = (stuffn != 0) & (self.ones == stuffn)

        r = dict(done=0, chain=0, stick=0, dtick=0, bit_v=0, bit_o=0)
        raw = None      # data write
        bit_v = 0
        se0 = j = 0
        sck = None

        if start:
            if not lead:
                raw = self._bit_of(start_data, 0, 0)
                bit_v = 1
            if not ext:
                sck = go_sck
        elif self.busy and ext:
            is_last = self.idx == last
            if lead:
                if edge_seen & ((self.lf ^ 1) | (is_last ^ 1)):
                    raw = self._bit_of(self.sh, mux(self.lf, self.idx + 1, 0), self.crc_pkt)
                    bit_v = self.crc_pkt ^ 1
                r["done"] = edge_opp & self.lf & is_last
            elif edge_seen:
                if is_last:
                    r["done"] = 1
                else:
                    raw = self._bit_of(self.sh, self.idx + 1, self.crc_pkt)
                    bit_v = self.crc_pkt ^ 1
        elif self.busy and not hold:
            if at_b:
                if self.eph != 0:
                    if self.eph == 2:
                        j = 1
                    elif self.eph == 3:
                        r["done"] = 1
                elif stuff_now:
                    raw = 0
                elif self.idx == last:
                    if not cpha:
                        sck = cpol                   # CPHA 0 returns SCK to idle at the end of a word
                    if c_chain & next_ok:
                        r["chain"] = 1
                        raw = self._bit_of(next_data, 0, next_crc)
                        bit_v = next_crc ^ 1
                        sck = go_sck
                    elif eop:
                        se0 = 1
                    else:
                        r["done"] = 1
                else:
                    raw = self._bit_of(self.sh, self.idx + 1, self.crc_pkt)
                    bit_v = self.crc_pkt ^ 1
                    r["dtick"] = 1
                    sck = go_sck
            elif at_h & (self.eph == 0):
                sck = vh
                r["stick"] = rel_h ^ 1
        r["stick"] = mux(self.busy & (start ^ 1) & release & self.sp, 1, r["stick"])

        # encoding
        m, m2 = mux(self.den, self._mask(self.dpin), 0), self.d2_mask
        wm = wv = 0
        enc = None
        if raw is not None:
            lvl = mux(start, 1, self.lvl)
            enc = mux(nrzi, mux(raw, lvl, lvl ^ 1), raw)
            v = enc ^ inv
            wm |= m | m2
            wv |= mux(v, m, 0) | mux(v ^ 1, m2, 0)
        if se0:
            wm |= m | m2
        if j:
            wm |= m | m2
            wv |= m
        if sck is not None:
            cm = mux(cen, self._mask(cpin), 0)
            wm |= cm
            wv |= mux(sck, cm, 0)
        r["pw_mask"], r["pw_val"] = wm, wv
        r["bit_v"] = 0 if raw is None else bit_v
        r["bit_o"] = 0 if raw is None else raw
        # for clock()
        r["_"] = dict(raw=raw, enc=enc, sck=sck, at_b=at_b, at_h=at_h, hold=hold, release=release, ext=ext,
                      edge_seen=edge_seen, lead=lead, ein=ein, se0=se0, stuff_now=stuff_now, cpin_in=cpin_in,
                      last=last, rel_h=rel_h, stretch=stretch & cen, next_data=next_data, next_crc=next_crc,
                      start=start, start_data=start_data, stop=stop)
        return r

    def clock(self, rst, cfg_we, cfg_idx, cfg_data, r):
        if rst:
            self.reset()
            return
        x = r["_"]
        self.eprev = x["ein"]
        start, stop = x["start"], x["stop"]
        done, chain = r["done"], r["chain"]
        if x["raw"] is not None:
            self.lvl = x["enc"]
            self.ones = mux(x["raw"], mux(start, 1, (self.ones + 1) & 15), 0)
        set_sp = x["at_h"] & self.busy & (x["ext"] ^ 1) & (x["hold"] ^ 1) & (x["at_b"] ^ 1) & x["rel_h"]
        self.sp = mux(set_sp, 1, mux(start | stop | (x["release"] & self.busy), 0, self.sp))
        if start:
            self.busy = 1
            self.sh = x["start_data"]
            self.idx = 0
            self.o = 1
            self.lf = self.sw = self.swc = self.crc_pkt = self.eph = 0
            if x["raw"] is None:
                self.ones = 0
        elif self.busy:
            if stop | done:
                self.busy = 0
                self.eph = 0
            elif chain:
                self.sh = x["next_data"]
                self.idx = 0
                self.o = 1
                self.crc_pkt = x["next_crc"]
                self.sw = self.swc = 0
            elif x["ext"]:
                first = x["edge_seen"] & x["lead"] & (self.lf ^ 1)
                step = x["edge_seen"] & (first ^ 1) & (self.idx != x["last"])
                self.lf = mux(first, 1, self.lf)
                self.idx = mux(step, (self.idx + 1) & 15, self.idx)
            elif not x["hold"]:
                if x["at_b"]:
                    self.o = 1
                    if self.eph != 0:
                        self.eph = (self.eph + 1) & 3
                    elif x["se0"]:
                        self.eph = 1
                    else:
                        self.idx = mux(x["stuff_now"], self.idx, (self.idx + 1) & 15)
                else:
                    self.o = (self.o + 1) & 0xFFFF
            if not chain:
                waiting = self.sw & (self.swc != 0)
                self.sw = mux(self.sw & (self.swc == 0) & x["cpin_in"], 0, self.sw)
                self.swc = mux(waiting, self.swc - 1, self.swc)
        if x["sck"] is not None:
            hit = x["sck"] & x["stretch"] & (stop ^ 1) & (done ^ 1)
            self.sw = mux(hit, 1, self.sw)
            self.swc = mux(hit, 2, self.swc)
        w = [cfg_we & (cfg_idx == k) for k in range(6)]
        self.pins = mux(w[0], cfg_data & 0x1F1F, self.pins)
        self.bit = mux(w[1], cfg_data, self.bit)
        self.frame = mux(w[2], cfg_data & 0x03F5, self.frame)
        self.clk = mux(w[3], cfg_data & 7, self.clk)
        self.tick = mux(w[4], cfg_data & 0x03F1, self.tick)
        self.line = mux(w[5], cfg_data & 0x0FF7, self.line)
