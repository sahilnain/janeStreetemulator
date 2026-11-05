# SPDX-License-Identifier: Apache-2.0
"""
txu: the TX unit, written from docs/spec/blocks.md Section 3.

What this file does
-------------------
comb() returns what the unit does in this cycle (pin writes, ticks, done, chain),
clock() updates its state. `o` counts the cycles of the current bit: 1 in the
cycle after the bit was written, B in the cycle the next write is due.
"""


def field(v, hi, lo):
    return (v >> lo) & ((1 << (hi - lo + 1)) - 1)


def nbits_of(frame):
    n = field(frame, 8, 4)
    return 8 if n == 0 else min(n, 16)


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
        return self._mask(field(self.line, 7, 4)) if self.diff else 0

    @property
    def diff(self):
        return (self.line >> 2) & 1 and self.den

    def _mask(self, pin):
        return (1 << pin) & self.nm

    def cfg_write(self, idx, data):
        if idx == 0:
            self.pins = data & 0x1F1F
        elif idx == 1:
            self.bit = data
        elif idx == 2:
            self.frame = data & 0x03F5
        elif idx == 3:
            self.clk = data & 7
        elif idx == 4:
            self.tick = data & 0x03F1
        elif idx == 5:
            self.line = data & 0x0FF7

    def read(self, idx):
        return (self.idx << 4) | self.busy if idx == 0 else 0

    def _bit_of(self, word, i, crc):
        n = nbits_of(self.frame)
        if crc:
            return (word >> (15 - i)) & 1
        return (word >> (n - 1 - i if self.frame & 1 else i)) & 1

    # --------------------------------------------------------------- comb
    def comb(self, start, start_data, stop, next_ok, next_crc, next_data, pin_in):
        ext = self.tick & 1
        epin = field(self.tick, 7, 4)
        eedge = (self.tick >> 8) & 1
        lead = ((self.tick >> 9) & 1) and ext
        ein = (pin_in >> epin) & 1
        edge_seen = ext and ein != self.eprev and ein == (eedge ^ 1)
        edge_opp = ext and ein != self.eprev and ein == eedge
        cpin = field(self.pins, 11, 8)
        cen = (self.pins >> 12) & 1
        cpin_in = (pin_in >> cpin) & 1
        cpol, cpha, stretch = self.clk & 1, (self.clk >> 1) & 1, (self.clk >> 2) & 1
        B = self.bit or 1
        H = B >> 1
        last = 15 if self.crc_pkt else nbits_of(self.frame) - 1
        stuffn = field(self.line, 11, 8)
        nrzi, eop = self.line & 1, (self.line >> 1) & 1
        inv = (self.frame >> 2) & 1
        c_chain = (self.frame >> 9) & 1

        vh = cpol if cpha else cpol ^ 1                     # the SCK level written at offset H
        rel_h = stretch and cen and vh                      # a release: the tick comes later
        go_sck = cpol ^ 1 if cpha else cpol                 # the SCK level written when a word begins
        release = self.sw and self.swc == 0 and cpin_in
        hold = self.sw and not release
        at_b = self.o == B
        at_h = H != 0 and self.o == H
        stuff_now = stuffn != 0 and self.ones == stuffn

        r = dict(done=0, chain=0, stick=0, dtick=0, bit_v=0, bit_o=0)
        raw = None      # data write
        bit_v = 0
        se0 = j = 0
        sck = None
        nxt = dict(idx=None, eph=None, word=None, crc=None, o=None)

        if start:
            lead0 = ((self.tick >> 9) & 1) and ext
            if not lead0:
                raw = self._bit_of(start_data, 0, 0)
                bit_v = 1
            if not ext:
                sck = go_sck
        elif self.busy and ext:
            if lead:
                if edge_seen and (not self.lf or self.idx != last):
                    i = self.idx + 1 if self.lf else 0
                    raw = self._bit_of(self.sh, i, 0)
                    bit_v = 1
                if edge_opp and self.lf and self.idx == last:
                    r["done"] = 1
            elif edge_seen:
                if self.idx == last:
                    r["done"] = 1
                else:
                    raw = self._bit_of(self.sh, self.idx + 1, 0)
                    bit_v = 1
        elif self.busy and not hold:
            if at_b:
                if self.eph:
                    if self.eph == 2:
                        j = 1
                    elif self.eph == 3:
                        r["done"] = 1
                elif stuff_now:
                    raw = 0
                elif self.idx == last:
                    if not cpha:
                        sck = cpol                   # CPHA 0 returns SCK to idle at the end of a word
                    if c_chain and next_ok:
                        r["chain"] = 1
                        raw = self._bit_of(next_data, 0, next_crc)
                        bit_v = 0 if next_crc else 1
                        sck = go_sck
                    elif eop:
                        se0 = 1
                    else:
                        r["done"] = 1
                else:
                    raw = self._bit_of(self.sh, self.idx + 1, self.crc_pkt)
                    bit_v = 0 if self.crc_pkt else 1
                    r["dtick"] = 1
                    sck = go_sck
            elif at_h and self.eph == 0:
                sck = vh
                if not rel_h:
                    r["stick"] = 1
        if self.busy and not start and release and self.sp:
            r["stick"] = 1

        if ext and start:
            sck = None

        # encoding
        wm = wv = 0
        enc = None
        if raw is not None:
            lvl = 1 if start else self.lvl
            enc = (lvl if raw else lvl ^ 1) if nrzi else raw
            if self.den:
                m = self._mask(self.dpin)
                wm |= m
                if enc ^ inv:
                    wv |= m
                if self.diff:
                    m2 = self.d2_mask
                    wm |= m2
                    if not enc ^ inv:
                        wv |= m2
        if se0 and self.den:
            wm |= self._mask(self.dpin) | self.d2_mask
        if j and self.den:
            wm |= self._mask(self.dpin) | self.d2_mask
            wv |= self._mask(self.dpin)
        if sck is not None and cen:
            m = self._mask(cpin)
            wm |= m
            if sck:
                wv |= m
        r["pw_mask"], r["pw_val"] = wm, wv
        r["bit_v"] = bit_v and raw is not None
        r["bit_o"] = raw or 0
        # for clock()
        r["_"] = dict(raw=raw, enc=enc, sck=sck, at_b=at_b, at_h=at_h, hold=hold, release=release, ext=ext,
                      edge_seen=edge_seen, lead=lead, ein=ein, se0=se0, stuff_now=stuff_now, cpin_in=cpin_in,
                      last=last, rel_h=rel_h, stretch=stretch and cen, next_data=next_data, next_crc=next_crc,
                      start=start, start_data=start_data, stop=stop)
        return r

    def clock(self, rst, cfg_we, cfg_idx, cfg_data, r):
        if rst:
            self.reset()
            return
        x = r["_"]
        self.eprev = x["ein"]
        start, stop = x["start"], x["stop"]
        if x["raw"] is not None:
            self.lvl = x["enc"]
            self.ones = (1 if start else (self.ones + 1) & 15) if x["raw"] else 0
        if start or stop:
            self.sp = 0
        if x["at_h"] and self.busy and not x["ext"] and not x["hold"] and not x["at_b"] and x["rel_h"]:
            self.sp = 1
        if x["release"] and self.busy:
            self.sp = 0
        if start:
            self.busy = 1
            self.sh = x["start_data"]
            self.idx = 0
            self.o = 1
            self.lf = self.sw = self.swc = self.crc_pkt = self.eph = 0
            if x["raw"] is None:
                self.ones = 0
        elif self.busy:
            if stop or r["done"]:
                self.busy = 0
                self.eph = 0
            elif r["chain"]:
                self.sh = x["next_data"]
                self.idx = 0
                self.o = 1
                self.crc_pkt = x["next_crc"]
                self.sw = self.swc = 0
            elif x["ext"]:
                if x["edge_seen"]:
                    if x["lead"] and not self.lf:
                        self.lf = 1
                    elif self.idx != x["last"]:
                        self.idx += 1
            elif not x["hold"]:
                if x["at_b"]:
                    self.o = 1
                    if self.eph:
                        self.eph = (self.eph + 1) & 3
                    elif x["se0"]:
                        self.eph = 1
                    elif not x["stuff_now"]:
                        self.idx += 1
                else:
                    self.o += 1
            if self.sw and not r["chain"]:
                if self.swc:
                    self.swc -= 1
                elif x["cpin_in"]:
                    self.sw = 0
        if x["sck"] == 1 and x["stretch"] and not stop and not r["done"]:
            self.sw = 1
            self.swc = 2
        if cfg_we:
            self.cfg_write(cfg_idx, cfg_data)
