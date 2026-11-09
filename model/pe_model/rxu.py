# SPDX-License-Identifier: Apache-2.0
"""
rxu: the RX unit, written from docs/spec/blocks.md Section 4.

What this file does
-------------------
comb() finds the raw sample of this cycle from the selected source (vote, TX tick,
external edge or the resynchronised timer), decodes it as Section 4.5 lists, and
gives the pin write, done and push. clock() updates the state.
"""

from .blocks import Vote, keep, mux
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
        return ((self.frame >> 3) & 1) & ((self.tick & 3) != 0) & (self.need ^ 1)

    def read(self, idx, ferr):
        status = self.busy | self.rxrdy << 1 | ferr << 2 | self.overrun << 3 | self.serr << 4 | self.nb << 8
        return mux(idx == 0, self.data_q, mux(idx == 1, status, 0))

    def comb(self, start, start_data, stop, pin_in, stick, dtick):
        ibit = (pin_in >> self.ipin) & 1
        i2 = (pin_in >> field(self.pins, 7, 4)) & 1
        src = self.tick & 3
        ein = (pin_in >> field(self.tick, 7, 4)) & 1
        eedge = (self.tick >> 8) & 1
        sdly = self.tick >> 12
        msb, hold, inv = self.frame & 1, (self.frame >> 1) & 1, (self.frame >> 2) & 1
        n = nbits_of(self.frame)
        B = mux(self.bit != 0, self.bit, 1)
        live = self.busy
        se0 = ((self.pins >> 13) & 1) & (ibit ^ 1) & (i2 ^ 1)
        odly, nrzi, huntcfg, stuffn = self.odly & 15, (self.odly >> 4) & 1, (self.odly >> 5) & 1, field(self.odly, 11, 8)

        # sample and drive events
        edge_i = ibit != self.iprev
        sel_edge = (ein != self.eprev) & (ein == (eedge ^ 1))
        opp_edge = (ein != self.eprev) & (ein == eedge)
        raw_t = live & mux(src == 1, stick, mux(src == 2, sel_edge, 0))
        tq_n = ((self.tq << 1) | raw_t) & 0xFFFF
        take_t = mux(sdly == 0, raw_t, live & ((self.tq >> ((sdly - 1) & 15)) & 1))
        raw_o = live & mux(src == 1, dtick, mux(src == 2, opp_edge, 0))
        oq_n = ((self.oq << 1) | raw_o) & 0xFFFF
        drv_d = mux(odly == 0, raw_o, live & ((self.oq >> ((odly - 1) & 15)) & 1))

        win_now = (src == 0) & ((start & (self.first == 0)) | (live & (self.wins < n) & (self.wcnt == 0)))
        take = live & (se0 ^ 1) & mux(src == 0, self.vote.done,
                                      mux(src == 3, (self.wcnt == 0) & (edge_i ^ 1), take_t))
        sample = mux(src == 0, self.vote.bit, ibit)

        # decoding, in the order of Section 4.5
        v = sample ^ inv
        d = mux(nrzi, v == self.sprev, v)
        hunting_t = take & self.hunting
        skip = (self.hunting ^ 1) & (stuffn != 0) & (self.rones == stuffn)
        data = take & (self.hunting ^ 1) & (skip ^ 1)
        found = hunting_t & d & (self.hz == 2)
        hunting = mux(found, 0, self.hunting)
        hz = mux(hunting_t, mux(d, 0, mux(self.hz < 2, self.hz + 1, self.hz)), self.hz)
        rones = mux(found, 1, mux(take & skip, 0, mux(data, mux(d, (self.rones + 1) & 15, 0), self.rones)))
        serr = mux(take & skip & d, 1, self.serr)
        sprev = mux(take, v, self.sprev)

        done = data & (self.nb == n - 1)
        word = mux(data, mux(msb, ((self.word << 1) | d) & 0xFFFF, self.word | (d << (self.nb & 15))), self.word)

        # drive
        drv_ev = live & (self.oidx != n - 1) & mux(src == 0, win_now & (self.wins != 0), drv_d)
        wr_o = self.need & (start | drv_ev)
        obit = mux(start, 0, self.oidx + 1)
        dsrc = mux(start, start_data, self.od)
        b = (dsrc >> (mux(msb, n - 1 - obit, obit) & 15)) & 1     # 4-bit position, as in the RTL
        om = (1 << field(self.pins, 11, 8)) & self.nm

        # A stop in cycle D is too late: the word is complete (the FIFO grant to the controller
        # depends on push, so push cannot depend on the controller's ustop).
        return dict(wm=mux(wr_o, om, 0), wv=mux(wr_o & (b ^ inv), om, 0), done=done, push=done & (hold ^ 1),
                    push_data=word & 0xFF, bit_v=data, bit_o=d,
                    _=dict(start=start, start_data=start_data, stop=stop, ibit=ibit, ein=ein, tq=tq_n, oq=oq_n,
                           win_now=win_now, data=data, word=word, hunting=hunting, hz=hz, rones=rones, serr=serr,
                           sprev=sprev, done=done, hold=hold, edge_i=edge_i, drv_ev=drv_ev, n=n, B=B, src=src,
                           huntcfg=huntcfg, inv=inv, pin_in=pin_in))

    def clock(self, rst, restart, cfg_we, cfg_idx, cfg_data, r, after_end, rd_data_now, clr_ovr):
        if rst:
            self.reset()
            self.vote = Vote(self.cfg)
            return
        x = r["_"]
        start, stop, n, B, src, done = x["start"], x["stop"], x["n"], x["B"], x["src"], x["done"]
        hold = x["hold"]
        v = self.vcfg
        self.vote.clock(stop | restart, x["win_now"], self.ipin, x["pin_in"], mux(v & 15, v & 15, 1), v >> 4, self.step)
        self.eprev, self.iprev = x["ein"], x["ibit"]
        clear = stop | restart | start
        self.tq, self.oq = mux(clear, 0, x["tq"]), mux(clear, 0, x["oq"])

        # results
        self.rbyte = mux(done, x["word"], self.rbyte)
        if restart:
            self.rxrdy = self.overrun = 0
        else:
            load = after_end & hold
            # a set of overrun wins over a clear in the same cycle
            self.overrun = mux(load & self.rxrdy & (rd_data_now ^ 1), 1, mux(clr_ovr, 0, self.overrun))
            self.data_q = mux(load, self.rbyte, self.data_q)
            self.rxrdy = mux(load, 1, mux(rd_data_now, 0, self.rxrdy))

        chain_on = self.chain_on
        if start:
            self.busy = 1
            self.word = self.nb = self.oidx = self.serr = 0
            self.od = x["start_data"]
            no_first = self.first == 0
            self.wins = mux(no_first, 1, 0)
            self.wcnt = mux(no_first, B - 1, self.first - 1)
            self.hunting, self.hz, self.rones = x["huntcfg"], 0, 0
            self.sprev = x["ibit"] ^ x["inv"]
        elif self.busy:
            self.busy = mux(stop | restart | (done & (chain_on ^ 1)), 0, 1)
            self.hunting, self.hz, self.rones, self.serr = x["hunting"], x["hz"], x["rones"], x["serr"]
            self.sprev = x["sprev"]
            next_word = done & chain_on
            self.word = mux(next_word, 0, x["word"])
            self.nb = mux(next_word, 0, mux(x["data"], (self.nb + 1) & 31, self.nb))
            self.oidx = mux(x["drv_ev"], (self.oidx + 1) & 15, self.oidx)
            zero = self.wcnt == 0
            more = zero & (self.wins < n)
            half = B >> 1
            own = mux(zero, mux(more, B - 1, self.wcnt), self.wcnt - 1)
            dp = mux(x["edge_i"], mux(half != 0, half - 1, 0), mux(zero, B - 1, self.wcnt - 1))
            self.wcnt = mux(src == 0, own, mux(src == 3, dp, self.wcnt))
            self.wins = mux((src == 0) & more, self.wins + 1, self.wins)
        w = [cfg_we & (cfg_idx == k) for k in range(8)]
        # Protocol support: the bits of a feature that is not built keep their value (Section 13)
        c = self.cfg
        ext, chn = c.spi or c.i2c, c.spi or c.usb
        self.pins = mux(w[0], keep(self.pins, cfg_data, 0x3FFF,
                                   0x000F | (0x20F0 if c.usb else 0) | (0x1F00 if c.i2c else 0)), self.pins)
        self.bit = mux(w[1], cfg_data, self.bit)
        self.frame = mux(w[2], keep(self.frame, cfg_data, 0x01FF, 0x01F7 | (0x0008 if chn else 0)), self.frame)
        self.vcfg = mux(w[3], cfg_data & 0xFF, self.vcfg)
        self.step = mux(w[4], cfg_data, self.step)
        self.first = mux(w[5], cfg_data, self.first)
        # SRC 0 is always there; 1 and 2 need SPI or I2C, 3 needs USB. A SRC not built is ignored.
        s_in = cfg_data & 3
        src_ok = (s_in == 0) | mux(s_in == 3, int(c.usb), int(ext))
        tick = keep(self.tick, cfg_data, 0xF1F0, 0xF1F0 if ext else 0)
        src = mux(src_ok, s_in, self.tick & 3) if (ext or c.usb) else (self.tick & 3)
        self.tick = mux(w[6], (tick & 0xF1F0) | src, self.tick)
        self.odly = mux(w[7], keep(self.odly, cfg_data, 0x0F3F, (0x000F if c.i2c else 0) | (0x0F30 if c.usb else 0)),
                        self.odly)
