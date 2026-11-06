# SPDX-License-Identifier: Apache-2.0
"""
i2c_lib: the I2C bus limits (NXP UM10204 Rev. 7, Tables 11 and 13), a target and a controller
written from that document, and helpers for the tests.

Levels on the bench are what a device reads, after `bench.Edges` applied the rise and fall time, so
each limit is measured at the input thresholds, as the standard measures it.
"""

from bench import NS, Edges

SCL, SDA = 6, 7
# ns. vd = tVD;DAT and tVD;ACK. tr, tf = the slowest edges the bus may have.
LIMITS = {
    "SM": dict(fscl=100, low=4700, high=4000, hd_sta=4000, su_sta=4700, su_sto=4000, buf=4700, su_dat=250, vd=3450,
               tr=1000, tf=300),
    "FM": dict(fscl=400, low=1300, high=600, hd_sta=600, su_sta=600, su_sto=600, buf=1300, su_dat=100, vd=900,
               tr=300, tf=300),
    "FM+": dict(fscl=1000, low=500, high=260, hd_sta=260, su_sta=260, su_sto=260, buf=500, su_dat=50, vd=450,
                tr=120, tf=120),
    "HS": dict(fscl=3400, low=160, high=60, hd_sta=160, su_sta=160, su_sto=160, buf=0, su_dat=10, vd=None,
               tr=40, tf=40),
}
# Fastest settings of the master firmware for each speed
MASTER_B = {"SM": 478, "FM": 126, "FM+": 48}


def cyc(ns):
    """ns to whole cycles, rounded up."""
    return int(-(-ns // NS))


def slow_edges(mode, scl_rise=True):
    """The slowest edges the standard allows, as seen at the thresholds."""
    lim = LIMITS[mode]
    return {SCL: Edges(cyc(lim["tr"]) if scl_rise else 0, cyc(lim["tf"])),
            SDA: Edges(cyc(lim["tr"]), cyc(lim["tf"]))}


def timing(samples, mode, scl=SCL, sda=SDA, start=0, stop=None, check=None):
    """Measures the limits of `mode` in a trace; returns {name: worst value in ns} of those broken.
    `check`: the names to look at, when the other side is not the one being tested."""
    lim = LIMITS[mode]
    worst = {}

    def need(name, cycles):
        ns = cycles * NS
        if ns < lim[name] and (name not in worst or ns < worst[name]):
            worst[name] = round(ns)

    rise = fall = sda_t = st = sp = None
    end = len(samples) if stop is None else stop
    for t in range(max(1, start), end):
        pc, c = (samples[t - 1] >> scl) & 1, (samples[t] >> scl) & 1
        pd, d = (samples[t - 1] >> sda) & 1, (samples[t] >> sda) & 1
        if pc and c and pd and not d:                       # START
            if sp is not None and (rise is None or rise < sp):
                if lim["buf"]:
                    need("buf", t - sp)
            elif rise is not None:
                need("su_sta", t - rise)
            st, sp = t, None
        elif pc and c and not pd and d and st is not None:  # STOP
            if rise is not None:
                need("su_sto", t - rise)
            sp = t
        elif pd != d and not c:
            sda_t = t
        if st is None:
            continue
        if not pc and c:                                    # SCL rises
            if fall is not None:
                need("low", t - fall)
            if sda_t is not None and fall is not None and sda_t >= fall:
                need("su_dat", t - sda_t)
            if rise is not None and 48000 / (t - rise) > lim["fscl"]:
                worst["fscl"] = max(worst.get("fscl", 0), round(48000 / (t - rise)))
            rise = t
        elif pc and not c:                                  # SCL falls
            if rise is not None:
                need("high", t - rise)
            if fall is None or fall < st:
                need("hd_sta", t - st)
            fall = t
    return {k: v for k, v in worst.items() if check is None or k in check}


def data_hold_valid(samples, mode, scl=SCL, sda=SDA, waive_stretched=None):
    """tHD;DAT and tVD: when SDA changes in a SCL low phase. Returns (min hold in cycles, max valid in ns)."""
    mn, mx = None, 0
    fall = None
    for t in range(1, len(samples)):
        pc, c = (samples[t - 1] >> scl) & 1, (samples[t] >> scl) & 1
        pd, d = (samples[t - 1] >> sda) & 1, (samples[t] >> sda) & 1
        if pc and not c:
            fall = t
        if pd != d and not c and fall is not None:
            dt = t - fall
            mn = dt if mn is None else min(mn, dt)
            mx = max(mx, dt)
    return mn, mx * NS


def expect_write(addr, data, stop=True, start="S"):
    ev = [start, ("AW", addr), "A"]
    for d in data:
        ev += [("DW", d), "A"]
    return ev + (["P"] if stop else [])


def expect_read(addr, data, stop=True, start="S"):
    ev = [start, ("AR", addr), "A"]
    for i, d in enumerate(data):
        ev += [("DR", d), "A" if i < len(data) - 1 else "N"]
    return ev + (["P"] if stop else [])


class Target:
    """A register device from UM10204 3.1: pointer byte, then data; ACKs its own address, leaves SDA
    alone otherwise. `stretch`: 'addr', 'read' (byte level) or 'bit' (every low phase): cycles to hold SCL."""

    def __init__(self, addr=0x50, mem=None, nack_after=None, stretch=None, addr10=None, scl=SCL, sda=SDA):
        self.scl, self.sda, self.addr, self.addr10 = scl, sda, addr, addr10
        self.mem = mem if mem is not None else [0] * 256
        self.nack_after, self.stretch = nack_after, stretch or {}
        self.pscl, self.psda = 1, 1
        self.state, self.bits, self.n, self.ptr, self.nw = "idle", 0, 0, 0, 0
        self.first, self.rw, self.rd_byte, self.master_ack, self.matched10 = True, 0, 0, False, False
        self.sda_o = self.scl_o = 1
        self.hold = 0
        self.seen, self.starts, self.stops = [], 0, 0
        self.stretched = 0

    def _falling(self):
        st = self.state
        if st == "ack_a10":
            self.sda_o, self.state, self.bits, self.n = 1, "addr2", 0, 0
        elif st in ("addr", "addr2", "wdata") and self.n == 8:
            byte, self.bits, self.n = self.bits & 0xFF, 0, 0
            if st == "addr2":
                if byte == self.addr10 & 0xFF:
                    self.matched10, self.rw, self.sda_o, self.state, self.first = True, 0, 0, "ack_a", True
                else:
                    self.state = "idle"
            elif st == "addr" and self.addr10 is not None:
                self.seen.append(byte)
                if byte & 0xFE == 0xF0 | ((self.addr10 >> 7) & 0x06):
                    if byte & 1 and self.matched10:
                        self.rw, self.sda_o, self.state = 1, 0, "ack_a"
                    elif not byte & 1:
                        self.sda_o, self.state = 0, "ack_a10"
                    else:
                        self.state = "idle"
                else:
                    self.state = "idle"
            elif st == "addr":
                self.seen.append(byte)
                if byte >> 1 == self.addr:
                    self.rw, self.sda_o, self.state, self.first = byte & 1, 0, "ack_a", True
                else:
                    self.state = "idle"
            else:
                ok = self.nack_after is None or self.nw < self.nack_after
                if ok:
                    if self.first:
                        self.ptr, self.first = byte, False
                    else:
                        self.mem[self.ptr & 0xFF] = byte
                        self.ptr += 1
                    self.nw += 1
                self.sda_o, self.state = (0, "ack_w") if ok else (1, "idle")
        elif st == "ack_a":
            if self.stretch.get("addr"):
                self.scl_o, self.hold = 0, self.stretch["addr"]
            if self.rw:
                self._send()
            else:
                self.sda_o, self.state = 1, "wdata"
        elif st == "ack_w":
            self.sda_o, self.state = 1, "wdata"
        elif st == "rdata":
            self.n += 1
            if self.n == 8:
                self.sda_o, self.state = 1, "rack"
            else:
                self.sda_o = (self.rd_byte >> (7 - self.n)) & 1
        elif st == "rack":
            if self.master_ack:
                self._send()
            else:
                self.sda_o, self.state = 1, "idle"
        if self.stretch.get("bit") and not self.hold:
            self.scl_o, self.hold = 0, self.stretch["bit"]

    def _send(self):
        if self.stretch.get("read"):
            self.scl_o, self.hold = 0, self.stretch["read"]
        self.rd_byte = self.mem[self.ptr & 0xFF]
        self.ptr += 1
        self.n, self.state = 0, "rdata"
        self.sda_o = (self.rd_byte >> 7) & 1

    def step(self, line, t):
        scl, sda = (line >> self.scl) & 1, (line >> self.sda) & 1
        if self.hold:
            self.hold -= 1
            self.stretched += 1
            if not self.hold:
                self.scl_o = 1
        if self.pscl and scl and self.psda and not sda:
            self.starts += 1
            self.state, self.bits, self.n, self.sda_o = "addr", 0, 0, 1
        elif self.pscl and scl and not self.psda and sda:
            self.stops += 1
            self.state, self.matched10, self.sda_o = "idle", False, 1
        elif not self.pscl and scl:
            if self.state in ("addr", "addr2", "wdata"):
                self.bits, self.n = (self.bits << 1) | sda, self.n + 1
            elif self.state == "rack":
                self.master_ack = sda == 0
        elif self.pscl and not scl:
            self._falling()
        self.pscl, self.psda = scl, sda
        m = ((1 - self.scl_o) << self.scl) | ((1 - self.sda_o) << self.sda)
        return m, 0


class Controller:
    """An I2C controller from UM10204, for Script: every phase at the minimum of `mode`, or `h` cycles
    per clock phase. Releases lines, as open drain, and waits for a stretched SCL."""

    def __init__(self, script, mode="FM", margin=2, h=None, scl=SCL, sda=SDA):
        lim = LIMITS[mode]
        self.s, self.scl, self.sda = script, scl, sda
        c = lambda n: cyc(n) + margin    # noqa: E731
        self.tlow, self.thigh = c(lim["low"]), c(lim["high"])
        self.hd_sta, self.su_sta, self.su_sto, self.tbuf = c(lim["hd_sta"]), c(lim["su_sta"]), c(lim["su_sto"]), c(lim["buf"])
        self.hold = max(2, cyc(lim["tf"]))       # SDA changes once SCL has fallen (Table 11 note 2)
        self.tr = cyc(lim["tr"])
        self.su = cyc(lim["su_dat"])
        if h:
            self.tlow = self.thigh = self.hd_sta = self.su_sta = self.su_sto = self.tbuf = h
            self.hold = max(2, h // 4)

    def _l(self, p):
        return (self.s.line >> p) & 1

    def _sda(self, v):
        return {self.sda: None if v else 0}

    def _rise_scl(self):
        yield {self.scl: None}
        n = 0
        while not self._l(self.scl):
            n += 1
            assert n < 2_000_000, "SCL stuck low"
            yield 1

    def _clock(self, bit):
        """One clock from SCL low: returns the SDA level read."""
        yield self._sda(bit)
        yield max(1, self.tlow - self.hold)
        yield from self._rise_scl()
        yield self.thigh
        v = self._l(self.sda)
        yield {self.scl: 0}
        yield self.hold
        return v

    def start(self):
        yield self._sda(1)
        yield self.tbuf
        yield from self._rise_scl()
        yield self.su_sta
        yield self._sda(0)
        yield self.hd_sta
        yield {self.scl: 0}
        yield self.hold

    def restart(self):
        yield self._sda(1)
        yield self.tlow
        yield from self._rise_scl()
        yield self.su_sta
        yield self._sda(0)
        yield self.hd_sta
        yield {self.scl: 0}
        yield self.hold

    def stop(self):
        yield self._sda(0)
        yield self.tlow
        yield from self._rise_scl()
        yield self.su_sto
        yield self._sda(1)
        yield self.tbuf

    def wbyte(self, b):
        for k in range(8):
            yield from self._clock((b >> (7 - k)) & 1)
        r = yield from self._clock(1)
        return r == 0

    def rbyte(self, ack):
        v = 0
        for _ in range(8):
            v = v << 1 | (yield from self._clock(1))
        yield from self._clock(0 if ack else 1)
        return v
