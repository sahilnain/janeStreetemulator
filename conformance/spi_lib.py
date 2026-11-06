# SPDX-License-Identifier: Apache-2.0
"""
spi_lib: SPI stimulus and measurements from the mode definitions alone.

CPOL: the level of SCK when idle. CPHA 0: data is sampled on the leading edge and changes on the
trailing edge; the first bit must be there before the first edge. CPHA 1: data changes on
the leading edge and is sampled on the trailing edge.

* `SpecSlave`: a device following that, with MISO released when not selected.
* `master_gen`: the same for a master, for Script.
* `check_frames`: what the decoder does not look at: idle level, edge count, when the data lines
  may change, CS lead and lag, clock phases.
"""

SCK, MOSI, MISO, CS = 2, 3, 4, 5


class SpecSlave:
    def __init__(self, cpol=0, cpha=0, msb=1, bits=8, tx=(), sck=SCK, mosi=MOSI, miso=MISO, cs=CS):
        self.cpol, self.cpha, self.msb, self.bits = cpol, cpha, msb, bits
        self.tx, self.rx = list(tx), []
        self.sck, self.mosi, self.miso, self.cs = sck, mosi, miso, cs
        self.p_sck, self.p_cs = cpol, 1
        self.cur, self.nout, self.nin, self.sh, self.out = 0, 0, 0, 0, 1
        self.sent = []

    def _load(self):
        self.cur = self.tx.pop(0) if self.tx else (1 << self.bits) - 1
        self.sent.append(self.cur)
        self.nout = 0

    def _bit(self):
        k = self.nout
        self.nout += 1
        return (self.cur >> (self.bits - 1 - k if self.msb else k)) & 1

    def step(self, line, t):
        sck, cs, mosi = (line >> self.sck) & 1, (line >> self.cs) & 1, (line >> self.mosi) & 1
        if self.p_cs and not cs:
            self.nin, self.sh = 0, 0
            self._load()
            if not self.cpha:
                self.out = self._bit()
        if not cs and sck != self.p_sck:
            leading = sck != self.cpol
            if leading != bool(self.cpha):                   # sampling edge
                self.sh = (self.sh << 1 | mosi) if self.msb else (self.sh | mosi << self.nin)
                self.nin += 1
                if self.nin == self.bits:
                    self.rx.append(self.sh)
                    self.nin, self.sh = 0, 0
            else:                                            # shift edge
                if self.nout == self.bits:
                    self._load()
                self.out = self._bit()
        self.p_sck, self.p_cs = sck, cs
        return (0, 0) if cs else (1 << self.miso, self.out << self.miso)


def master_gen(s, frames, bits=8, h=16, cpol=0, cpha=0, msb=1, lead=32, sck=SCK, mosi=MOSI, miso=MISO, cs=CS,
               gap=32, got=None, abort_after=None):
    """Clock `frames` (a list of lists of words), one CS low per frame. Replies go in `got`."""
    yield {cs: 1, sck: cpol, mosi: 0}
    yield gap
    for fr in frames:
        yield {cs: 0}
        yield lead
        for w in fr:
            v = 0
            for k in range(bits):
                i = bits - 1 - k if msb else k
                if abort_after is not None and k == abort_after:
                    break
                if not cpha:
                    yield {mosi: (w >> i) & 1}
                    yield max(1, h - 1)
                    v |= ((s.line >> miso) & 1) << i
                    yield {sck: 1 - cpol}
                    yield max(1, h - 1)
                    yield {sck: cpol}
                else:
                    yield {sck: 1 - cpol, mosi: (w >> i) & 1}
                    yield max(1, h - 1)
                    v |= ((s.line >> miso) & 1) << i
                    yield {sck: cpol}
                    yield max(1, h - 1)
            if got is not None:
                got.append(v)
        yield h
        yield {cs: 1}
        yield gap


def check_frames(samples, cpol, cpha, nedges=None, min_phase=1, sck=SCK, cs=CS, data=(MOSI,), min_lead=None, min_lag=None,
                 check_change=True):
    """Walks every frame (CS low). Raises on the first thing that breaks the mode definition."""
    lv = lambda t, p: (samples[t] >> p) & 1     # noqa: E731
    n = len(samples)
    t, frames, total = 1, 0, 0
    while t < n:
        if lv(t - 1, cs) and not lv(t, cs):
            start = t
            assert lv(t, sck) == cpol, f"CS fell at {t} with SCK not at CPOL"
            edges = []
            prev = {p: lv(t, p) for p in data}
            t += 1
            while t < n and not lv(t, cs):
                e = lv(t, sck) != lv(t - 1, sck)
                if e:
                    edges.append(t)
                for p in data:
                    if lv(t, p) != prev[p]:
                        if check_change and edges:
                            lead = (len(edges) % 2 == 1)        # the last edge was a leading one
                            assert not (e and lead != bool(cpha)), \
                                f"pin {p} changed exactly on a sampling edge at {t}"
                            if lead != bool(cpha):
                                raise AssertionError(f"pin {p} changed at {t}, after a sampling edge")
                        elif check_change and cpha and not edges:
                            raise AssertionError(f"CPHA 1: pin {p} changed at {t}, before the first edge")
                        prev[p] = lv(t, p)
                t += 1
            assert t < n, "frame runs off the trace"
            assert lv(t, sck) == cpol, f"CS rose at {t} with SCK not at CPOL"
            if edges:
                if min_lead is not None:
                    assert edges[0] - start >= min_lead, f"CS lead {edges[0] - start} < {min_lead}"
                if min_lag is not None:
                    assert t - edges[-1] >= min_lag, f"CS lag {t - edges[-1]} < {min_lag}"
                for a, c in zip(edges, edges[1:]):
                    assert c - a >= min_phase, f"SCK phase of {c - a} cycles at {a}, minimum {min_phase}"
                assert len(edges) % 2 == 0, f"odd number of SCK edges: {len(edges)}"
            total += len(edges)
            frames += 1
        t += 1
    if nedges is not None:
        assert total == nedges, f"{total} SCK edges, expected {nedges}"
    return frames
