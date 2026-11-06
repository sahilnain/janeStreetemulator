# SPDX-License-Identifier: Apache-2.0
"""
uart_lib: helpers for test_uart.py.

`frame(word, wbits, stop, inv)` gives the levels of one frame and `Line` plays them on a pin
with any bit time, fractional if needed, from the UART definition and nothing in the chip.
`check_grid` measures a trace: every edge of a frame must lie within `tol` of the ideal grid.
"""

import pe_fw as F
from bench import A_RXF, A_STATUS, A_TXF, ST_RXF_EMPTY, ST_TXF_FULL, Bench, OWN_TX, Script, owners


def fw(B, bits=8, parity=F.PAR_NONE, stop_b2=2, inv=0, msb=0, txd=0, rxd=1):
    n = 9 if B >= 10 else max(1, B - 1)
    s = max(1, B // (n + 1))
    return F.uart_ex(txd=txd, rxd=rxd, B=B, N=n, K=1, S=s, bits=bits, parity=parity, stop_b2=stop_b2, inv=inv, msb=msb)


def boot(B, **kw):
    b = Bench(1)
    nd = b.nodes[0]
    nd.load(fw(B, **kw), own=owners(p0=OWN_TX))
    b.run(200)                  # the firmware finishes its setup, as a host would wait for
    return b, nd


def word_bits(data, bits, parity, msb):
    """The bits of a word in wire order, parity included."""
    order = list(range(bits - 1, -1, -1)) if msb else list(range(bits))
    d = [(data >> i) & 1 for i in order]
    if parity == F.PAR_EVEN:
        d.append(sum(d) & 1)
    elif parity == F.PAR_ODD:
        d.append(1 - (sum(d) & 1))
    elif parity == F.PAR_MARK:
        d.append(1)
    elif parity == F.PAR_SPACE:
        d.append(0)
    return d


def frame(data, bits=8, parity=F.PAR_NONE, stop=1.0, inv=0, msb=0, bad_stop=False, bad_parity=False):
    """[(level, length in bit times)] of one frame."""
    idle = 0 if inv else 1
    w = word_bits(data, bits, parity, msb)
    if bad_parity:
        w[-1] ^= 1
    out = [(1 - idle, 1.0)] + [(b ^ inv, 1.0) for b in w]
    out.append((1 - idle if bad_stop else idle, 1.0 if bad_stop else stop))
    return out


def play(pin, segments, bit, idle=1, lead=0):
    """Generator for Script: drive `segments` with bit time `bit` (float cycles)."""
    def g():
        yield {pin: idle}
        if lead:
            yield lead
        t, cyc = 0.0, 0
        for lv, n in segments:
            t += n * bit
            k = int(round(t)) - cyc
            cyc += k
            yield {pin: lv}
            if k > 1:
                yield k - 1
        yield {pin: idle}
    return g()


def transfer(b, nd, tx=(), n_rx=0, limit=4_000_000, on_tx_bytes=1):
    """Host loop: write tx to TXF and read n_rx bytes from RXF, as real host software does."""
    tx, got = list(tx), []
    start = len(b.samples)
    while (tx or len(got) < n_rx) and len(b.samples) - start < limit:
        st = nd.read(A_STATUS)
        if not st & ST_RXF_EMPTY:
            got.append(nd.read(A_RXF))
        elif tx and not st & ST_TXF_FULL:
            nd.write(A_TXF, tx.pop(0))
        else:
            b.run(8)
    return got


def drain(b, nd, B, frame_bits=14):
    """Wait until everything queued has left the pin."""
    b.until(lambda: nd.chip.txf.empty and not nd.chip.tx.busy, what="TX drain")
    b.run(frame_bits * B)


def tx_words(words, bits):
    """Words to the bytes the firmware's TXF wants: bits > 8 is two bytes, low first."""
    out = []
    for w in words:
        out.append(w & 0xFF)
        if bits > 8:
            out.append(w >> 8)
    return out


def rx_words(got, bits):
    if bits <= 8:
        return list(got)
    return [got[i] | (got[i + 1] << 8) for i in range(0, len(got) - 1, 2)]


def check_grid(samples, pin, B, wbits, inv=0, tol=None, start_extra=1):
    """Each frame: every edge within `tol` cycles of start + k * B. Returns the worst error."""
    idle = 0 if inv else 1
    tol = max(1, B * 0.02) if tol is None else tol
    lv = [(s >> pin) & 1 for s in samples]
    i, worst, n = 1, 0.0, len(lv)
    while i < n:
        if lv[i - 1] == idle and lv[i] != idle:
            s = i
            end = min(n, s + (wbits + 1) * B + 2)
            for t in range(s + 1, end):
                if lv[t] != lv[t - 1]:
                    k = round((t - s) / B)
                    worst = max(worst, abs(t - s - k * B))
            i = end
        else:
            i += 1
    assert worst <= tol, f"edge {worst} cycles off the grid, limit {tol}"
    return worst


__all__ = ["fw", "boot", "frame", "play", "transfer", "drain", "tx_words", "rx_words", "check_grid", "Script"]
