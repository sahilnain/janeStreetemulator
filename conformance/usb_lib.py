# SPDX-License-Identifier: Apache-2.0
"""
usb_lib: USB low and full speed packets and bus levels from the USB 2.0 specification, chapters 7 and 8.

`encode` makes the levels of a packet: sync, bit stuffing, NRZI, EOP, with errors on request.
`Peer` plays levels on the bus. `packets` measures what a trace holds: when each packet starts and
ends, and its SE0 length, so the tests can compare the times with the limits in LIMITS.
"""

from bench import NS, Bench, Script

J, K, SE0 = "J", "K", "0"
SPEEDS = {"FS": dict(B=4, jp=0, kp=1), "LS": dict(B=32, jp=1, kp=0)}    # D+ = P0, D- = P1
LIMITS = {
    # ns, Tables 7-9 and 7-10. eop = source SE0 of EOP; eop_rx = the shortest a receiver must accept;
    # se0_glitch = SE0 in a differential transition, which is not an EOP (7.1.4); rate_pct = ppm as %.
    "FS": dict(eop=(160, 175), eop_rx=82, se0_glitch=14, rate_pct=0.25, jitter_next=20),
    "LS": dict(eop=(1250, 1500), eop_rx=670, se0_glitch=210, rate_pct=1.5, jitter_next=141),
}
TIPD = 2.0            # bit times, the shortest inter-packet delay (7.1.18.1)
TRSPIPD1 = 6.5        # the longest device response, detachable cable
TRSPIPD2 = 7.5        # the longest device response, captive cable; also the host's
TIMEOUT = (16.0, 18.0)  # a waiting side does not give up before 16 and gives up before 18 (7.1.19.1)
TDETRST_NS = 2500     # a device detects a reset within this (Table 7-14)
TDRST_MS = 10         # a host drives reset at least this long
TDRSMDN_MS = 20       # a host drives resume at least this long
OUT, IN, SOF, SETUP = 0xE1, 0x69, 0xA5, 0x2D
DATA0, DATA1, ACK, NAK, STALL = 0xC3, 0x4B, 0xD2, 0x5A, 0x1E


def crc5(bits):
    r = 0x1F
    for b in bits:
        fb = ((r >> 4) & 1) ^ b
        r = (r << 1) & 0x1F
        if fb:
            r ^= 0x05
    return [((~r) >> (4 - i)) & 1 for i in range(5)]


def crc16_bits(data):
    r = 0xFFFF
    for d in data:
        for i in range(8):
            fb = ((r >> 15) & 1) ^ ((d >> i) & 1)
            r = (r << 1) & 0xFFFF
            if fb:
                r ^= 0x8005
    v = ~r & 0xFFFF
    return [(v >> (15 - i)) & 1 for i in range(16)]


def to_bytes(bits):
    return [sum(b << i for i, b in enumerate(bits[k:k + 8])) for k in range(0, len(bits), 8)]


def token(pid, addr, ep, bad_crc=False):
    bits = [(addr >> i) & 1 for i in range(7)] + [(ep >> i) & 1 for i in range(4)]
    c = crc5(bits)
    if bad_crc:
        c[2] ^= 1
    return [pid] + to_bytes(bits + c)


def sof(frame):
    bits = [(frame >> i) & 1 for i in range(11)]
    return [SOF] + to_bytes(bits + crc5(bits))


def data_packet(pid, data, bad_crc=False):
    c = crc16_bits(data)
    if bad_crc:
        c[3] ^= 1
    return [pid] + list(data) + to_bytes(c)


def encode(pkt, sync_bits=8, no_stuff_first=False, dribble=False):
    """Levels of a packet, one per bit time, ending SE0 SE0 J."""
    bits = [0] * (sync_bits - 1) + [1]
    for b in pkt:
        bits += [(b >> i) & 1 for i in range(8)]
    out, level, ones, n = [], J, 0, 0
    for b in bits:
        level = level if b else (K if level == J else J)
        out.append(level)
        ones = ones + 1 if b else 0
        if ones == 6:
            if no_stuff_first:           # an error on purpose: this stuff bit is left out
                no_stuff_first = False
            else:
                level = K if level == J else J
                out.append(level)
            ones = 0
        n += 1
    if dribble:
        out.append(out[-1])
    return out + [SE0, SE0, J]


def _pins(jp, kp, lv):
    return {J: {jp: 1, kp: 0}, K: {jp: 0, kp: 1}, SE0: {jp: 0, kp: 0}}[lv]


def next_jitter(speed):
    """The receiver's budget to the next transition (Tables 7-4, 7-5), in cycles, rounded to nearest:
    FS 20 ns is 1 cycle, LS 141 ns is 7. Rounding down would make the full-speed value 0."""
    return round(LIMITS[speed]["jitter_next"] / NS)


def play(jp, kp, levels, bit, jitter=0, rnd=None, se0_cycles=None, glitch=0, release=True, next_j=0):
    """Generator for Script. bit: cycles per bit, may be fractional. jitter: largest error of each edge
    on its own, in cycles, so neighbours can differ by twice that. next_j: each edge is late by 0 or
    next_j cycles, so neighbours differ by at most next_j: the budget as the standard states it.
    se0_cycles: length of the EOP's SE0. glitch: cycles of SE0 put into each J-K change."""
    def g():
        yield _pins(jp, kp, J)
        t, cyc = 0.0, 0
        prev = J
        i = 0
        while i < len(levels):
            lv = levels[i]
            n = 1
            while i + n < len(levels) and levels[i + n] == lv:
                n += 1
            length = n * bit
            if lv == SE0 and se0_cycles is not None:
                length = se0_cycles
            t += length
            end = t + (rnd.randint(-jitter, jitter) if jitter and lv != SE0 else 0)
            if next_j and lv != SE0:
                end = t + rnd.choice((0, next_j))
            k = max(1, int(round(end)) - cyc)
            cyc += k
            if glitch and prev != SE0 and lv != SE0 and prev != lv:
                yield _pins(jp, kp, SE0)
                yield glitch
                k = max(1, k - glitch)
            yield _pins(jp, kp, lv)
            if k > 1:
                yield k - 1
            prev = lv
            i += n
        if release:
            yield {jp: None, kp: None}
    return g()


def bench(speed, n=1):
    sp = SPEEDS[speed]
    return Bench(n, pull=0xFF & ~(1 << sp["kp"]))


def send(b, speed, levels, bit=None, **kw):
    """Play levels on the bus, wait until they have ended, returns the trace index of the SE0-to-J edge."""
    sp = SPEEDS[speed]
    s = Script(play(sp["jp"], sp["kp"], levels, bit or sp["B"], **kw))
    b.devices.append(s)
    b.until(lambda: s.done, limit=1_000_000, what="peer send")
    b.devices.remove(s)
    return last_eop_end(b.samples, speed)


def level_at(x, jp, kp):
    j, k = (x >> jp) & 1, (x >> kp) & 1
    return "1" if j and k else (J if j else (K if k else SE0))


def last_eop_end(s, speed):
    sp = SPEEDS[speed]
    jp, kp = sp["jp"], sp["kp"]
    for t in range(len(s) - 1, 0, -1):
        if level_at(s[t], jp, kp) == J and level_at(s[t - 1], jp, kp) == SE0:
            return t
    return None


def packets(s, speed, start=0):
    """(sop, se0 start, se0 end) of each packet in the trace."""
    sp = SPEEDS[speed]
    jp, kp = sp["jp"], sp["kp"]
    out, t, n = [], max(1, start), len(s)
    st = None
    while t < n:
        lv = level_at(s[t], jp, kp)
        prev = level_at(s[t - 1], jp, kp)
        if st is None and lv == K and prev == J:
            st = t
        elif st is not None and lv == SE0 and prev != SE0:
            e = t
            while t < n and level_at(s[t], jp, kp) == SE0:
                t += 1
            out.append((st, e, t))
            st = None
            continue
        t += 1
    return out


def ns(cycles):
    return cycles * NS
