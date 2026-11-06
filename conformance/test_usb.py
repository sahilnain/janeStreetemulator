# SPDX-License-Identifier: Apache-2.0
"""
test_usb: the chip model against USB 2.0 low and full speed (chapters 7 and 8), judged by the sigrok
usb_signalling + usb_packet decoders and the time limits of chapter 7. Packets from the peer are built
in usb_lib and also go through the decoder, so a mistake there shows up too.
Sections: docs/protocol_conformance.md Section 5.
"""

import random

import pe_fw as F
import pytest
import sigrok
import usb_lib as U
from bench import OWN_TX, CLOCK_HZ, Script, owners
from usb_lib import ACK, DATA0, DATA1, IN, NAK, OUT, SETUP, SPEEDS, STALL, data_packet, encode, packets, send, sof, token

ADDR, EP = 0x2A, 3
BOTH = ("FS", "LS")
MAXLEN = {"FS": 64, "LS": 8}
CSTAT = 0x24
RXD = F.UD_RXDATA + 1       # the PID comes first
WAIT = 40                   # bit times: the longest answer starts by 7.5, a handshake takes 19


def payload(n, seed=0):
    r = random.Random(n * 7 + seed)
    return [r.randint(0, 255) for _ in range(n)]


def device(speed, inlen=0, inpid=DATA1, indata=(), ouths=ACK, inhs=NAK, **kw):
    sp = SPEEDS[speed]
    b = U.bench(speed)
    nd = b.nodes[0]
    nd.dmem(F.UD_ADDR, [ADDR, 0, 0, 0, inlen, inpid, ouths, inhs, 0])
    nd.dmem(F.UD_INDATA, list(indata))
    nd.load(F.usb_device(B=sp["B"], jp=sp["jp"], kp=sp["kp"], **kw),
            own=owners(**{f"p{sp['jp']}": OWN_TX, f"p{sp['kp']}": OWN_TX}))
    b.run(150)
    return b, nd


def host(speed, **kw):
    sp = SPEEDS[speed]
    b = U.bench(speed)
    nd = b.nodes[0]
    nd.load(F.usb_host(B=sp["B"], jp=sp["jp"], kp=sp["kp"], **kw),
            own=owners(**{f"p{sp['jp']}": OWN_TX, f"p{sp['kp']}": OWN_TX}))
    b.run(60)
    return b, nd


def clean(s, speed):
    d = sigrok.usb(s, 0, 1, speed)
    assert not d["errors"], d["errors"][:4]
    return d


def names(d):
    return [p[1] for p in d["packets"]]


def bits_after(s, speed, t, what="answer"):
    """Bit times from the SE0-to-J edge at t to the next packet, None if there is none."""
    p = packets(s, speed, t)
    return (p[0][0] - t) / SPEEDS[speed]["B"] if p else None


def check_eops(s, speed, start=0):
    lo, hi = U.LIMITS[speed]["eop"]
    for sop, e, f in packets(s, speed, start):
        w = U.ns(f - e)
        assert lo <= w <= hi, f"EOP SE0 of {w:.0f} ns at {e}, limits {lo}..{hi}"


def turn_ok(t, speed, label=""):
    assert t is not None, f"no answer {label}"
    assert t >= U.TIPD, f"answer after {t} bit times: under 2 {label}"
    assert t <= U.TRSPIPD1, f"answer after {t} bit times: over 6.5 {label}"


# ---------------------------------------------------------------- device: OUT, SETUP
@pytest.mark.parametrize("n", [0, 1, 2, 3, 7, 8, 9, 16, 31, 32, 33, 48, 63, 64])
@pytest.mark.parametrize("speed", BOTH)
def test_device_out(speed, n):
    if n > MAXLEN[speed]:
        pytest.skip("over the maximum packet size")
    b, nd = device(speed)
    B = SPEEDS[speed]["B"]
    d = payload(n)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    t = send(b, speed, encode(data_packet(DATA1, d)))
    b.run(WAIT * B)
    b.until(lambda: nd.peek(F.UD_EVENT)[0] == F.EV_OUT, limit=4000, what="event")
    assert nd.peek(RXD, n) == d
    s = b.samples
    turn_ok(bits_after(s, speed, t), speed)
    x = clean(s[s0:], speed)
    assert names(x) == ["OUT", "DATA1", "ACK"], names(x)
    check_eops(s, speed, t)
    assert not b.contention
    assert nd.peek(F.UD_TOKEN)[0] == OUT and nd.peek(F.UD_EP)[0] == EP
    assert nd.peek(F.UD_RXLEN)[0] == n + 3     # PID, data, CRC


@pytest.mark.parametrize("speed", BOTH)
def test_device_setup(speed):
    """8.5.3: a SETUP is always answered with ACK, whatever handshake the endpoint would give to an OUT."""
    b, nd = device(speed, ouths=NAK)
    B = SPEEDS[speed]["B"]
    d = [0x80, 6, 0, 1, 0, 0, 18, 0]
    s0 = len(b.samples)
    send(b, speed, encode(token(SETUP, ADDR, 0)))
    b.run(2 * B)
    t = send(b, speed, encode(data_packet(DATA0, d)))
    b.run(WAIT * B)
    x = clean(b.samples[s0:], speed)
    assert names(x) == ["SETUP", "DATA0", "ACK"], names(x)
    turn_ok(bits_after(b.samples, speed, t), speed)


@pytest.mark.parametrize("hs", (ACK, NAK, STALL), ids=("ack", "nak", "stall"))
@pytest.mark.parametrize("speed", BOTH)
def test_device_out_handshakes(speed, hs):
    b, nd = device(speed, ouths=hs)
    B = SPEEDS[speed]["B"]
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    t = send(b, speed, encode(data_packet(DATA0, [1, 2, 3])))
    b.run(WAIT * B)
    assert names(clean(b.samples[s0:], speed))[-1] == {ACK: "ACK", NAK: "NAK", STALL: "STALL"}[hs]
    turn_ok(bits_after(b.samples, speed, t), speed)


@pytest.mark.parametrize("speed", BOTH)
def test_device_stuffing_patterns(speed):
    """7.1.9: runs of ones, at every position, in both directions; also a stuff bit as the last bit."""
    n = MAXLEN[speed]
    pats = [[0xFF] * n, [0x7F] * n, [0xFE] * n, [0x3F, 0xFC] * (n // 2), [0x00] * n]
    for pat in pats:
        b, nd = device(speed, inlen=len(pat) + 1, indata=pat)
        B = SPEEDS[speed]["B"]
        s0 = len(b.samples)
        send(b, speed, encode(token(OUT, ADDR, EP)))
        b.run(2 * B)
        send(b, speed, encode(data_packet(DATA0, pat)))
        b.run(WAIT * B)
        assert nd.peek(RXD, n) == pat
        t = send(b, speed, encode(token(IN, ADDR, EP)))
        b.run((n * 12 + 30) * B)
        x = clean(b.samples[s0:], speed)
        assert names(x)[:5] == ["OUT", "DATA0", "ACK", "IN", "DATA1"], names(x)
        assert "crc16-ok" in x["crc_ok"]


def stuff_at_end_payload():
    """A payload whose CRC16 ends in six ones: the stuff bit is the very last bit before EOP."""
    for n in range(1, 9):
        r = random.Random(n)
        for _ in range(200000):
            d = [r.randint(0, 255) for _ in range(n)]
            c = U.crc16_bits(d)
            if c[-6:] == [1] * 6 and c[-7] == 0:
                return d
    raise AssertionError


@pytest.mark.parametrize("speed", BOTH)
def test_device_stuff_bit_before_eop(speed):
    """7.1.9: 'a zero bit will be inserted even if it is the last bit before the EOP'. Both directions."""
    d = stuff_at_end_payload()
    b, nd = device(speed, inlen=len(d) + 1, indata=d)
    B = SPEEDS[speed]["B"]
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    t = send(b, speed, encode(data_packet(DATA0, d)))
    b.run(WAIT * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_OUT, "device refused it"
    t = send(b, speed, encode(token(IN, ADDR, EP)))
    b.run((len(d) * 12 + 40) * B)
    x = clean(b.samples[s0:], speed)
    assert names(x)[-1] == "DATA1" and x["crc_ok"].count("crc16-ok") == 2


# ---------------------------------------------------------------- device: IN
@pytest.mark.parametrize("n", [0, 1, 2, 7, 8, 9, 31, 32, 63, 64])
@pytest.mark.parametrize("pid", (DATA0, DATA1), ids=("d0", "d1"))
@pytest.mark.parametrize("speed", BOTH)
def test_device_in(speed, pid, n):
    if n > MAXLEN[speed]:
        pytest.skip("over the maximum packet size")
    d = payload(n, 5)
    b, nd = device(speed, inlen=n + 1, inpid=pid, indata=d)
    B = SPEEDS[speed]["B"]
    s0 = len(b.samples)
    t = send(b, speed, encode(token(IN, ADDR, EP)))
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) != 0, limit=40 * B, what="device starts")
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) == 0, limit=(n + 8) * 14 * B, what="device releases")
    t2 = U.last_eop_end(b.samples, speed)
    turn_ok(bits_after(b.samples, speed, t), speed)
    b.run(2 * B)
    send(b, speed, encode([ACK]))
    b.run(10 * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_IN_ACK, nd.peek(F.UD_EVENT)
    x = clean(b.samples[s0:], speed)
    assert names(x) == ["IN", "DATA0" if pid == DATA0 else "DATA1", "ACK"], names(x)
    assert "crc16-ok" in x["crc_ok"]
    got = [int(a.text.split()[-1], 16) for a in sigrok.run(b.samples[s0:], f"usb_signalling:dp=0:dm=1:signalling={'full-speed' if speed == 'FS' else 'low-speed'},usb_packet:signalling={'full-speed' if speed == 'FS' else 'low-speed'}", {"usb_packet": ["data"]})]
    assert got == d
    check_eops(b.samples, speed, t)
    # the J after the SE0 lasts one bit time, then the driver lets go (7.1.7.4.1)
    assert not b.contention


@pytest.mark.parametrize("speed", BOTH)
def test_device_in_nak_when_nothing_is_ready(speed):
    b, nd = device(speed, inlen=0)
    B = SPEEDS[speed]["B"]
    s0 = len(b.samples)
    t = send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    assert names(clean(b.samples[s0:], speed)) == ["IN", "NAK"]
    turn_ok(bits_after(b.samples, speed, t), speed)
    check_eops(b.samples, speed, t)


@pytest.mark.parametrize("speed", BOTH)
def test_device_in_no_ack_keeps_the_data(speed):
    """8.6.4: no handshake from the host: the data must stay for the retry."""
    d = [1, 2, 3]
    b, nd = device(speed, inlen=4, indata=d)
    B = SPEEDS[speed]["B"]
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(150 * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_IN_NOACK
    assert nd.peek(F.UD_INLEN)[0] == 4


@pytest.mark.parametrize("speed", BOTH)
def test_device_waits_for_the_handshake_as_long_as_it_must(speed):
    """7.1.19.1: after sending data the device does not time out before 16 bit times."""
    d = [1, 2, 3]
    b, nd = device(speed, inlen=4, indata=d)
    B = SPEEDS[speed]["B"]
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) != 0, limit=40 * B, what="device starts")
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) == 0, limit=100 * 14 * B, what="device releases")
    b.run(15 * B - 2)
    send(b, speed, encode([ACK]))
    b.run(10 * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_IN_ACK, "timed out before 16 bit times"


# ---------------------------------------------------------------- device: what must get no answer
@pytest.mark.parametrize("speed", BOTH)
def test_device_answers_only_its_address(speed):
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    for a in range(0, 128, 1 if speed == "FS" else 7):
        s0 = len(b.samples)
        send(b, speed, encode(token(IN, a, EP)))
        b.run(WAIT * B)
        n = len(packets(b.samples, speed, s0))
        assert n == (2 if a == ADDR else 1), f"address {a:#x}: {n} packets"


@pytest.mark.parametrize("speed", BOTH)
def test_device_ignores_sof(speed):
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    for fr in (0, 1, 0x2A, 0x7FF, 0x555):
        s0 = len(b.samples)
        send(b, speed, encode(sof(fr)))
        b.run(WAIT * B)
        assert len(packets(b.samples, speed, s0)) == 1, f"frame {fr}"
    s0 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    assert names(clean(b.samples[s0:], speed)) == ["IN", "NAK"]


@pytest.mark.parametrize("speed", BOTH)
def test_device_token_with_bad_crc5(speed):
    """8.3.5: a corrupted token is ignored."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP, bad_crc=True)))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 1
    assert "crc5-err" in [e[1] for e in sigrok.usb(b.samples[s0:], 0, 1, speed)["errors"]]


@pytest.mark.parametrize("speed", BOTH)
def test_device_data_with_bad_crc16(speed):
    """8.4.6: a corrupted data packet gets no handshake at all."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, [1, 2, 3], bad_crc=True)))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 2
    assert nd.peek(F.UD_EVENT)[0] != F.EV_OUT


@pytest.mark.parametrize("pid", (OUT ^ 0x10, IN ^ 0x40, SETUP ^ 0x01, 0xFF, 0x00), ids=lambda p: f"{p:02x}")
@pytest.mark.parametrize("speed", BOTH)
def test_device_ignores_a_bad_pid(speed, pid):
    """8.3.1: PID check bits that are not the complement mean a corrupted packet: it is ignored."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed, inlen=2, indata=[9])
    s0 = len(b.samples)
    send(b, speed, encode(token(pid, ADDR, EP)))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 1, "answered a packet with a bad PID"
    assert nd.peek(F.UD_EVENT)[0] == 0
    s1 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(60 * B)
    assert len(packets(b.samples, speed, s1)) >= 2, "no answer to the next good token"


@pytest.mark.parametrize("pid", (DATA0, DATA1, ACK, NAK, STALL), ids=("d0", "d1", "ack", "nak", "stall"))
@pytest.mark.parametrize("speed", BOTH)
def test_device_ignores_an_unexpected_packet(speed, pid):
    """A packet with no token before it is not a transaction."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(data_packet(pid, [1, 2]) if pid in (DATA0, DATA1) else [pid]))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 1, "answered it"
    s1 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    assert names(clean(b.samples[s1:], speed)) == ["IN", "NAK"]


@pytest.mark.parametrize("speed", BOTH)
def test_device_stuff_error(speed):
    """7.1.9.1: seven ones in a row: the packet is ignored."""
    B = SPEEDS[speed]["B"]
    d = [0xFF, 0xFF, 0x12]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, d), no_stuff_first=True))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 2, "answered a packet with a stuff error"
    s = sigrok.usb(b.samples[s0:], 0, 1, speed)
    assert any("stuff" in e[2].lower() for e in s["errors"]), "the decoder must see it too"


@pytest.mark.parametrize("pid", (DATA0 ^ 0x10, DATA1 ^ 0x01, 0xFF, 0x00, ACK), ids=lambda p: f"{p:02x}")
@pytest.mark.parametrize("speed", BOTH)
def test_device_data_with_a_bad_pid(speed, pid):
    """8.3.1 and 8.4.6: after an OUT token, a packet that is not a good DATA packet gets no handshake."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    send(b, speed, encode(data_packet(pid, [1, 2, 3])))
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 2, "answered it"
    assert nd.peek(F.UD_EVENT)[0] != F.EV_OUT


@pytest.mark.parametrize("gap", (18, 19, 20, 25))
@pytest.mark.parametrize("speed", BOTH)
def test_device_timeout_does_not_eat_the_next_token(speed, gap):
    """7.1.19.1: the host may send the next token 18 bit times after the data. By then the device has
    given up waiting for a handshake (before 18), so the token is answered."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed, inlen=2, indata=[9])
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) != 0, limit=40 * B, what="device starts")
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) == 0, limit=100 * B, what="device releases")
    b.run((gap - 1) * B)                         # its J bit is over; the rest of the gap
    s1 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(60 * B)
    assert len(packets(b.samples, speed, s1)) == 2, "the next token got no answer"


# ---------------------------------------------------------------- device: as a receiver
@pytest.mark.parametrize("sync", (8, 7, 6))
@pytest.mark.parametrize("speed", BOTH)
def test_device_short_sync(speed, sync):
    """8.2: a received sync may be shorter than 8 bits. Chapter 7 gives a figure for high speed only,
    so 6 here is a margin, not a limit of the standard."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP), sync_bits=sync))
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, [5, 6, 7]), sync_bits=sync))
    b.run(WAIT * B)
    assert nd.peek(RXD, 3) == [5, 6, 7] and len(packets(b.samples, speed, s0)) == 3


@pytest.mark.parametrize("pct", (-1.0, -0.5, 0.0, 0.5, 1.0))
@pytest.mark.parametrize("speed", BOTH)
def test_device_clock_tolerance(speed, pct):
    """7.1.11: a source may be off by 0.25% at full speed and 1.5% at low speed: `pct` of that."""
    B = SPEEDS[speed]["B"]
    bit = B * (1 + pct * U.LIMITS[speed]["rate_pct"] / 100)
    b, nd = device(speed)
    d = [0xFF, 0x00, 0x7F, 0xFE, 0xAA, 0x55, 0x3C, 0xC3]
    send(b, speed, encode(token(OUT, ADDR, EP)), bit=bit)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, d)), bit=bit)
    b.run(WAIT * B)
    assert nd.peek(RXD, 8) == d
    assert nd.peek(F.UD_EVENT)[0] == F.EV_OUT


@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize("speed", BOTH)
def test_device_jitter(speed, seed):
    """Tables 7-4 and 7-5, receiver budget to the next transition: FS 20 ns = 1 cycle, LS 141 ns = 6 cycles."""
    B = SPEEDS[speed]["B"]
    j = int(U.LIMITS[speed]["jitter_next"] // U.NS)
    b, nd = device(speed)
    r = random.Random(seed)
    d = payload(MAXLEN[speed], seed)
    send(b, speed, encode(token(OUT, ADDR, EP)), jitter=j, rnd=r)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, d)), jitter=j, rnd=r)
    b.run(WAIT * B)
    assert nd.peek(RXD, len(d)) == d and nd.peek(F.UD_EVENT)[0] == F.EV_OUT


def dribble_payload(speed):
    """A payload whose last CRC bits are five ones, so a dribble bit makes a sixth (Figure 7-33)."""
    r = random.Random(1)
    while True:
        d = [r.randint(0, 255) for _ in range(4)]
        c = U.crc16_bits(d)
        if c[-5:] == [1] * 5 and c[-6] == 0:
            return d


@pytest.mark.parametrize("speed", BOTH)
def test_device_accepts_dribble(speed):
    """7.1.9.1: 'the receiver must accept a packet for which there are up to six full bit times at the port
    with no transitions prior to the EOP'."""
    B = SPEEDS[speed]["B"]
    d = dribble_payload(speed)
    b, nd = device(speed)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, d), dribble=True))
    b.run(WAIT * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_OUT and nd.peek(RXD, len(d)) == d, "dribble refused"


@pytest.mark.parametrize("speed", BOTH)
def test_device_accepts_shortest_eop(speed):
    """7.1.13.2: a receiver accepts an SE0 of 82 ns at full speed, 670 ns at low speed."""
    B = SPEEDS[speed]["B"]
    c = int(-(-U.LIMITS[speed]["eop_rx"] // U.NS))
    b, nd = device(speed)
    send(b, speed, encode(token(OUT, ADDR, EP)), se0_cycles=c)
    b.run(2 * B)
    t = send(b, speed, encode(data_packet(DATA0, [1, 2, 3])), se0_cycles=c)
    b.run(WAIT * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_OUT, "short EOP not accepted"


@pytest.mark.parametrize("speed", BOTH)
def test_device_ignores_se0_in_transitions(speed):
    """7.1.4: D+ and D- may both be low in a differential transition, up to 14 ns at full speed and 210 ns at
    low speed: 'logic in the receiver must ensure that this is not interpreted as an SE0'."""
    B = SPEEDS[speed]["B"]
    g = int(U.LIMITS[speed]["se0_glitch"] // U.NS)
    if g < 1:
        pytest.skip("under one clock period: the input never shows it for a whole cycle")
    b, nd = device(speed)
    send(b, speed, encode(token(OUT, ADDR, EP)), glitch=g)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA0, [0xA5, 0x5A, 0x00])), glitch=g)
    b.run(WAIT * B)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_OUT and nd.peek(RXD, 3) == [0xA5, 0x5A, 0x00]


@pytest.mark.parametrize("gap", (2.0, 2.5, 3.0, 4.0, 8.0, 40.0))
@pytest.mark.parametrize("speed", BOTH)
def test_device_token_to_data_gap(speed, gap):
    """7.1.18.1: the host leaves 2 bit times or more between its packets (from SE0-to-J)."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    send(b, speed, encode(token(OUT, ADDR, EP)))
    b.run(int((gap - 1) * B))                  # the J bit of the EOP counts
    send(b, speed, encode(data_packet(DATA0, [1, 2])))
    b.run(WAIT * B)
    assert names(clean(b.samples[s0:], speed)) == ["OUT", "DATA0", "ACK"]


@pytest.mark.parametrize("speed", BOTH)
def test_device_back_to_back_transactions(speed):
    """After the device's answer, a host may start the next token 2 bit times later. Then an SOF and a token."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    s0 = len(b.samples)
    for k in range(4):
        send(b, speed, encode(token(OUT, ADDR, EP)))
        b.run(B)
        send(b, speed, encode(data_packet(DATA0, [k, 1])))
        b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) != 0, limit=40 * B, what="start of ACK")
        b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) == 0, limit=40 * B, what="end of ACK")
        b.run(B)
        send(b, speed, encode(sof(k)))
        b.run(B)
    assert names(clean(b.samples[s0:], speed)) == ["OUT", "DATA0", "ACK", "SOF"] * 4
    assert nd.peek(RXD, 2) == [3, 1]


@pytest.mark.parametrize("speed", BOTH)
def test_device_reset_detect(speed):
    """Table 7-14: SE0 of 2.5 us or more is a reset. An SE0 of an EOP is not."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    send(b, speed, [U.SE0, U.SE0, U.J])          # a keep-alive
    b.run(400)
    assert nd.peek(F.UD_EVENT)[0] == 0, "an EOP was taken for a reset"
    c = U.TDETRST_NS / U.NS
    send(b, speed, [U.SE0, U.J], se0_cycles=int(c))
    b.run(400)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_RESET
    s1 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    assert names(clean(b.samples[s1:], speed)) == ["IN", "NAK"]


@pytest.mark.parametrize("speed", BOTH)
def test_device_long_reset(speed):
    """A 10 ms reset, as hosts drive: reported, and the device is fine after it."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    send(b, speed, [U.SE0, U.J], se0_cycles=480000)
    b.run(400)
    assert nd.peek(F.UD_EVENT)[0] == F.EV_RESET
    s1 = len(b.samples)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    assert names(clean(b.samples[s1:], speed)) == ["IN", "NAK"]


@pytest.mark.parametrize("speed", BOTH)
def test_device_suspend_and_resume(speed):
    """7.1.7.6: 3 ms of idle bus means suspend (CSTAT.ETX twice); resume K, then normal work."""
    B = SPEEDS[speed]["B"]
    b, nd = device(speed)
    assert not nd.read(CSTAT) & 0x20
    b.run(int(0.003 * CLOCK_HZ))
    assert nd.read(CSTAT) & 0x20, "suspend not seen"
    s1 = len(b.samples)
    send(b, speed, [U.K, U.SE0, U.SE0, U.J], bit=1000 * B)     # resume K
    b.run(20 * B)
    send(b, speed, encode(token(IN, ADDR, EP)))
    b.run(WAIT * B)
    p = sigrok.usb(b.samples[s1:], 0, 1, speed)
    assert [x[1] for x in p["packets"]][-2:] == ["IN", "NAK"], p["packets"]


# ---------------------------------------------------------------- host
def htok(nd, t, dpid=0, data=(), op=0):
    nd.dmem(0, [op] + list(t) + [dpid, len(data)])
    nd.dmem(0x10, list(data))
    nd.doorbell()


def hwait(b, nd, limit=300000):
    b.until(lambda: nd.chip.regs.st2 & 0x10, limit=limit, what="host firmware")
    nd.write(0x0E, 0x10)


def wait_oe_off(b, nd, B, limit):
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) != 0, limit=limit, what="host starts")
    b.until(lambda: nd.chip.pins.pad_oe(nd.chip.regs.od) == 0, limit=limit, what="host releases")


@pytest.mark.parametrize("n", [0, 1, 8, 9, 32, 63, 64])
@pytest.mark.parametrize("speed", BOTH)
def test_host_out(speed, n):
    if n > MAXLEN[speed]:
        pytest.skip("over the maximum packet size")
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    d = payload(n, 3)
    s0 = len(b.samples)
    htok(nd, token(OUT, ADDR, EP), DATA1, d)
    lim = (n + 12) * 14 * B
    wait_oe_off(b, nd, B, lim)                      # token
    t1 = U.last_eop_end(b.samples, speed)
    wait_oe_off(b, nd, B, lim)                      # data
    t2 = U.last_eop_end(b.samples, speed)
    b.run(int(3 * B))
    send(b, speed, encode([ACK]))
    hwait(b, nd)
    assert nd.peek(F.UH_RPID)[0] == ACK and nd.peek(F.UH_STATUS)[0] == F.UH_OK
    x = clean(b.samples[s0:], speed)
    assert names(x) == ["OUT", "DATA1", "ACK"], names(x)
    gap = bits_after(b.samples, speed, t1)
    assert U.TIPD <= gap < U.TIMEOUT[0], f"token to data: {gap} bit times (the device waits for 16)"
    check_eops(b.samples[:t2 + 8 * B], speed, s0)
    assert not b.contention


@pytest.mark.parametrize("turn", (2.0, 3.0, 5.0, 6.5, 7.5))
@pytest.mark.parametrize("n", (0, 1, 8))
@pytest.mark.parametrize("speed", BOTH)
def test_host_in(speed, n, turn):
    """Every device response time from 2 to 7.5 bit times; the host's ACK comes within 7.5 (7.1.18.1)."""
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    d = payload(n, 4)
    s0 = len(b.samples)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(int((turn - 1) * B))
    t = send(b, speed, encode(data_packet(DATA1, d)))
    hwait(b, nd, limit=40 * B + 2000)
    b.run(4 * B)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_OK and nd.peek(F.UH_RPID)[0] == DATA1
    assert nd.peek(0x81, n) == d
    x = clean(b.samples[s0:], speed)
    assert names(x) == ["IN", "DATA1", "ACK"], names(x)
    g = bits_after(b.samples, speed, t)
    assert g is not None and U.TIPD <= g <= U.TRSPIPD2, f"the host's ACK after {g} bit times"
    assert not b.contention


@pytest.mark.parametrize("speed", BOTH)
def test_host_in_all_lengths_and_a_stuff_bit_at_the_end(speed):
    B = SPEEDS[speed]["B"]
    for n in list(range(0, MAXLEN[speed] + 1, 1 if speed == "LS" else 8)):
        for d in (payload(n, 1), [0xFF] * n):
            b, nd = host(speed)
            htok(nd, token(IN, ADDR, EP))
            wait_oe_off(b, nd, B, 60 * B)
            b.run(2 * B)
            send(b, speed, encode(data_packet(DATA0, d)))
            hwait(b, nd, limit=40 * B + 2000)
            assert nd.peek(F.UH_STATUS)[0] == F.UH_OK and nd.peek(0x81, n) == d


@pytest.mark.parametrize("speed", BOTH)
def test_host_timeout(speed):
    """7.1.19.1: no time-out before 16 bit times, one before 18; a new transaction no earlier than 18."""
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    t = U.last_eop_end(b.samples, speed)
    b.until(lambda: nd.chip.rx.busy, limit=40 * B, what="listening")
    b.until(lambda: not nd.chip.rx.busy, limit=400 * B, what="time-out")     # it stops listening
    at = (len(b.samples) - t) / B
    hwait(b, nd, limit=4000)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_TIMEOUT
    assert U.TIMEOUT[0] <= at, f"timed out after {at:.1f} bit times: under 16"
    assert at <= U.TIMEOUT[1], f"timed out after {at:.1f} bit times: over 18"


@pytest.mark.parametrize("speed", BOTH)
def test_host_accepts_a_late_answer(speed):
    """The answer may start up to 16 bit times after the token, as seen at the host."""
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(15 * B - 2)                 # send() takes 2 cycles to its first K: it starts at 16 bit times
    send(b, speed, encode(data_packet(DATA1, [1, 2])))
    hwait(b, nd, limit=60 * B + 2000)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_OK and nd.peek(0x81, 2) == [1, 2]


@pytest.mark.parametrize("speed", BOTH)
def test_host_bad_crc_no_ack(speed):
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    s0 = len(b.samples)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA1, [1, 2, 3], bad_crc=True)))
    hwait(b, nd, limit=40 * B + 2000)
    b.run(WAIT * B)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_BADPKT
    assert len(packets(b.samples, speed, s0)) == 2, "ACK to a corrupted packet"


@pytest.mark.parametrize("pid", (DATA1 ^ 0x10, 0xFF), ids=("flip", "ff"))
@pytest.mark.parametrize("speed", BOTH)
def test_host_bad_pid_no_ack(speed, pid):
    """8.3.1: a packet whose PID check fails is corrupted: no ACK, and the status says so."""
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    s0 = len(b.samples)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(2 * B)
    send(b, speed, encode(data_packet(pid, [1, 2, 3])))
    hwait(b, nd, limit=40 * B + 2000)
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 2, "ACK to a packet with a bad PID"
    assert nd.peek(F.UH_STATUS)[0] == F.UH_BADPKT


@pytest.mark.parametrize("speed", BOTH)
def test_host_stuff_error_no_ack(speed):
    B = SPEEDS[speed]["B"]
    b, nd = host(speed)
    s0 = len(b.samples)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA1, [0xFF, 0xFF, 0x12]), no_stuff_first=True))
    hwait(b, nd, limit=40 * B + 2000)
    b.run(WAIT * B)
    assert len(packets(b.samples, speed, s0)) == 2
    assert nd.peek(F.UH_STATUS)[0] == F.UH_BADPKT


@pytest.mark.parametrize("speed", BOTH)
def test_host_accepts_dribble(speed):
    B = SPEEDS[speed]["B"]
    d = dribble_payload(speed)
    b, nd = host(speed)
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA1, d), dribble=True))
    hwait(b, nd, limit=40 * B + 2000)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_OK, "dribble refused"
    assert nd.peek(0x81, len(d)) == d


@pytest.mark.parametrize("speed", BOTH)
def test_host_ignores_se0_in_transitions(speed):
    B = SPEEDS[speed]["B"]
    g = int(U.LIMITS[speed]["se0_glitch"] // U.NS)
    if g < 1:
        pytest.skip("under one clock period")
    b, nd = host(speed)
    d = [0xA5, 0x5A, 0x00]
    htok(nd, token(IN, ADDR, EP))
    wait_oe_off(b, nd, B, 60 * B)
    b.run(2 * B)
    send(b, speed, encode(data_packet(DATA1, d)), glitch=g)
    hwait(b, nd, limit=40 * B + 2000)
    assert nd.peek(F.UH_STATUS)[0] == F.UH_OK and nd.peek(0x81, 3) == d


@pytest.mark.parametrize("speed", BOTH)
def test_host_reset_is_long_enough(speed):
    """Table 7-13: a host drives reset for at least 10 ms."""
    b, nd = host(speed)
    want = U.TDRST_MS * CLOCK_HZ // 1000
    n = want // 256 + 1
    s0 = len(b.samples)
    nd.dmem(0, [1, 0, 0, 0, 0, n & 0xFF, n >> 8])
    nd.doorbell()
    hwait(b, nd, limit=want * 2)
    b.run(200)
    p = sigrok.usb(b.samples[s0:], 0, 1, speed)
    assert p["reset"], "the decoder saw no reset"
    jp, kp = SPEEDS[speed]["jp"], SPEEDS[speed]["kp"]
    se0 = sum(1 for x in b.samples[s0:] if U.level_at(x, jp, kp) == U.SE0)
    assert se0 >= want, f"reset of {se0 * U.NS / 1e6:.2f} ms"


@pytest.mark.parametrize("speed", ("LS",))
def test_host_keep_alive(speed):
    """Table 7-2: a keep-alive is a low-speed EOP: SE0 for 1.25 to 1.5 us, then J."""
    b, nd = host(speed)
    s0 = len(b.samples)
    nd.dmem(0, [2])
    nd.doorbell()
    hwait(b, nd, limit=4000)
    b.run(100)
    p = sigrok.usb(b.samples[s0:], 0, 1, speed)
    assert p["keep"], "the decoder saw no keep-alive"
    assert not p["errors"], p["errors"]
    jp, kp = SPEEDS[speed]["jp"], SPEEDS[speed]["kp"]
    w = U.ns(sum(1 for x in b.samples[s0:] if U.level_at(x, jp, kp) == U.SE0))
    lo, hi = U.LIMITS[speed]["eop"]
    assert lo <= w <= hi, f"keep-alive SE0 {w:.0f} ns"


@pytest.mark.parametrize("speed", BOTH)
def test_host_resume_is_long_enough(speed):
    """Table 7-13 and 7.1.7.7: resume: K for at least 20 ms, ended by a low-speed EOP, with no SE0 glitch."""
    b, nd = host(speed)
    want = U.TDRSMDN_MS * CLOCK_HZ // 1000
    n = want // 256 + 1
    s0 = len(b.samples)
    nd.dmem(0, [3, 0, 0, 0, 0, n & 0xFF, n >> 8])
    nd.doorbell()
    hwait(b, nd, limit=want * 2)
    b.run(300)
    jp, kp = SPEEDS[speed]["jp"], SPEEDS[speed]["kp"]
    lv = [U.level_at(x, jp, kp) for x in b.samples[s0:]]
    k = lv.count(U.K)
    assert k >= want, f"K for {k * U.NS / 1e6:.2f} ms"
    first_k = lv.index(U.K)
    assert U.SE0 not in lv[first_k:first_k + 100], "SE0 glitch going into K"


# ---------------------------------------------------------------- both ends are chips
@pytest.mark.parametrize("speed", BOTH)
def test_chip_to_chip_control_transfer(speed):
    """GET_DESCRIPTOR: SETUP stage, IN data stage, OUT status stage, with the chip at both ends.
    The sigrok usb_request decoder must read it as a device-to-host request."""
    sp = SPEEDS[speed]
    B = sp["B"]
    b = U.bench(speed, 2)
    h, d = b.nodes
    own = owners(**{f"p{sp['jp']}": OWN_TX, f"p{sp['kp']}": OWN_TX})
    desc = [0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08]
    d.dmem(F.UD_ADDR, [ADDR, 0, 0, 0, len(desc) + 1, DATA1, ACK, NAK, 0])
    d.dmem(F.UD_INDATA, desc)
    d.load(F.usb_device(B=B, jp=sp["jp"], kp=sp["kp"]), own=own)
    h.load(F.usb_host(B=B, jp=sp["jp"], kp=sp["kp"]), own=own)
    b.run(200)
    s0 = len(b.samples)
    setup = [0x80, 6, 0, 1, 0, 0, 8, 0]
    htok(h, token(SETUP, ADDR, 0), DATA0, setup)
    hwait(b, h, limit=40000 * (B // 4))
    assert h.peek(F.UH_RPID)[0] == ACK and h.peek(F.UH_STATUS)[0] == F.UH_OK
    assert d.peek(RXD, 8) == setup
    b.run(40 * B)
    htok(h, token(IN, ADDR, 0))
    hwait(b, h, limit=40000 * (B // 4))
    assert h.peek(F.UH_STATUS)[0] == F.UH_OK and h.peek(F.UH_RPID)[0] == DATA1
    assert h.peek(0x81, 8) == desc
    b.run(60 * B)
    assert d.peek(F.UD_EVENT)[0] == F.EV_IN_ACK
    d.dmem(F.UD_EVENT, [0])
    htok(h, token(OUT, ADDR, 0), DATA1, [])
    hwait(b, h, limit=40000 * (B // 4))
    assert h.peek(F.UH_RPID)[0] == ACK
    b.run(40 * B)
    s = b.samples[s0:]
    x = clean(s, speed)
    assert names(x) == ["SETUP", "DATA0", "ACK", "IN", "DATA1", "ACK", "OUT", "DATA1", "ACK"], names(x)
    assert not b.contention
    sp2 = "full-speed" if speed == "FS" else "low-speed"
    r = sigrok.run(s, f"usb_signalling:dp=0:dm=1:signalling={sp2},usb_packet:signalling={sp2},usb_request",
                   {"usb_request": ["request-setup-read", "errors"]})
    assert [a for a in r if a.cls == "request-setup-read"], f"the request decoder saw no request: {r[:4]}"
    assert not [a for a in r if a.cls == "errors"], r[:4]
