# SPDX-License-Identifier: Apache-2.0
"""
uart: UART line helpers for tests.

What this file does
-------------------
* `decode(samples, B)`: turns a per-clock-cycle list of line levels into
  bytes. It also checks that every edge lies on the bit grid, so TX timing
  must be cycle-exact: an edge off the grid raises AssertionError.
* `frame_bits(byte, stop=1)`: start + 8 data bits (LSB first) + stop, used
  to drive a line from the test (including bad stop bits).
"""


def frame_ex(word, wbits, stop_bits=1, idle=1):
    """start + wbits data LSB first + stop_bits stop (levels, before inversion)."""
    return [1 - idle] + [(word >> i) & 1 for i in range(wbits)] + [idle] * stop_bits


def parity_of(data, parity):
    """Parity bit value; parity is pe_fw.PAR_* (1 even, 2 odd, 3 mark, 4 space)."""
    odd = bin(data).count("1") & 1
    return {1: odd, 2: 1 - odd, 3: 1, 4: 0}[parity]


def decode_ex(samples, B, wbits, stop_b2=2, inv=0, msb=0, start=None):
    """Decode frames of wbits bits. Every edge inside a frame must be on the bit
    grid, and the line must stay idle for at least the stop time between frames.
    start: length of the start bit, if not B (pe_fw.uart_ex with an odd B)."""
    start = B if start is None else start
    idle = 0 if inv else 1
    stop = B * stop_b2 // 2
    out = []
    i, n = 1, len(samples)
    while i < n:
        if samples[i - 1] == idle and samples[i] != idle:
            s = i
            end = s + start + wbits * B
            if end + stop > n:
                break
            for t in range(s + 1, end):
                if samples[t] != samples[t - 1]:
                    assert (t - s - start) % B == 0, f"edge at {t - s} cycles into the frame, B={B}"
            for t in range(end, end + stop):
                assert samples[t] == idle, f"stop time broken at {t - end} after the data, stop={stop}"
            bits = [samples[s + start + B // 2 + k * B] ^ inv for k in range(wbits)]
            word = 0
            for k, b in enumerate(bits):
                word |= b << (wbits - 1 - k if msb else k)
            out.append(word)
            i = end + stop
        else:
            i += 1
    return out


def frame_bits(byte, stop=1):
    return [0] + [(byte >> i) & 1 for i in range(8)] + [stop]


def decode(samples, B, exact=True):
    """Decode 8N1 frames from per-cycle samples (idle high)."""
    out = []
    i = 1
    n = len(samples)
    while i < n:
        if samples[i - 1] == 1 and samples[i] == 0:
            s = i
            if s + 10 * B > n:
                break
            if exact:
                # The line may only change on the bit grid s + m*B.
                for t in range(s + 1, s + 10 * B):
                    if samples[t] != samples[t - 1]:
                        assert (t - s) % B == 0, f"edge at {t - s} cycles into frame, B={B}"
            bits = [samples[s + B // 2 + k * B] for k in range(10)]
            assert bits[0] == 0, "start bit"
            assert bits[9] == 1, "stop bit"
            out.append(sum(b << k for k, b in enumerate(bits[1:9])))
            i = s + 9 * B + B // 2
        else:
            i += 1
    return out


async def drive_bits(chip, pin, bits, B, glitch=None):
    """Drive `bits` (one level per bit, B cycles each) on `pin` through chip.drive.
    glitch: optional (offset, length) inverting each bit for `length` cycles
    starting `offset` cycles into the bit (noise test)."""
    from cocotb.triggers import FallingEdge
    for b in bits:
        for t in range(B):
            lv = b
            if glitch and glitch[0] <= t < glitch[0] + glitch[1]:
                lv ^= 1
            chip.drive[pin] = lv
            await FallingEdge(chip.dut.clk)


def stream_bits(data, stop=1, idle=0, bad=()):
    """Frames for every byte in data, each followed by `idle` extra idle bits;
    bytes whose index is in `bad` get a 0 stop bit."""
    bits = []
    for i, d in enumerate(data):
        bits += frame_bits(d, stop=0 if i in bad else stop) + [1] * idle
    return bits
