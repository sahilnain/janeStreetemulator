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
