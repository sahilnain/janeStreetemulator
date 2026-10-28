# SPDX-License-Identifier: Apache-2.0
"""
pe_fw: test firmware for the Protocol Emulator (UART TX / RX).

What this file does
-------------------
Builds instruction lists (via pe_isa) for the UART programs used by the
pe_core unit test and the top-level test, and does the cycle arithmetic the
future assembler will do.

Timing model (from pe_core.v / pe_ctx.v, NUM_SLOTS = 2):
  An instruction that loads a stall of d cycles (delay field + `delay n`)
  occupies cost(d) = NUM_SLOTS * ceil((d + 1) / NUM_SLOTS) cycles.
  So `delay c-1` costs exactly c cycles when c is a multiple of NUM_SLOTS.

UART TX (LSB first, bit period B cycles, B even, B >= 12):
  Every edge on the line is exactly B cycles after the previous one, and a
  back-to-back frame is exactly 10*B cycles.

UART RX (oversample + vote; bit period B_rx = N*S + 4):
  wait for the start edge -> vote the start bit (glitch -> restart) ->
  8 x (vote, djnz) -> wait to the middle of the stop bit -> jpin checks the
  stop bit (framing error -> drop the byte, wait for idle) -> push.
"""

import pe_isa as I

NUM_SLOTS = 2


def delay_for(cycles):
    """`delay` instruction that occupies exactly `cycles` cycles."""
    assert cycles >= NUM_SLOTS and cycles % NUM_SLOTS == 0, cycles
    return I.delay(cycles - 1)


def uart_tx(pin, B, base=0):
    """UART TX on `pin`, bit period B cycles. Bytes come from the TX FIFO."""
    assert B % 2 == 0 and B >= 12
    L0 = base + 2
    BIT = base + 6
    return [
        I.set_(pin, I.SRC_1),         # base+0: idle high
        I.setoe(pin, 1),              # base+1: drive the line
        I.pull(),                     # L0: wait for a byte
        I.set_(pin, I.SRC_0),         # start bit edge
        I.mov(0, 8),
        delay_for(B - 4),             # set + mov + delay = B
        I.out(pin, 1),                # BIT: data bit edge
        delay_for(B - 4),             # out + delay + djnz = B
        I.djnz(0, BIT),
        I.set_(pin, I.SRC_1),         # stop bit edge
        delay_for(B - 6),             # set + delay + jmp + pull = B
        I.jmp(L0),
    ]


def uart_rx(pin, N, S, base=0):
    """UART RX on `pin` with the vote unit (N samples, S cycles each).
    Matching bit period: N*S + 4 cycles."""
    B = N * S + 4
    R0 = base + 0
    BIT = base + 4
    FERR = base + 10
    mid = max(2, (B // 2 - 4) // 2 * 2)
    return [
        I.mov(0, 8),                  # R0
        I.wait(pin, 0),               # start edge
        I.vote(pin),                  # start bit
        I.jmp(R0, I.C),               # voted 1 -> glitch, restart
        I.vote(pin),                  # BIT: data bit -> ISR
        I.djnz(0, BIT),
        delay_for(mid),               # to the middle of the stop bit
        I.jpin(pin, 0, FERR),         # stop bit low -> framing error
        I.push(),
        I.jmp(R0),
        I.wait(pin, 1),               # FERR: wait for idle, drop the byte
        I.jmp(R0),
    ]


def rx_bit_period(N, S):
    return N * S + 4
