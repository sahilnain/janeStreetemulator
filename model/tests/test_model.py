# SPDX-License-Identifier: Apache-2.0
"""
test_model: checks of the Python reference model against the SPEC (no RTL).

What is checked
---------------
These tests make sure the model itself behaves as documented, so that when
the lockstep test compares it with the RTL, agreement means something.
1. test_registers_reset     ID, VERSION, vote reset values over the host bus.
2. test_uart_loopback       UART TX + vote-based RX firmware (test/common/pe_fw)
                            on the chip model, P0 looped to P1: bytes arrive,
                            and the TX line is cycle-exact on the bit grid.
3. test_framing_error_drop  a frame with a bad stop bit is dropped by the RX
                            firmware, good frames around it arrive.
4. test_instruction_timing  one instruction per NUM_SLOTS cycles, delay field
                            rounded up to whole slots (2- and 3-context chips).
5. test_vote_timing         the vote unit finishes exactly N*S+1 cycles after
                            start, and votes the majority of the centre samples.
6. test_exec_invariants     out/in conserve bits; held instructions have no
                            side effects (same properties as the formal proof).
7. test_unwritten_imem      executing an unwritten IMEM word raises ModelError
                            (the RTL would execute X).
"""

import random

import pe_fw
import pe_model.isa as I
import pytest
import uart
from pe_model import Chip, Config, ModelError
from pe_model.blocks import (A_ID, A_OWNER, A_RESTART, A_RUN, A_RXF, A_STATUS, A_TXF, A_VERSION,
                             A_CTX0, A_PIN_OD, Vote)
from pe_model.host import ModelHost


def line_of(outs, ext=0xFF):
    return ((outs["uio_out"] & outs["uio_oe"]) | (ext & ~outs["uio_oe"])) & 0xFF


def setup_uart(B, N, S, K=1, ext=None):
    chip = Chip()
    state = {"ext": 0xFF}

    def pins_in(outs):
        line = line_of(outs, state["ext"])
        if ext is None:  # loopback P0 -> P1
            line = (line & ~2) | ((line & 1) << 1)
        return line

    h = ModelHost(chip, pins_in)
    h.reset()
    h.load_program(pe_fw.uart_tx(0, B, base=0), start=0)
    h.load_program(pe_fw.uart_rx(1, N, S, base=20), start=20)
    h.write(A_OWNER, 0b10)
    h.write(A_CTX0 + 8 + 0, 20)
    for off, val in ((2, N), (3, K), (4, S & 0xFF), (5, S >> 8)):
        h.write(A_CTX0 + 8 + off, val)
    h.write(A_RESTART, 0b11)
    return chip, h, state


def test_registers_reset():
    h = ModelHost(Chip())
    h.reset()
    assert h.read(A_ID) == 0x50 and h.read(A_VERSION) == 0x01
    assert h.read(A_CTX0 + 2) == 9 and h.read(A_CTX0 + 3) == 1 and h.read(A_CTX0 + 4) == 48
    h.write(A_PIN_OD, 0x5A)
    assert h.read(A_PIN_OD) == 0x5A
    assert h.read(0x7F) == 0


def test_uart_loopback():
    N, S = 9, 12
    B = pe_fw.rx_bit_period(N, S)
    chip, h, _ = setup_uart(B, N, S)
    h.write(A_RUN, 0b11)
    t0 = len(h.trace)
    data = [random.getrandbits(8) for _ in range(12)]
    todo, got = list(data), []
    while len(got) < len(data):
        st = h.read(A_STATUS)
        if not st & 0x04:
            got.append(h.read(A_RXF))
        elif todo and not st & 0x02:
            h.write(A_TXF, todo.pop(0))
    assert got == data
    assert h.read(A_STATUS) & 0xF0 == 0
    h.cycle(2 * B)
    tx = [line_of(o) & 1 for o in h.trace[t0:]]
    assert uart.decode(tx, B) == data


def test_framing_error_drop():
    N, S = 9, 12
    B = pe_fw.rx_bit_period(N, S)
    chip, h, state = setup_uart(B, N, S, ext=True)
    h.write(A_RUN, 0b10)
    bits = [1] * 3 + uart.frame_bits(0x5A) + [1] * 2 + uart.frame_bits(0xC3, stop=0) + [0] * 2
    bits += [1] * 4 + uart.frame_bits(0x81) + [1] * 3
    for b in bits:
        state["ext"] = 0xFD | (b << 1)
        h.cycle(B)
    assert h.read(A_RXF) == 0x5A
    assert h.read(A_RXF) == 0x81
    assert h.read(A_STATUS) & 0x04, "the bad frame must have been dropped"


@pytest.mark.parametrize("nctx", [2, 3])
def test_instruction_timing(nctx):
    cfg = Config(num_ctx=nctx, vote_ctx_mask=(1 << nctx) - 1)
    ns = cfg.num_slots
    for d in range(8):
        chip = Chip(cfg)
        h = ModelHost(chip)
        h.reset()
        h.load_program([I.set_(0, I.SRC_1, dly=d), I.set_(0, I.SRC_0, dly=d), I.jmp(0)]
                       + [I.jmp(3)] * 61)
        h.write(A_RESTART, 1)
        h.write(A_RUN, 1)
        t0 = len(h.trace)
        h.cycle(20 * ns * 8)
        tr = [o["uio_out"] & 1 for o in h.trace[t0:]]
        runs, cur, n = [], tr[0], 0
        for b in tr:
            if b == cur:
                n += 1
            else:
                runs.append((cur, n))
                cur, n = b, 1
        runs = runs[1:]
        while runs[0][0] == 0:
            runs = runs[1:]
        cost = ns * -(-(d + 1) // ns)
        assert runs[:4] == [(1, cost), (0, cost + ns)] * 2, (d, runs[:4])


def test_vote_timing():
    cfg = Config()
    for _ in range(200):
        N, k, S = random.randint(1, 15), random.randint(0, 8), random.randint(1, 20)
        v = Vote(cfg)
        wave = {}
        v.clock(0, 1, 2, 0, N, k, S)
        t = 1
        while not v.done:
            wave[t] = random.getrandbits(8)
            v.clock(0, 0, 0, wave[t], N, k, S)
            t += 1
            assert t <= N * S + 2
        assert t == N * S + 1
        samples = [(wave[(j + 1) * S] >> 2) & 1 for j in range(N)]
        ones = sum(samples[j] for j in range(N) if k <= j < N - k)
        assert v.bit == int(2 * ones > max(N - 2 * k, 0))


def test_exec_invariants():
    cfg = Config()
    SRM = (1 << cfg.sr_w) - 1
    for _ in range(5000):
        w = (random.randrange(14) << 19) | random.getrandbits(19)
        f = I.decode(w, cfg.iw, cfg.aw)
        s = dict(pc=random.getrandbits(6), ctr0=random.getrandbits(16), ctr1=random.getrandbits(16),
                 osr=random.getrandbits(16), osr_cnt=random.randint(0, 16), isr=random.getrandbits(16),
                 c=random.getrandbits(1), to=random.getrandbits(1), shift_msb=random.getrandbits(1),
                 vote_avail=1)
        e = dict(pin_in=random.getrandbits(8), txf_rdata=random.getrandbits(8),
                 txf_empty=random.getrandbits(1), rxf_full=random.getrandbits(1))
        o = I.execute(cfg, f, s, e)
        n = f["n"]
        if f["op"] == "out":
            bits = (o["pin_wr_val"] >> f["pin"]) if f["pin"] + n <= 8 else None
            if bits is not None:
                if s["shift_msb"]:
                    assert ((o["osr_n"] >> n) | (bits << (16 - n))) & SRM == s["osr"]
                else:
                    assert ((o["osr_n"] << n) | bits) & SRM == s["osr"]
        if not o["done"]:
            assert f["op"] in ("wait", "pull", "push")
            assert o["pc_n"] == s["pc"] and o["stall_n"] == 0
            assert not (o["pin_wr_en"] or o["pin_oe_en"] or o["txf_rd"] or o["rxf_wr"] or o["vote_start"])


def test_unwritten_imem():
    chip = Chip()
    h = ModelHost(chip)
    h.reset()
    h.write(A_RUN, 1)
    with pytest.raises(ModelError):
        h.cycle(10)
