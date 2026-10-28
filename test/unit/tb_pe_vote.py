# SPDX-License-Identifier: Apache-2.0
"""
tb_pe_vote: unit test for src/core/pe_vote.v (oversample + majority vote).

What is checked (cycle-exact, against the timing in the pe_vote.v header)
---------------------------------------------------------------------------
Let edge R0 be the clock edge where `start` is seen. Then:
  * sample j (j = 0..N-1) is the selected pin's value at edge R_{(j+1)*S};
  * only samples with k <= j < N-k are counted;
  * done = 1 for exactly one cycle, right after edge R_{N*S};
    bit_out = ones > (N - 2k) / 2;
  * busy = 1 from after R0 up to and including the done cycle, then 0.
1. random_votes: 300 votes with random N, k, S, pin and random (biased)
   pin waveforms. Includes out-of-range pin selects (they read 0) and
   2k >= N (empty window, so the result is 0).
2. start_ignored_while_busy: a second start during a vote does not restart it.
3. uart_like: N=9, k=1, S=12 on clean levels 0 and 1, plus a vote with
   glitches on the edge samples only (k=1 ignores them).
"""

import random

import cocotb
from tbutil import fall, param, start, v


async def init(dut):
    for s in ("start", "pin_sel", "pin_in", "n_cfg", "k_cfg", "s_cfg"):
        getattr(dut, s).value = 0
    await start(dut)


async def run_vote(dut, N, k, S, sel, wave):
    """wave(m) -> pin_in vector driven before edge R_m (m >= 0). Returns (bit, checks ok)."""
    NP = param("NUM_PINS", 8)
    dut.n_cfg.value, dut.k_cfg.value, dut.s_cfg.value, dut.pin_sel.value = N, k, S, sel
    dut.start.value = 1
    vals = {0: wave(0)}
    dut.pin_in.value = vals[0]
    await fall(dut)  # after R0
    dut.start.value = 0
    total = N * S
    m = 1
    while True:
        # Now just after edge R_{m-1}.
        done, busy = v(dut.done), v(dut.busy)
        if m - 1 == total:
            assert done == 1, "done missing"
            assert busy == 1, "busy must cover the done cycle"
            got = v(dut.bit_out)
            break
        assert done == 0, f"early done at R{m - 1}"
        assert busy == 1, f"busy dropped at R{m - 1}"
        vals[m] = wave(m)
        dut.pin_in.value = vals[m]
        await fall(dut)
        m += 1
    await fall(dut)
    assert v(dut.busy) == 0 and v(dut.done) == 0
    samples = [((vals[(j + 1) * S] >> sel) & 1) if sel < NP else 0 for j in range(N)]
    ones = sum(samples[j] for j in range(N) if k <= j < N - k)
    exp = int(2 * ones > max(N - 2 * k, 0))
    assert got == exp, f"N={N} k={k} S={S} sel={sel} samples={samples} got={got}"
    return got


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def random_votes(dut):
    await init(dut)
    NP = param("NUM_PINS", 8)
    nmax = (1 << param("N_W", 4)) - 1
    smax = min((1 << param("S_W", 12)) - 1, 20)
    for _ in range(300):
        N = random.randint(1, nmax)
        k = random.randint(0, N // 2 + 1)
        S = random.randint(1, smax)
        sel = random.randrange(min(NP + 2, 16)) if random.random() < 0.1 else random.randrange(NP)
        p1 = random.random()
        await run_vote(dut, N, k, S, sel,
                       lambda m: sum((random.random() < p1) << i for i in range(NP)))


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def start_ignored_while_busy(dut):
    await init(dut)
    N, S = 5, 4
    dut.n_cfg.value, dut.k_cfg.value, dut.s_cfg.value, dut.pin_sel.value = N, 0, S, 0
    dut.pin_in.value = 1
    dut.start.value = 1
    await fall(dut)
    dut.start.value = 0
    await fall(dut, 3)
    dut.start.value = 1  # must be ignored
    await fall(dut)
    dut.start.value = 0
    cycles = 4
    while v(dut.done) == 0:
        await fall(dut)
        cycles += 1
        assert cycles <= N * S, "vote restarted"
    assert cycles == N * S and v(dut.bit_out) == 1


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def uart_like(dut):
    await init(dut)
    N, k, S = 9, 1, 12
    assert await run_vote(dut, N, k, S, 3, lambda m: 0xFF) == 1
    assert await run_vote(dut, N, k, S, 3, lambda m: 0x00) == 0
    # level 1 with glitches only on samples 0 and N-1 (ignored with k = 1)
    glitch = {S * 1, S * N}
    assert await run_vote(dut, N, k, S, 3, lambda m: 0x00 if m in glitch else 0xFF) == 1
