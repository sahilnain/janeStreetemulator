# Future improvements

Ideas that are not part of the current plan, kept so they can come back later.

## 1. Time-sliced contexts (barrel scheduling) to save area

### What it is
This is the architecture of the first RTL version (v1, verified; see `docs/modules.md` and `docs/architecture.html`).
- One shared fetch/decode/execute datapath and a single instruction-memory read port serve all contexts.
- A slot counter gives each context a fixed turn: NUM_SLOTS = max(2, NUM_CTX).
- The memory read is registered, so the core fetches one slot ahead: `im_raddr` = PC of the next slot's context.
- Each context (`pe_ctx`) holds only state: PC, ctr0/ctr1, OSR/ISR, osr_cnt, flags, stall counter.
- Only the slot owner's write-enable is set, so only that context latches the result. Pin writes are tagged with the context.

### Why it is attractive
- **Area:** a new protocol costs only one more `pe_ctx` (about 140 flip-flops) and a vote unit, not a whole engine. The v1 core with 2 contexts is 283 flip-flops.
- **Exact timing:** every context gets the same fixed share of cycles whether the others are busy or stalled. Firmware delay constants never depend on what other contexts are doing.
- **Scales by parameter:** a NUM_CTX = 3 configuration (UART + SPI + I2C on 8 pins) passed the `pe_regs` equivalence proof. The slot logic and fairness are formally proved (`formal/pe_core.sby`).

### Why it was set aside
Per-context speed is `clk / NUM_SLOTS`, at best one instruction every 2 cycles (25 MIPS at 50 MHz), and slower with each added context:

| Protocol | Time-sliced, 2 contexts | Time-sliced, 3 contexts |
|---|---|---|
| SPI master (with side-set + xfer) | 12.5 MHz SCK | about 8 MHz |
| SPI master (current ISA) | about 5 MHz | about 3 MHz |
| SPI slave (polling) | about 2-3 MHz | lower |
| I2C (all modes up to 1 MHz) | fine | fine |

The current goal is maximum speed for every protocol regardless of area. That needs one instruction per cycle for each protocol (see `hw_plan.md`).

### When to bring it back
- If the design does not fit the tile budget, or a smaller tile count is wanted.
- As a hybrid: fast engines for SPI and UART RX, and one time-sliced engine shared by slow protocols (I2C, UART TX at low baud, GPIO-style tasks).
- To support more protocols than there are engines (JTAG, SWD, PS/2, CAN), when they are slow enough to share.

### What can be reused
- `src/core/pe_core.v` slot scheduler, with its formal properties (slot sequence, attribution, bounded service).
- The `pe_ctx`, `pe_decode` and `pe_exec` split: the same per-context state, decode and execute blocks drop into a per-engine design unchanged.
- The Python model (`model/pe_model/core.py`), the lockstep test and the equivalence harness all take NUM_CTX as a parameter.

## 2. Complex instructions to shrink IMEM

### The idea
IMEM and DMEM are most of the chip: after the `dly` removal, IMEM (162 × 29) is about 415,000 µm² and DMEM (256 × 8) about 177,000 µm², of about 721,000 µm² in all. IMEM costs about **2,600 µm² per word**, and its depth is set by the largest program (the full-speed USB device, 162 words). Instructions that do the work of two common pairs would make programs shorter, so IMEM could be shallower. The decoder and the controller get more complex, but they are small (`pe_ctrl` with `pe_decode` is about 32,000 µm², 4% of the chip), so a little more logic there can pay for many IMEM words.

### Where the words go (counted over the firmware, 2026-11-15)
| Pair | All 15 programs | USB device (162) | USB host (159) | I2C multi-master (155) |
|---|---|---|---|---|
| `cmp` + `jmp` (compare and branch) | 61 | 14 | 11 | 9 |
| `mov rB, #addr` + `ld`/`st` (fixed DMEM address) | 34 | 8 | 7 | 2 |
| `ucfg` + `ucfg` on the same unit | 110 | 14 | 14 | 1 |
| `set` + `delay` | 27 | 0 | 1 | 9 |
| `setoe` + `setoe` | 11 | 2 | 7 | 0 |

### Candidates, and whether they fit the 29-bit word
- **`ld`/`st` with an absolute address** (`ld rd, [imm8]`): fits. imm16 has room for the address, so one word replaces `mov` + `ld`/`st`. USB device saves 8 words.
- **Compare and branch** (`jeq/jne rA, #imm, target`): imm16 already holds the condition (5 bits) and the target (8 used of 10), so the compare value needs another place. 6 bits (A unused, I, B) give compares against 0-63, which covers most uses (PIDs and status codes do not fit; small counts and flags do). USB device saves up to 14.
- **`set` with a delay** brings back part of what `dly` did, but only for `set`. Worth it for I2C (9 in the multi-master build), not for USB.
- **Two `ucfg` in one word** is not possible: each carries a 16-bit value.

With the first two, the USB device would be about 140 words: IMEM 162 → 140 saves about 57,000 µm² (8% of the chip). IMEM depth must still cover the largest program, so the saving is set by USB, the protocol that is tight on space.

### Things to keep in mind
- This shrinks IMEM only. DMEM's size is set by the host buffers (USB packets of up to 64 bytes in and out, I2C and SPI transfers), not by the firmware. Shrinking DMEM (for example to 128 bytes) is a separate decision about the largest transfer.
- Every new instruction needs the RTL, the model, the z3 obligation, the assembler and the firmware changed together, as in the `dly` removal.
- Timing: compare and branch puts a 16-bit compare into the next-pc path, which now also drives the IMEM read address. That path had 11 ns of slack at the slow corner (after CTS); check it again.

## 3. Other parked ideas
- `assist` opcode (15) for CRC, NRZI/Manchester and bit stuffing: needed for the stretch goals (low-speed USB, 10BASE-T).
- Timestamp counter and capture FIFO.
- SRAM-based instruction memory instead of flip-flops (area).
- Symbolic proof of the chip composition. Each module is proved against the model (`model/tests/test_equiv.py`), but the wiring is covered by lockstep simulation only.
