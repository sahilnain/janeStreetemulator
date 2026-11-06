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

## 2. Other parked ideas
- `assist` opcode (15) for CRC, NRZI/Manchester and bit stuffing: needed for the stretch goals (low-speed USB, 10BASE-T).
- Timestamp counter and capture FIFO.
- SRAM-based instruction memory instead of flip-flops (area).
- Symbolic proof of the chip composition. Each module is proved against the model (`model/tests/test_equiv.py`), but the wiring is covered by lockstep simulation only.
