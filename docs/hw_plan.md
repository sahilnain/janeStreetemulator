# Hardware implementation plan

Plan for building the architecture in `../Claude.md` (sections 2, 3 and 6, "Architecture v2").
Starting point: the unmodified Tiny Tapeout Verilog template. Clock assumed 50 MHz (`src/config.json` CLOCK_PERIOD = 20).

## 0. Architecture v2 → RTL modules

| Architecture block (Claude.md §6) | Module | Notes |
|---|---|---|
| Host interface and registers | `pe_host_if.v`, `pe_regs.v` | Program load, config (N, S, k, pin map), context start/stop, status, FIFO access. Every software-visible value is a register |
| Instruction memory (wide, one shared read port) | `pe_imem.v` | Width and depth are parameters. Flop, latch or SRAM implementation behind one interface |
| Execution contexts | `pe_ctx.v` × NUM_CONTEXTS | PC, ctr, OSR, ISR, shift count, stall counter, flags. Both OSR and ISR are in every context (in the diagram, ctx0 shows only OSR and ctx1 only ISR), so a single context can do full-duplex SPI |
| Stall counter (per context) | inside `pe_ctx.v` | Loaded by `delay n` or the word's 3-bit delay field. Counts raw clock cycles. The context is not fetched while it is non-zero |
| Vote unit | `pe_vote.v`, instantiated per `VOTE_CTX_MASK` | ones/n counters, N, S, k from registers. Stalls its context for N×S cycles, then shifts the result into ISR |
| Decoder and shared datapath | `pe_core.v` | One decoder and shift/ALU unit, shared round-robin between contexts (see fetch scheme below) |
| TX FIFO, RX FIFO | `pe_fifo.v` × 2 | Byte FIFOs between host and firmware: one TX (host→`pull`) and one RX (`push`→host), shared by all contexts as drawn. Any context can `pull`/`push`. Firmware decides who uses them |
| Pin fabric | `pe_pins.v`, `pe_sync.v` | Out reg, output-enable (open-drain), 2-FF sync, **static per-pin owner map** set by the host. A write from a context that does not own the pin is ignored and sets a sticky error flag |
| Optional assists | — | Reserved `ASSIST` opcode only in v1 |
| Optional capture | `pe_capture.v` (M9, if area allows) | Timestamp counter plus capture FIFO for debug |

Design rules carried into the RTL (Claude.md §6):
- Pin ownership is a static map, so two contexts can never drive one line. A formal property checks this.
- Each context stalls independently. A stalled context never blocks another context's slot.
- Anything firmware relies on must be host-configurable: N/S/k, shift direction, pin owner/OE/open-drain mode and per-context start PC are all in `regmap.md`.

## 1. Proposed answers to the open items

These are defaults so work can start. Each one is cheap to change until the milestone named.

| # | Open item | Proposal | Why | Lock by |
|---|-----------|----------|-----|---------|
| 1 | `push` on a full RX FIFO | Per-instruction `block` bit, like PIO: `push` stalls; `push nb` drops and sets a sticky overrun flag | SPI master wants stall (it owns the clock). UART RX wants drop + flag (the line will not wait) | M4 |
| 2 | Vote unit on every context? | Parameter `VOTE_CTX_MASK`. `full` = all contexts, `min` = context 1 only | About 20 flops each. Measure in M7 before deciding | M7 |
| 3 | Host bus | 4-bit parallel nibble bus on `ui`/`uo`, strobe/ack toggles through 2-FF syncs | All 8 `uio` pins stay free for protocols, and those are the only pins with per-pin output-enable (needed for I2C open-drain). A nibble bus does not look like SPI or UART | M1 |
| 4 | RTL language | Plain Verilog-2005 plus one generated parameter include | Matches the template, iverilog, yosys and the TT linter. The config generator gives the parameterisation that Amaranth etc. would | M0 |
| 5 | Instruction width | 24 bits for now (table below). Freeze after the I2C/SPI opcodes are in | Side-set needs about 4-5 bits. 16 bits does not fit opcode + delay + side-set + operand | M8 |
| 6 | CRC/NRZI in v1 | No. Leave encoding space and an `ASSIST` opcode slot | Stretch goals only. Area is better spent on instruction memory depth | M8 |

Two new decisions the RTL forces that the handoff does not cover:

- **Fetch scheme: fixed time-slot (barrel) interleave.** This is the "round-robin" from §6, made strict: a slot is never given to another context, even when its own context is stalled. Context *i* issues on cycles where `slot == i`. Each context gets one instruction slot every `NUM_CONTEXTS` cycles. Advantages:
  - Timing stays fully deterministic, so the assembler can compute delays exactly.
  - A single imem read port is enough.
  - The interleave hides a registered-read SRAM's latency for free.

  The cost is that per-context speed is `clk / max(2, NUM_CONTEXTS)`. Because the imem read is registered, there are always at least 2 slots, so `NUM_CONTEXTS=1` does not give full speed (see `modules.md` §2). Fast SPI depends on side-set/`xfer` instead.
- **Stall counter counts raw clock cycles** (as in the handoff), and the delay is rounded to whole slots. The assembler does that rounding and reports the error against the target baud rate.

### Pin allocation (proposal)

| Pins | Use |
|------|-----|
| `uio[7:0]` | Protocol pins P0-P7. Each pin has an out reg, OE reg, open-drain mode and a 2-FF input sync |
| `ui[3:0]` | Host data in (nibble) |
| `ui[5:4]` | Host op: 00 = set address nibble, 01 = write nibble, 10 = read nibble, 11 = command |
| `ui[6]` | Host strobe (toggle, synchronised) |
| `ui[7]` | Extra protocol input P8 (input only) |
| `uo[3:0]` | Host data out (nibble) |
| `uo[4]` | Host ack (toggle) |
| `uo[5]` | IRQ (FIFO level / error, maskable) |
| `uo[7:6]` | Extra protocol outputs P9-P10 (output only) |

## 2. Instruction word v0 (24 bits, not frozen)

```
[23:19] opcode (5)   [18:16] delay 0-7 (3)   [15:0] operands (16)
```

| Op | Operands | Notes |
|----|----------|-------|
| `set pin,val` / `set pin,C` | pin(4), src(2) | src = 0, 1, carry, ~carry |
| `oe pin,val` | pin(4), val(1) | output enable, for open-drain |
| `out pin,n` / `in pin,n` | pin(4), n(5) | shift OSR→pin / pin→ISR. Direction from the per-context shift-dir config bit |
| `wait pin,level[,timeout]` | pin(4), level(1), ctr sel(1) | timeout counts down a counter, then sets the timeout flag (I2C clock stretching) |
| `vote pin` | pin(4) | result → carry and ISR. Context stalls N×S cycles |
| `jpin pin,lvl,target` | pin, lvl, addr | |
| `jmp [cond],target` | cond(3), addr | cond: always, C, !C, Z, !Z, timeout, FIFO-empty, FIFO-full |
| `djnz ctr,target` | ctr(1), addr | |
| `mov ctr,imm` | ctr(1), imm(16) | 16-bit counters, so 9600 baud (5208 cycles) fits |
| `delay n` | n(16) | |
| `pull [nb]` / `push [nb]` | block(1) | |
| `ASSIST` | reserved | |
| side-set field | (M8) | carved from operands, or the word grows to 28/32 |

Address width = log2(IMEM_DEPTH), from the config.

## 3. Repository layout

```
config/            min.yaml, full.yaml, tapeout.yaml   (single source of truth)
tools/gen_config.py  → src/pe_params.vh + asm/encoding_gen.py
asm/pe_asm.py      assembler (encoding tables imported from generated file)
model/pe_iss.py    cycle-accurate Python reference model
firmware/          uart_tx.s, uart_rx.s, later i2c_*.s, spi_*.s
src/               tt_um_*.v (top), pe_host_if.v, pe_regs.v, pe_imem.v, pe_core.v,
                   pe_ctx.v, pe_vote.v, pe_fifo.v, pe_pins.v, pe_sync.v, (pe_capture.v)
test/              cocotb: one test module per RTL block + top-level tests
formal/            SymbiYosys .sby per leaf block
docs/spec/         isa.md, regmap.md, vote.md, host_bus.md
```

Config fields: `NUM_CONTEXTS, IMEM_DEPTH, IMEM_IMPL (flop|latch|sram), INSTR_WIDTH, FIFO_DEPTH, NUM_PINS, VOTE_CTX_MASK, CTR_WIDTH, VOTE_N_MAX, VOTE_S_WIDTH`.

## 4. Milestones

Each milestone ends with tests passing in CI for both `min` and `full`.

### M0: Toolchain and skeleton
- Install iverilog, verilator (lint), yosys, SymbiYosys, cocotb (`test/requirements.txt`). None are installed now. Alternatively use the template's `.devcontainer`.
- `git init` and push to GitHub so the template's GDS/test actions run.
- Rename the top module to `tt_um_<github user>_protoemu`. Fill in `info.yaml`.
- Write `gen_config.py`, the 3 configs and a Makefile target that regenerates. CI matrix over configs.
- **Exit:** empty top with generated params lints clean and passes a smoke test.

### M1: Specs (short, but written before RTL)
- `isa.md`: final v0 encoding, per-op cycle cost, behaviour for stalls, FIFOs and flags.
- `vote.md`: cycle-by-cycle. On issue the context stalls, the sample counter loads S, the ones/n counters clear. Every S cycles sample the synced pin, and if `k ≤ n < N−k` add it to ones. After N samples, carry = (ones > (N−2k)/2), shift into ISR, release the stall. State the exact cycle count, including the issue slot, so the assembler can subtract it.
- `regmap.md` and `host_bus.md`: address map (imem window, per-context control/PC/config, vote N/S/k, pin modes, FIFO data/status, global run/halt, IRQ mask, ID/version register).
- **Exit:** reviewed docs. The assembler and ISS are written against them.

### M2: Python reference model and assembler
- `pe_iss.py`: models the barrel slots, stall counters, FIFOs, vote and pins cycle by cycle. Pins are exposed as callbacks so tests can wire TX→RX.
- `pe_asm.py`: labels, `.baud`/`.clk` directives that compute delay constants minus instruction overhead, error report.
- UART TX/RX firmware runs on the ISS in loopback at 9600 and 115200 baud.
- **Exit:** ISS loopback passes, with injected ±2% baud error, and catches a forced framing error.

### M3: Leaf RTL blocks (each with a cocotb unit test, plus formal where marked)
1. `pe_sync`: 2-FF synchroniser.
2. `pe_fifo`: sync FIFO, parametric depth. Formal: no overflow/underflow, count invariant.
3. `pe_pins`: out/OE regs, open-drain mode, static per-pin owner map (non-owner writes are ignored and flagged). Formal: a pin only changes when its owner writes it.
4. `pe_vote`: matches `vote.md` exactly. Formal: output = majority of the centre samples. Random test against the ISS.
5. `pe_imem`: wrapper with flop and latch implementations behind one interface (a single read port, and a write port from the host). Keep the SRAM-macro option open behind the same interface.
- **Exit:** all units green. First yosys `synth` cell count recorded per block (generic, then IHP cells).

### M4: Core with a single context
- `pe_ctx` holds the state: PC, ctr0/ctr1, OSR, ISR, shift count, stall counter, flags C/Z/timeout/overrun.
- `pe_core`: fetch → decode → execute on the shared datapath, muxed by the slot counter.
- Bring up op by op: `set/jmp/mov/djnz/delay` → `out/pull` → `wait/jpin` → `in/push` → `vote`.
- Lockstep test: run the same program on RTL and ISS and compare architectural state every cycle (pins, PC, counters). Add constrained-random instruction streams on top.
- **Exit:** UART TX firmware gives bit-exact waveforms against the ISS. Random lockstep passes 10k programs.

### M5: Multi-context and UART full duplex
- Instantiate `NUM_CONTEXTS`. Turn on the slot interleave. Connect the shared TX/RX FIFOs (ctx0 pulls, ctx1 pushes in the UART firmware).
- Full-duplex loopback test matrix: baud {9600, 57600, 115200, 1M}, error {−2, 0, +2}%, plus framing errors, back-to-back bytes and RX FIFO overrun (`push nb`).
- **Exit:** matrix passes on `min` and `full`.

### M6: Host interface and top-level integration
- `pe_host_if` (nibble bus), `pe_regs`, IRQ, and the top-level pin mapping.
- A Python host driver used both by cocotb (bus model) and later on the RP2040 (MicroPython): load program, configure, start, read/write FIFOs.
- **Exit:** all M5 tests rerun **through the pins only**, with the host bus driving everything.

### M7: Synthesis and area (do not wait until the end)
- Run the TT GDS action on `full`. Record cells, utilisation and timing at 50 MHz.
- Size IMEM_DEPTH × INSTR_WIDTH against the 24-tile budget. Compare flop, latch and the TT SRAM example.
- Settle `VOTE_CTX_MASK`, `FIFO_DEPTH` and `NUM_CONTEXTS` for tapeout from these numbers.
- **Exit:** `full` hardens in 6x4 with positive slack. Area table is in the docs.

### M8: I2C and SPI features, then freeze the ISA
- I2C: `wait` timeout, open-drain via OE, `set pin,C`, ACK/NACK, clock stretching. Firmware plus a cocotb I2C target model.
- SPI: side-set field, combined `xfer` (drive MOSI + sample MISO in the same slot), programmable MISO sample delay, all 4 modes, CS. Firmware plus a cocotb SPI slave model.
- Freeze the instruction width and encoding. Regenerate everything.
- **Exit:** UART, I2C and SPI all pass through the top-level pins on the `tapeout` config.

### M9: Tapeout readiness
- Gate-level sim (`GATES=yes`) of the full test suite.
- `docs/info.md` datasheet, pinout in `info.yaml`, demo firmware for the RP2040 board.
- Optional if area remains: JTAG/SWD/PS/2 firmware (no new hardware needed), and `pe_capture` (timestamp counter plus capture FIFO).
- **Target:** submission well before Jan 18, 2027.

## 5. Verification strategy (summary)

- **Golden model:** the Python ISS. Every RTL test compares against it in lockstep, not against hand-written expectations.
- **Formal (SymbiYosys):** fifo, vote, sync, host-bus handshake, and core invariants (PC in range, stall counter never underflows, no two contexts drive one pin).
- **Constrained-random:** random legal instruction streams and random host-bus timing.
- **Protocol scoreboards:** the UART/I2C/SPI bus-functional models in cocotb are independent of the ISS, so they catch shared spec bugs.
- **CI:** every config × every test, and lint with verilator `--lint-only -Wall`.

## 6. Risks

| Risk | Mitigation |
|------|------------|
| Instruction memory dominates area | Measure in M3/M7. Latch or SRAM implementation behind a wrapper |
| The barrel halves per-context speed | Side-set/`xfer` for SPI. A same-cycle (async-read) flop imem option could allow 1-slot operation later |
| 50 MHz timing on the shared datapath | Synth from M3 on; the interleave lets decode be registered |
| Spec drift between RTL, ISS and assembler | Everything generated from `config/*.yaml`. Lockstep tests |
| Tiny Tapeout format changes (8x4, SRAM availability) | Keep tiles and IMEM_IMPL as config, and re-check the TT announcements monthly |
