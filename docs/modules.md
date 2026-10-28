# Module breakdown (RTL v0)

Splits Architecture v2 (`../../Claude.md` §6, `hw_plan.md` §0) into Verilog modules with fixed interfaces, so each module can be written and tested on its own.
Port lists are the contract between modules. Bit-level instruction encoding and the register map go in `docs/spec/` (M1), not here.

## 0. Files (RTL v0, written)

```
src/tt_um_sahil_protoemu.v   top: TT wrapper, config localparams, all wiring
src/host/pe_host_if.v        nibble host bus
src/host/pe_regs.v           register file (register map in the header)
src/mem/pe_imem.v            instruction memory (IMPL 0 = flops, 1 = latches, experimental)
src/mem/pe_fifo.v            TX/RX FIFO
src/io/pe_sync.v             2-FF synchroniser
src/io/pe_pins.v             pin fabric
src/core/pe_core.v           slot scheduler, wires contexts + decode + exec + vote
src/core/pe_ctx.v            one context's state
src/core/pe_decode.v         decoder (instruction encoding table in the header)
src/core/pe_exec.v           shared datapath (instruction semantics in the header)
src/core/pe_vote.v           vote unit (cycle-exact timing in the header)
```

Until `docs/spec/` is written, the header comments of `pe_decode.v`, `pe_exec.v`, `pe_vote.v` and `pe_regs.v` are the source of truth for the encoding, semantics, vote timing and register map.

## 1. Hierarchy

```
tt_um_<user>_protoemu            top: TT pin mapping, reset sync
├── pe_host_if                   nibble host bus  →  internal register bus
├── pe_regs                      register file / address decode, config fan-out, imem staging
├── pe_imem                      instruction memory (1 write port, 1 registered read port)
├── pe_fifo  u_txf               host → firmware byte FIFO   (pull)
├── pe_fifo  u_rxf               firmware → host byte FIFO   (push)
├── pe_pins                      out/OE regs, open-drain, owner map, input sync
│   └── pe_sync ×NUM_PINS        2-FF synchroniser
└── pe_core                      slot scheduler + shared decode/execute
    ├── pe_ctx  ×NUM_CTX         per-context architectural state + stall counter
    ├── pe_decode                instruction word → control signals (combinational)
    ├── pe_exec                  shared datapath: next state for the context in its slot
    └── pe_vote ×popcount(VOTE_CTX_MASK)   oversample + majority vote
```

Data flow per instruction: `pe_core` sends the PC of the next slot's context to `pe_imem`. One cycle later the word arrives, then `pe_decode`, `pe_exec` and writeback into that `pe_ctx`, `pe_pins` and the FIFOs. Everything happens on one clock edge.

## 2. Global parameters (generated from `config/*.yaml` into `src/pe_params.vh`)

| Parameter | min | full | Meaning |
|---|---|---|---|
| `NUM_CTX` | 2 | 2 (maybe 3) | execution contexts |
| `IMEM_DEPTH` / `IMEM_AW` | 32 / 5 | 64 / 6 (sized in M7) | words, address bits |
| `IW` | 24 | 24 | instruction width (frozen in M8) |
| `IMEM_IMPL` | flop | latch / sram | `pe_imem` implementation |
| `NUM_PINS` | 8 | 8 | protocol pins (= `uio[7:0]`) |
| `DATA_W` | 8 | 8 | FIFO width |
| `FIFO_DEPTH` | 2 | 4 | entries per FIFO |
| `SR_W` | 16 | 16 | OSR/ISR width |
| `CTR_W` | 16 | 16 | counters and stall counter (9600 baud = 5208 cycles fits) |
| `VOTE_CTX_MASK` | 2'b10 | 2'b11 | which contexts own a vote unit |
| `VOTE_N_W`, `VOTE_S_W` | 4, 10 | 4, 12 | widths of N and S |

`CTX_W = clog2(NUM_CTX)`, `PIN_W = clog2(NUM_PINS)`, `NUM_SLOTS = max(2, NUM_CTX)`.

### Timing decision this breakdown fixes

`pe_imem` has a **registered read** for every implementation, so that an SRAM macro fits the same interface. Fetch therefore happens one cycle before execute, for the context that owns the *next* slot. This needs at least 2 slots. With `NUM_CTX=1`, the second slot is idle, so **an instruction always costs `NUM_SLOTS` cycles (2 for the min and full configs)**. This replaces the claim in `hw_plan.md` that `NUM_CTX=1` gives full speed.

Consequences:
- The stall counter still counts raw cycles (Claude.md). A context only issues in its slot, so the real delay is rounded up to a multiple of `NUM_SLOTS`. The assembler accounts for this.
- `wait`/`jpin` re-check once per slot (every 2 cycles at 50 MHz = 40 ns resolution).
- `vote` sampling runs every cycle inside `pe_vote`, independent of slots, so `S` keeps 1-cycle resolution.

## 3. Modules

Conventions for every module: `clk` and active-high synchronous `rst` (generated in the top from `rst_n`). There are no other clocks. Every module has `default_nettype none`.

### 3.1 `tt_um_<user>_protoemu` (top)
- **Does:** the TT port list. Registers `rst_n` into `rst`. Instantiates everything and maps pins:
  `uio_out/uio_oe/uio_in` ↔ `pe_pins`. `ui[6:0]`, `uo[5:0]` ↔ `pe_host_if`. `ui[7]` and `uo[7:6]` are reserved for v1 (tied off).
- **Test:** top-level cocotb only (M6).

### 3.2 `pe_host_if`
- **Does:** turns the nibble bus into single-cycle register reads and writes. The strobe is a toggle, so the host can be slow and asynchronous (RP2040 PIO or GPIO): it goes through `pe_sync`, an edge detect, the op, then ack toggle. It holds an 8-bit address register (filled from nibbles) and an 8-bit data assembly register.
- **Ports:**
  ```
  in  [3:0] h_din      in  [1:0] h_op      in  h_stb
  out [3:0] h_dout     out h_ack           out h_irq
  out [7:0] r_addr     out [7:0] r_wdata   out r_we   out r_re
  in  [7:0] r_rdata    in  irq
  ```
- **Notes:** ops: set-address-nibble, write-nibble (the second nibble completes the byte and sends `r_we`), read-nibble (the first read sends `r_re` and latches `r_rdata`, the second returns the other half), command (reserved). The exact sequence goes in `docs/spec/host_bus.md`.
- **Test:** cocotb with a host bus-functional model at random strobe timing. Formal: exactly one `r_we` per completed byte, and the ack toggles once per strobe.

### 3.3 `pe_regs`
- **Does:** address decode for the 8-bit register space. Holds every host-visible config register. Exposes status. Stages 24-bit imem words (three byte writes, then a commit with auto-increment). Connects the host side of both FIFOs.
- **Ports:**
  ```
  reg bus  : r_addr, r_wdata, r_we, r_re → r_rdata, irq
  imem     : out im_we, out [IMEM_AW-1:0] im_waddr, out [IW-1:0] im_wdata
  fifos    : out txf_wr, out [7:0] txf_wdata, in txf_full
             out rxf_rd, in [7:0] rxf_rdata, in rxf_empty, in rxf_overrun
  core cfg : out [NUM_CTX-1:0] ctx_run, out [NUM_CTX-1:0] ctx_restart,
             out [NUM_CTX*IMEM_AW-1:0] ctx_start_pc, out [NUM_CTX-1:0] ctx_shift_msb,
             out vote N/S/k per vote unit
  core stat: in [NUM_CTX*IMEM_AW-1:0] ctx_pc, in [NUM_CTX-1:0] ctx_stalled, in err flags
  pin cfg  : out [NUM_PINS*CTX_W-1:0] pin_owner, out [NUM_PINS-1:0] pin_od,
             in [NUM_PINS-1:0] pin_in (readback), in owner_err
  ```
- **Test:** cocotb write/readback of every register, plus a check that reserved addresses read 0.

### 3.4 `pe_imem`
- **Does:** `IMEM_DEPTH × IW` memory with one write port (host) and one registered read port (core). `IMEM_IMPL` picks flops, latches or an SRAM-macro wrapper.
- **Ports:** `we, [AW-1:0] waddr, [IW-1:0] wdata, [AW-1:0] raddr → [IW-1:0] rdata` (valid 1 cycle after `raddr`).
- **Notes:** the host writes only while the contexts are halted. `pe_regs` enforces this; a write while running is dropped and flagged. That avoids read/write collision logic.
- **Test:** cocotb random write/read. The same test runs against each implementation.

### 3.5 `pe_fifo`
- **Does:** synchronous FIFO with parameters `W` and `DEPTH`. If `wr` comes while full, the data is dropped and a sticky `overrun` is set. If `rd` comes while empty, nothing happens. Blocking is the core's job: it stalls before asserting `wr`/`rd`.
- **Ports:** `wr, [W-1:0] wdata, full, rd, [W-1:0] rdata (first-word fall-through), empty, [clog2(DEPTH+1)-1:0] count, overrun, clr_overrun`.
- **Test:** formal (count invariant, data order, no change on illegal ops) plus cocotb random.

### 3.6 `pe_sync`
- **Does:** 2-FF synchroniser with a reset value parameter. Used by `pe_pins` and `pe_host_if`.

### 3.7 `pe_pins`
- **Does:** the pin fabric. Each pin has an `out` reg and an `oe` reg, which hold until written. Open-drain mode (`pin_od[i]`): writing 1 releases (oe=0), writing 0 drives low (oe=1, out=0). An owner check lets a write from context `c` affect only pins with `pin_owner[i]==c`. Other bits are ignored and `owner_err` is set (sticky). Inputs pass through `pe_sync`.
- **Ports:**
  ```
  from core: in wr_en, in [CTX_W-1:0] wr_ctx,
             in [NUM_PINS-1:0] wr_mask, in [NUM_PINS-1:0] wr_val,     // set / out / set C / side-set
             in oe_en, in [NUM_PINS-1:0] oe_mask, in [NUM_PINS-1:0] oe_val
  to core  : out [NUM_PINS-1:0] pin_in_sync
  cfg      : in pin_owner, in pin_od, out owner_err, in clr_err
  pads     : out [NUM_PINS-1:0] pad_out, out [NUM_PINS-1:0] pad_oe, in [NUM_PINS-1:0] pad_in
  ```
- **Notes:** a mask/value write port covers `set` (1 bit), `out pin,n` (n consecutive bits) and side-set (M8) without changing the interface. Only one context executes per cycle, so a single port is enough.
- **Test:** formal: a pin changes only on an owner write. In OD mode, `pad_out` is always 0 when `pad_oe` is 1. Plus cocotb.

### 3.8 `pe_core`
- **Does:** the slot counter and fetch scheduling. It muxes the slot's context state into `pe_exec` and writes the result back. It routes FIFO, pin and vote requests. It holds no architectural state of its own apart from the slot counter.
- **Ports:** `imem raddr/rdata`, the pin port (3.7), the firmware sides of both FIFOs (`txf_rd, txf_rdata, txf_empty, rxf_wr, rxf_wdata, rxf_full`), core cfg/stat (3.3), vote cfg.
- **Test:** lockstep against the Python ISS (M4/M5), plus constrained-random programs.

### 3.9 `pe_ctx`
- **Does:** one context's state:
  - `pc`, `ctr0`, `ctr1`, `osr`, `isr`, `osr_cnt`, `isr_cnt`, flags `C Z TO`
  - `stall_cnt`, and `busy` (waiting on a vote)

  `stall_cnt` counts down every cycle, independent of slots. `ready = run && stall_cnt==0 && !busy`. Takes `restart` (PC ← start_pc, clear state) and a writeback bus from `pe_exec` (`we` + the next value of each field).
- **Test:** cocotb unit test of stall counting, restart and writeback.

### 3.10 `pe_decode`
- **Does:** purely combinational. Takes an `IW`-bit word and produces opcode one-hot, delay, pin index, n, counter select, condition, target, immediate and the block bit. It is generated from the same encoding table as the assembler, so the two cannot diverge.
- **Test:** exhaustive in cocotb: decode(assemble(x)) == x for every op.

### 3.11 `pe_exec`
- **Does:** purely combinational, shared by all contexts. Takes the decoded instruction, the current context state, `pin_in_sync`, FIFO status and vote results. Produces:
  - the next context state
  - pin write requests and FIFO `rd`/`wr`
  - vote start
  - the stall decision

  If the instruction cannot complete (`wait` not met, blocking `pull` on empty, blocking `push` on full), the PC is held and the state is not changed. Otherwise the stall counter is loaded from the delay field, or from `delay n`.
- **Test:** through `pe_core` with the ISS. Formal: a held instruction has no side effects.

### 3.12 `pe_vote`
- **Does:** after `start` (from `pe_exec`, with the pin index), takes a sample every `S` cycles, `N` samples in total. It counts `ones` only for samples with index in `[k, N-k)`. When done it raises `done` with `bit = ones > (N-2k)/2`. While running it holds the context's `busy`. `pe_core` shifts `bit` into ISR and sets `C`.
- **Ports:** `start, [PIN_W-1:0] pin_sel, [NUM_PINS-1:0] pin_in_sync, [VOTE_N_W-1:0] N, [VOTE_S_W-1:0] S, [VOTE_N_W-1:0] k → busy, done, bit`.
- **Test:** formal: `bit` equals the majority of the centre samples. The total cycle count is exactly N×S. Random test against the ISS.

## 4. Build order (dependencies)

```
pe_sync → pe_fifo → pe_pins → pe_vote → pe_imem      leaf blocks, M3
pe_decode → pe_ctx → pe_exec → pe_core               M4–M5 (needs isa.md)
pe_host_if → pe_regs → top                           M6 (needs regmap.md, host_bus.md)
```

The leaf blocks need no ISA decisions, so RTL can start with them now. `pe_decode`/`pe_exec` wait on `docs/spec/isa.md`.
