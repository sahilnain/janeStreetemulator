# Area estimate

Post-synthesis cell area of the RTL in the process Tiny Tapeout will use. This is an estimate: placement, clock tree, buffering and routing come after this step and have not been run. The Python model is not synthesizable, so the RTL stands in for it; the z3 proof and lockstep tests tie the two together.

## Setup

| Item | Value |
|---|---|
| Design | commit `efd4314`, plus `pe_crc.v` added to `info.yaml` |
| Process | IHP SG13G2, PDK revision `cb7daaa8`, typical corner 1.2 V 25 °C |
| Flow | LibreLane 3.0.5 (the version the `ttihp26b` GDS action installs), stopped after `Yosys.Synthesis` |
| Config | The `src/config_merged.json` that `tt_tool.py --create-user-config --ihp` generates: 6x4, 48 MHz, `AREA 0` |
| Lint and synthesis checks | 0 errors, 0 warnings, 0 inferred latches |

## Total

| Measure | Value |
|---|---|
| Cell area, flat (what the real flow builds) | **755,159 µm²** |
| Of which flip-flops | 408,616 µm² (54%), 8,341 flip-flops |
| Of which tie cells | 60,542 µm² (8%) |
| Cells | 42,552 |
| 6x4 die, 1289.28 × 710.64 µm | 916,220 µm² |
| Core, die less margins (calculated, not from a floorplan) | about 902,400 µm² |
| **Core used by cells, before place and route** | **about 84%** |

## Breakdown by module

From a second run with hierarchy kept (`SYNTH_HIERARCHY_MODE=keep`), because a flat netlist has no modules. Its total is 770,087 µm², 2% above flat, as logic cannot be optimized across module boundaries. Each row is that instance alone, without its children, and the rows add up to the total.

| Instance | Module | Area (µm²) | Share | Flip-flops | FF area | Tie area | Logic area | Cells |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `u_imem` | `pe_imem` | 459,562 | 59.68% | 5,152 | 252,390 | 37,391 | 169,780 | 21,881 |
| `u_dmem` | `pe_dmem` | 176,929 | 22.98% | 2,048 | 100,329 | 14,864 | 61,736 | 9,294 |
| `u_ctrl` | `pe_ctrl` | 31,954 | 4.15% | 194 | 9,504 | 1,408 | 21,042 | 2,548 |
| `u_rxu` | `pe_rxu` | 24,326 | 3.16% | 239 | 11,708 | 1,735 | 10,883 | 1,624 |
| `u_txu` | `pe_txu` | 13,099 | 1.70% | 106 | 5,193 | 769 | 7,137 | 989 |
| `u_erx` | `pe_det` | 9,562 | 1.24% | 91 | 4,458 | 660 | 4,443 | 673 |
| `u_regs` | `pe_regs` | 9,141 | 1.19% | 90 | 4,409 | 653 | 4,079 | 622 |
| `u_etx` | `pe_det` | 7,920 | 1.03% | 74 | 3,625 | 537 | 3,758 | 558 |
| `u_rxf` | `pe_fifo` | 7,120 | 0.92% | 78 | 3,821 | 566 | 2,732 | 370 |
| `u_txf` | `pe_fifo` | 7,120 | 0.92% | 78 | 3,821 | 566 | 2,732 | 370 |
| `u_rxu/u_vote` | `pe_vote` | 6,519 | 0.85% | 55 | 2,694 | 399 | 3,425 | 464 |
| `u_crc` | `pe_crc` | 5,383 | 0.70% | 52 | 2,547 | 377 | 2,458 | 385 |
| `u_pins` | `pe_pins` | 5,281 | 0.69% | 41 | 2,009 | 298 | 2,974 | 407 |
| top level | `tt_um_sahil_protoemu` | 2,499 | 0.32% | 2 | 98 | 15 | 2,386 | 261 |
| `u_host_if` | `pe_host_if` | 2,063 | 0.27% | 24 | 1,176 | 174 | 713 | 133 |
| `u_pins/u_sync` | `pe_sync` | 1,045 | 0.14% | 16 | 784 | 116 | 145 | 48 |
| `u_ctrl/u_decode` | `pe_decode` | 435 | 0.06% | 0 | 0 | 0 | 435 | 59 |
| `u_host_if/u_sync_stb` | `pe_sync` | 131 | 0.02% | 2 | 98 | 15 | 18 | 6 |
| **Total** | | **770,087** | **100%** | **8,342** | **408,665** | **60,543** | **300,879** | **40,692** |

Grouped:

| Group | Instances | Area (µm²) | Share |
|---|---|---:|---:|
| Instruction memory | `u_imem` | 459,562 | 59.7% |
| Data memory | `u_dmem` | 176,929 | 23.0% |
| Controller | `u_ctrl`, `u_decode` | 32,390 | 4.2% |
| RX unit | `u_rxu`, `u_vote` | 30,845 | 4.0% |
| Edge detectors | `u_erx`, `u_etx` | 17,482 | 2.3% |
| FIFOs | `u_rxf`, `u_txf` | 14,239 | 1.8% |
| TX unit | `u_txu` | 13,099 | 1.7% |
| Registers and host bus | `u_regs`, `u_host_if`, `u_sync_stb` | 11,335 | 1.5% |
| Pins | `u_pins`, `u_sync` | 6,326 | 0.8% |
| CRC unit | `u_crc` | 5,383 | 0.7% |
| Top-level logic | | 2,499 | 0.3% |

## What drives it

- **Memory is 83% of the design.** IMEM is 160 × 32 = 5,120 bits and DMEM is 256 × 8 = 2,048 bits, both in flip-flops with a read multiplexer.
- **Cost per bit:** about 90 µm² in IMEM and 86 µm² in DMEM. One IMEM word costs about 2,870 µm², or 0.31% of the die.
- **Every flip-flop has its own tie cell.** The only flip-flops the flow maps to have an asynchronous reset pin. The RTL uses synchronous reset, so each pin is tied high: 49 µm² for the flip-flop and 7.3 µm² for its tie cell.

## The latch option for IMEM

`pe_imem` has a latch version, `IMEM_IMPL = 1`. Synthesized the same way, changing only that value:

| | Flip-flop IMEM (current) | Latch IMEM |
|---|---:|---:|
| Total, flat | 755,159 µm² | 509,903 µm² |
| `u_imem`, hierarchy kept | 459,562 µm² | 203,245 µm² |
| One IMEM word | about 2,870 µm² | about 1,270 µm² |
| Share of the 6x4 core | about 84% | about 57% |

The latch version saves 245,255 µm², but it is **not verified**:

- Its source marks it experimental and not yet checked in the IHP flow.
- It is proved at depths 16 and 128, not 160, and the chip regression and lockstep run on the flip-flop version.
- Its timing depends on the low phase of the clock, which place and route must confirm.
- It adds a lint warning, an unused `wr_in`.
- It has not been checked against Tiny Tapeout's rules for latches.

## Does it fit?

Not measured yet. A place-and-route run is the only real test. The target placement density in `src/config.json` is 60%, and its comments say values up to 80% have worked. Clock tree, buffers and timing repair add cell area; a rough allowance is 10–25%, which is a rule of thumb and not from this design.

| Option | Core used at synthesis | With a 10–25% allowance | View |
|---|---:|---:|---|
| 6x4, flip-flop IMEM (current) | about 84% | about 92–105% | Very unlikely to fit |
| 6x4, latch IMEM | about 57% | about 62–71% | Plausible, once verified |
| 8x4, flip-flop IMEM | about 62% | about 69–78% | Plausible, at a higher tile cost |

## Not covered

- **Timing.** Synthesis does not tell us whether 48 MHz closes.
- **Final area.** Nothing here includes clock tree, hold buffers, repair or routing congestion.
- **`AREA 0` is the flow's default strategy.** No other strategies were tried.

## To reproduce

Work on a copy of the repository, as this generates files in `src/`:

```sh
python tt/tt_tool.py --create-user-config --ihp
mkdir -p runs/flat runs/keep
# inside ghcr.io/librelane/librelane:3.0.5, with PDK_ROOT set:
python3 -m librelane --pdk ihp-sg13g2 --run-tag flat --force-run-dir runs/flat \
    --to Yosys.Synthesis src/config_merged.json
python3 -m librelane --pdk ihp-sg13g2 --run-tag keep --force-run-dir runs/keep \
    --to Yosys.Synthesis -c SYNTH_HIERARCHY_MODE=keep src/config_merged.json
```

The reports are in `runs/*/06-yosys-synthesis/reports/stat.rpt`.
