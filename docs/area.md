# Area

Cell area of the RTL in the process Tiny Tapeout will use, after synthesis and after placement with the clock tree. Routing, hold repair and fill have not been run, so the final number is still to come (see "Does it fit?"). The Python model is not synthesizable, so the RTL stands in for it; the z3 proof and lockstep tests tie the two together.

> **v3 (2026-11-17):** DMEM moved from flip-flops to the IHP SRAM macro (`docs/v3_design.md`), and a mailbox and two data movers were added. The numbers below are for the v2 design with memory clock gating, which placed but did not route. The v3 synthesis and place-and-route results will be added here.

The target is **6x4 tiles**: a die of 1289.28 × 710.64 µm = 916,220 µm², of which about **902,400 µm² is core** for cells.

## Setup

| Item | Value |
|---|---|
| Design | the `test` branch after the memory clock gating (`MEM_CLOCK_GATE = 1`): 1 instruction per cycle, 29-bit word, IMEM 162 words |
| Process | IHP SG13G2, PDK revision `c4b8b4e5`, typical corner 1.2 V 25 °C |
| Flow | LibreLane 3.0.5 (the version the `ttihp26b` GDS action installs) |
| Config | The `src/config_merged.json` that `tt_tool.py --create-user-config --ihp` generates: 6x4, 48 MHz, `AREA 0` |
| Lint and synthesis checks | 0 errors |

## History

Each step of the optimization so far, measured the same way (flat synthesis):

| Step | Commit | Cell area | Change |
|---|---|---:|---:|
| Starting point | `efd4314` | 755,159 µm² | |
| Pipelined fetch, 1 instruction per cycle | `f612094` | 760,053 µm² | +4,894 |
| `dly` field removed: 29-bit word, IMEM 162 words | `c07568a` | 721,200 µm² | −38,853 |
| Memory clock gating | this change | **580,697 µm²** | **−140,503** |
| **Total** | | | **−174,462 (−23%)** |

## Total

| Measure | Value |
|---|---|
| Cell area, flat synthesis | **580,697 µm²**: 569,321 µm² reported by the flow, plus 11,376 µm² for the 418 clock gates (see below) |
| Of which flip-flops | 388,000 µm² (67%), 7,915 flip-flops |
| Of which tie cells | 57,415 µm² (10%) |
| Of which clock gates | 11,376 µm², 418 × `sg13g2_lgcp_1` (27.2 µm² each) |
| **After detailed placement** | **604,810 µm², 67.0% of the core** |
| **After clock-tree synthesis** | **690,597 µm², 76.5% of the core** |
| Timing after clock-tree synthesis, slow corner (1.08 V, 125 °C) | setup +6.18 ns worst |

**The flow undercounts the clock gates.** The IHP PDK's LibreLane config lists `sg13g2_lgcp_1` in both `synth_exclude.cells` and `pnr_exclude.cells` (no reason given there). Synthesis keeps an instance written in the RTL, but it removes the cell from the library it counts area with, so its report says "Area for cell type sg13g2_lgcp_1 is unknown". Its 27.2 µm² is added back by hand above. Placement and clock-tree synthesis place, size and time the cells normally.

## Breakdown by module

From a second run with hierarchy kept (`SYNTH_HIERARCHY_MODE=keep`), because a flat netlist has no modules. Its total is 583,951 µm² (572,575 reported + 11,376 of clock gates), 0.6% above flat, as logic cannot be optimized across module boundaries. Each row is that instance alone, without its children, and the rows add up to the total. The memories' rows include their clock gates.

| Instance | Module | Area (µm²) | Share | Flip-flops | FF area | Tie area | Logic area | Clock gates |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `u_imem` | `pe_imem` | 308,391 | 52.81% | 4,727 | 232,000 | 34,300 | 38,139 | 162 = 4,409 |
| `u_dmem` | `pe_dmem` | 142,596 | 24.42% | 2,048 | 100,000 | 14,900 | 20,433 | 256 = 6,967 |
| `u_ctrl` | `pe_ctrl` | 31,343 | 5.37% | 193 | 9,450 | 1,400 | 20,480 | |
| `u_rxu` | `pe_rxu` | 24,326 | 4.17% | 239 | 11,700 | 1,730 | 10,880 | |
| `u_txu` | `pe_txu` | 13,099 | 2.24% | 106 | 5,190 | 849 | 7,052 | |
| `u_erx` | `pe_det` | 9,562 | 1.64% | 91 | 4,460 | 973 | 4,131 | |
| `u_regs` | `pe_regs` | 9,141 | 1.57% | 90 | 4,410 | 653 | 4,079 | |
| `u_etx` | `pe_det` | 7,920 | 1.36% | 74 | 3,630 | 631 | 3,663 | |
| `u_rxf` | `pe_fifo` | 7,120 | 1.22% | 78 | 3,820 | 566 | 2,736 | |
| `u_txf` | `pe_fifo` | 7,120 | 1.22% | 78 | 3,820 | 566 | 2,736 | |
| `u_rxu/u_vote` | `pe_vote` | 6,519 | 1.12% | 55 | 2,690 | 399 | 3,425 | |
| `u_crc` | `pe_crc` | 5,383 | 0.92% | 52 | 2,550 | 377 | 2,458 | |
| `u_pins` | `pe_pins` | 5,281 | 0.90% | 41 | 2,010 | 298 | 2,974 | |
| top level | `tt_um_sahil_protoemu` | 2,499 | 0.43% | 2 | 98 | 102 | 2,299 | |
| `u_host_if` | `pe_host_if` | 2,063 | 0.35% | 24 | 1,180 | 174 | 713 | |
| `u_pins/u_sync` | `pe_sync` | 1,045 | 0.18% | 16 | 784 | 116 | 145 | |
| `u_ctrl/u_decode` | `pe_decode` | 414 | 0.07% | 0 | 0 | 0 | 414 | |
| `u_host_if/u_sync_stb` | `pe_sync` | 131 | 0.02% | 2 | 98 | 15 | 18 | |
| **Total** | | **583,951** | **100%** | **7,916** | | | | **418 = 11,376** |

Grouped:

| Group | Instances | Area (µm²) | Share |
|---|---|---:|---:|
| Instruction memory | `u_imem` | 308,391 | 52.8% |
| Data memory | `u_dmem` | 142,596 | 24.4% |
| Controller | `u_ctrl`, `u_decode` | 31,757 | 5.4% |
| RX unit | `u_rxu`, `u_vote` | 30,845 | 5.3% |
| Edge detectors | `u_erx`, `u_etx` | 17,482 | 3.0% |
| FIFOs | `u_rxf`, `u_txf` | 14,239 | 2.4% |
| TX unit | `u_txu` | 13,099 | 2.2% |
| Registers and host bus | `u_regs`, `u_host_if`, `u_sync_stb` | 11,335 | 1.9% |
| Pins | `u_pins`, `u_sync` | 6,326 | 1.1% |
| CRC unit | `u_crc` | 5,383 | 0.9% |
| Top-level logic | | 2,499 | 0.4% |

## What drives it

- **Memory is still 77% of the design.** IMEM is 162 × 29 = 4,698 bits and DMEM is 256 × 8 = 2,048 bits, both in flip-flops.
- **Cost per bit,** with clock gating: about 66 µm² in IMEM and 70 µm² in DMEM, of which 49 µm² is the flip-flop and 7.3 µm² its tie cell. One IMEM word costs about 1,900 µm², or 0.21% of the core.
- **Clock gating removed the hold multiplexers.** A flip-flop takes its D input on every clock edge, and the IHP library has no flip-flop with an enable, so without gating each memory bit needs a 2:1 multiplexer feeding its own value back. With `MEM_CLOCK_GATE`, each IMEM word and DMEM byte has one clock gate that only lets an edge through when it is written. IMEM's logic fell from 150,218 to 38,139 µm² (what is left is the read multiplexer and the write decode), and DMEM's from 60,121 to 20,433 µm².
- **Every flip-flop has its own tie cell.** The only flip-flops the flow maps to have an asynchronous reset pin. The RTL uses synchronous reset, so each pin is tied high: 57,415 µm² in all.
- **The clock tree is bigger with gating.** Clock-tree synthesis builds a tree to the 418 gates and a small tree behind each one: 420 clock roots and 2,471 buffers, against 736 buffers without gating. It added 85,787 µm² (604,810 → 690,597), against 23,737 µm² without gating. Each gated row has only 29 (IMEM) or 8 (DMEM) flip-flops, so clock-tree settings may be able to cut this; not tried yet.

## Rolling back the clock gating

`MEM_CLOCK_GATE` (top level, default 1) selects it. Defining it as 0 gives the ungated memories, with the same behaviour:
- RTL: `-DMEM_CLOCK_GATE=0`
- tests: `make MEM_CG=0` in `test/`
- the flow: `"VERILOG_DEFINES": ["MEM_CLOCK_GATE=0"]` in `src/config.json`

Without it the design was 721,200 µm² after synthesis, and an estimated 87% of the core after clock-tree synthesis.

## The latch option for IMEM

`pe_imem` also has a latch version, `IMEM_IMPL = 1`, which needs no clock gates. Synthesized alone, at 162 × 29: 187,149 µm², against 416,125 µm² for flip-flops without gating and about 308,000 µm² with it. It is **not verified**:

- Its source marks it experimental and not yet checked in the IHP flow.
- It is proved at depths 16 and 128, not 162, and the chip regression and lockstep run on the flip-flop version.
- Its timing depends on the low phase of the clock, which place and route must confirm.
- It has not been checked against Tiny Tapeout's rules for latches.

## Does it fit?

**Probably, but not yet shown.** After clock-tree synthesis the cells use 76.5% of the core. The flow's target placement density is 60%, and its comments say values up to 80% have worked. Still to come, and each adds cells or needs space:

- hold repair after clock-tree synthesis (the hold slack is −0.05 ns typical and −0.12 ns fast before it, the same as without gating)
- global and detailed routing, where congestion shows up
- fill and the final checks, including how Tiny Tapeout's checks treat a cell the PDK excludes

The next step is a complete run to GDS. If it does not route, the clock-tree settings above and the options in `docs/future_improvements.md` (complex instructions, a smaller DMEM) are the next levers.

## Not covered

- **Routing.** Only placement and the clock tree have run.
- **Final timing.** Setup closes at the slow corner after the clock tree; hold repair and routed parasitics come later.
- **`AREA 0` is the flow's default strategy.** No other strategies were tried.

## To reproduce

Work on a copy of the repository, as this generates files in `src/`:

```sh
python tt/tt_tool.py --create-user-config --ihp
mkdir -p runs/flat runs/keep runs/cts
# inside ghcr.io/librelane/librelane:3.0.5, with PDK_ROOT set:
python3 -m librelane --pdk ihp-sg13g2 --run-tag flat --force-run-dir runs/flat \
    --to Yosys.Synthesis src/config_merged.json
# with "SYNTH_HIERARCHY_MODE": "keep" added to the config:
python3 -m librelane --pdk ihp-sg13g2 --run-tag keep --force-run-dir runs/keep \
    --to Yosys.Synthesis src/config_merged.json
# placement and clock tree:
python3 -m librelane --pdk ihp-sg13g2 --run-tag cts --force-run-dir runs/cts \
    --to OpenROAD.STAMidPNR-1 src/config_merged.json
```

The synthesis reports are in `runs/*/06-yosys-synthesis/reports/stat.rpt`; the placement and clock-tree metrics in `runs/cts/34-openroad-detailedplacement/` and `runs/cts/35-openroad-cts/`.
