![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Protocol Emulator

A small programmable I/O processor for Tiny Tapeout (IHP SG13G2, 6x4 tiles, 48 MHz). Hardware units put bits on the wires with exact timing, and a controller running firmware does the protocol rules between packets. UART, SPI, I2C and USB (low and full speed) are all programs; no block is specific to one of them.

- Datasheet: [docs/info.md](docs/info.md)
- How it works, module by module: [docs/architecture_guide.md](docs/architecture_guide.md)
- Instruction set: [docs/spec/isa.md](docs/spec/isa.md). Cycle-level block rules: [docs/spec/blocks.md](docs/spec/blocks.md)
- The v3 changes (SRAM data memory, mailbox, data movers): [docs/v3_design.md](docs/v3_design.md)
- Area: [docs/area.md](docs/area.md). Conformance to the protocol standards: [docs/protocol_conformance.md](docs/protocol_conformance.md)

## Layout

| Folder | Contents |
|---|---|
| `src/` | the RTL, and the Tiny Tapeout flow config (`config.json`, `pdn_cfg.tcl`) |
| `macro/` | the IHP SRAM macro used for the data memory (LEF, GDS, Liberty, CDL) |
| `model/` | a cycle-accurate Python model of the chip, and the z3 proofs that each RTL module matches it |
| `test/` | the chip tests (cocotb), unit benches for each module (`test/unit`), the assembler and all firmware (`test/common`) |
| `formal/` | SymbiYosys proofs of every module and of the whole chip |
| `conformance/` | the firmware run on the model against the protocol standards, with sigrok decoders |
| `docs/` | the documentation above |

## Running the checks

All tools come with the Tiny Tapeout dev container (`.devcontainer/Dockerfile`).

| Check | Command |
|---|---|
| Chip tests | `cd test && make` (options in [test/README.md](test/README.md)) |
| Chip tests in lockstep with the model | `cd test && make LOCKSTEP=1` |
| Unit benches | `cd test/unit && pytest` |
| z3 model-against-RTL proofs | `cd model && pytest tests` |
| Formal proofs | `cd formal && ./run_formal.sh` |
| Conformance | `cd conformance && ./run.sh` |

## About Tiny Tapeout

Tiny Tapeout is an educational project that makes it easier and cheaper to get digital and analog designs manufactured on a real chip. See https://tinytapeout.com.
