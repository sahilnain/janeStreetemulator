# Tests

The chip tests drive `tt_um_sahil_protoemu` through its host bus with [cocotb](https://docs.cocotb.org/en/stable/), load firmware from `common/pe_fw.py`, and check the protocol pins against peers written in Python (UART, SPI, I2C and USB devices and hosts).

## Layout

| Path | Contents |
|---|---|
| `tb.v`, `tb_dual.v` | the test benches: one chip, or two chips wired together |
| `test.py` | registers, firmware loading, errors and IRQ |
| `test_uart.py`, `test_spi.py`, `test_i2c.py`, `test_usb.py`, `test_modes.py` | each protocol and its options |
| `test_fabric.py`, `test_gaps.py` | pin fabric, protocol switches, corner cases |
| `test_lockstep.py`, `test_cross.py` | the RTL against the Python model (`../model`) |
| `test_dual.py` | two emulators talking to each other (run with `TB=tb_dual`) |
| `common/pe_isa.py` | the assembler: instruction encodings, condition and unit numbers, config field helpers |
| `common/pe_fw.py` | all firmware, with the mailbox and DMEM conventions in its header |
| `common/pe_host.py` | the host-bus driver and request helpers (mailbox, DMEM, doorbell) |
| `common/lockstep.py` | runs the model next to the RTL and compares every register each cycle |
| `models/` | the PDK's behavioural model of the SRAM macro, for simulation |
| `unit/` | a bench for each module (`pytest` in that folder) |

## Running

```sh
make                          # every chip test, RTL simulation (Icarus)
make LOCKSTEP=1               # the same, comparing the RTL with the model on every cycle
make COCOTB_TEST_MODULES=test_spi        # one test file
make PROTOCOLS=uart,i2c       # a build with only some protocols; tests for the others are skipped
make DMEM_IMPL=0              # flip-flop data memory instead of the SRAM macro
make MEM_CG=0                 # memories without clock gating
make TB=tb_dual COCOTB_TEST_MODULES=test_dual MODULE=test_dual   # two chips
cd unit && pytest             # unit benches
```

Each combination of `PROTOCOLS`, `MEM_CG` and `DMEM_IMPL` builds in its own `sim_build/` folder.

## Gate-level simulation

After the GDS flow, copy the gate-level netlist to `gate_level_netlist.v` and run:

```sh
make -B GATES=yes
```

The SRAM macro's behavioural model is included in this build as well.

## Waveforms

The benches write `tb.fst` (or `tb_dual.fst`). View with `gtkwave tb.fst tb.gtkw` or `surfer tb.fst`. To write VCD instead, edit `tb.v` to use `$dumpfile("tb.vcd");` and run `make -B FST=`.
