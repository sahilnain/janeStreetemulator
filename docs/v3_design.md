# Protocol Emulator v3: SRAM data memory, mailbox and data movers

Decided 2026-11-16. The block-level definitions are in `spec/isa.md` (Sections 1, 3.4, 3.7) and `spec/blocks.md` (Sections 8, 9, 14).

## Why

The clock-gated v2 design (580,697 µm² after synthesis) placed at 79% of the 6x4 core, but did not route: detailed routing stalled near 6,000 violations, mostly shorts on Metal2, where wires meet cell pins. About 6,700 memory flip-flops and their read multiplexers crowd the pins. The competition rules allow the IHP SRAM macros, and Tiny Tapeout has proven one on silicon. DMEM was the first memory to move:

| DMEM 256 × 8 | Area |
|---|---:|
| Flip-flops with clock gates (v2) | 142,596 µm² |
| `RM_IHPSG13_1P_256x8_c3_bm_bist` (236.8 × 74.1 µm) | 17,547 µm² |

## What changed

1. **DMEM is the SRAM macro and holds payload only.** Bytes to send start at `dmem[0x00]`, received bytes at `dmem[0x80]`. The macro has one port and a registered read: the byte comes out the cycle after the read. `DMEM_IMPL` = 0 gives a flip-flop DMEM with the same behaviour, for roll-back.
2. **Control values go through a mailbox**: 8 registers of 16 bits (M0–M7), read and written by the host (0x30–0x3F) and by the firmware (`mbr`, `mbw`, opcodes 35 and 36). The list came from reading every program's requests and results and merging the common ones:

   | Reg | Name | Use |
   |---|---|---|
   | M0 | OP | operation (USB host); token PID (USB device result) |
   | M1 | ADDR | I2C target address (7 or 10 bits), USB device address |
   | M2 | NTX | bytes to send; USB device IN length (the firmware clears it); USB host count; UART break length |
   | M3 | NRX | bytes to receive; USB device endpoint (result) |
   | M4 | PID | data PID |
   | M5 | HS | USB device handshakes: OUT [7:0], IN [15:8] |
   | M6 | STATUS | status or event |
   | M7 | INFO | received length; UART parity errors |

   The instruction word stays 29 bits: the opcode field had room.
3. **Two data movers** (units 5 and 6) move payload between DMEM and the existing FIFOs on their own. The firmware gives a start address and a count and then only checks "done" (jump conditions 27–30). The TX mover reads ahead into a one-byte buffer, so the SRAM latency never reaches the line; it needs about 3 cycles a byte, and SPI at 24 MHz needs one every 16. The existing 8-entry FIFOs do the job a ping-pong buffer would. The CRC append (USB) waits until the TX mover is idle.
4. **One DMEM port, four users**, in this priority: host, RX mover, TX mover, controller.
5. **`ld` / `st` stay** for programs that need a byte in a register (16-bit SPI words, the I2C master, the I2C slave's register space). `ld` takes 2 cycles; `st` 1.
6. **Host DMEM reads are answered one cycle later**: the host bus waits for `h_ack` anyway, so nothing changes for the host software.

## Firmware

| Program | Payload | Control |
|---|---|---|
| `spi_master`, `spi_master_stream`, `spi_master_3wire` | TXM / RXM | mailbox |
| `spi_master_words` | `ld` / `st` (16-bit words) | mailbox |
| `i2c_master` | `ld` / `st` (bytes go through registers) | mailbox (a 10-bit address is one value now) |
| `i2c_slave` | `ld` / `st` (the register space) | none |
| `usb_device` | IN data by TXM; a received packet by RXM after its PID | mailbox |
| `usb_host` | token and data by TXM; a received packet by RXM after its PID | mailbox |
| `uart_ex` | FIFOs | mailbox (counters, break length) |

Every program still fits the 162-word IMEM, with room to spare, because one `mbr` replaces `mov` + `ld`:

| Program | v2 | v3 |
|---|---:|---:|
| `usb_device` | 162 | 149 |
| `usb_host` | 159 | 144 |
| `i2c_master` with `arb` | 155 | 154 |
| `i2c_master` with `arb` and `addr10` | 167 (did not fit) | 162 (fits; no test yet) |

The comments at the top of `test/common/pe_fw.py` explain the mailbox, the DMEM layout and the movers, with a table of what each program reads and writes.

## Verification added

- `tb_pe_dmem`: the same checks on flip-flops, clock-gated flip-flops and the SRAM macro's behavioural model (registered read, latency, neighbours, glitches).
- `tb_pe_dmov`: the block against the Python model every cycle under random traffic; TXM and RXM end to end with back-pressure and host interference; every arbitration combination.
- `tb_pe_regs` (mailbox, grant), `tb_pe_host_if` (late reads), `tb_pe_ctrl` (2-cycle `ld`, `mbr`, `mbw`, the new conditions).
- z3: `pe_dmem` and `pe_dmov` proved equal to the model; `pe_regs`, `pe_host_if` and `pe_ctrl` re-proved.
- Formal: `pe_dmov` (new), `pe_dmem`, `pe_ctrl`, `pe_host_if`, `pe_regs`, the chip.
- Chip tests in lockstep with the model, with the SRAM model (`make`) and with flip-flops (`make DMEM_IMPL=0`).
