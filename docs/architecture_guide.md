# Protocol Emulator: architecture guide

This explains how the chip works, from the instruction set through each module to the registers. It describes the code as it is now. For exact cycle-level rules, see `docs/spec/isa.md` and `docs/spec/blocks.md`.

## 1. The idea

A protocol such as UART, SPI, I2C or USB has two parts:

- **Bits:** putting each bit on a wire at exactly the right time. At USB full speed a bit lasts 4 clock cycles.
- **Rules:** what happens between the bits. Send a START, wait for an ACK, check a CRC, decide what to answer.

The chip splits the work the same way:

- **Small hardware units do the bits.** They know nothing about any protocol. A unit is told "send 8 bits, 4 cycles each, with these options" and it does so with exact timing.
- **A small processor does the rules, in firmware.** It runs one instruction every 2 cycles (42 ns at 48 MHz), far too slow to time fast bits itself. It only has to act between packets.

So UART, SPI, I2C and USB are all just different programs. No block in the hardware is specific to one of them. The closest are generic options such as NRZI coding or a CRC register, which firmware can switch on.

## 2. The chip at a glance

```
         external host (a microcontroller)
                  │ 4-bit bus + IRQ
           ┌──────▼──────┐
           │ pe_host_if  │  nibbles → one register access
           └──────┬──────┘
           ┌──────▼──────┐
           │   pe_regs   │  the host's register map
           └┬───┬───┬───┬┘
   program  │   │   │   │  mailbox
       ┌────▼─┐ │   │ ┌─▼──────┐
       │ IMEM │ │   │ │  DMEM  │
       └────┬─┘ │   │ └─▲──────┘
            │   │   │   │ ld / st
       ┌────▼───┼───┼───┴───┐
       │    pe_ctrl         │  the processor: runs the firmware
       │    (pe_decode)     │
       └─┬───┬──┼───┼───┬───┘
  commands│   │ ┌▼───▼┐  │ commands
          │   │ │FIFOs│  │  TX FIFO, RX FIFO (byte queues)
          │   │ └┬───┬┘  │
     ┌────▼┐  │  │   │  ┌▼────┐     ┌───────┐
     │ ETX ├──┼─►│   │◄─┤ ERX │     │pe_crc │ checks bits as they pass
     └──┬──┘  │  │   │  └──┬──┘     └───▲───┘
  start │     │  │   │     │ start      │
     ┌──▼─────▼──▼┐ ┌▼─────▼─────┐      │
     │  pe_txu    │ │  pe_rxu    ├──────┘
     │  TX unit   │ │  RX unit   │
     └──────┬─────┘ └─────┬──────┘
            │  writes     │  reads pin_in, writes in half duplex
       ┌────▼─────────────▼────┐
       │       pe_pins         │  owner check, open drain, sync, filter
       └───────────┬───────────┘
                   │
              8 protocol pins, uio[7:0]
```

**Pins.** These come from the Tiny Tapeout wrapper, `src/tt_um_sahil_protoemu.v`:

| Pins | Use |
|---|---|
| `ui[3:0]` | host data in |
| `ui[5:4]` | host operation |
| `ui[6]` | host strobe (toggles) |
| `uo[3:0]` | host data out |
| `uo[4]` | host ack (toggles) |
| `uo[5]` | IRQ |
| `uio[7:0]` | protocol pins P0–P7 |
| `ui[7]`, `uo[7:6]` | reserved |

**The memories and queues:**

| Name | Size | Holds |
|---|---|---|
| IMEM | 160 × 32 bits | the firmware |
| DMEM | 256 × 8 bits | requests and results between the host and the firmware |
| TX FIFO, RX FIFO | 8 × 8 bits each | bytes going to and coming from the units |

## 3. The instruction set

### 3.1 What the processor has

| Item | Size | Notes |
|---|---|---|
| `pc` | 8 bits | 160 words need 8 bits. A pc past the end reads as `nop` |
| `r0`–`r7` | 16 bits each | general registers, all equal |
| `Z`, `C` | 1 bit each | zero and carry, set by the ALU |
| `TO` | 1 bit | a `wait` timed out |
| `DB` | 1 bit | doorbell from the host |
| return stack | 4 levels | for `call` and `ret` |
| stall counter | 17 bits | counts down the delay after an instruction |

It also reads flags that live elsewhere: `ETX`, `ERX`, `FERR`, `RXRDY`, and the state of the units and FIFOs.

### 3.2 The instruction word

Every instruction is 32 bits:

```
 31      26 25  23 22  20 19 18  16 15                 0
┌──────────┬──────┬──────┬──┬──────┬────────────────────┐
│  opcode  │ dly  │  A   │I │  B   │       imm16        │
└──────────┴──────┴──────┴──┴──────┴────────────────────┘
```

| Field | Meaning |
|---|---|
| `opcode` | which instruction. 34 of 64 are used; a reserved one acts as `nop` and sets `ill_err` |
| `dly` | 0–7 extra cycles after this instruction |
| `A` | usually the destination register, or a unit number |
| `I` | 1: the operand is `imm16`. 0: the operand is the register `imm16[2:0]` |
| `B` | a sub-operation, second register or config register number |
| `imm16` | a number, a jump target, or pin fields |

Jumps use `imm16` as condition `[15:11]` and target `[9:0]`. Pin instructions use pin `[15:12]` and level `[11]`.

**Example:** `mov r3, 0x80` has opcode 7, A = 3, I = 1, imm16 = 0x80:

```
(7 << 26) | (3 << 20) | (1 << 19) | 0x80 = 0x1C380080
```

This is what the assembler in `test/common/pe_isa.py` does. In firmware it is written `a.mov(3, 0x80)`.

### 3.3 Timing

- **Every instruction takes 2 cycles.** The memory read takes one cycle and execution the next.
- **A delay** of `n` extra cycles takes `2 × ceil((n + 1) / 2)` in total, so delays round up to an even number.
- **A blocked instruction** has no effect and is tried again every 2 cycles. `wait`, `uwait`, `evt wait`, and `fget` or `fput` when the FIFO is empty or full all block. This is how firmware idles: there is no halt instruction.

### 3.4 The instructions

**Flow**

| Op | Instruction | What it does |
|---|---|---|
| 0 | `nop` | nothing |
| 1 | `jmp [cond,] target` | jump, if the condition holds |
| 2 | `djnz rd, target` | `rd - 1`; jump if not zero |
| 3 | `call target` | push the return address and jump |
| 4 | `ret` | pop the return address |
| 6 | `delay op2` | wait `op2` extra cycles |

`op2` means `imm16` or a register, depending on `I`. A `call` on a full stack or a `ret` on an empty one acts as `nop` and sets `stk_err`.

**Jump conditions**

| Condition | True when |
|---|---|
| `Z`, `C` | the flag is set |
| `TO` | the last `wait` timed out |
| `ETX`, `ERX` | that detector has fired |
| `TXU_BUSY`, `RXU_BUSY` | that unit is running |
| `FERR` | the last stop-bit check failed |
| `TXF_EMPTY`, `TXF_FULL`, `RXF_EMPTY`, `RXF_FULL` | FIFO state |
| `DB` | the host rang the doorbell |
| `RXRDY` | the RX unit holds a finished word |

Most have an opposite too (`NZ`, `NC`, `NERX`, `TXU_IDLE`, and so on), except the two "full" conditions.

**Registers and ALU** (16 bits)

| Op | Instruction | What it does | Flags |
|---|---|---|---|
| 7 | `mov rd, op2` | copy | none |
| 8, 9 | `add`, `sub rd, op2` | add, subtract | Z, C |
| 10–12 | `and`, `or`, `xor rd, op2` | logic | Z |
| 13, 14 | `shl`, `shr rd, n` | shift by `n` | Z, C = last bit out |
| 15 | `cmp rd, op2` | subtract, keep only the flags | Z, C = `rd < op2` |
| 34 | `par rd` | parity | C = odd number of 1s |

**Pins**

| Op | Instruction | What it does |
|---|---|---|
| 16 | `set pin, src` | drive the pin to 0, 1, C or ~C |
| 17 | `setoe pin, v` | output enable on or off |
| 18 | `wait pin, lvl [, rd]` | block until the pin has that level; with `rd`, time out and set `TO` |
| 19 | `jpin pin, lvl, target` | jump if the pin has that level |
| 20 | `rdpins rd` | read all pins |
| 21, 22 | `bitin`, `bitout` | move one bit between a pin and a register |

None of the current firmware uses `rdpins`, `bitin`, `bitout` or `xor`.

**FIFOs and data memory**

| Op | Instruction | What it does |
|---|---|---|
| 23 | `fget rd, fifo [,nb]` | take a byte. Blocks if empty; with `nb`, sets C instead |
| 24 | `fput rs, fifo [,nb]` | add a byte. Blocks if full; with `nb`, drops it and sets C |
| 25 | `ld rd, [ra] [,+]` | read DMEM, optionally `ra + 1` |
| 26 | `st rs, [ra] [,+]` | write DMEM, optionally `ra + 1` |

**Units and events**

| Op | Instruction | What it does |
|---|---|---|
| 27 | `ucfg u, reg, op2` | write a unit's config register |
| 28 | `urd rd, u, reg` | read a unit's status or data |
| 29 | `ustart mask [, op2]` | start one packet on TX, RX or both, in the same cycle |
| 30 | `ustop mask` | abort at once |
| 31 | `uwait mask` | block until the units are idle |
| 32 | `evt d, sub` | on a detector: `wait`, `waitc` (wait and clear), `clr`, `arm`, `disarm` |
| 33 | `host sub` | `irq` raises the IRQ to the host; `dbclr` clears the doorbell |

`ustart` is the key instruction: **one start sends or receives one packet, then the unit stops.** That gives the firmware its chance to do the next step. The exception is CHAIN (Section 4.5), for streams that must not pause.

## 4. The modules

The order follows an instruction from the host to the wires. The numbers in brackets are flip-flops, from `docs/area.md`.

### 4.1 `pe_host_if` (24, plus 2 in its synchroniser): the host bus

It turns a slow 4-bit bus into one-cycle register accesses. The host can be much slower than the chip and on a different clock, because of the handshake: the host sets the operation and data, then toggles the strobe. The chip synchronises the strobe, performs the operation and toggles ack.

| `h_op` | Name | Effect |
|---|---|---|
| 0 | ADDR | shift a nibble into the address. Two make an address |
| 1 | WRITE | the first nibble is the high half; the second completes a write |
| 2 | READ | the first read fetches the byte and returns the high half; the second returns the low half |
| 3 | SYNC | reset the nibble phase, if the host lost track |

The address does not increment, so reading a FIFO repeatedly only needs the data operations.

### 4.2 `pe_regs` (90): the host's view

The host's register map (Section 5.3). It holds RUN, the pin configuration and the error flags, and turns accesses into actions: push to a FIFO, write IMEM, ring the doorbell.

**Loading firmware.** A word is 32 bits but the bus carries bytes, so 3 bytes wait in a staging register, and writing the fourth (0x13) stores the whole word and advances the address. A store while RUN = 1 is dropped and sets `imem_err`, so firmware cannot change while it runs.

### 4.3 `pe_imem` (5,152): the program

160 words of 32 bits, one write port for the host and one read port for the controller. The read is registered: the word arrives one cycle after the address, which is why an instruction takes 2 cycles. The 32 extra flip-flops are that output register. It is built from flip-flops; a latch version exists but is not verified (`docs/area.md`).

### 4.4 `pe_ctrl` (194) and `pe_decode` (0): the processor

`pe_decode` is pure logic that splits a word into its fields. `pe_ctrl` does everything else:

- **Issue:** a `slot` bit alternates each cycle, and an instruction can only start in one of the two. This is the 2-cycle rhythm.
- **Execute:** all effects happen in one cycle, called E: registers, pins, FIFOs and unit commands.
- **Block:** if an instruction cannot finish, it has no effect at all and is retried, so a blocked `fget` can never half-happen. A blocked `wait` with a timeout is the one exception: it counts its register down.
- **Stall:** `dly` or `delay` loads the stall counter.
- **Restart:** the host's RESTART loads `pc` from START_PC and clears the registers, flags and stack. It aborts the units and disarms the detectors. Config registers survive.

### 4.5 `pe_txu` (106): the TX unit

Sends one packet of 1–16 bits. It takes the word from the TX FIFO or from `ustart`'s operand.

**Timing.** With start cycle S and B cycles per bit, bit `i` is written at `S + i × B`, exactly. Software cannot disturb that.

**Options**, all set with `ucfg` and none protocol-specific:

| Option | What it does | Used for |
|---|---|---|
| clock output (CPOL, CPHA) | makes a clock on a second pin | SPI, I2C |
| STRETCH | holds the timer until the clock pin really reads high | I2C clock stretching |
| external clock | advances on edges of a pin, not a timer | SPI slave, I2C slave |
| NBITS, MSB, INV | word size, bit order, inverted line | all |
| NRZI | a 0 toggles the line, a 1 keeps it | USB |
| STUFFN | an extra 0 after N 1s in a row | USB |
| DIFF | a second pin with the opposite level | USB |
| EOP | at the end, both pins low for 2 bits, then idle for 1 | USB |
| CHAIN | the next FIFO byte follows with no gap | USB, SPI streaming |

**Ticks.** It sends the RX unit a *sample tick* at each sampling edge and a *shift tick* at each data change. This keeps both units in step for SPI and I2C masters.

### 4.6 `pe_rxu` (239) and `pe_vote` (55): the RX unit

Receives one packet of 1–16 bits. The main choice is what tells it when to sample (`SRC`):

| SRC | Timing comes from | Used for |
|---|---|---|
| 0 | its own timer, with `pe_vote` | UART |
| 1 | the TX unit's sample tick | SPI and I2C masters |
| 2 | edges on a pin | SPI and I2C slaves |
| 3 | its own timer, re-timed on every data edge (clock recovery) | USB |

**`pe_vote`** takes N samples across a bit and returns the majority, ignoring K samples at each end, so a noise spike cannot flip the bit. That is what lets UART survive a noisy or slightly off-speed line.

**Drive mode.** It can drive a pin with a word while sampling that pin. I2C writes use this: the unit drives SDA and reads it back.

**Line decoding** undoes the TX options, in this order: DIFF, INV, NRZI, HUNT (drop samples until the sync pattern ends), STUFFN (drop stuff bits, and flag a stuff error).

**Result:**
- `HOLD = 0`: the byte goes straight to the RX FIFO.
- `HOLD = 1`: the word waits in a register and sets `RXRDY`, so firmware can check it first, then keep or drop it.

### 4.7 `pe_det` (ETX 74, ERX 91): the event detectors

Two copies of one module, each paired with one unit. A detector handles what firmware is too slow for.

**1. It watches a pin.** It fires on a level, or an edge, that holds for DUR cycles. An optional second pin qualifies it, so "SDA changes while SCL is high" can be one condition: that is how I2C START and STOP are caught.

**2. When it fires,** it always sets its flag, and optionally:
- **START:** starts its unit in that same cycle. A UART start bit can begin reception with no firmware involved.
- **STOP:** stops its unit in that same cycle. USB end-of-packet uses this, so no bit is lost or added.
- **AUTOARM:** arms itself again after the packet, so reception keeps running in hardware.

**3. After its unit's packet,** it can do one more thing:
- **ETX:** drive the data pin to 0 or 1, or release it, at the exact end of the last bit. This is the UART stop edge and the USB bus release.
- **ERX:** sample the pin ADLY cycles later and set `FERR` if it is wrong. This is the UART stop-bit check.

The detector-to-unit delay is 2 cycles (`k = 2`, from the pin synchroniser). The firmware's timing constants allow for it.

### 4.8 `pe_crc` (52): the CRC unit

One 16-bit CRC register with any polynomial, fed with the TX or RX unit's data bits, after stuff bits are removed. It can skip the first bits (USB's sync and PID are not in the CRC). With APPEND, the TX unit sends the inverted CRC when the FIFO runs out. On receive, the firmware reads the register and compares it with the known residual value. A CRC shorter than 16 bits is aligned to the top, so USB's CRC5 uses the same register.

### 4.9 `pe_pins` (41, plus 16 in its synchroniser): the pin fabric

Everything that touches a wire goes through here.

- **Outputs:** each pin has an `out` and an `oe` register, which keep their value until written.
- **Ownership:** each pin belongs to the controller, the TX unit or the RX unit. A unit can only write its own pins. The controller can also write a unit's pin **while that unit is idle**. Firmware uses this for I2C START and STOP on pins the units own. Any other write is dropped and sets `owner_err`.
- **Open drain:** with `PIN_OD` set, a 0 pulls low and a 1 releases, as I2C requires. `oe` is ignored.
- **Inputs:** `pe_sync` passes each input through 2 flip-flops, since the outside world is not on this clock. Every block reads this same `pin_in`.
- **Filter:** with `PIN_FILT` set, a new level must be seen on 4 samples in a row. That rejects I2C's 50 ns spikes and adds 4 cycles of delay.

### 4.10 `pe_fifo` (78 each) and `pe_dmem` (2,048): queues and memory

**FIFOs** have fixed priority, so the units and the host never wait; the controller blocks and retries instead:

| FIFO | Writers, highest first | Readers, highest first |
|---|---|---|
| TX | host, controller | units, controller |
| RX | RX unit, controller | host, controller |

A host write to a full TX FIFO or a unit push to a full RX FIFO is dropped and flagged.

**DMEM** is the mailbox. The host has priority, and a controller `ld` or `st` in the same cycle retries.

### 4.11 The top level

`tt_um_sahil_protoemu` holds the reset synchroniser and the glue between blocks:
- One TX FIFO pop serves every unit starting in a cycle, so TX and RX in one `ustart` get the same byte.
- A detector start with no byte available is dropped and sets `tx_underrun`.
- RX counts as busy through the ERX stop-bit check.
- In CHAIN, the next TX word comes from the FIFO, or the CRC if that is empty and APPEND is set.

### 4.12 Protocol switches

`SUPPORT_SPI`, `SUPPORT_I2C` and `SUPPORT_USB` (top-level macros, default 1) leave a protocol's features out of the build. UART is always built. A feature that is left out keeps its config field at 0, as if firmware never set it, and synthesis removes the logic behind it. To test a reduced build: `make PROTOCOLS=uart,i2c`. Details: `spec/blocks.md` Section 13.

## 5. The registers

There are five groups, depending on who can see them.

### 5.1 Firmware's registers

`r0`–`r7`, `Z`, `C`, `TO`, `DB`, `pc`, stack and stall counter, as in Section 3.1. The hardware gives `r0`–`r7` no special meaning. Each program chooses its own:

| Program | r0 | r1 | r2 | r3 | r4 | r5 | r6 | r7 |
|---|---|---|---|---|---|---|---|---|
| `uart` | byte in | | | | | | | TX state |
| `uart_ex` | word | scratch | parity test | | | break count | parity errors | |
| `spi_master` | byte out | data pointer | bytes left | reply pointer | byte in | | | |
| `spi_master_stream` | byte out | data pointer | bytes to send | reply pointer | byte in | replies left | status | |
| `spi_master_3wire` | byte out | data pointer | writes left | reply pointer | byte in | reads left | | |
| `spi_slave_words` | next word | scratch | scratch | | word in | | | |
| `i2c_master` | byte | write pointer | writes left | read pointer | address | result, counter | status, busy flag | reads left |
| `i2c_slave` | byte | scratch | pointer-next flag | register pointer | | | 10-bit match | |
| `usb_device` | byte, event | token byte, handshake | token byte, EP | pointer | CRC, status | IN length, timeout | PID | our address |
| `usb_host` | byte | status | | pointer | CRC, status | count, timeout | data PID | |

`spi_slave` uses no registers, `spi_master_words` uses the same ones as `spi_master`, and `spi_slave_3wire` only uses `r0`. The comments in `test/common/pe_fw.py` give these for each program.

### 5.2 Unit config registers

Firmware writes these with `ucfg` and reads status with `urd`. They are 16 bits and keep their values through RESTART. The unit numbers are TX 0, RX 1, ETX 2, ERX 3, CRC 4.

**TX unit (0)**

| # | Name | Bits |
|---|---|---|
| 0 | PINS | `[3:0]` DPIN data pin, `[4]` DEN uses it, `[11:8]` CPIN clock pin, `[12]` CEN uses it |
| 1 | BIT | `[15:0]` B, cycles per bit |
| 2 | FRAME | `[0]` MSB first, `[2]` INV, `[8:4]` NBITS (0 means 8), `[9]` CHAIN |
| 3 | CLK | `[0]` CPOL, `[1]` CPHA, `[2]` STRETCH |
| 4 | TICK | `[0]` EXT external clock, `[7:4]` EPIN, `[8]` EEDGE (0 rising, 1 falling), `[9]` LEAD |
| 5 | LINE | `[0]` NRZI, `[1]` EOP, `[2]` DIFF, `[7:4]` D2PIN second pin, `[11:8]` STUFFN |
| read 0 | STAT | `[0]` busy, `[7:4]` bits sent |

**RX unit (1)**

| # | Name | Bits |
|---|---|---|
| 0 | PINS | `[3:0]` IPIN sampled pin, `[7:4]` I2PIN, `[11:8]` OPIN driven pin, `[12]` OEN drive mode, `[13]` DIFF |
| 1 | BIT | `[15:0]` B, cycles per bit |
| 2 | FRAME | `[0]` MSB, `[1]` HOLD, `[2]` INV, `[3]` CHAIN, `[8:4]` NBITS |
| 3 | VOTE | `[3:0]` N samples per bit, `[7:4]` K ignored at each end |
| 4 | STEP | `[15:0]` cycles between vote samples |
| 5 | FIRST | `[15:0]` cycles from the start to bit 0's window |
| 6 | TICK | `[1:0]` SRC, `[7:4]` EPIN, `[8]` EEDGE, `[15:12]` SDLY sample delay |
| 7 | ODLY | `[3:0]` drive delay, `[4]` NRZI, `[5]` HUNT, `[11:8]` STUFFN |
| read 0 | DATA | the held word; reading clears RXRDY |
| read 1 | STAT | `[0]` busy, `[1]` RXRDY, `[2]` FERR, `[3]` overrun, `[4]` stuff error, `[12:8]` bits of the current word |

**Detectors: ETX (2) and ERX (3)**

| # | Name | Bits |
|---|---|---|
| 0 | COND | `[3:0]` PIN, `[5:4]` MODE (0 level, 1 rising, 2 falling, 3 any edge), `[6]` LVL, `[7]` START, `[8]` AUTOARM, `[12:9]` QPIN qualifier, `[13]` QEN, `[14]` QLVL, `[15]` STOP |
| 1 | DUR | `[15:0]` cycles the condition must hold |
| 2 | AFTER | ETX `[1:0]`: 0 none, 1 drive 0, 2 drive 1, 3 release. ERX: `[0]` CHECK, `[1]` expected level |
| 3 | ADLY | ERX only: `[15:0]` cycles from the unit's end to the check |
| read 0 | STAT | `[0]` armed, `[1]` flag, `[2]` arm pending, `[3]` waiting on its unit or after-step, `[4]` FERR |

**CRC unit (4)**

| # | Name | Bits |
|---|---|---|
| 0 | CTRL | `[1:0]` SRC (0 off, 1 TX, 2 RX), `[2]` APPEND, `[15:8]` SKIP bits |
| 1 | POLY | `[15:0]` polynomial |
| 2 | INIT | `[15:0]` start value. Writing it loads the CRC, so write CTRL and POLY first |
| read 0 | value | the CRC register |
| read 1 | STAT | `[0]` the CRC has been sent |

### 5.3 The host's register map

The external host reaches these over the 4-bit bus. Addresses that are not mapped read 0.

| Addr | Name | Access | Meaning |
|---|---|---|---|
| 0x00 | ID | R | 0x50 |
| 0x01 | VERSION | R | 0x02 |
| 0x02 | RUN | RW | `[0]` the controller runs |
| 0x03 | RESTART | W | `[0]` = 1: restart the controller, units and detectors |
| 0x04 | STATUS | R | `[0]` txf_empty, `[1]` txf_full, `[2]` rxf_empty, `[3]` rxf_full, `[4]` txf_overrun, `[5]` rxf_overrun, `[6]` owner_err, `[7]` imem_err |
| 0x05 | ERR_CLR | W | write 1 to clear STATUS bits 4–7 |
| 0x06 | IRQ_EN | RW | `[0]` RX FIFO not empty, `[1]` TX FIFO not full, `[2]` any error, `[3]` firmware `host irq` |
| 0x07 | FIFO_CNT | R | `[3:0]` TX count, `[7:4]` RX count |
| 0x08 | TXF_DATA | W | push to the TX FIFO |
| 0x09 | RXF_DATA | R | pop from the RX FIFO |
| 0x0A | IMEM_ADDR | RW | word address `[7:0]`, advances on each stored word |
| 0x0B | PIN_IN | R | `pin_in[7:0]` |
| 0x0C | PIN_IN_H | R | `pin_in[15:8]`, 0 with 8 pins |
| 0x0D | IMEM_ADDR_H | RW | word address `[9:8]` |
| 0x0E | STATUS2 | R, write 1 to clear | `[0]` ill_err, `[1]` stk_err, `[2]` rx_overrun, `[3]` tx_underrun, `[4]` ctrl_irq |
| 0x0F | DOORBELL | W | any write sets the controller's `DB` flag |
| 0x10–0x13 | IMEM_DATA | W | instruction bytes, low first. Writing 0x13 stores the word |
| 0x14–0x15 | PIN_OD | RW | open-drain enable, one bit per pin |
| 0x16–0x17 | PIN_FILT | RW | input filter enable, one bit per pin |
| 0x18–0x1B | PIN_OWNER | RW | 2 bits per pin: 0 controller, 1 TX unit, 2 RX unit, 3 controller |
| 0x20–0x21 | START_PC | RW | where RESTART starts |
| 0x22–0x23 | PC | R | current `pc` |
| 0x24 | CSTAT | R | `[0]` run, `[1]` blocked, `[2]` Z, `[3]` C, `[4]` TO, `[5]` ETX, `[6]` ERX, `[7]` DB |
| 0x28 | DMEM_ADDR | RW | DMEM address |
| 0x29 | DMEM_DATA | RW | read or write DMEM, then address + 1 |

The IRQ pin is high when any enabled source is active:

```
(IRQ_EN[0] & RX FIFO not empty) | (IRQ_EN[1] & TX FIFO not full)
  | (IRQ_EN[2] & an error in STATUS[7:4] or STATUS2[3:0]) | (IRQ_EN[3] & ctrl_irq)
```

**Starting a program**, as the test benches do:

1. Write RUN = 0.
2. Load the words through IMEM_ADDR and IMEM_DATA.
3. Set PIN_OD, PIN_FILT and PIN_OWNER.
4. Write START_PC, then RESTART = 1, then RUN = 1.

The firmware then sets up the units with `ucfg` by itself.

### 5.4 DMEM maps: the host and firmware agree here

These are conventions set by the firmware, not hardware registers. DMEM-driven programs follow this pattern: the host writes a request, writes DOORBELL, and waits for the IRQ.

| Program | Request | Result |
|---|---|---|
| `spi_master` | `[0]` n, `[1..n]` data | `[0x80..]` replies |
| `spi_master_words` | `[0]` n, then 2 bytes a word, low first | `[0x80..]` 2 bytes a reply |
| `spi_master_stream` | `[0]` n, `[1..n]` data | `[0x80..]` replies. `[0x7F]`: 0 ok, 1 SCK paused, 2 reply lost |
| `spi_master_3wire` | `[0]` writes, `[1]` reads, `[2..]` data | `[0x80..]` replies |
| `i2c_master` | `[0]` address, `[1]` writes, `[2]` reads, `[3..]` data. With `addr10`, `[0x7E]` is address bits 9:8 | `[0x80..]` read data. `[0x7F]`: 0 ok, 1 address NACK, 2 data NACK, 3 arbitration lost, 4 SDA stuck low |
| `uart_ex` | `[3]` break length; the doorbell sends a break | `[1]` parity errors, `[2]` breaks |
| `usb_host` | `[0]` operation (0 transaction, 1 reset, 2 keep-alive, 3 resume), `[1..3]` token, `[4]` data PID (0 for IN), `[5]` data count or low byte of a time count, `[6]` its high byte, `[0x10..]` data | `[0x80]` first byte received, `[7]` status (0 OK, 1 time-out, 2 bad packet), `[8]` bytes received |
| `usb_device` | `[0]` address, `[4]` IN length (PID + data, 0 = NAK), `[5]` IN PID, `[6]` OUT handshake, `[7]` IN handshake, `[0x10..]` IN data | `[1]` token, `[2]` endpoint, `[3]` length, `[8]` event (1 OUT, 2 IN acked, 3 reset, 4 IN not acked), `[0x80..]` packet received |

`i2c_slave` uses DMEM as its register space, and general-call data goes to `[0xF0..]`. `uart`, `uart_ex` and the SPI slaves move their data through the FIFOs.

### 5.5 Internal state

These cannot be accessed directly, apart from what the status registers show. They are mostly copies of the config registers (named `c_*`), plus:

| Module | Registers |
|---|---|
| `pe_ctrl` | `pc`, `r[0:7]`, `z c to db`, `stack`, `sp`, `stall`, `slot`, `blocked` |
| `pe_txu` | `sh` shift register, `idx` bit index, `o` cycle within the bit, `sw swc` stretch wait, `sp` sample tick pending, `lvl` last NRZI level, `ones` 1s in a row, `eph` EOP phase, `crc_pkt` sending the CRC, `lf` LEAD state, `eprev` last level of the external clock pin |
| `pe_rxu` | `sh` shift register, `od` word to drive, `data_q` held word, `nbits` bits taken, `wcnt` cycles to the next sample, `wins` windows started, `tq oq` delay lines, `lp` last NRZI sample, `hunting hz` sync hunt, `rones` 1s in a row, `serr` stuff error, `rxrdy`, `overrun`, `eprev iprev` last pin levels |
| `pe_vote` | `running`, `sel`, `n_l k_l s_l` latched settings, `idx`, `ones`, `scnt` |
| `pe_det` | `armed`, `pend` arm pending, `mine` it started this packet, `prev` last pin level, `hold hlvl` edge mode state, `cnt` duration count, `apend acnt` after-step and its count, `flag`, `ferr` |
| `pe_crc` | `crc`, `skip`, `sent` |
| `pe_pins` | `out_r`, `oe_r`, `filt_out filt_c0 filt_c1` filter state, `owner_err` |
| `pe_fifo` | `mem`, `wptr`, `rptr`, `cnt`, `overrun` |
| `pe_regs` | `run`, `start_pc`, `pin_owner`, `pin_od`, `pin_filt`, `dm_addr`, `imaddr`, `stage`, `irq_en`, `imem_err`, `st2` |
| `pe_host_if` | `r_addr`, `wbuf wphase` write nibbles, `rlow rphase` read nibbles, `stb_q`, `h_dout`, `h_ack` |

## 6. Three examples, end to end

### 6.1 Receiving a UART byte (`uart`)

No instruction runs during the byte.

1. **Setup:** RX with `HOLD`, voting, and ERX set to "falling edge, low for half a bit, then START with AUTOARM".
2. **Start bit:** RXD falls. After half a bit, ERX fires and starts the RX unit in that cycle.
3. **Data bits:** the RX unit votes across each bit, B cycles apart.
4. **Stop bit:** after the last bit, ERX samples the pin ADLY cycles later, in the middle of the stop bit, and sets or clears `FERR`.
5. **Hand-over:** `RXRDY` rises and ERX re-arms itself.
6. **Firmware:** its loop sees `RXRDY`, reads DATA with `urd`, skips the byte if `FERR` is set, and otherwise does `fput` to the RX FIFO.
7. **Host:** the IRQ, if enabled in IRQ_EN, tells it the RX FIFO is not empty, and it reads RXF_DATA.

### 6.2 An I2C write, single master (`i2c_master`)

1. **Request:** the host writes address and data to DMEM, then DOORBELL.
2. **Check:** the firmware sees `DB` and reads the request. If SDA is already low, it runs the bus clear first.
3. **START:** SCL and SDA belong to the units, but the units are idle, so the firmware may drive them. It pulls SDA low while SCL is high.
4. **A byte:** `ustart TX|RX`. TX makes 8 SCL clocks with STRETCH, so a slave can hold the clock. RX in drive mode puts each bit on SDA a little after SCL falls. Both pins are open drain.
5. **ACK:** `uwait`, then the firmware releases SDA, raises SCL by hand and reads SDA with `jpin`.
6. **STOP:** after the last byte, SDA rises while SCL is high.
7. **Result:** the status goes to `dmem[0x7F]`, and `host irq` tells the host.

The units time the bits, and the firmware only acts between bytes.

### 6.3 A USB IN transaction, chip as device (`usb_device`)

The hard part is time: the device must answer within 6.5 bit times (26 cycles, or 13 instructions).

1. **Hunting:** RX runs with NRZI, HUNT, STUFFN and clock recovery. ERX watches for SE0.
2. **Token:** the sync is dropped in hardware and bytes reach the FIFO. SE0 stops the unit in the same cycle.
3. **Check:** the firmware reads the PID and the address byte, reads the CRC register, and compares it with the CRC5 residual. It checks the address, then sees PID = IN.
4. **Answer:** it starts sending the sync **before** deciding between data and NAK, since both begin the same way. That buys time.
5. **Data:** with bytes ready, it sets up the CRC with APPEND, then sends the PID and data through the FIFO. CHAIN sends them with no gap, the unit adds the CRC, and EOP ends the packet. ETX releases the bus.
6. **Handshake:** it hunts again and waits for K for less than 18 bit times. ACK clears `UD_INLEN`, and the event goes to DMEM with the IRQ.

## 7. The firmware

All of it is in `test/common/pe_fw.py`, in the same shape: set up, wait for work, run the protocol, report.

| Function | What it is |
|---|---|
| `uart` | full duplex, 8 data bits, hardware-timed start bit |
| `uart_ex` | 5–9 data bits, any parity, 1, 1.5 or 2 stop bits, inverted line, break |
| `spi_master` | bytes, all 4 modes |
| `spi_master_words` | words of 1–16 bits |
| `spi_master_stream` | no gap between bytes |
| `spi_master_3wire`, `spi_slave_3wire` | one data pin |
| `spi_slave` | words of 1–8 bits, all 4 modes |
| `spi_slave_words` | words of 1–16 bits |
| `i2c_master` | with options for multi-master, 10-bit addressing and Hs mode |
| `i2c_slave` | register-style device, with options for 10-bit addressing and general call |
| `usb_device`, `usb_host` | low and full speed, at transaction level |

Options are Python arguments: they choose which instructions get assembled, so each build only costs IMEM for what it uses.

## 8. How it was built and checked

1. **Python model** (`model/pe_model`): a cycle-accurate model of the chip, in plain Python.
2. **RTL** (`src/`): the Verilog of the chip.
3. **Model against RTL:**
   - A z3 proof (`model/equiv`) shows each module's next state matches the model's, for all inputs.
   - Lockstep (`make LOCKSTEP=1`) compares every flip-flop on every cycle during the tests.
4. **RTL on its own:**
   - Formal proofs in `formal/` check the properties in `blocks.md` Section 10.
   - Unit benches in `test/unit` test each module.
   - Chip tests in `test/` run the firmware against peers.
5. **Against the standards:** `conformance/` runs the firmware on the model, with sigrok's third-party decoders reading the bus and a checker measuring times against the standards. Bugs found and fixed are in `docs/protocol_conformance.md`.

All this was digital simulation. No real device or silicon is involved; `docs/protocol_conformance.md` Section 8 lists what that leaves unproven.

## 9. Where to look

| To find | Read |
|---|---|
| Exact instruction rules | `docs/spec/isa.md` |
| Exact cycle timing of each block | `docs/spec/blocks.md` |
| The assembler and config helpers | `test/common/pe_isa.py` |
| The firmware | `test/common/pe_fw.py` |
| Each module's rules | the header comment of its file in `src/` |
| What was wrong and how it was fixed | `docs/protocol_conformance.md`, `docs/problem_log.md` |
| Area | `docs/area.md` |
