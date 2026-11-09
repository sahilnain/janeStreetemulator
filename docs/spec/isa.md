# Protocol Emulator v2: instruction set

Status: **final** (2026-11-03, step 1 of the v2 plan; encodings of config registers follow in the block specs). Extended 2026-11-04 for all UART frames, SPI word sizes and USB (NBITS, CHAIN, line coding, CRC unit, `par`). Architecture: `docs/v2_arch_study.md`.

Decisions included (2026-10-30):
- 8 general 16-bit registers and an ALU (optimise after UART, I2C, SPI and USB work).
- `call` / `ret` are implemented.
- No `halt`. The TX and RX units transfer **one packet per start** and then stop. The controller starts a packet, waits for it, then does the protocol step (START/STOP, ACK, error check) and repeats. The one exception is CHAIN (Section 4), for bitstreams that must not pause between words, such as USB.

## 1. Programming model

One controller executes the instruction stream. Bit-level work is done by four blocks that the controller configures and commands:

| Block | Unit id | Job |
|---|---|---|
| TX unit | 0 | sends one packet: 1–16 data bits, optional clock output, optional line coding |
| RX unit | 1 | receives one packet: 1–16 data bits; can also drive its pin (half duplex); optional line decoding |
| TX event detector (ETX) | 2 | level/edge (+ duration) on a pin, optionally qualified by a second pin; can start the TX unit; after a TX packet, can drive the pin to an idle level |
| RX event detector (ERX) | 3 | same trigger logic; can start the RX unit; after an RX packet, can check one more sample (stop bit → FERR) |
| CRC unit | 4 | 16-bit CRC, any polynomial, over the TX or RX data bits (Section 4) |
| (reserved) | 5–7 | future units |

Controller state:

| State | Width | Notes |
|---|---|---|
| `pc` | AW = log2(IMEM_DEPTH) | IMEM_DEPTH ≤ 1024 |
| `r0`–`r7` | 8 × 16 bits | general registers (counters, data, pointers, timeouts) |
| `Z`, `C` | 1 + 1 | zero and carry/borrow flags, set by ALU ops |
| `TO` | 1 | timeout flag, set by `wait` with a timeout |
| `ETX`, `ERX` | 1 + 1 | sticky event flags of the TX and RX event detectors |
| `DB` | 1 | host doorbell (set by a host register write) |
| return stack | STACK_DEPTH × AW (default 4) | for `call` / `ret` |
| stall counter | 17 bits | loaded by the delay field and `delay`: up to 7 + 65535 cycles |

Shared resources: TX FIFO, RX FIFO (8-bit entries), data memory (256 × 8), pin fabric.

The host starts and stops the controller with the RUN register, as in v1. The controller has no halt instruction: idle firmware loops, typically blocked in `wait`, `uwait`, `evt wait` or `fget`.

**Timing.** The front end is unchanged from v1 (registered IMEM read, 2 slots). Each instruction takes **2 cycles**. A delay of d extra cycles is rounded up to whole slots: cost = 2·ceil((d+1)/2). Blocking instructions retry every 2 cycles until their condition is met: `wait`, `fget`/`fput` on an empty or full FIFO, `ustart TX` on an empty TX FIFO, `uwait`, and `evt wait`.

## 2. Instruction word (32 bits)

```
 31      26 25  23 22  20 19 18  16 15                              0
┌──────────┬──────┬──────┬──┬──────┬─────────────────────────────────┐
│  opcode  │ dly  │  A   │I │  B   │              imm16              │
└──────────┴──────┴──────┴──┴──────┴─────────────────────────────────┘
     6        3      3    1    3                  16
```

| Field | Use |
|---|---|
| `opcode` | 64 opcodes: 34 used, 30 reserved |
| `dly` | 0–7 extra stall cycles after the instruction (as in v1) |
| `A` | destination/source register `rd`/`rs`, or unit id / unit mask |
| `I` | 1: the operand is `imm16`. 0: the operand is register `rs = imm16[2:0]` |
| `B` | sub-field: sub-op, second register `ra`, or config register index |
| `imm16` | immediate; jump target `[9:0]` with condition `[15:11]`; or pin fields (`pin = [15:12]`, `lvl = [11]`) |

`op2` below means "`imm16` if I = 1, else `r[imm16[2:0]]`".

## 3. Instructions

### 3.1 Flow control

| Op | Mnemonic | Fields | Operation |
|---|---|---|---|
| 0 | `nop` | — | nothing (plus `dly`) |
| 1 | `jmp [cond,] target` | cond = imm[15:11], target = imm[9:0] | if cond: pc ← target |
| 2 | `djnz rd, target` | A = rd | rd ← rd − 1; if rd ≠ 0: pc ← target |
| 3 | `call target` | target = imm[9:0] | push pc + 1; pc ← target |
| 4 | `ret` | — | pc ← pop |
| 6 | `delay op2` | I, imm16 / rs | stall `op2` extra cycles (`delay rX` for run-time values) |

`call` with a full stack, or `ret` with an empty stack, executes as `nop` and sets the sticky STATUS.stk_err bit.

Jump conditions (5 bits, 32 codes):

| Code | Condition | Code | Condition |
|---|---|---|---|
| 0 | always | 15 / 16 | FERR / !FERR (ERX stop-bit check ≠ expected level, Section 4) |
| 1 / 2 | Z / !Z | 17 / 18 | TX FIFO empty / not empty |
| 3 / 4 | C / !C | 19 | TX FIFO full |
| 5 / 6 | TO / !TO | 20 / 21 | RX FIFO empty / not empty |
| 7 / 8 | ERX / !ERX | 22 | RX FIFO full |
| 9 / 10 | TX unit busy / idle | 23 / 24 | DB / !DB (host doorbell) |
| 11 / 12 | RX unit busy / idle | 25 / 26 | RXRDY / !RXRDY (RX unit holds an unread byte, after the ERX check if enabled; cleared by `urd` of DATA) |
| 13 / 14 | ETX / !ETX | 27–31 | reserved |

### 3.2 Registers and ALU (all 16-bit)

| Op | Mnemonic | Operation | Flags |
|---|---|---|---|
| 7 | `mov rd, op2` | rd ← op2 | — |
| 8 | `add rd, op2` | rd ← rd + op2 | Z, C = carry |
| 9 | `sub rd, op2` | rd ← rd − op2 | Z, C = borrow |
| 10 | `and rd, op2` | rd ← rd & op2 | Z |
| 11 | `or rd, op2` | rd ← rd \| op2 | Z |
| 12 | `xor rd, op2` | rd ← rd ^ op2 | Z |
| 13 | `shl rd, n` | rd ← rd << n (n = imm[3:0]) | Z, C = last bit out |
| 14 | `shr rd, n` | rd ← rd >> n | Z, C = last bit out |
| 15 | `cmp rd, op2` | rd − op2, result discarded | Z, C = borrow (rd < op2) |
| 34 | `par rd` | rd unchanged | C = 1 if rd has an odd number of 1s (UART parity at full baud rate) |

### 3.3 Pins

| Op | Mnemonic | Fields | Operation |
|---|---|---|---|
| 16 | `set pin, src` | pin = imm[15:12], src = imm[1:0] | drive pin = 0 / 1 / C / ~C |
| 17 | `setoe pin, v` | v = imm[0] | output enable (open-drain release when 0) |
| 18 | `wait pin, lvl [, rd]` | I = timeout enable, A = rd | stall until pin = lvl. With a timeout, rd decrements on each retry; when it reaches 0, TO ← 1 and continue. On success TO ← 0 |
| 19 | `jpin pin, lvl, target` | lvl = imm[11], target = imm[9:0] | if pin = lvl: pc ← target |
| 20 | `rdpins rd` | A = rd | rd ← synchronised pins (zero-extended) |
| 21 | `bitin rd, pin, dir` | dir = imm[0] | bit-bang input: MSB-first rd ← (rd << 1) \| pin; LSB-first rd ← (rd >> 1) \| (pin << 15) |
| 22 | `bitout pin, rd, dir` | dir = imm[0] | bit-bang output: pin ← rd[15] then rd <<= 1 (MSB-first), or pin ← rd[0] then rd >>= 1 |

**Pin sharing.** Each pin has one owner in the owner map: the controller, the TX unit or the RX unit. The controller may also drive a pin owned by a unit **while that unit is idle**. This is how firmware makes I2C START/STOP on SDA, the 9th (ACK) clock on SCL, and SPI CS. A write to a pin owned by a busy unit, or a unit write to a pin it does not own, is ignored and sets STATUS.owner_err. The ETX drive action counts as a TX unit write; it acts on the TX unit's data pin only.

### 3.4 FIFOs and data memory

| Op | Mnemonic | Fields | Operation |
|---|---|---|---|
| 23 | `fget rd, fifo [,nb]` | A = rd, B[0] = fifo (0 TX, 1 RX), imm[0] = nb | rd ← pop(fifo). Blocks while empty; with nb: C ← 1 and continue if empty, else C ← 0 |
| 24 | `fput rs, fifo [,nb]` | A = rs | push(fifo, rs[7:0]). Blocks while full; with nb: drop, C ← 1 |
| 25 | `ld rd, [ra] [,+]` | A = rd, B = ra, imm[0] = post-increment | rd ← dmem[ra[7:0]]; if +: ra ← ra + 1 |
| 26 | `st rs, [ra] [,+]` | A = rs, B = ra | dmem[ra[7:0]] ← rs[7:0]; if +: ra ← ra + 1 |

### 3.5 Units and events

| Op | Mnemonic | Fields | Operation |
|---|---|---|---|
| 27 | `ucfg u, reg, op2` | A = unit id, B = config register index (0–7) | write unit config register (16 bits) |
| 28 | `urd rd, u, reg` | A = rd, B = unit id, imm[2:0] = status register index | rd ← unit status / data register (e.g. the RX unit's last byte) |
| 29 | `ustart mask [, op2]` | A[1:0] = mask (bit 0 TX, bit 1 RX), B[0] = TX data from op2 (16 bits) | start **one packet** on every unit in the mask, **in the same cycle** (see below) |
| 30 | `ustop mask` | A[1:0] = mask | abort: the units go idle at once and drop a partial packet |
| 31 | `uwait mask` | A[1:0] = mask | stall until every unit in the mask is idle |
| 32 | `evt ETX\|ERX, sub` | A[0] = detector (0 ETX, 1 ERX); B: 0 `wait`, 1 `waitc` (wait, then clear), 2 `clr`, 3 `arm`, 4 `disarm` | wait for / clear the detector's flag; arm or disarm the detector |
| 33 | `host sub` | B: 0 `irq` (set the host IRQ flag), 1 `dbclr` (clear DB) | controller ↔ host signalling |

`ustart` per unit:
- **TX:** sends one packet. The data byte comes from the TX FIFO (B[0] = 0; `ustart` blocks while the FIFO is empty) or from `op2[7:0]` (B[0] = 1). The unit then goes idle and holds the pin (at its idle level, Section 4).
- **RX:** receives one packet. The byte is pushed to the RX FIFO if the unit's AUTOPUSH is set; otherwise it is held for `urd`, so the controller can check FERR and then `fput` it or drop it.
The two event detectors are separate instances of the same block, each configured through its own unit id (`ucfg ETX, …` / `ucfg ERX, …`) and paired with one unit: ETX can start only the TX unit, ERX only the RX unit. A detector is armed with `evt d, arm`. When it fires, it sets its flag (ETX/ERX) and, if configured, starts one packet on its own unit in the same cycle. It then disarms itself. Each detector also has an optional **after-packet action** (Section 4), run when its unit finishes a packet. A detector re-arms in one of two ways:
- by firmware: `evt d, arm`. An arm issued while the detector's unit is busy, or while its after-packet action is pending, takes effect when both have finished;
- by itself, if AUTOARM is set: when the packet it started and the after-packet action have finished. AUTOARM keeps the RX side running in hardware at high baud rates, where firmware could not re-arm in time.

The detectors watch the pin's input, which follows the driven level on an output pin, so ETX can time the TX line itself.

The unit config and status register indices are defined in the block specifications (step 2).

### 3.6 Reserved

| Opcodes | Planned use |
|---|---|
| 5 | (was `halt` in the first draft; not used) |
| 35–47 | general extensions |
| 48–63 | line-coding and CRC assists (see Section 5) |

A reserved opcode executes as `nop` and sets the sticky STATUS.ill_err bit.

## 4. Packet format and the after-packet actions

A packet is **NBITS data bits**, 1 to 16 (8 by default), MSB- or LSB-first. There are no framing bits in the units. Framing is built from the two event detectors and firmware (UART example below). A word over 8 bits comes from `ustart`'s op2 and is read with `urd`, because the FIFOs hold bytes.

**Stream options.** These are generic config bits in the units, not protocol logic; firmware still decides everything per transaction.

| Option | Unit | Effect |
|---|---|---|
| CHAIN | TX, RX | when a word ends, the next starts with no gap. TX takes the next word from the TX FIFO. The packet ends when the FIFO is empty |
| INV | TX, RX | inverted line |
| NRZI | TX, RX | 0 = toggle, 1 = keep |
| STUFFN | TX, RX | one 0 after N data 1s in a row; removed and checked on RX |
| DIFF | TX | second pin = inverse of the data pin |
| EOP | TX | at the end of a packet: both pins low for 2 bits, then idle for 1 bit (USB SE0, SE0, J) |
| HUNT | RX | drop samples until a 1 follows two or more 0s (USB sync) |
| SRC 3 | RX | sample timer resynchronised on data edges (clock recovery) |
| DIFF | RX | no sample while both pins are low |
| STOP | ETX, ERX | the detector's fire stops its unit in the same cycle (USB end of packet) |
| CRC APPEND | CRC | after the last FIFO word the TX unit sends ~CRC, 16 bits MSB-first |

Each detector has one **after-packet action**, set by its AFTER config register (approved 2026-11-03). It runs when the detector's unit finishes a packet, whether the packet was started by the detector or by `ustart`:

| Detector | Action | What it does | Why the controller cannot do it |
|---|---|---|---|
| **ETX** | **Drive** `DRIVE(lvl)` / `RELEASE` | at the end of the last bit, drive the TX unit's data pin to a level (0 or 1), or release it (output enable off; with DIFF, both pins). The TX unit then holds that level while idle | the UART stop edge must be exactly at the end of the last bit, while the controller may be busy with RX |
| **ERX** | **Check** `CHECK(lvl)` + `ADLY n` | `n` cycles after the RX unit finishes the last bit (n ≈ B/2 for the middle of the UART stop bit, `blocks.md` Section 6), sample the RX pin once more; set **FERR** if it differs from `lvl` | the UART stop-bit window is one bit long: 12 cycles at 4 Mbaud, shorter than one pass of the firmware loop |

- Both actions only produce a level or report a status; the decision (send the next byte, keep or drop a byte) stays in firmware.
- With the ERX check enabled, **RXRDY rises when the check completes**, so FERR is valid whenever RXRDY is set. FERR is updated by every check.
- With AFTER = none, the detector does nothing after the packet, and the unit holds its last bit.

## 5. Examples

These use symbolic config register names and field macros; the indices and bit fields are in `blocks.md`.

**UART, full duplex on one chip** (same behaviour as v1: the host writes TXF and reads RXF; a byte with a bad stop bit is dropped).
```asm
        ucfg  TX,  PINS,  TXD          ; unit setup (once)
        ucfg  TX,  BIT,   417          ; cycles per bit: 115200 baud at 48 MHz
        ucfg  TX,  FRAME, LSB
        ucfg  ETX, AFTER, DRIVE(1)     ; stop edge right after bit 8
        ucfg  RX,  PINS,  RXD
        ucfg  RX,  BIT,   417
        ucfg  RX,  FRAME, LSB|HOLD     ; no auto-push: firmware checks FERR first
        ucfg  RX,  VOTE,  N(9)|K(1)    ; 9 samples, 46 cycles apart
        ucfg  RX,  STEP,  46
        ucfg  RX,  FIRST, 210          ; B - DUR + 1 (blocks.md Section 6)
        ucfg  ERX, COND,  RXD|FALL|START|AUTOARM   ; start bit: falling edge ...
        ucfg  ERX, DUR,   208                      ; ... then low for B/2 (glitch filter)
        ucfg  ERX, AFTER, CHECK(1)                 ; stop bit must be 1 -> else FERR
        ucfg  ERX, ADLY,  210                      ; middle of the stop bit
        set   TXD, 1
        setoe TXD, 1
        evt   ERX, arm
        ucfg  ETX, COND,  TXD|LEVEL(1)             ; "line idle >= 1 bit"
        ucfg  ETX, DUR,   417
        evt   ETX, arm
        mov   r7, 0                    ; TX state: 0 = waiting for idle, 1 = start bit
loop:   cmp   r7, 1
        jmp   Z, tx_run
        jmp   !ETX, rx_chk             ; TX side: line idle >= 1 bit?
        jmp   TXF_EMPTY, rx_chk        ;          and a byte to send?
        evt   ETX, clr
        ucfg  ETX, COND,  TXD|LEVEL(0)|START       ; time the start bit, then start TX
        ucfg  ETX, DUR,   415                      ; B - k, k = 2
        evt   ETX, arm
        set   TXD, 0                   ; start bit (exact moment not critical)
        mov   r7, 1
        jmp   rx_chk
tx_run: jmp   !ETX, rx_chk             ; TX unit not started yet
        evt   ETX, clr                 ; TX unit is sending; ETX drives the stop edge
        ucfg  ETX, COND,  TXD|LEVEL(1)             ; stop bit + idle >= 1 bit
        ucfg  ETX, DUR,   417
        evt   ETX, arm                 ; takes effect after the stop edge
        mov   r7, 0
rx_chk: jmp   !RXRDY, loop             ; RX side: a byte received and checked?
        urd   r0, RX, DATA             ; read it (clears RXRDY)
        jmp   FERR, loop               ; bad stop bit: drop the byte
        fput  r0, RX
        jmp   loop
```
`k` = 2 is the fixed detector-to-unit latency (pin synchroniser), see `blocks.md` Section 5.3; the exact cycle timing of this program is worked out in `blocks.md` Section 6. Every edge on the TX line comes from hardware: the start bit length and the stop edge from ETX, the data bits from the TX unit. The RX side runs in hardware: ERX finds the start bit, the RX unit reads 8 bits, ERX checks the stop bit and re-arms. The controller acts once per frame on each side, never at an exact cycle, with about one frame of slack. The firmware never blocks, so TX and RX are serviced in the same loop.

**SPI master, mode 0, 24 MHz SCK, r1 bytes.**
```asm
        ucfg  TX,  PINS,  MOSI|SCK
        ucfg  TX,  BIT,   2            ; 2 cycles per bit -> SCK = 24 MHz
        ucfg  TX,  FRAME, MSB
        ucfg  TX,  CLK,   CPOL0|CPHA0
        ucfg  RX,  PINS,  MISO
        ucfg  RX,  TICK,  FROM_TX|SDLY(3) ; sample on the TX unit's sample edge
        ucfg  RX,  FRAME, MSB          ; auto-push to the RX FIFO
        set   CS, 0
byte:   ustart TX|RX                   ; one byte out and one byte in, together
        uwait  TX|RX
        djnz  r1, byte
        set   CS, 1
```
There is a gap of a few instructions between bytes (SCK pauses); SPI allows this.

**I2C master, write one byte, read ACK, STOP.**
```asm
        set   SDA, 0      dly 7        ; START: SDA falls while SCL is high
        set   SCL, 0                   ; (both units idle, so the controller may drive)
        ustart TX|RX                   ; TX: 8 SCL clocks; RX: 8 bits out on SDA (from TX FIFO)
        uwait TX|RX
        setoe SDA, 0                   ; release SDA for the ACK
        set   SCL, 1      dly 7        ; 9th clock high
        jpin  SDA, 1, nack             ; read ACK
        set   SCL, 0
        ...                            ; next byte, or:
stop:   set   SDA, 0
        set   SCL, 1      dly 7
        set   SDA, 1                   ; STOP: SDA rises while SCL is high
```

## 6. USB and what remains

USB low speed and full speed are implemented (`blocks.md` Section 12). Every bit-level step is one of the options in Section 4; transactions are firmware (`pe_fw.usb_device`, `pe_fw.usb_host`).

**Clock.** USB needs a clock that divides evenly:

| Mode | At 48 MHz | At 50 MHz |
|---|---|---|
| Low speed, 1.5 Mb/s | B = 32, exact | B = 33: 1.0% fast. Inside the ±1.5% device limit, outside the host limit |
| Full speed, 12 Mb/s | B = 4, exact | B = 4: 4.2% fast, far outside ±0.25%. Receive works; transmit is not USB-compliant |

High speed (480 Mb/s) is not possible on this chip. The chip clock is 48 MHz (`info.yaml`) for this reason; the 50 MHz column is why it is not 50.

Ethernet remains open: Manchester (10BASE-T) at 20 Mbaud half-bits is 2.5 cycles and is not feasible at this clock without a faster sampler.

## 7. Changes from v1

| v1 | v2 |
|---|---|
| 24-bit word, 5-bit opcode | 32-bit word, 6-bit opcode |
| 2 time-sliced contexts | 1 controller |
| ctr0/ctr1, OSR/ISR per context | r0–r7, return stack, TX/RX units |
| `out`/`in`/`vote` bit loops | one packet per `ustart` (bit loops still possible with `bitin`/`bitout`) |
| `pull`/`push` (FIFO ↔ shift register) | units take/give FIFO bytes; `fget`/`fput` for the controller |
| `jmp` 3-bit condition | 5-bit condition |
| — | ALU, `call`/`ret`, data memory, unit/event/host instructions |
