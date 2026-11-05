# v2 architecture study: controller + TX unit + TRX unit

Status: study, nothing implemented. Base: v1 (`v1.0` tag, branch `v1-uart`).
Goal: run UART, SPI and I2C (one at a time) at the highest speed the 50 MHz
clock and the pads allow, without worrying about area.

## 1. The proposal

```
             (unchanged from v1)
 host ──► pe_host_if ──► pe_regs ──► IMEM ──► fetch ──► decode
                                                          │
                                                          ▼
                                                 ┌──────────────────┐
                                                 │  CONTROL UNIT    │  instructions:
                                                 │ (v1 exec, trimmed│  flow, counters, flags,
                                                 │  + unit commands)│  pins, unit commands
                                                 └──┬────────────┬──┘
                                    cmd / status    │            │   cmd / status
                                        ┌───────────▼──┐      ┌──▼────────────┐
                    TX FIFO ───────────►│   TX UNIT    │ tick │   TRX UNIT    │◄───────── TX FIFO (I2C)
                                        │ UART TX      ├─────►│ UART RX       │
                                        │ SPI MOSI+SCK │      │ SPI MISO      │──────────► RX FIFO
                                        │ I2C SCL      │      │ I2C SDA in/out│
                                        └──────┬───────┘      └──────┬────────┘
                                               │      pin fabric     │
                                               └────────► uio[7:0] ◄─┘
 RX FIFO ──► (controller) ──► higher memory level / host
```

- **Front end unchanged:** IMEM, fetch and decode stay as in v1, including the registered IMEM read. Contexts are removed (Section 13), so there is one instruction stream.
- **Control unit:** sits after decode and runs the instruction stream. Bit-level work no longer goes through it. It configures and commands the two units, handles framing and decisions, and moves data.
- **TX unit:** a serializer with its own bit timer and clock generator. It covers UART TX, SPI MOSI + SCK, and the I2C SCL generator.
- **TRX unit:** a deserializer with an output path. It covers UART RX, SPI MISO (master) and MOSI (slave), and I2C SDA in both directions; I2C is half duplex, so one unit is enough.
- **Data path:** the units read the TX FIFO and write the RX FIFO directly. The controller moves data from the FIFOs to the higher level.

## 2. The key effect: bit timing moves out of the instruction stream

In v1, every bit costs at least one instruction, and an instruction costs 2 cycles. That caps every protocol at roughly clk/4 per bit once loop overhead is included.

Here, the units shift bits from hardware timers or clock edges. The instruction stream only acts per byte or per transaction:

| Work | v1 | This proposal |
|---|---|---|
| Per bit | 2–5 instructions | none (unit hardware) |
| Per byte | — | 0 with auto-pull/auto-push, otherwise ~2–3 instructions |
| Per transaction | a few instructions | a few instructions (CS, START/STOP, ACK decision) |

So the v1 front end at 25 M instructions/s per context is no longer the speed limit, and it can stay as it is. That matches the "keep IMEM, fetch, decode the same" requirement, and it avoids the hardest timing risk of the 3-engine idea: a combinational 64:1 IMEM read plus decode plus execute in 20 ns.

## 3. Blocks

### 3.1 Control unit (v1 `pe_exec` + `pe_ctx`, extended)
- Keeps the v1 instructions: `nop set setoe out in wait jpin jmp djnz mov delay pull push vote`. They stay for bit-banging protocols the units do not cover (Section 8), and for CS, START/STOP and similar edges.
- New instructions (proposed encodings come later):

| Instruction | Function |
|---|---|
| `ucfg u, reg, imm` | write a unit configuration register (bit period, length, order, pins, clock mode) |
| `ugo u, n [,mode]` | start unit `u` for `n` bits, or continuously (auto-pull/auto-push) |
| `uwait u` | stall until unit `u` is idle (or `jmp` on `u` busy/done) |
| `ustop u` | abort unit `u` (SPI slave CS release, errors) |
| `jmp ack` / `jmp nack`, `jmp ferr` | branch on unit status bits |
| `mvrx` / `mvtx` | move one byte between a FIFO and data memory (only if the "higher memory" option in Section 6 is chosen) |

- One instruction stream (no contexts, Section 13). UART full duplex works because both units auto-pull/auto-push on their own.

### 3.2 TX unit (serializer + clock generator)

| Part | Width | Function |
|---|---|---|
| shift register | 8 (or 16) | loaded from the TX FIFO (auto-pull) |
| bit counter | 5 | bits left in this frame |
| bit timer | 16 | half-bit period in cycles; 1 means one level per cycle → SCK = clk/2 |
| clock generator | — | drives SCK/SCL with CPOL, and shifts data on the CPHA edge |
| tick out | 2 | `edge_a`/`edge_b` pulses exported to the TRX unit, so it samples in the same cycle and needs no pad loop-back |
| clock readback | — | optional: waits for the clock pin to really go high (I2C clock stretching) |
| frame bits | config | prefix/suffix bits around the data: UART start 0 / stop 1 / parity |
| outputs | 2 pins | data pin + clock pin, through the pin fabric owner map |

### 3.3 TRX unit (deserializer + half-duplex output)

| Part | Width | Function |
|---|---|---|
| shift register | 8 (or 16) | auto-push to the RX FIFO when full |
| tick source | mux | (a) its own bit timer (UART RX), (b) TX unit ticks (SPI/I2C master), (c) synchronised edge of an external pin (SPI/I2C slave) |
| start detect + vote | v1 `pe_vote` | UART RX: wait for the falling edge, vote mid-bit |
| output path | 1 pin | open-drain or push-pull, driven from its shift register (I2C write, SPI slave MISO, ACK) |
| status | — | `ack`, `ferr`, `busy`, `done` → control unit |

The TX and RX units are separate blocks, each optimised for its own job (decision 2). Because only one protocol runs at a time, the TX FIFO is statically muxed to whichever unit is sending.

## 4. Protocol walk-throughs

**UART**
- TX unit: frame = start(0) + 8 data, LSB first + stop(1); bit timer = B; auto-pull.
- TRX unit: own timer, start detect, vote at mid-bit, stop check → `ferr`; auto-push.
- Control: starts both units, then only handles errors.

**SPI master (modes 0–3)**
- TX unit: SCK generator (CPOL/CPHA), MOSI, auto-pull; it keeps SCK running as long as the FIFO has data.
- TRX unit: samples MISO on the TX unit's sample-edge tick, in the same cycle; auto-push.
- Control: `set CS,0` → `ugo` both → `uwait` → `set CS,1`.
- In write-only transfers, a TRX "discard" mode keeps dummy bytes out of the RX FIFO.

**SPI slave**
- TRX unit: tick = synchronised external SCK edge; captures MOSI.
- TX unit: data-only mode on the external SCK opposite edge; drives MISO.
- Control: `wait CS,0` → `ugo` → `wait CS,1` → `ustop` both, which aborts a partial byte.

**I2C master**
- START/STOP: control unit with `set`/`setoe` on SDA and SCL (open-drain).
- Byte write: TX unit runs SCL for 9 clocks. TRX shifts 8 bits out on SDA, then releases SDA and samples ACK on the 9th clock → `ack` flag.
- Byte read: TRX samples 8 bits, then drives ACK or NACK on clock 9; the control unit sets which before the byte.
- Clock stretching: clock readback in the TX unit.
- Gaps: between bytes the master holds SCL low (legal), so the 2–4 cycles of control-unit decision latency are harmless.

**I2C slave**
- TRX unit: tick = external SCL edges.
- START/STOP detection (SDA changing while SCL is high) is needed. It can be firmware (`wait`/`jpin` polling, enough up to 400 kHz) or a generic "level change on pin A while pin B is high" event in the TRX unit (needed for 1 MHz+).

## 5. Expected maximum speeds (50 MHz, estimates)

| Protocol | v1 | This proposal | Limited by |
|---|---|---|---|
| UART TX | ~4 Mbaud (firmware needs B ≥ 12 cycles) | **25 Mbaud** | 1 bit per 2 cycles (clk/2); pad speed |
| UART RX | ~3–6 Mbaud (B = N·S + 4) | **6–12 Mbaud** | 4–8 samples per bit |
| SPI master | ~5 MHz SCK | **25 MHz SCK**, continuous | clk/2; pads + slave tco ≤ ~15 ns for MISO |
| SPI slave | ~2–3 MHz | **~6–10 MHz** | 2-FF sync + edge detect + output register ≈ 4 cycles before MISO changes |
| I2C master | 1 MHz | **3.4 MHz** (HS timing) | external pull-ups / pads in practice |
| I2C slave | ~400 kHz | 400 kHz (firmware START/STOP), 1 MHz+ with the hardware event | |

These are architecture limits. Tiny Tapeout's pad and mux path must still be checked; 25 MHz on `uio` is at the edge of what is usually quoted.

## 6. Data path to the host ("higher memory levels")

**Host bus throughput (v1 nibble bus, estimate).**
- Each nibble is a toggle handshake through 2-FF synchronisers on both sides: about 4 chip cycles plus host latency, roughly 150–200 ns with an RP2040 PIO host.
- The address does not auto-increment, but the FIFO register is read repeatedly at the same address, so one byte is 2 READ ops: about 0.3–0.4 µs per byte, or **~2.5–3 MB/s** best case for one direction.
- Full-duplex SPI at 25 MHz needs 3.1 MB/s in **each** direction (6.25 MB/s total), so the host bus cannot sustain it. It can sustain UART and I2C, and SPI up to roughly 10–12 MHz.

**Options for "the controller shares FIFO data with higher memory levels":**

| Option | What it is | Pros | Cons |
|---|---|---|---|
| H0 | Deeper FIFOs only (e.g. 64 bytes); the host reads the FIFO register | simplest; no controller work | full speed only for bursts ≤ FIFO depth |
| H1 | **Data memory (e.g. 256 B)**; the controller copies RX FIFO → memory (`mvrx`) and memory → TX FIFO, the host reads/writes memory with address auto-increment | whole packets at line rate; packet logic in firmware; also gives I2C/SPI **slave register maps** served from memory | +2 k flip-flops (fine); controller copy at ~4 cycles/byte = 12.5 MB/s, which is enough |
| H2 | Faster host bus (8-bit data or a source-synchronous clock) | sustained full-duplex rate | uses more `ui`/`uo` pins; redesign of `pe_host_if` |

Recommendation: H1. It matches the proposal (the controller owns data movement), it handles 25 MHz SPI bursts up to the memory size, and slave-mode register maps are a real use for it. H2 can come later if sustained throughput matters.

## 7. Why this is still a programmable design (decided)

The units only use the pattern every serial protocol shares: a burst of bits clocked from a timer or an edge. They are generic serializers and deserializers, in the same spirit as RP2040 PIO's shift registers with auto-pull and auto-push. The controller firmware keeps full control: it sets bit periods, lengths, bit order, framing bits and tick sources, and it handles CS, START/STOP, ACK, addressing, errors and retries. The same units are the basis for USB and Ethernet later, where the controller adds encoding (NRZI, bit stuffing, Manchester) as a future step.

Design rules that keep it that way:
1. No protocol names, modes or protocol state machines in the RTL. Units are configured by fields such as length, order, prefix/suffix bits, tick source and clock polarity.
2. Protocol decisions live in firmware.
3. The v1 bit-level instructions (`set out in wait jpin vote ...`) stay in the controller. Protocols the units do not fit can still be bit-banged.
4. The README shows the same units running all three protocols with different firmware.

## 8. Flexibility check: protocols outside the units

| Protocol | Fits the units? | Otherwise |
|---|---|---|
| JTAG | yes (TMS/TCK via control; TDI = TX, TDO = TRX) | — |
| SWD | yes (half duplex, like I2C SDA) | — |
| PS/2 | yes (external clock, 11-bit frame) | — |
| 1-Wire, WS2812 | partly (pulse-width coding) | bit-bang with v1 instructions |
| CAN | no (bit stuffing, arbitration) | bit-bang, slow |
| USB LS, 10BASE-T | no (NRZI, stuffing, Manchester) | needs the `assist` idea (future_improvements.md) |

## 9. Area and timing (rough)

| Block | Flip-flops (est.) |
|---|---|
| v1 total (imem 1560, core 283, regs 98, FIFOs 80, pins 33, host 26, rst 2) | 2,082 |
| TX unit | ~70 |
| TRX unit (incl. vote) | ~110 |
| Unit config registers | ~60 |
| FIFOs 4 → 16 deep (both) | +200 |
| H1 data memory 256 B | +2,048 |
| **Total (est.)** | **~4,600** |

- Timing: the front end is unchanged, and the units are small, local counters and shift registers. The critical path stays where it is in v1, so 50 MHz is not at risk. That is a clear advantage over the 3-engine option, which needs a combinational IMEM read.
- 6×4 tiles: probably fits with H1. This needs a synthesis check.

## 10. Comparison

| | v1 (time-sliced) | 3 engines (1 instr/clk each) | **This proposal** |
|---|---|---|---|
| SPI master max | ~5 MHz | 25 MHz | 25 MHz |
| SPI slave max | ~2–3 MHz | ~6–8 MHz | ~6–10 MHz |
| UART RX max | ~3–6 Mbaud | ~12 Mbaud | 6–12 Mbaud |
| Front-end change | — | large (3 fetch ports, combinational IMEM) | **none** |
| 50 MHz timing risk | low | **high** | low |
| Firmware size per protocol | large (bit loops) | large (bit loops, 3 programs) | **small** (config + framing) |
| Bit-level work | firmware | firmware | generic serializers, firmware-controlled (Section 7) |
| v1 verification reuse | — | partial | high (front end, pins, FIFOs, host) + new unit proofs |
| Area | ~2.1 k FF | ~3–4 k FF + 3 read muxes | ~4.6 k FF with H1 |

## 11. Decisions (2026-10-30)

1. **Programmability:** accepted. The units are generic byte-burst serializers under full controller control (Section 7). The same units serve USB/Ethernet later.
2. **Units:** the TX unit and the RX unit are **separate blocks**, each optimised for its own job (no shared `pe_shu` module).
3. **Higher memory level:** H1 data memory, for now.
4. **Contexts removed.** A single controller instruction stream. The per-context roles move into the TX and RX units.
5. **Pin-event detector:** a separate block (Section 12) that raises an event when a configured pin holds a configured level for a configured time. Two instances (decision 14.3, revised 2026-11-03).

## 12. Pin-event detectors (new block, two instances: ETX and ERX)

Function: raise an event when a configured pin holds a configured level for a configured number of cycles. Optionally, count only while a second (qualifier) pin is at a given level, or trigger on an edge instead of a held level.

| Field | Width | Meaning |
|---|---|---|
| `en` | 1 | enable; while 0 the counter is cleared |
| `pin` | 4 | pin to watch (synchronised input) |
| `mode` | 2 | 0 level, 1 rising edge, 2 falling edge, 3 any edge |
| `lvl` | 1 | level mode: level to detect |
| `dur` | 16 | level mode: cycles the level must hold continuously (0 = first cycle seen) |
| `qual_en`, `qual_pin`, `qual_lvl` | 1 + 4 + 1 | only count / accept edges while `qual_pin == qual_lvl` |
| `start` | 1 | on the event, also start this detector's own unit (ETX → TX unit, ERX → RX unit) |
| `autoarm` | 1 | re-arm as soon as the packet it started (and the after-packet action) has finished |
| `after` (ETX) | 2 | after a TX packet: none / drive 0 / drive 1 / release the TX data pin, at the end of bit 8 |
| `after` (ERX) | 1 + 1 + 16 | after an RX packet: check enable, expected level, delay; sample the RX pin once more and set FERR on a mismatch |

Output: a **sticky event flag**, one per detector (ETX, ERX), tested with `evt d, wait` / `jmp ETX|ERX` and cleared with `evt d, clr`. With `start` set, the event also starts the detector's own unit in the same cycle, with no controller latency. No vectored interrupt.

**Two instances** (decision 14.3, revised 2026-11-03): ETX is paired with the TX unit and ERX with the RX unit. They are the same module instantiated twice, with separate configuration (unit ids 2 and 3) and separate flags. The second instance is needed because full-duplex UART on one chip watches two conditions at the same time: the RX start bit (ERX, AUTOARM) and the TX start-bit / idle timing (ETX). Protocols that need only one condition at a time use either detector:
- **I2C START and STOP** are both "SDA changes while SCL is high", so one detector in `any edge` mode with qualifier SCL = 1 catches both. The controller then reads SDA: low means START, high means STOP.
- **SPI slave CS:** ERX watches CS assert and starts the RX unit; ETX can watch the same CS pin and start the TX unit in the same cycle, so both units start together.

Uses across protocols:

| Use | Configuration |
|---|---|
| UART start bit, glitch-filtered → start RX unit | ERX: RX pin, level 0, dur ≥ B/2, start, autoarm |
| UART TX start bit / stop edge / minimum idle | ETX: TX pin, level 0 for B−k then start; after = drive 1; level 1 for ≥ B |
| UART stop-bit check | ERX: after = check 1, delay B → FERR |
| UART break / line idle | RX pin, level 0 (or 1), dur ≥ 1 frame |
| SPI slave CS assert → start both units | ETX and ERX both on the CS pin, level 0, dur 0, start |
| SPI slave CS release (abort) | CS pin, level 1, dur 0; controller aborts the units |
| I2C START / STOP | SDA pin, any edge, qualifier SCL = 1 |
| I2C bus stuck / SMBus timeout | SCL pin, level 0, dur ≥ 25 ms (needs a wider `dur`, or a prescaler) |
| USB SE0 (later) | D+ pin, level 0, qualifier D− = 0, dur ≥ n |
| Ethernet link pulse / idle (later) | pin, level, dur |

Note: `dur` is 16 bits (up to 1.3 ms at 50 MHz). The 25 ms SMBus timeout needs either a 21-bit `dur` or a prescaler; decide when that use comes up.

## 13. Consequences of removing contexts

- **Front end:** IMEM, fetch and decode stay as they are. With one instruction stream, `NUM_SLOTS = max(2, 1) = 2` still applies (registered IMEM read), so the controller runs **1 instruction every 2 cycles**, and every other slot is idle. That is fine because the controller only acts per byte or per transaction (25 M instructions/s; an 8-bit SPI byte at 25 MHz lasts 16 cycles, room for 8 instructions). A later optimisation would fetch PC+1 ahead to reach 1 instruction/cycle.
- **Pin owner map:** owners become {controller, TX unit, RX unit} (2 bits per pin) instead of context numbers.
- **Per-context registers** (START_PC, CFG, VOTE_*, PC, CSTAT) become controller registers plus unit configuration registers.
- **UART full duplex** works with one stream because the TX and RX units run on their own with auto-pull/auto-push.
- **Reuse:** `pe_ctx`/`pe_exec` become the controller (one instance), the slot logic in `pe_core` shrinks, and `pe_vote` moves into the RX unit. The v1 time-sliced design is kept in `future_improvements.md`.

## 14. Detector decisions (2026-10-30)

1. Qualifier pin and edge modes: added.
2. Events: sticky flag to the controller plus an optional direct start of the TX/RX unit. No vectored interrupt.
3. ~~One detector instance.~~ Revised 2026-11-03: **two detector instances**, ETX (paired with the TX unit) and ERX (paired with the RX unit), each with AUTOARM. Same date: the TX idle level is the ETX **drive** after-packet action, and the RX stop-bit sample is the ERX **check** after-packet action (FERR); they are not in the units.

Next step: the v2 specification (block interfaces and register map) and the full ISA with encodings.
