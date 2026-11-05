# Protocol Emulator v2: block specifications

Status: implemented, tested and formally proved (see Section 10). Instruction set: `docs/spec/isa.md` (final). Architecture: `docs/v2_arch_study.md`.

## 0. Conventions

- One clock, 48 MHz (`info.yaml`; exact USB bit times). Synchronous active-high reset `rst` (from `rst_n` through the 2-FF reset synchroniser, as in v1).
- **Write cycle.** "Written in cycle w" means a register is updated at the clock edge that ends cycle w; the new value is visible from cycle w+1. A pin written in cycle w is on the pad from cycle w+1.
- **`pin_in`** is the synchronised pin input (2 FFs, `pe_sync`). A pad change in cycle p is seen in `pin_in` from cycle p+2. The controller, both units and both detectors all read the same `pin_in`.
- **Strobe.** A 1-cycle pulse. Controller → block strobes are combinational in the controller's execute cycle E. The block acts on them at the end of E.
- Bit order: LSB-first unless the MSB bit of the block's FRAME register is set.
- All config registers are 16 bits and are written with `ucfg u, idx, op2`. Unused bits are reserved: write 0, read 0. Config registers keep their value across RESTART; only chip reset clears them. Reset values are 0, except BIT = 1 (TX, RX), STEP = 1 and VOTE N = 1 (RX).

## 1. Block list and reuse

| Module | Block | From v1 |
|---|---|---|
| `tt_um_sahil_protoemu` | top level, pin map | rewired; pin map unchanged |
| `pe_host_if` | 4-bit host bus | **unchanged** |
| `pe_regs` | host register map | rewritten (Section 9) |
| `pe_imem` | instruction memory | unchanged, IW = 32 |
| `pe_ctrl` | controller: fetch, issue, state, execute | new, built from `pe_core` + `pe_ctx` + `pe_exec` |
| `pe_decode` | 32-bit instruction decode | rewritten |
| `pe_txu` | TX unit | new |
| `pe_rxu` | RX unit (contains `pe_vote`) | new; `pe_vote` unchanged |
| `pe_det` | event detector, two instances: ETX and ERX | new |
| `pe_crc` | CRC unit | new |
| `pe_pins` | pin fabric | changed: 3 write ports, 2-bit owners, busy rule |
| `pe_fifo` | TX FIFO, RX FIFO | **unchanged**, depth 8 |
| `pe_dmem` | data memory | new |
| `pe_sync` | synchroniser | **unchanged** |

Top-level parameters (defaults):

| Parameter | Default | Notes |
|---|---|---|
| `IMEM_DEPTH` | 160 | ≤ 1024; 160 × 32 = 5120 bits. `pc` and IMEM_ADDR have AW = 8 bits: an address past the end reads as 0 (`nop`) and a write there does nothing, so a runaway `pc` runs to 255 and wraps to 0. Raised from 64 for the I2C master (about 105 words) and from 128 on 2026-11-04 for the USB device firmware (148 words); area to be revisited |
| `DMEM_DEPTH` | 256 | ≤ 256; may be reduced if area is short |
| `FIFO_DEPTH` | 8 | ≤ 15 (FIFO_CNT is 4 bits per FIFO) |
| `STACK_DEPTH` | 4 | return stack |
| `NUM_PINS` | 8 | = uio width; pin fields are 4 bits (room for 16) |

```
 host pins ─ pe_host_if ─ pe_regs ─┬─ pe_imem ──► pe_ctrl ──┬── cmd ──► pe_txu ◄── start ── pe_det (ETX)
                                   ├─ TX FIFO ◄─────────────┤          pe_rxu ◄── start ── pe_det (ERX)
                                   ├─ RX FIFO ◄─────────────┤            │  ▲
                                   └─ pe_dmem ◄─────────────┘            ▼  │ pin_in
                                                              pe_pins (CTRL, TXP, RXP write ports) ── uio
```

## 2. Controller (`pe_ctrl`, `pe_decode`)

### 2.1 State
`pc` (AW bits), `r0`–`r7` (16 bits), flags `Z C TO DB`, return stack (STACK_DEPTH × AW, plus a depth counter), stall counter (16 bits). The flags `ETX`, `ERX` live in the detectors, and `RXRDY` and `FERR` in the RX unit and ERX. The controller reads them as inputs.

### 2.2 Issue timing
- The front end is v1's with one stream. In the cycle before an issue, `im_raddr = pc`; the word arrives in the issue cycle (registered IMEM read). The controller **issues at most every 2 cycles**. The cycle in which an instruction executes is called **E**. All its effects are written in E.
- `dly` d (or `delay`): the next issue is at E + 2·ceil((d+1)/2).
- **Blocking.** A blocked instruction has no effect, and is retried at E+2, E+4, … until its condition holds. Blocking instructions: `wait`, `fget`/`fput` (empty/full, or lost arbitration), `ld`/`st` (lost arbitration to the host), `ustart` (unit busy, or TX FIFO empty when a byte is needed), `uwait`, `evt wait/waitc`.
- RUN = 0: no issue. A blocked instruction is abandoned and re-executed when RUN returns to 1. RESTART (host): `pc ← START_PC`; r0–r7, flags, stack and stall counter are cleared; both units are aborted (as `ustop`); both detectors are disarmed and their flags, FERR and RXRDY cleared.

### 2.3 Outputs to the other blocks (combinational in E)

| Port | Width | Meaning |
|---|---|---|
| `cfg_we`, `cfg_unit`, `cfg_idx`, `cfg_data` | 1, 3, 3, 16 | `ucfg` write |
| `rd_unit`, `rd_idx`, `rd_data` (in) | 3, 3, 16 | `urd` read mux; `rd_strobe` marks a real read (clears RXRDY for RX DATA) |
| `start_mask`, `start_imm`, `start_data` | 2, 1, 8 | `ustart`; issued only when not blocked |
| `stop_mask` | 2 | `ustop` |
| `det_sel`, `det_op` | 1, 3 | `evt` clr / arm / disarm (wait is local) |
| `pin_*` | — | CTRL port of `pe_pins` (Section 7) |
| FIFO, DMEM ports | — | Section 8 |
| `host_irq_set`, `db_clr` | 1, 1 | `host irq`, `host dbclr` |
| `ill_err`, `stk_err` | 1, 1 | sticky-set strobes to `pe_regs` |

### 2.4 Rules
- `ucfg` / `urd` to unit ids 4–7, or to an unused register index: write ignored / read 0. No error.
- A config write to a busy unit takes effect immediately. Firmware must not reconfigure a running unit; the result is unspecified, but it can never violate pin ownership.
- `ustart` with mask 0 is a `nop`.

## 3. TX unit (`pe_txu`, unit id 0)

Sends one packet of n = NBITS bits per start on a data pin, with an optional clock on a second pin. Below, bit i runs over 0…n−1.

### 3.1 Config registers

| Idx | Name | Bits |
|---|---|---|
| 0 | PINS | [3:0] DPIN, [4] DEN (data pin used), [11:8] CPIN, [12] CEN (clock pin used) |
| 1 | BIT | [15:0] B, cycles per bit (0 is treated as 1; ≥ 2 when CEN = 1) |
| 2 | FRAME | [0] MSB (1 = MSB-first), [2] INV (data bits inverted), [8:4] NBITS (0 = 8, 1–16, above 16 = 16), [9] CHAIN (3.6) |
| 3 | CLK | [0] CPOL, [1] CPHA, [2] STRETCH (wait for the clock pin to read high, I2C) |
| 4 | TICK | [0] EXT (0 = own timer, 1 = external clock), [7:4] EPIN, [8] EEDGE (0 = rising, 1 = falling), [9] LEAD (SPI slave CPHA 1, see 3.4) |
| 5 | LINE | [0] NRZI, [1] EOP, [2] DIFF, [7:4] D2PIN, [11:8] STUFFN (0 = off), see 3.6 |

Status (`urd rd, TX, idx`): 0 STAT = [0] busy, [7:4] bits sent.

### 3.2 Start
- Start sources: `ustart` (mask bit 0) or ETX (START). The start cycle is **S** (`ustart`: S = E; detector: S = its fire cycle).
- Data byte: `op2[7:0]` if `ustart` used B[0] = 1, else one pop from the TX FIFO in cycle S. If the TX FIFO is empty: `ustart` blocks, and a detector start is dropped and sets STATUS2.tx_underrun.

### 3.3 Timing, own timer (EXT = 0)
Let H = floor(B/2).
- Data bit i is written in cycle **S + i·B**, so it is on the pad during cycles S+1+i·B … S+(i+1)·B.
- Clock (CEN = 1). "Active" = !CPOL.
  - CPHA = 0: SCK written idle in S, active in S + i·B + H, and idle in S + (i+1)·B.
  - CPHA = 1: SCK written active in S + i·B, and idle in S + i·B + H.
- **Sample tick** `tx_stick`: a pulse in cycle S + i·B + H, the write cycle of the sampling edge (leading for CPHA 0, trailing for CPHA 1). Exported to the RX unit.
- **Shift tick** `tx_dtick`: a pulse in cycle S + i·B, i = 1…n−1, exported to the RX unit for its drive mode.
- **Done** D = S + n·B: a pulse to ETX (after-packet action) and to the controller (busy ends, see 3.5). Nothing is written by the unit in D except the CPHA-0 SCK return to idle.
- With B = 2, CPHA 0: SCK = 24 MHz, one cycle high and one low.

STRETCH (I2C clock stretching): after the unit writes SCK to the released level (1) in cycle w, the bit timer holds until `pin_in[CPIN]` reads 1 (earliest w+3). Every SCL high phase is therefore at least 2 cycles longer than programmed, plus any time a slave holds SCL low. The tick and done times move by the same amount. If the write at offset H is a release (CPOL 0 / CPHA 0, as in I2C, or CPOL 1 / CPHA 1), `tx_stick` is not given in w but in the cycle SCK is first seen high, so the RX unit never samples while a slave still holds SCL low (added 2026-11-03).

### 3.4 Timing, external clock (EXT = 1, SPI slave)
- Bit 0 is written in S. Bit i (i = 1…n−1) is written in the cycle in which the selected edge (EEDGE) of EPIN is seen in `pin_in`.
- D = the cycle of the n-th edge; the last bit stays on the pin. No clock output, no ticks.
- This is SPI slave CPHA 0: EEDGE = trailing edge, so bit 0 is out before the first leading edge.
- **LEAD = 1, SPI slave CPHA 1** (added 2026-11-04): EEDGE = leading edge. Nothing is written in S. Bit i is written in the cycle selected edge i+1 is seen. D = the first opposite (trailing) edge seen after the last bit, so that bit stays until the master has sampled it, and MISO only ever changes after a leading edge.

### 3.5 Busy and stop
- `busy` = 1 from S+1 up to and including D. The ETX drive action (Section 5) happens in D, so the controller sees TX idle from D+1, with the stop level already on the pin.
- `ustop` (cycle E): the unit goes idle in E+1. The pin keeps its last written level (a write already due in E still happens), and no done pulse is generated.

### 3.6 Line coding and chaining (own timer, added 2026-11-04)
A clock output is not meaningful together with these options.
- **Order:** data bit → stuffing → NRZI → INV → pins.
- **NRZI:** a 0 toggles the level, a 1 keeps it. The level is 1 at S.
- **STUFFN = N:** after N data 1s in a row, a 0 is sent for one bit time before the next bit. The count starts at 0 in S, carries across chained words, and a stuff bit due at the end of a packet is sent before the EOP. `tx_stick` keeps its timing, so do not use it with STUFFN.
- **DIFF:** every data write also writes D2PIN with the inverse of DPIN. ETX release covers both pins.
- **CHAIN:** when the last bit of a word has run for B cycles and a next word exists, that word starts in the same cycle: its bit 0 is written and no done is given. The next word is the next TX FIFO byte (popped in that cycle) or, if the FIFO is empty and the CRC unit has APPEND set and has not sent yet, ~CRC, 16 bits MSB-first. Otherwise the packet ends. The firmware must keep the FIFO non-empty until the last byte; `ustart` still takes the first word from the FIFO or op2.
- **EOP:** when the packet ends: both pins written 0 at the end of the last bit, DPIN written 1 (D2PIN written 0) two bit times later, done one bit time after that (busy until then).
- **CRC stream:** `bit_v`/`bit_o` give every data bit to the CRC unit, not stuffed bits and not the CRC word.

## 4. RX unit (`pe_rxu`, unit id 1)

Receives one packet of n = NBITS bits per start. It can also drive a pin with a word while it samples (half duplex: I2C SDA).

### 4.1 Config registers

| Idx | Name | Bits |
|---|---|---|
| 0 | PINS | [3:0] IPIN (sampled pin), [7:4] I2PIN, [11:8] OPIN (driven pin), [12] OEN (drive mode), [13] DIFF (no sample while IPIN and I2PIN are both low) |
| 1 | BIT | [15:0] B, cycles per bit (own timer) |
| 2 | FRAME | [0] MSB, [1] HOLD (0 = push the low byte to the RX FIFO, 1 = hold the word for `urd`), [2] INV, [3] CHAIN (SRC ≠ 0, no drive), [8:4] NBITS (as TX) |
| 3 | VOTE | [3:0] N samples per bit (0 → 1), [7:4] K samples ignored at each end |
| 4 | STEP | [15:0] cycles between vote samples (0 → 1) |
| 5 | FIRST | [15:0] cycles from the start to the beginning of bit 0's window |
| 6 | TICK | [1:0] SRC: 0 own timer, 1 TX sample tick, 2 external edge, 3 resynchronised timer; [7:4] EPIN; [8] EEDGE (0 rising, 1 falling); [15:12] SDLY |
| 7 | ODLY | [3:0] drive delay (added 2026-11-03): with SRC = 1 or 2, drive bit i ≥ 1 is written ODLY cycles after its shift event. For I2C, SDA then changes after SCL is low, never at the same clock edge. Must be < B/2. [4] NRZI, [5] HUNT, [11:8] STUFFN (0 = off), see 4.5 |

Status / data (`urd rd, RX, idx`):
- 0 DATA: the held word, right-aligned. A read clears RXRDY.
- 1 STAT: [0] busy, [1] RXRDY, [2] FERR, [3] overrun (sticky; cleared by RESTART or a host write to STATUS2), [4] stuff error (sticky, cleared by a start), [12:8] bits of the current word.

### 4.2 Sampling
Start cycle S, as for the TX unit (`ustart` mask bit 1, or ERX START).
- **SRC = 0, own timer (UART).** Bit i's window starts at W_i = S + FIRST + i·B. It is voted by `pe_vote` with start = W_i: sample j (j = 0…N−1) is `pin_in[IPIN]` in cycle W_i + (j+1)·STEP. Samples with K ≤ j < N−K are counted, and the bit is the majority (ties → 0). Constraint: N·STEP + 1 ≤ B. The last bit is known in cycle **D = W_(n−1) + N·STEP + 1**.
- **SRC = 1, TX sample tick (SPI/I2C master).** Bit i = `pin_in[IPIN]` in cycle T_i + SDLY, where T_i is the i-th `tx_stick` pulse. D = T_(n−1) + SDLY. SDLY compensates the synchroniser and the external round trip: SDLY = 3 samples the pad value that is present when the clock edge reaches the pad.
- **SRC = 2, external edge (SPI slave).** Bit i = `pin_in[IPIN]` in the cycle in which the i-th selected edge of EPIN is seen, plus SDLY. D = the cycle the last bit is taken.
- **SRC = 3, resynchronised timer (USB, added 2026-11-04).** One sample per B cycles. An edge of IPIN seen in cycle e moves the next sample to e + floor(B/2), and cancels a sample due in e (without that, a long run that is slightly long counts one bit too many). FIRST sets the first sample as for SRC 0.

### 4.3 Drive mode (OEN = 1)
- The byte comes from `op2` or the TX FIFO, exactly as for the TX unit (3.2). If both units in one `ustart` need a byte, they get the **same** byte (one pop).
- Bit 0 is written to OPIN in S. Bit i ≥ 1 is written in cycle:
  - SRC = 0: W_i
  - SRC = 1: ODLY cycles after the i-th `tx_dtick` (ODLY = 0: the same cycle in which the TX unit writes its data bit i)
  - SRC = 2: ODLY cycles after the i-th opposite edge of EPIN (the edge that does not sample)
- The unit still samples. The received byte is the readback (I2C arbitration check: DATA ≠ sent byte).

### 4.4 Result
In D the received byte is complete. Then:
- HOLD = 0: the byte is pushed to the RX FIFO in D. A full FIFO drops it and sets rxf_overrun, as in v1.
- HOLD = 1: the byte is written to the DATA register, and **RXRDY** is set, at the end of the ERX after-packet step (Section 5.4): in D if ERX has no check, or in D + ADLY with the check. Until then DATA keeps the previous byte, so a read during the check is consistent. If RXRDY was already set, the old byte is overwritten and STAT.overrun is set.
- `busy` = 1 from S+1 up to and including the end of the ERX after-packet step.
- `ustop` in cycle E: idle in E+1, a partial word is dropped, RXRDY unchanged. A stop in cycle D itself is too late: the word is complete and is pushed or held as usual. (The controller's FIFO grant depends on the push, so the push cannot depend on `ustop`.)

### 4.5 Line decoding and chaining (added 2026-11-04)
On every raw sample, in this order:
1. **DIFF:** no sample at all while IPIN and I2PIN are both low. A sample can fall in SE0 one cycle before a detector can stop the unit, when the sender is slightly slower than the local clock.
2. **INV**, then **NRZI:** decoded bit = 1 if the sample equals the previous sample. The previous sample starts as the pin level in S.
3. **HUNT:** samples are dropped until a 1 follows two or more 0s. That 1 is not data, but counts as the first 1 for STUFFN (as USB requires).
4. **STUFFN = N:** the sample after N data 1s in a row is dropped. If it is a 1, the stuff error is set.

**CHAIN:** after the last bit of a word the unit stays busy and the next word begins at once. Every word gives a done pulse and a FIFO push. The packet ends only by `ustop` or a detector with STOP. A done in CHAIN is not the end of a packet, so the ERX CHECK does not run.

## 5. Event detectors (`pe_det`; ETX = unit id 2, ERX = unit id 3)

One module, instantiated twice with parameter `KIND` (0 = ETX, 1 = ERX). The trigger logic is identical in both. The after-packet action differs: ETX can drive, ERX can check.

### 5.1 Config registers

| Idx | Name | Bits |
|---|---|---|
| 0 | COND | [3:0] PIN, [5:4] MODE (0 level, 1 rising, 2 falling, 3 any edge), [6] LVL (level mode), [7] START, [8] AUTOARM, [12:9] QPIN, [13] QEN, [14] QLVL, [15] STOP |
| 1 | DUR | [15:0] cycles the condition must hold (0 → 1) |
| 2 | AFTER | ETX: [1:0] 0 none, 1 drive 0, 2 drive 1, 3 release. ERX: [0] CHECK, [1] check level |
| 3 | ADLY | ERX only: [15:0] check delay after D |

Status (`urd rd, ETX|ERX, 0`): STAT = [0] armed, [1] flag, [2] arm pending, [3] waiting for its unit / after-step, [4] FERR (ERX).

### 5.2 Trigger
The detector keeps `prev = pin_in[PIN]` from the previous cycle at all times, armed or not. `qual = !QEN || pin_in[QPIN] == QLVL`.
- **Level mode:** a counter counts consecutive cycles in which `pin_in[PIN] == LVL && qual`, and resets otherwise. The detector fires in the cycle the count reaches DUR (the DUR-th consecutive cycle). With DUR ≤ 1, it fires in the first such cycle.
- **Edge modes:** an edge is `pin_in[PIN] != prev` in the selected direction, in a cycle with `qual` = 1. The edge cycle counts as 1. The count continues while the pin keeps the new level and resets if the pin changes back. The detector fires at count DUR. This is "edge, then hold for DUR cycles". It needs a real edge after arming, so arming on a line that is already low does not fire a falling-edge detector.
- Counting only happens while armed. The counter is cleared when the detector is armed, disarmed, or has its COND/DUR written.

### 5.3 Fire (cycle F)
- `flag ← 1` (the ETX/ERX flag, sticky until `evt clr` / `waitc`), and the detector disarms.
- If START: a start strobe to its own unit in F, so the unit's S = F. If the unit is busy, the start is ignored (the flag is still set).
- If STOP: its own unit stops in F, exactly as `ustop` in cycle F (USB: SE0 ends the packet with no bit lost or added).
- **Detector-to-unit latency.** Watching its own TX pin in level mode, a pad edge in cycle p is in `pin_in` from p+2. The detector fires at F = p+2+DUR−1, and TX bit 0 is on the pad from F+1. The pad level therefore lasted **DUR + 2** cycles: **k = 2**. For a UART start bit of exactly B cycles, DUR = B − 2.

### 5.4 After-packet action (runs on its unit's done D, whoever started the packet)
- ETX, drive 0/1: writes the TX unit's DPIN out = level, oe = 1, in cycle **D** through the TXP port. The level is on the pad from D+1, which is exactly the end of bit 7.
- ETX, release: writes oe = 0 on DPIN, and on D2PIN with DIFF, in D.
- ERX, CHECK: in cycle D + ADLY, `FERR ← (pin_in[IPIN of the RX unit] != check level)`. FERR keeps its value until the next check (or RESTART). The after-step ends in D + ADLY. RX busy ends, and RXRDY is set, at that cycle.
- None: the after-step ends in D.

### 5.5 Arming
- `evt d, arm` in cycle E: if its unit is idle and no after-step is pending, the detector is armed from E+1. Otherwise **arm pending** is set, and the detector arms in the cycle after the after-step ends.
- AUTOARM: after a packet **this detector started**, it re-arms in the cycle after the after-step ends.
- `evt d, disarm` clears armed and arm pending. `evt d, clr` clears the flag. RESTART clears all of this.

### 5a. CRC unit (`pe_crc`, unit id 4)

One 16-bit register, top-aligned: a CRC of width w < 16 uses POLY << (16−w) and INIT << (16−w).

| Idx | Name | Bits |
|---|---|---|
| 0 | CTRL | [1:0] SRC (0 off, 1 TX bits, 2 RX bits), [2] APPEND, [15:8] SKIP |
| 1 | POLY | [15:0] |
| 2 | INIT | [15:0]: writing loads the register, reloads the SKIP counter and clears `sent`. Write CTRL and POLY first |

Read (`urd rd, CRC, idx`): 0 the register, 1 [0] sent. On each data bit: if SKIP bits remain, count one down; else `crc ← (crc << 1) ^ ((crc[15] ^ bit) ? POLY : 0)`. USB: CRC16 POLY 0x8005, INIT 0xFFFF, residual 0x800D; CRC5 POLY 0x2800, INIT 0xF800, residual 0x6000.

## 6. Worked timing: UART at B = 417 (115200 baud at 48 MHz), the ISA Section 5 program

TX side (ETX watches TXD):
- Idle check: COND = TXD, level 1, DUR = 417. ETX fires once TXD has been high for 417 cycles.
- Firmware: `set TXD,0` (pad low from p), then sets COND = TXD, level 0, START, with DUR = 415 (B − k), and arms. ETX fires at p+416, so the TX unit has S = p+416. Bit 0 is on the pad from p+417, which makes the start bit **417** cycles.
- Bit i is on the pad from p + 417·(i+1). D = S + 8·417, and ETX AFTER = drive 1 writes the stop level in D. The stop bit is on the pad from p + 9·417, which is exact.
- Firmware has re-armed ETX with COND = TXD, level 1, DUR = 417 while the TX unit was busy (arm pending). This arms in D+1, and then fires after the stop bit plus 2 cycles. The stop and idle time is therefore ≥ B + 2, and firmware adds its own reaction time. The decoder in `test/common/uart.py` allows idle gaps between frames.

RX side (ERX watches RXD, falling edge):
- Pad falling edge in cycle p. COND = RXD, falling, START, AUTOARM; DUR = 208 (floor(B/2), glitch filter). It fires at F = p + 2 + 207 = S.
- Bit 0 in `pin_in` starts at p + 2 + 417 = S + 210, so **FIRST = B − DUR + 1 = 210**. VOTE N = 9, K = 1, STEP = 46 (9·46 + 1 ≤ 417).
- D = S + 210 + 7·417 + 415. The middle of the stop bit in `pin_in` is at p + 2 + 9.5·417. That gives **ADLY = floor(1.5·B) − N·STEP − 1 = 210**, with AFTER = CHECK, level 1.
- In D + 210: FERR is valid, RXRDY = 1, ERX re-arms. The next start edge is ≥ B/2 later. A bad stop bit (line held low) does not re-trigger, because a falling edge is required.

These numbers become the firmware constants (`pe_fw.py`) and cycle-exact unit-test checks.

## 7. Pin fabric (`pe_pins`)

- Per pin: `out` and `oe` registers. Reset: out = 1, oe = 0. Open-drain mode (`PIN_OD[i]`) as v1: out = 0 drives low, out = 1 releases; oe is ignored.
- Owner map, 2 bits per pin: 0 controller, 1 TX unit, 2 RX unit, 3 = controller. Reset: all 0.
- A unit pin field holds 4 bits. With 8 pins, an index of 8 or more selects no pin: writes do nothing and do not set `owner_err`, reads return 0.
- Three write ports, each with an out mask/value and an oe mask/value:

| Port | Source | May write pin i when |
|---|---|---|
| CTRL | controller (`set`, `setoe`, `bitout`) | owner = controller, **or** the owner unit is idle |
| TXP | TX unit (DPIN, CPIN) and ETX drive/release | owner = TX unit |
| RXP | RX unit (OPIN) | owner = RX unit |

- Any write that is not allowed is dropped for that pin and sets the sticky `owner_err`. By construction at most one allowed port writes a pin in a cycle (CTRL needs the owner unit idle, and the units only write their own pins). The RTL still uses a fixed priority TXP > RXP > CTRL, which the formal proof shows is never needed.
- `pin_in` is `pe_sync` of `uio_in`.

## 8. FIFOs and data memory

**FIFOs** (`pe_fifo`, unchanged, 8 bits × 8). Each FIFO has one write and one read port per cycle. Arbitration is fixed and the losers block, so no data is lost:

| FIFO | Writers (priority order) | Readers (priority order) |
|---|---|---|
| TX FIFO | host TXF write > controller `fput TX` | TX/RX unit start pop > controller `fget TX` |
| RX FIFO | RX unit push > controller `fput RX` | host RXF read > controller `fget RX` |

A unit or host access never waits. A host write to a full TXF, or a unit push to a full RXF, is dropped and sets the v1 overrun flag.

**Data memory** (`pe_dmem`, DMEM_DEPTH × 8, flip-flops, combinational read, one write per cycle).
- Controller: `ld`/`st` with address `ra[7:0]` (upper bits ignored when DMEM_DEPTH < 256).
- Host: DMEM_ADDR / DMEM_DATA registers (Section 9). Each DMEM_DATA access auto-increments the address. The host has priority; a controller `ld`/`st` in the same cycle blocks and retries.

## 9. Host register map (`pe_regs`, VERSION = 0x02)

v1 addresses are kept where the meaning is the same. Unmapped addresses read 0.

| Addr | Name | Access | Bits |
|---|---|---|---|
| 0x00 | ID | R | 0x50 |
| 0x01 | VERSION | R | 0x02 |
| 0x02 | RUN | RW | [0] controller runs |
| 0x03 | RESTART | W | [0] = 1: restart (2.2) |
| 0x04 | STATUS | R | as v1: [0] txf_empty [1] txf_full [2] rxf_empty [3] rxf_full [4] txf_overrun [5] rxf_overrun [6] owner_err [7] imem_err |
| 0x05 | ERR_CLR | W | as v1: write 1 to clear bits 4–7 of STATUS |
| 0x06 | IRQ_EN | RW | [0] RXF not empty [1] TXF not full [2] any error [3] controller IRQ (`host irq`) |
| 0x07 | FIFO_CNT | R | [3:0] TX count, [7:4] RX count |
| 0x08 | TXF_DATA | W | push to the TX FIFO |
| 0x09 | RXF_DATA | R | pop from the RX FIFO |
| 0x0A | IMEM_ADDR | RW | [7:0] word address (auto-increments on commit) |
| 0x0B | PIN_IN | R | `pin_in[7:0]` |
| 0x0C | PIN_IN_H | R | `pin_in[15:8]` |
| 0x0D | IMEM_ADDR_H | RW | [1:0] word address [9:8] |
| 0x0E | STATUS2 | R, W1C | [0] ill_err [1] stk_err [2] rx_overrun (RX STAT.overrun) [3] tx_underrun [4] ctrl_irq. Write 1 to clear |
| 0x0F | DOORBELL | W | any write sets the controller's DB flag |
| 0x10–0x13 | IMEM_DATA | W | instruction staging, byte 0 = bits [7:0]; writing 0x13 commits. A commit while RUN = 1 is dropped and sets imem_err (as v1) |
| 0x14–0x15 | PIN_OD | RW | open-drain enable, bit i = pin i |
| 0x18–0x1B | PIN_OWNER | RW | 2 bits per pin, pin 0 at bit 0 of 0x18 |
| 0x20–0x21 | START_PC | RW | [9:0] |
| 0x22–0x23 | PC | R | [9:0] |
| 0x24 | CSTAT | R | [0] run [1] blocked [2] Z [3] C [4] TO [5] ETX [6] ERX [7] DB |
| 0x28 | DMEM_ADDR | RW | [7:0] |
| 0x29 | DMEM_DATA | RW | read or write dmem[DMEM_ADDR], then DMEM_ADDR + 1 |

IRQ pin (uo[5]): `(IRQ_EN[0] & !rxf_empty) | (IRQ_EN[1] & !txf_full) | (IRQ_EN[2] & any error bit in STATUS[7:4] or STATUS2[3:0]) | (IRQ_EN[3] & ctrl_irq)`.

Top-level pins are unchanged from v1: ui[3:0] host data in, ui[5:4] op, ui[6] strobe; uo[3:0] data out, uo[4] ack, uo[5] IRQ; uio[7:0] = protocol pins P0–P7.

## 10. Properties for step 5 (formal), per block

| Block | Properties to prove |
|---|---|
| `pe_txu` | bit i is written exactly at S + i·B; SCK edges at the 3.3 times; D = S + nB; busy exactly S+1…D; `ustop` → idle next cycle; never writes a pin outside {DPIN, CPIN, D2PIN}; never more than STUFFN data 1s in a row; every bit of a word before done / chain / EOP; EOP timing |
| `pe_rxu` | sample times as in 4.2; for all n the received word equals the decoded values; nothing taken while hunting, in SE0 or from a stuffed sample; HOLD/push exclusive; RXRDY / overrun rules |
| `pe_crc` | register update for any POLY; SKIP; INIT; `sent` |
| `pe_det` | fires exactly at the DUR-th qualifying cycle; edge mode never fires without an edge after arming; START strobe only in F; drive only in D; arm-pending / AUTOARM rules |
| `pe_pins` | no pin written by a non-owner; CTRL to a busy unit's pin is dropped and sets owner_err; at most one allowed writer per pin per cycle |
| `pe_ctrl` | issue spacing ≥ 2; a blocked instruction has no side effects; stack never over/underflows (stk_err instead) |
| `pe_fifo`, `pe_dmem` | arbitration losers have no effect and retry; no data loss under arbitration |
| model ↔ RTL | cycle-by-cycle lockstep (`test/test_lockstep.py`, `make LOCKSTEP=1`) and protocol cross tests in both directions (`test/test_cross.py`) |

Status (2026-11-04): the properties above are in the RTL (`ifdef FORMAL`) and proved with `formal/*.sby`, by k-induction, so they are unbounded. `formal/top.sby` proves the whole chip with every module's properties and contracts asserted together; only the host-bus protocol on the chip pins is assumed. Timing properties hold when no config register is written during a packet, as Section 2.4 requires.

## 11. How the v1 top-level tests carry over (step 4)

The functional behaviour seen from the host bus is the same as v1. The tests keep their scenarios; only the setup changes (one controller program instead of two contexts, new register addresses).

| v1 test | v2 form |
|---|---|
| test_registers | VERSION = 0x02, new RW registers |
| test_uart_loopback / 115200 / baud_error | same data and checks; firmware = ISA Section 5 UART program; TX waveform still checked cycle-exact by `uart.decode` |
| test_framing_error | same bit stream; the bad byte is dropped via FERR |
| test_imem_err_irq | same |
| test_open_drain | same (controller-owned pin) |
| test_owner_map | controller `set` on a pin owned by a **busy** unit is dropped, sets owner_err |
| new | two-chip loopback (emulator 1 TX → emulator 2 RX); Python model ↔ RTL cross tests (both directions) |

## 12. Firmware recipes for the new modes

**UART, any frame (`pe_fw.uart_ex`).** 5–9 data bits, parity none / even / odd / mark / space, 1, 1.5 or 2 stop bits, normal or inverted line, either bit order. NBITS = data + parity. Stop time = the ETX idle DUR. Parity uses `par`. The start bit is timed by the instruction stream, since the word comes from a register: `set`, `delay B−3`, `ustart`, exact for even B. A frame with no parity and at most 8 bits can use the hardware-timed `pe_fw.uart`.

**SPI words.** Master, 1–16 bits (`spi_master_words`). Slave, 1–8 bits (`spi_slave`), since the unit restarts itself from the byte-wide FIFO. A 16-bit slave word is two 8-bit words in one frame.

**UART break (`pe_fw.uart_ex`).** A frame of all zeros with a bad stop bit counts in dmem[2] and raises the controller IRQ. ERX needs a new edge, so a long break counts once. A doorbell sends a break of dmem[3] bit times between frames. With an odd B (115200 baud at 48 MHz is B = 417), the start bit is B + 1 cycles, and every other edge stays on the bit grid.

**SPI, more forms.**
- `spi_master_stream`: no pause between bytes, using TX and RX CHAIN. Any B up to 9 bytes (they fit the FIFO and the shift register); longer streams need B ≥ 4, since a pass that refills TXF and empties RXF takes 26 cycles. At B = 3, RXF slowly fills: 80 bytes pass, but by 100 it overruns, a reply is lost, and the firmware waits for it forever.
- `spi_master_3wire`, `spi_slave_3wire`: one data pin. The master writes, releases the pin, then reads, within one CS frame. For the slave the first byte is a command with MSB = 1 for a read; it takes the pin only after the master's last clock edge plus 10 cycles. SCK half period ≥ 16 cycles.
- `spi_slave_words`: words of 1–16 bits, as two FIFO bytes each, low first. SCK half period ≥ 12 cycles.
- Every slave prepares the next word before its first clock, so at CS high that word has already left the FIFO and is lost. Load one extra word, or none if only reading.

**I2C, options of `pe_fw.i2c_master` and `pe_fw.i2c_slave`.** Chosen when the firmware is built, so each only costs IMEM when used (master 105 words, up to 138 with `addr10` and `arb`).
- Timing: `i2c.check_timing` measures the bus against every limit of the I2C standard. The controller's START, STOP and ACK phases last at least H = floor(B/2), the same as a data bit, and the SCL period is B + 2. Fastest legal B: 478 (100 kHz), 126 (375 kHz), 48 (960 kHz), `hs_B` 16 (2.67 MHz; a 50% duty clock cannot reach 3.4 MHz here). These allow for ideal edges, so a real bus needs a margin for its rise time.
- `addr10`: 10-bit addressing, master and slave. A slave read needs a write address phase first, then a repeated START.
- `arb` (master): waits for a free bus, then checks every written bit: sent 1 and read 0 means lost. The loser releases both lines at once, sends no STOP, and reports status 3. The writes are clocked by firmware rather than the units, since a unit would keep driving the rest of its byte: B ≥ 48. A loser does not then answer as a slave if it is the one addressed.
- `hs_B` (master): HS mode. The master code at B, which nobody acknowledges, then a repeated START, and the data bytes at `hs_B`. ACK clocks keep their F/S length, which is legal. There is no current-source pull-up on this chip; the bus pull-up must allow the rate.
- Slave clock stretching is always on: SCL is held low from each ACK until the next byte is set up. `hold` adds cycles for slow processing.
- `general_call` (slave): address 0 with W is acknowledged, data goes to dmem[0xF0..], and the first byte raises the controller IRQ.

**USB (`pe_fw.usb_device`, `pe_fw.usb_host`).**
- Pins: jp is high in J (full speed D+, low speed D−), kp is the other. Both are owned by the TX unit; external pull-up and transceiver as USB requires.
- TX: NRZI, STUFFN = 6, DIFF, EOP, CHAIN; ETX AFTER = release. RX: SRC 3, NRZI, HUNT, STUFFN = 6, CHAIN, DIFF; ERX = SE0 (level 0 on jp with qualifier kp = 0, DUR = max(2, B/4)), STOP.
- A packet: `setoe` both pins, `ustart TX, 0x80` (sync), then PID and data through the FIFO. With data, the CRC unit has APPEND set, SKIP = PID + sync bits still to come.
- Scope: transaction level, one transaction at a time. Host: token, data, handshake, bus reset, keep-alive EOP. Device: address match, IN / OUT / SETUP, ACK / NAK / STALL chosen by the host side, reset detection. The external host MCU handles enumeration, data toggles, endpoint state, SOF timing and tokens' CRC5 in the host role.
- Suspend: ETX times the idle bus in hardware (4 words, as a firmware timer does not fit IMEM). CSTAT.ETX set means J for at least 1.37 ms since the last packet, and any packet clears it. The host reads it twice, 1.7 ms apart: still set means suspend. Resume shows as K on PIN_IN, and the firmware keeps hunting throughout. Remote wakeup is not implemented, so the device must not advertise it.
- Host resume (UH_OP = 3): K for UH_N × 256 cycles, then an EOP. J to K goes through the TX unit with DIFF, since only a unit write changes both pins in one edge.
- Timing (full speed): answer within 7.5 bit times is met by starting the sync before deciding between data and NAK, and by hunting before the CRC setup.
