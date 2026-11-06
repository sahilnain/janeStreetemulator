# Protocol conformance

This document records how the chip was checked against the industry standards for UART, SPI, I2C and USB, what was wrong, and how each problem was fixed. The conformance tests run on the Python model of the hardware; Section 7 records what was then carried over to the RTL and checked there. Everything here is on the `test` branch.

## 1. Method

Earlier checks compared our own code with our own code: the model with the RTL, and the firmware with test peers written in this project. None of them could catch a misreading of a standard. This work adds outside references:

| Layer | What it is | Who wrote it |
|---|---|---|
| Decoders | sigrok `uart`, `spi`, `i2c`, `usb_signalling`, `usb_packet`, `usb_request`, reading the recorded bus trace | Third party |
| Standards | NXP UM10204 Rev. 7, USB 2.0, PC16550D data sheet | Third party |
| CRC check | The published CRC-5/USB and CRC-16/USB check values, and `crcmod` for CRC-16 | Third party |
| Time limits checker | Measures traces against the numbers in the standards. sigrok decoders look at bits, not at times | This project, numbers from the standards |
| Test stimulus | Bus peers written from the standards. Their traces also go through the decoders | This project |
| Device under test | `model/pe_model` (the chip) running `test/common/pe_fw.py` (the firmware) | This project |

**The rule for fixes:** fix the firmware first, because that leaves the RTL and its proof valid. Change the model only if the standard cannot be met any other way. That happened once (I5).

**Why a pass means something:** `test_controls.py` feeds the decoders and checkers traces that are wrong on purpose (bad parity, bad stop bit, bad CRC, missing stuff bit, wrong SPI phase, I2C too fast), and they must flag them. Each bug in Section 6 was also seen failing before its fix.

**Result:** 1,799 cases: 1,765 pass, 34 are skipped, none fail. The skips are not requirements: packets over the maximum size for low speed, SE0 glitches shorter than one clock period, UART bit times too short for the firmware, and SPI slave half periods under the documented limit.

| File | Cases |
|---|---|
| `test_uart.py` | 745 |
| `test_spi.py` | 486 |
| `test_usb.py` | 273 |
| `test_i2c.py` | 282 |
| `test_controls.py` | 13 |

To run them: `docker build -t pe-conformance conformance`, then `conformance/run.sh -n 12`. A full run takes about 9 minutes.

## 2. UART

There is no formal UART standard. The reference is the de facto behaviour of the PC16550D, which is what other UARTs are compatible with.

| Requirement | Source | Check | Result |
|---|---|---|---|
| Idle high, start low, data LSB first, stop high | 16550 Section 8 | sigrok, TX and RX | Pass |
| 5–9 data bits, parity none / even / odd / mark / space, stop 1 / 1.5 / 2, plus inverted and MSB first: all 300 formats, both directions | 16550 LCR | sigrok | Pass |
| Standard baud rates 9600 to 3,000,000, within 0.2% | Common practice | sigrok, plus the receiver at the exact rate | Pass |
| All 256 byte values, back to back | | sigrok | Pass |
| Edges within 2% of the bit grid | Common practice | `check_grid` | Pass |
| Receiver reads only the first stop bit | 16550 LCR bit 2 | 1 stop bit sent to a 2-stop receiver | Pass |
| False start: a low pulse gone by mid-bit is not a start | 16550 features | Pulses of 10%, 25%, 40% | Pass |
| Framing error detected, then in step again | 16550 LSR bit 3 | Bad stop bit, then a good frame | Pass |
| Parity error detected | 16550 LSR bit 2 | Each parity mode | Pass |
| Break: low for longer than a frame, reported once; and sent | 16550 LSR bit 4 | sigrok `rx-break` both ways | Pass |
| Overrun: oldest bytes kept, and reported | 16550 LSR bit 1 | 12 frames into an 8-byte FIFO | Pass |
| Baud tolerance | No figure in the data sheet; ±3% is the usual budget | ±3% in 8N1, 8E1, 9O1 | Pass. Measured limit in 8N1: −4.5% to +5.5% |
| Joining a stream mid-frame | | In step after one idle frame | Pass |
| Full duplex, out of phase | | Down to bit time 16 (3 Mbaud) | Pass |

**No UART bugs were found.** A spike in the start bit looked like one at first, but it only does harm at mid-bit, which is exactly where a 16550 would reject the start bit too.

Two behaviours differ from a 16550, and neither breaks anything:

- After a framing error a 16550 treats the low stop bit as the next start bit. The chip waits for a new falling edge, so it can lose that frame if the sender really did run on without idle.
- Bytes with a bad stop bit or parity are dropped and counted, not delivered with a flag.

## 3. SPI

SPI has no formal standard either. The reference is the CPOL / CPHA mode definition that started with Motorola and that every vendor repeats.

I could not get a copy of the Motorola SPI Block Guide. The mode definitions come from sigrok and from vendor documents. The half-period CS lead and lag used in S2 is my memory of that guide, so treat it as a convention to confirm, not a quoted limit.

| Requirement | Check | Result |
|---|---|---|
| All 4 modes, MSB and LSB first, as master and slave | sigrok on both data lines | Pass |
| Master at bit time 2 to 48 | sigrok, data both ways | Pass |
| Word sizes: master 1–16; slave 1–8 and 9–16 | sigrok `wordsize` | Pass |
| Stream with no pause (B ≥ 3) | No gap in SCK | Pass |
| 3-wire, master and slave | sigrok, no bus contention | Pass |
| SCK at CPOL whenever CS changes; even number of edges; right count | `check_frames` | Pass |
| Data changes only on the shift edge, never on or after a sampling edge | `check_frames` | Pass |
| **MISO released when not selected** | Output enable checked | Pass after S1 |
| CS lead and lag at least half an SCK period | `check_frames` | Pass after S2 |
| CS high in mid-word ends the frame; next frame starts at bit 0 | Bit positions 1, 3, 5, 7 | Pass |
| Chip as master talking to chip as slave | sigrok | Pass |

Measured limits: the slave needs at least 6 cycles from CS low to the first SCK edge (125 ns, CPHA 1), and half periods of at least 6 cycles for CPHA 0 and 4 for CPHA 1.

## 4. I2C

Reference: NXP UM10204 Rev. 7. The master runs at Standard mode (B = 478), Fast mode (B = 126) and Fast-mode Plus (B = 48), each tested with ideal edges and with the slowest rise and fall times the standard allows. Hs mode is master only.

| Requirement | Source | Check | Result |
|---|---|---|---|
| START, address, R/W, ACK, data, repeated START, STOP | 3.1.4–3.1.6 | sigrok event sequence | Pass |
| fSCL, tLOW, tHIGH, tHD;STA, tSU;STA, tSU;STO, tBUF, tSU;DAT | Table 11 | Limits checker at thresholds | Pass |
| Controller data hold: SDA waits for SCL to fall | Table 11 note 2 | Chip's own edges against tf | Pass after I6 |
| Every 7-bit address; every data byte | 3.1.10 | sigrok | Pass |
| NACK on address or data gives STOP and a status | 3.1.6 | sigrok | Pass |
| Controller waits for stretched SCL, byte and bit level | 3.1.9 | Up to 20,000 cycles | Pass |
| 10-bit addressing, both roles | 3.1.11 | 5 addresses | Pass |
| Hs mode: master code, NACK, Sr, then Hs timing | 5.3.2, Table 13 | sigrok and limits | Pass |
| **Bus is busy from START to STOP** | 3.1.8 | Slow controller, long SCL high | Pass after I1 |
| Arbitration on address and data; loser stops at once, no STOP | 3.1.8 | Two chips, start skew 0–8 cycles | Pass |
| **Clock synchronization** | 3.1.7 | Two chips at 64 / 96 / 200 | Pass after I2 |
| Target ignores other addresses | 3.1.10 | All 127 others | Pass |
| No answer to reserved addresses and START byte | Table 4 | 0x00–0x0F | Pass |
| START byte sequence, then a real transfer | 3.1.15 | sigrok | Pass |
| **START or STOP at any bit** | 3.1.4 | After 1, 2, 3, 5, 7 bits | Pass after I3 |
| Target ignores Hs traffic and recovers | 5.3.5 | Master code, fast bits | Pass |
| General call, when built with it | 3.1.13 | | Pass |
| **Target's data ready before it releases a stretched SCL** | Table 11 notes 3, 4 | Slow SDA, fast SCL | Pass after I4 |
| tVD;DAT, tVD;ACK | Table 11 | Measured | Pass |
| **Spikes under 50 ns suppressed** | Table 10, tSP | 1–3 cycles on SCL and SDA, both roles | Pass after I5 |
| **Bus clear: SDA stuck low** | 3.1.16 | Device letting go after 0–8 clocks, or never; a read abandoned at bits 1–7; another controller's transfer must not be clocked over | Pass after I7 |

Measured top SCL speeds with the filter on: 99 kHz (SM), 364 kHz (FM), 889 kHz (FM+).

## 5. USB

Reference: USB 2.0 chapters 7 and 8, low speed (B = 32) and full speed (B = 4). The chip works at transaction level: higher layers such as enumeration and data toggles belong to the host software.

| Requirement | Source | Check | Result |
|---|---|---|---|
| Sync, PID, NRZI, bit stuffing, CRC5, CRC16, EOP | 7.1, 8.3 | sigrok, both directions | Pass |
| A stuff bit even as the last bit before EOP | 7.1.9 | Searched payload whose CRC ends in 6 ones | Pass |
| EOP SE0: 160–175 ns FS, 1.25–1.5 µs LS | Tables 7-9, 7-10 | Measured: 167 ns, 1.33 µs | Pass |
| **Device response between 2 and 6.5 bit times** | 7.1.18.1 | Measured below | Pass after U1 |
| **Host response at most 7.5 bit times** | 7.1.18.1 | Measured below | Pass after U2 |
| **Waiting side times out between 16 and 18 bit times** | 7.1.19.1 | Measured below | Pass after U3, U11 |
| **Reset at least 10 ms, resume at least 20 ms** | Table 7-13 | sigrok reset; K measured | Pass after U4 |
| LS keep-alive | Table 7-2 | sigrok `keep-alive` | Pass |
| Device answers only its own address; ignores SOF | 8.4 | All 128 addresses at FS | Pass |
| Bad CRC5, CRC16, stuffing: no answer | 8.3.5, 7.1.9.1 | | Pass |
| **Bad PID: no answer** | 8.3.1 | Tokens and data, both roles | Pass after U5, U6 |
| **Dribble bit accepted** | 7.1.9.1 | Both roles | Pass after U7 |
| **SETUP always gets ACK** | 8.5.3 | With the endpoint set to NAK | Pass after U8 |
| Shortest EOP accepted: 82 ns FS, 670 ns LS | 7.1.13.2 | | Pass |
| **SE0 in a transition is not an EOP** | 7.1.4 | Up to 210 ns at LS | Pass after U9 |
| **Reset detected at 2.5 µs**; an EOP is not a reset | Table 7-14 | | Pass after U10 |
| Clock tolerance: ±0.25% FS, ±1.5% LS | 7.1.11 | Full range | Pass |
| **Jitter: 20 ns FS, 141 ns LS, to the next transition** | Tables 7-4, 7-5 | Neighbouring edges differ by up to 1 cycle (FS), 7 cycles (LS); data and response limits checked | Pass after U14, U15 |
| **Every byte stored and counted, whatever the phase of the last byte against the EOP** | | All 16 clock phases of a bit pair, 8 payloads, two transactions in a row, both roles | Pass after U13 |
| **Next packet 2 bit times after the last** | 7.1.18.1 | Token after SOF, data after token | Pass after U12 |
| Suspend after 3 ms, resume, then normal work | 7.1.7.6 | | Pass |
| Control transfer, chip at both ends | 8.5.3 | sigrok `usb_request` reads the request | Pass |

Measured response times, in bit times. Each is the range over clock phase and random payloads, with and without jitter at the budget. An earlier version of this table gave a single phase, which hid the worst cases: see U15.

| | Full speed | Low speed | Limit |
|---|---|---|---|
| Device ACK | 4.25–6.0 | 2.22–2.53 | 2 to 6.5 |
| Device NAK | 6.25–6.5 | 2.53–2.75 | 2 to 6.5 |
| Device IN data | 6.25–6.5 | 2.53–2.75 | 2 to 6.5 |
| Host ACK | 3.75–6.75 | 2.47–2.81 | 2 to 7.5 |
| Host time-out (not swept) | 17.5 | 17.4 | 16 to 18 |

Device NAK and IN data at full speed reach the limit exactly, with no margin.

The firmware still fits IMEM, out of 160 words: device 160 (FS) and 159 (LS), host 158 and 155. **The full-speed device has no spare word left.**

## 6. Bugs and solutions

All of these passed the old tests, because those tests shared the firmware's assumptions or never reached the case.

### SPI

| ID | Problem | Effect | Solution | Where |
|---|---|---|---|---|
| S1 | The slave drove MISO all the time | Could not share a bus with another slave | Enable MISO after CS falls and release it after CS rises. The controller cannot change OE on a pin while its unit is busy, so ETX is now armed after MISO is on | Firmware |
| S2 | CS lead and lag were a fixed few cycles (2–10), under half an SCK period at slower speeds | Slow devices could miss the first or last bit | Wait half a period after CS falls and before it rises | Firmware |

### I2C

| ID | Problem | Effect | Solution | Where |
|---|---|---|---|---|
| I1 | A controller took "both lines high for half a period" to mean the bus was free | It started in the middle of a slower controller's transfer | Track START and STOP with ERX. Busy lasts from one to the other | Firmware |
| I2 | Clock synchronization was missing, in four ways: the high phase did not end when another controller pulled SCL low; SDA was read at the end of the high phase, when the target may already have let go; START hold ignored SCL; and a first START came two half periods after the free check | Two controllers at different speeds lost arbitration to themselves or saw false NACKs | A synchronized high phase that reads SDA at its start and ends early if SCL falls; a first START straight after the free check; reads bit by bit in multi-master firmware | Firmware |
| I3 | A START or STOP right after bit 8 was missed, and the target drove ACK into it | The target pulled SDA low in the middle of the next transfer | Check for START or STOP until SCL falls before driving ACK; clear stale RX data | Firmware |
| I4 | The target released stretched SCL 250 ns after its data | On a bus with slow rise times SDA was not yet valid | Wait tr(max) + tSU;DAT = 1250 ns, as Table 11 note 4 says | Firmware |
| I5 | Inputs had no spike filter | A 50 ns spike on SCL or SDA was taken as a clock or as a START or STOP | New `PIN_FILT` register: a new level must be seen on 4 samples in a row. This rejects 3 samples, the most a 49 ns pulse can touch at 48 MHz. Off by default; adds 4 cycles of input delay on that pin | **Model** |
| I6 | The master moved SDA 125 ns after pulling SCL low, when the units finished | With a 300 ns fall time, SCL could still be high, which makes a false STOP or START | Wait `ODLY` (312 ns at SM and FM) before moving SDA | Firmware |
| I7 | No bus clear: with SDA held low by a device stuck in a read, the master waited for ever | The bus stayed dead until a reset | Up to nine SCL pulses until SDA is released, then a STOP and the request. If it is not released: status 4 and the IRQ, with no START. The multi-master build first waits for SDA low and SCL high to last about 8 ms (`stuck`), so it does not clock over another controller's zeros | Firmware |

### USB

| ID | Problem | Effect | Solution | Where |
|---|---|---|---|---|
| U1 | Device answered in 6.75–7 bit times | Over the 6.5 limit for a detachable cable | Shorter paths: a tighter poll loop and fewer instructions before the answer. Now 6.25 at most | Firmware |
| U2 | Host ACK took 11.75 bit times | Over the 7.5 limit, so a device would have timed out | Check the PID as bytes arrive, so only CRC and stuffing remain after EOP. Now 3.75 | Firmware |
| U3 | Host timed out after 26–35 bit times, and an answer starting near the limit was missed | Over the 18 limit | Watch for the start of a packet, with a time-out of 17.5 bit times | Firmware |
| U4 | Reset lasted at most 0.44 ms and resume 0.88 ms, as the count was 8 bits | The required 10 ms and 20 ms could not be reached | 16-bit count (`UH_N`, `UH_NH`), and a loop of exactly 256 cycles | Firmware |
| U5 | Host sent ACK to a data packet with a bad PID | Against 8.3.1 | First byte must be DATA0 or DATA1, otherwise no ACK and `UH_BADPKT` | Firmware |
| U6 | Device answered a data packet with a bad PID | Against 8.3.1 | Same check | Firmware |
| U7 | A dribble bit made the CRC fail, in both roles | Good packets lost behind hubs | The extra 1 bit gives a known CRC value (`CRC5_DRIB`, `CRC16_DRIB`), so accept it as well as the residual. No new hardware | Firmware |
| U8 | SETUP got whatever handshake the endpoint was set to | Against 8.5.3 | Always ACK a SETUP | Firmware |
| U9 | SE0 of more than 8 cycles counted as an EOP at low speed, but up to 210 ns (10 cycles) is legal in a transition | Good packets cut short at low speed | Detector time B/2: 16 cycles (333 ns) at LS | Firmware |
| U10 | Reset took 2.7 µs to detect | A 2.5 µs reset was missed | Decide earlier, and still after the longest EOP | Firmware |
| U11 | After sending data the device waited 40 bit times for a handshake | It took the host's next token for a handshake and lost it | Give up before 18 bit times, by the start of a packet | Firmware |
| U12 | Returning to listen took too long | A token 2 bit times after an SOF was missed | Start hunting first and do the rest after. The next bits reach the FIFO only once the hunt ends | Firmware |
| U13 | At full speed the last byte of a packet can reach the FIFO just after the loop finds it empty and just before it sees the EOP. About 1 packet in 3 lost it, depending on the phase | `UH_RXLEN` and `UD_RXLEN` one short, and the last CRC byte not stored, so software working out the payload size from the length loses a data byte. In the host the byte stayed in the FIFO and was read as the PID of the **next** packet: that transaction reported a handshake the device never sent | The CRC unit has already seen the byte, so the answer does not depend on it: answer first, then store what is left | Firmware |
| U14 | The same race on a one-byte packet in the host | With jitter inside the budget, 7 of 300 good handshakes were reported as `UH_BADPKT` | Read the FIFO once more before judging a one-byte packet | Firmware |
| U15 | The device's ACK could take 6.75 bit times with jitter inside the budget, over the 6.5 limit | Over the limit for a detachable cable, by 1 clock cycle | Look for the EOP every 2 instructions instead of 3, using the FIFO-empty jump condition | Firmware |

**How U13–U15 were missed.** Three gaps in these tests, all now closed:

- The tests ran each packet at one clock phase, and U13 depends on the phase. It was found by accident on the RTL, when a new random seed moved a chip-to-chip test onto a bad phase.
- The full-speed jitter test applied no jitter: its cycle count was `int(20 ns // 20.83 ns)`, which is 0. The earlier "Jitter: Pass" at full speed meant nothing. The count is now rounded to nearest and the test asserts it is at least 1.
- The jitter tests did not check response times.

Fixing U13 by reading the FIFO once more on the way to the handshake was tried first. It put the device's ACK at up to 7.5 bit times and the conformance tests rejected it, which is why the stored byte now comes after the answer. On a bad packet with a good CRC, the host now turns the CRC unit off instead of testing a flag, so its ACK path did not get longer.

Changes to how firmware is used:

- `UH_RPID` is now `0x80`, the first byte received. The separate copy was removed to fit IMEM. The RTL tests use the new `pe_host.usb_result`.
- `dmem[UH_NH]` is the high byte of the reset and resume count.
- A one-byte packet in the host, whatever its PID, is now reported as `UH_OK` with `UH_RXLEN` = 1, and gets no ACK. A lone DATA PID used to give `UH_BADPKT`. Software must look at `UH_RPID`, as it already must for handshakes.
- The I2C master's status 4 means SDA is stuck low (I7).
- A device now needs a sync of at least 6 bits at full speed, where the old firmware took 4 when the packets were 6 bit times apart. The standard gives no shorter figure for full speed, so this is not a violation, but the margin is smaller.

## 7. What was carried over to the RTL

Only one model change needed new hardware: `PIN_FILT` (I5). The RTL now has it: registers at 0x16–0x17, and a filter per pin in `src/io/pe_pins.v`, 32 flip-flops in all (8 enable, 8 output, 16 count). All the other fixes are firmware.

| Check, on the final RTL and firmware | Result | What it shows |
|---|---|---|
| Chip tests (cocotb) | 138 of 139 pass, 1 skipped (optional soak), on each of 3 random seeds | The firmware fixes, the filter and bus clear work on the RTL |
| Same suite with lockstep on every test (`make LOCKSTEP=1`) | 138 of 139 pass, 1 skipped. One seed. First full run of this | RTL and model agree on every flip-flop, every cycle, across all of those tests |
| Dual-chip tests | 5 of 5 pass | |
| Unit benches | 28 of 28 pass | Includes the filter's pulse widths |
| Formal (`pe_pins`, `pe_regs`, top level) | All pass | |
| z3 proof and model tests | 58 of 58 pass | The model matches the RTL **with the filter included**: the harness now compares `PIN_FILT` and the filter state |

The unit, formal and z3 runs were made on this RTL before the last firmware fixes, which changed firmware and tests only.

The spike test on the RTL has a negative control: with the filter off, SDA spikes of 1 cycle corrupt a read and spikes of 2 and 3 cycles cause a NACK. SCL spikes do not trouble a master, as it drives SCL itself, so they do not serve as the control.

## 8. What this does not show

- **Analog behaviour.** The bench is digital, one sample per 48 MHz clock. Rise and fall times are modelled as a delay at the input threshold. Voltages, drive strength, USB signal quality and the pads are not covered.
- **Real devices.** No real USB host, I2C part or SPI part has talked to this. Decoders check that a trace follows the rules; they don't prove every real device accepts it.
- **sigrok's own limits.** The decoders look at bits. Most timing checks are my code using numbers from the standards. The decoders are also not strict everywhere, so a clean decode is necessary but not sufficient.
- **My reading of the standards.** The requirements in Sections 2–5 are the ones I picked out. I read the relevant clauses and not every word, and SPI rests partly on memory (Section 3).
- **Chip-level timing.** The model is cycle-accurate at 48 MHz but says nothing about gate delays.

Features not implemented:

| Feature | Standard | Note |
|---|---|---|
| I2C device ID, software reset | 3.1.14, 3.1.17 | Optional. General call data is stored, not acted on |
| I2C Hs mode as target | 5.3 | Master only |
| USB remote wakeup | 7.1.7.7 | The device must not advertise it |
| USB high speed | 7.1 | Far above the 48 MHz clock |
| USB data toggles, enumeration, frame timing | 8.6, 9, 11 | Left to host software |

Known limits of what is implemented:

- **SCL stuck low is not cleared.** The standard offers only a hardware reset or a power cycle for that case (3.1.16), and the master waits.
- **Bus clear in the multi-master build** waits about 8 ms before acting. Another controller that pauses for longer than that with SDA low and SCL high could be clocked over.
- **I2C multi-master with 10-bit addressing does not fit**: 166 words against 160. The other builds fit: single 126, 10-bit 138, multi-master 154, Hs 132, Hs with 10-bit 144.
- **Bug U13 depended on timing that random tests hit only sometimes.** The phase sweeps now cover it, but other timing races could remain outside what was swept.
