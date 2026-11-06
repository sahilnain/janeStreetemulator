# Problem log

Every significant problem met while building the Protocol Emulator, what caused it, and how it was resolved. It runs from v1 to the current v2 (November 2026). Where a cause was not recorded at the time, the entry gives only the symptom and the fix.

Words used below:
- **B**: clock cycles per bit, at 48 MHz in v2 (50 MHz before the clock change).
- **Mutant**: a bug put in on purpose, to confirm that a test can fail.
- **Lockstep**: the Python model and the RTL compared on every clock cycle.

## What to take from this

1. **A passing test only shows what the test checks.** The I2C master broke the I2C standard's timing at every speed while every test passed, because the test device never measured timing. Formal proofs did not catch it either, since they check the RTL against its spec and the fault was in the firmware.
2. **Prove that a check can fail.** Mutants exposed a weak fuzz test, a check that could not fail, and runs that had not run at all. An empty result is not a pass.
3. **Check independently written versions against each other.** Writing the Python model from the spec found one RTL bug, two undefined behaviours and six spec errors that nothing else had found.
4. **Measure a claimed speed.** Three published speed figures were wrong: one worked out by hand, one label left over from the old clock, and one that ignored the extra 2 cycles in each I2C clock.

## 1. Bugs in the chip design (RTL)

| Problem | Cause | Fix | Found by |
|---|---|---|---|
| Writing `VOTE N = 0` made RX read all zeros; the spec says 0 means 1 | The value went to the vote block unchanged | `pe_rxu` treats 0 as 1 | Python model |
| A TX start with an empty FIFO loaded stale memory | The FIFO output was used without checking for empty | The unit gets 0 when the FIFO is empty | Python model |
| IMEM addresses 160–255 had undefined contents | The 8-bit `pc` can address more words than the memory holds | Past the end reads as `nop`, and writes are ignored; formal property added | Python model |
| RX kept stale events across a start | The tick delay line was not cleared | Cleared on start | Testing |
| SDA changed in the same cycle SCL fell | No delay between the clock edge and the data write | New `ODLY` drive delay in RX | Bus checker |
| RX sampled too early when a device stretched SCL | The sample tick was timed from the write to SCL, not from SCL actually rising | The tick waits until SCL is seen high | Testing |
| IMEM too small, and programs wrapped silently | Firmware outgrew the memory | Raised from 64 to 128, then to 160 words; `load_program` now checks the size | Testing |
| `RESTART` or `ustop` lost to a start in the same cycle | No priority between them | The start is gated at the top level | Testing |
| With the unit's own timer, every bit came out as bit 0 | `LEAD` was not gated by `EXT` | `lead_on` gating | Formal |
| `RESTART` did not win over overrun | Priority order in `pe_rxu` | Corrected | Formal |
| USB RX sampled inside SE0 when the sender was slightly slow | A sample can land one cycle before the detector stops the unit | `DIFF` gate: no sample while both lines are low | USB clock error test |
| USB RX took an extra bit when a 7-bit run lasted exactly 30 cycles at 4.17-cycle bits | A sample and a resync edge fell in the same cycle | The edge cancels the sample due then | USB clock error test |
| `eph` stayed set after a stop during EOP | Not cleared on stop | Cleared | Testing |
| Whole-chip proof failed: the ERX check stayed pending while RX was busy | RX `CHAIN` combined with ERX `CHECK` | No check in `CHAIN` (`unit_chain`), with a new module test | Formal |

## 2. Firmware bugs

| Problem | Cause | Fix |
|---|---|---|
| **I2C master broke the standard's timing at every speed.** At `B = 125` the ACK low phase was 833 ns against a 1,300 ns minimum, and the clock reached 457 kHz against a 400 kHz limit | ACK, START and STOP clocks are run by firmware with `B/4` steps, while data bits use `B/2` | These phases now last at least `B/2`. Fastest legal `B`: 478, 126, 48, and `hs_B` 16. New checker `i2c.check_timing` |
| **I2C slave gave too little data setup with `hold`** (42–62 ns against 100–250 ns) | SCL was released 2 cycles after the first data bit | 12-cycle wait before the release, about 290 ns of setup |
| **SPI stream hung at `B = 3`** after 80–100 bytes | A pass took 26 cycles and a byte arrives every 24, so RXF overflowed. The firmware counted replies with no way out | Loop reduced to 18 cycles. Once TX is idle the firmware no longer waits for a reply that isn't there: it reports status 2 and raises the IRQ |
| USB full-speed IN answered after 9 bits (limit 7.5) | Deciding and setting up the CRC before sending took too long | Send the sync first, decide after, and adjust the CRC `SKIP` to match |
| USB short-packet check gave wrong results | `cmp` overwrote the C flag being used | Compare with `cmp r6, 0` |
| USB RX hunt started before the CRC setup | Instruction order | Reordered |
| USB host J-to-K went through an SE0 glitch | Two separate pin writes | One write through the TX unit with `DIFF` changes both pins on one edge |
| `spi_slave_words` lost data | The next word was started in the wrong order relative to storing the last one | Start first, store after |
| Full-duplex UART example missed the RX re-arm deadline | A blocking `uwait TX` stopped the controller | `AUTOARM`, the RXRDY condition, and a non-blocking loop |

## 3. Architecture problems

| Problem | Resolution |
|---|---|
| One controller cannot place the TX stop edge to the exact cycle while also handling the RX stop check and re-arm; this gets worse at high baud | Two event detectors that act in hardware, with `AUTOARM`, a TX idle level and an RX stop check. Chosen by the user over lead/trail bits or using two chips |
| USB full speed at 50 MHz: `B = 4` is 4.2% fast, which fails the transmit tolerance | Clock changed to 48 MHz, which gives exact USB bit times. All UART timing was recalculated (115200 baud is now `B = 417`) |
| Gap-free streams and USB do not fit "one packet per start" | `CHAIN` added as a flagged exception, accepted by the user |
| UART formats beyond 8N1 and SPI words beyond 8 bits need a variable packet length | `NBITS` added to TX and RX |
| Odd `B` in `uart_ex` could not be timed exactly | The start bit is `B + 1` cycles, and all other edges stay on the bit grid |
| Suspend detection did not fit in IMEM as a firmware timer | The ETX detector times the idle bus in hardware, in 4 words |
| I2C arbitration could not use the units, which would keep driving the rest of a byte after losing | Firmware clocks written bits when `arb` is on |

## 4. Errors in the spec and documents

| Problem | Correct behaviour |
|---|---|
| When RX DATA is written | At the end of the after-step |
| What a stop in cycle D does | It is too late: the word still completes |
| SCK at the start of a CPHA 0 word | Written idle in cycle S |
| Which writes at offset H count as a release | Corrected rule for CPOL and CPHA |
| EOP J | It also writes D2 low |
| Reset values | Were missing, now listed |
| Long SPI streams "need `B ≥ 3`" | Measurement showed a hang at `B = 3`. After the firmware fix, `B ≥ 3` is true |
| I2C tests labelled 3.4 MHz and 400 kHz | Those settings ran at 3 MHz and 378 kHz, because the SCL period is `B + 2` |
| Speed labels left from the 50 MHz clock (for example 4.2 Mbaud) | Corrected to 48 MHz values (4 Mbaud) |
| v1 study claimed UART TX of about 12 Mbaud | Corrected to about 4 Mbaud (`B ≥ 12`), and RX to 3–6 Mbaud |
| Early study described this design as not being "protocols directly in hardware" | The user challenged the argument, and that section was rewritten |

## 5. Test and verification weaknesses

| Problem | Fix |
|---|---|
| The I2C test device never measured timing, which hid the master timing bug | `check_timing` covers rate, low and high times, START and STOP setup and hold, bus-free time and data setup. A permanent test confirms it flags bad settings |
| The first check for the slave setup fault could not fail: the test master set the clock edge, not the chip | New test using `hold`, confirmed to fail without the fix |
| First attempt at that test judged the chip against a master with an illegal 229 ns low phase | Rewritten with a legal master |
| The first fuzz test missed mutants: packets rarely finished and `st2` was not compared | Added structured random scenarios and compared more state |
| A hung test blocked a mutation run | Added cocotb timeouts |
| One planted mutant behaved the same as the original, so it proved nothing | Replaced with a real bug |
| A CRC mutant first changed a comment | Moved to the code, and it was then caught |

**Python model bugs**, found when lockstep disagreed with the RTL. Each was checked against the spec to decide which side was wrong:
- stop and vote reset, and delay lines
- `FIRST = 0`
- TX clock corner cases
- `QUAL` applying only to the edge
- TX reset value `BIT = 1`
- overrun set winning over clear

**Testbench bugs:**
- `I2cMaster.start()` produced an illegal SCL low pulse of about 40 ns
- a repeated-START low phase was wrong
- a 3-wire test left a command byte in RXF
- an SCK period count was off by one
- a loopback helper forced P0
- inputs were X at time 0 under lockstep
- reading an empty RXF gave X on `uo_out`, so comparison is now bitwise and the host bus tolerates X
- UART decode ran before the last stop bit had finished
- a USB stuff-error test used bit index 20, not 19
- USB turnaround was measured from the wrong edge
- an early run-phase assumption in `runs()` was wrong
- register and immediate operands were mixed up in some tests
- `st` addresses were wrong in some tests
- harness reset and clock tasks were not restarted per test

**Formal property errors** (the design was right, the property was not):
- the detector `!armed` counter check was too strict
- the invariant `!lf → idx == 0` was missing
- an ETX cover was unreachable
- the chip proof needed a shared reset flag and a busy contract
- `bit_v` has to be allowed in the start cycle
- the stuffing property must exclude `EXT`

## 5a. Model bugs found by the v2 z3 proof

The proof compares the model and the RTL in every state, including states no test reaches. All of these needed firmware to change a unit's config in the middle of a word, or two events to land in the same cycle. The RTL was right each time, and no RTL changed.

| Problem | Fix |
|---|---|
| TX bit timer `o`, bit index `idx` and RX bit count `nb` kept counting in the model but wrap in the RTL | Masked to the RTL's width |
| TX and RX bit position went negative in the model after `NBITS` shrank mid-word, while the RTL wraps at 4 bits | 4-bit position |
| RX LSB-first shift used all of `nb` in the model, the RTL only uses 4 bits | 4-bit shift |
| TX with external clock ignored `crc_pkt` in the model, if the clock mode changed during a CRC word | Same handling as the timer path |
| TX `sp`: a stretch release and the half-bit set in the same cycle cleared it in the model, set it in the RTL | Set wins |

| Proof problem | Fix |
|---|---|
| Each independent `if` in the model doubled the paths: `pe_regs` took 27,820 paths and 22 minutes, `pe_det` never finished | Model logic moved to a small `mux` helper, which adds no path in the proof. `pe_regs` now takes 46 paths and 4 seconds, `pe_det` 13 paths and 1 second |
| Register and memory indexing split the path once per index | `SymList` turns indexing into a mux, and fails if the index can be out of range |
| A proof could pass because it left a register out, or because its assumptions contradict | The proof now fails if any flip-flop is not compared, or if the assumptions cannot all hold |
| My first mutant run reported 28 of 29 as missed | My script read the wrong output line. Fixed: 29 of 29 caught. One mutant had to be replaced because it did not change behaviour |

## 6. Tool and environment problems

| Problem | Fix |
|---|---|
| A z3 equivalence run hung for over 11 hours, and its timeout did not end the container | Stopped that container by name and gave each case its own time limit |
| A proof timeout was hidden by my `grep` | Check exit codes, not just filtered output |
| A `\|` in a test filter broke the run; it happened twice, and the second time gave empty mutant results | Use one filter, and treat an empty result as a failed run |
| `${2:+VAR=1} make` did not set the variable | Use `env` |
| A soak run hit my own time limit | Split into shorter runs |
| `pe_txu` proof `p8` was too slow with one solver | Moved to Bitwuzla, with Boolector as a second check; `top_prf` also uses Bitwuzla |
| `$global_clock` is not allowed inside a generate block | Used a `(* gclk *)` wire |
| Chip cover failed or hung on submodule covers | Wrapped module covers in `ifdef <MOD>_TOP` |
| Yosys does not support hierarchical references | Added formal-only ports on `pe_pins` |
| Yosys memory pass created a duplicate `rdreg`; `DEPTH=1` pointers were removed; `$scopeinfo` cells appeared | `-nordff`, default handling, and skipping those cells |
| Yosys reported "no driver" | My script lacked `read_liberty -lib` |
| Verilator reported width warnings, a false combinational loop, and a latch from a shared loop variable | Typed localparams; busy outputs made independent of start and stop; a separate loop variable |
| A patch left stray text in `pe_rxu` | Removed and re-tested |
| Unit tests could not import `pe_isa` | Set `sys.path` in the runner |
| `git push` failed because a branch and a tag were both named `v1-uart` | Tag renamed to `v1.0`, with explicit refspecs |
| `git push` for v2 failed: no GitHub login in this environment | The user pushes from their own terminal |
| The image has no OpenROAD or magic | Place and route is still to do |

## 7. Still open

These are known limits, not fixed problems.

**Design and verification**
- The design needs about 178% of the 6x4 area in synthesis, so it does not fit yet. This is the next task.
- Place and route, pad timing and gate-level simulation have not been run, so every speed comes from simulation with ideal peers.
- The z3 proof covers each module, not the wiring between them. The chip top (`chip.py` against `tt_um_sahil_protoemu.v`) is still only checked by lockstep and fuzz, for what they run. A few proof assumptions rest on that wiring, such as a unit only getting a start while idle.
- The IMEM proof covers the flip-flop version. The latch version is not proved against the model, and would need it if the area work moves to latches.
- One branch is uncovered: `pe_txu.v:377`, which needs a mid-word `EEDGE` change.
- `docs/modules.md` still describes v1, and `docs/info.md` is an empty template.

**Protocols**
- **I2C:** `check_timing` assumes ideal edges, so a real bus needs a margin for rise time. High speed tops out at 2.67 MHz because SCL has a roughly even duty cycle. There is no current-source pull-up. A master that loses arbitration does not then answer as a slave. As a slave, the chip is only checked for data setup.
- **SPI:** slaves lose the word already prepared when CS rises. At `B = 2`, long streams pause SCK.
- **USB:** no high speed and no remote wakeup. Suspend is read by the host, not an IRQ.
- **UART:** break handling is only in `uart_ex`. With odd `B` the start bit is one cycle long.
