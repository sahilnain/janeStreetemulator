# SPDX-License-Identifier: Apache-2.0
"""
pe_fw: the firmware. Every protocol the chip speaks is one of the programs in this file.

How to read it
--------------
Each function builds one program with the assembler in pe_isa.py and returns an Asm; asm.words()
gives the 32-bit words the host loads into IMEM. Options such as `arb` are plain Python: they choose
which instructions get assembled, so a program only costs IMEM for what it uses. a.L("name") places
a label, and the instruction methods return the Asm, so several can be chained on one line.

Every program has the same shape:
1. ucfg ...       Set up the units once: pins, bit time, frame, line coding, detectors.
2. idle loop      Wait for work: a doorbell from the host, a byte in a FIFO, or an event on a pin.
3. the protocol   Start a unit (ustart), wait for it (uwait), do the step between packets, repeat.
4. host irq       Report the result, and go back to the idle loop.

The hardware does the bits and the firmware does the rules. The controller runs one instruction per
2 clock cycles (42 ns at 48 MHz), far too slow to time the bits of a fast protocol, so the units
time them. The firmware decides what happens between packets: START, STOP, ACK, error checks.

Conventions
-----------
* Timing: an instruction takes 2 cycles. delay(n) or dly=n takes 2 * ceil((n + 1) / 2) in total.
* The host talks to a DMEM-driven program through DMEM: it writes a request, writes DOORBELL, and
  gets the IRQ when the result is there. Each program's docstring gives its DMEM map.
* Carry as a result: several subroutines return C = 1 for "something else happened". cmp(x, 0)
  gives C = 0 for any x, and with x = 0, cmp(x, 1) borrows, which gives C = 1.
* fget(..., nb=1) does not block: C = 1 means the FIFO was empty, C = 0 means a byte was read.

k = 2: detector-to-unit latency (pin synchroniser), docs/spec/blocks.md Section 5.3.
Full explanation: docs/architecture_guide.md. Instructions: docs/spec/isa.md. Units: docs/spec/blocks.md.
"""

import pe_isa as I
from pe_isa import Asm

K_LAT = 2


def uart_consts(B, N=1, K=0, S=None):
    """Timing constants for bit period B and an RX vote of N samples, S apart."""
    if S is None:
        S = max(1, B // (N + 1))
    assert N * S + 1 <= B, "vote window must fit in a bit"
    dur = B // 2
    return dict(B=B, N=N, K=K, S=S, DUR=dur, FIRST=B - dur + 1,
                ADLY=B + B // 2 - N * S - 1, TXDUR=B - K_LAT)


def uart(txd=0, rxd=1, B=112, N=9, K=1, S=12, B_rx=None, tx=True, rx=True, stop_lvl=1):
    """Full-duplex UART: TX on `txd`, RX on `rxd`. Returns an Asm."""
    # Registers: r0 the received byte. r7 the TX state: 0 waiting for the line to be idle, 1 start bit running.
    # The loop never blocks, so TX and RX are both served by it. Each edge on the line comes from hardware.
    c = uart_consts(B if B_rx is None else B_rx, N, K, S)
    a = Asm()
    # Setup. The TX unit sends the data bits; ETX adds the stop edge the moment the last bit ends.
    if tx:
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(txd))
        a.ucfg(I.TX, I.TX_BIT, B)
        a.ucfg(I.TX, I.TX_FRAME, 0)
        a.ucfg(I.ETX_U, I.D_AFTER, I.A_DRIVE1 if stop_lvl else I.A_DRIVE0)
    # RX runs in hardware from here on: ERX finds the start bit and starts the RX unit, the RX unit votes on
    # each bit, ERX checks the stop bit and re-arms itself (AUTOARM). RXRDY means a byte was received and checked.
    if rx:
        a.ucfg(I.RX, I.RX_PINS, I.rx_pins(rxd))
        a.ucfg(I.RX, I.RX_BIT, c["B"])
        a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(hold=1))
        a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(N, K))
        a.ucfg(I.RX, I.RX_STEP, c["S"])
        a.ucfg(I.RX, I.RX_FIRST, c["FIRST"])
        # Start bit: a falling edge, then low for half a bit, so a short glitch is not taken for a start.
        a.ucfg(I.ERX_U, I.D_COND, I.cond(rxd, I.FALL, start=1, autoarm=1))
        a.ucfg(I.ERX_U, I.D_DUR, c["DUR"])
        a.ucfg(I.ERX_U, I.D_AFTER, I.erx_after(1, 1))
        a.ucfg(I.ERX_U, I.D_ADLY, c["ADLY"])
        a.evt(I.DET_RX, I.EV_ARM)
    # Idle the line high, and have ETX report when it has been high for one bit time.
    if tx:
        a.set(txd, I.SRC_1)
        a.setoe(txd, 1)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # line idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)
    a.mov(7, 0)                                                  # TX state
    a.L("loop")
    if tx:
        # TX side, state 0: when the line has been idle and a byte is waiting, drive the start bit. ETX then times
        # it, and starts the TX unit by itself after B - k cycles, so the start bit is exact.
        a.cmp(7, 1)
        a.jmp("tx_run", I.Z)
        a.jmp("rx_chk", I.NETX)                                  # idle >= 1 bit?
        a.jmp("rx_chk", I.TXF_EMPTY)                             # and a byte?
        a.evt(I.DET_TX, I.EV_CLR)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 0, start=1))
        a.ucfg(I.ETX_U, I.D_DUR, B - K_LAT)
        a.evt(I.DET_TX, I.EV_ARM)
        a.set(txd, I.SRC_0)                                      # start bit
        a.mov(7, 1)
        a.jmp("rx_chk")
        # TX side, state 1: ETX has fired, so the TX unit is sending. Set ETX up to time the stop bit and the idle
        # after it. The arm takes effect after the stop edge, which ETX drives.
        a.L("tx_run")
        a.jmp("rx_chk", I.NETX)                                  # TX not started yet
        a.evt(I.DET_TX, I.EV_CLR)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # stop + idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)                                # deferred to after stop edge
        a.mov(7, 0)
    # RX side: take a finished byte. Reading DATA clears RXRDY.
    a.L("rx_chk")
    if rx:
        a.jmp("loop", I.NRXRDY)
        a.urd(0, I.RX, I.RX_DATA)
        a.jmp("loop", I.FERR)                                    # bad stop bit: drop
        a.fput(0, I.FIFO_RX)
    a.jmp("loop")
    return a


# Parity modes of uart_ex
PAR_NONE, PAR_EVEN, PAR_ODD, PAR_MARK, PAR_SPACE = range(5)


def uart_ex(txd=0, rxd=1, B=112, N=9, K=1, S=12, bits=8, parity=PAR_NONE, stop_b2=2, inv=0, msb=0):
    """UART, any frame: 5-9 data bits (any value 1-15 fits), parity none / even /
    odd / mark / space, stop_b2 half bits of stop (2 = 1, 3 = 1.5, 4 = 2), inverted
    line, either bit order.
    The host writes data to TXF and reads it from RXF; with bits > 8 a word is
    two FIFO bytes, low byte first. Frames with a bad stop bit or parity are
    dropped; dmem[1] counts parity errors.
    Break: a frame of all zeros with a bad stop bit counts in dmem[2] and raises the
    controller IRQ. A doorbell sends a break of dmem[3] bit times (> 0), between frames.
    The start bit is timed by the instruction stream, because a word with parity or 9
    bits comes from a register, not the FIFO: with an odd B it is B + 1 cycles long."""
    # Registers: r0 the word being sent or received. r1 scratch: second FIFO byte, DMEM pointer, break length.
    # r2 parity test. r5 break count, kept in dmem[2]. r6 parity error count, kept in dmem[1].
    # Unlike uart(), a word with parity or 9 bits comes from a register, so the start bit is timed by instructions.
    idle, active = (0, 1) if inv else (1, 0)
    c = uart_consts(B, N, K, S)
    wbits = bits + (parity != PAR_NONE)
    stop = B * stop_b2 // 2
    pbit = 1 if msb else 1 << bits              # the parity bit in the word
    mask = (1 << bits) - 1
    a = Asm()
    # Setup. NBITS is data + parity, so parity needs no framing hardware. The stop time is the idle time ETX waits for.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(txd))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, inv, wbits))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_DRIVE0 if inv else I.A_DRIVE1)
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(rxd))
    a.ucfg(I.RX, I.RX_BIT, B)
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, inv=inv, nbits=wbits))
    a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(N, K))
    a.ucfg(I.RX, I.RX_STEP, c["S"])
    a.ucfg(I.RX, I.RX_FIRST, c["FIRST"])
    a.ucfg(I.ERX_U, I.D_COND, I.cond(rxd, I.RISE if inv else I.FALL, start=1, autoarm=1))
    a.ucfg(I.ERX_U, I.D_DUR, c["DUR"])
    a.ucfg(I.ERX_U, I.D_AFTER, I.erx_after(1, idle))
    a.ucfg(I.ERX_U, I.D_ADLY, c["ADLY"])
    a.evt(I.DET_RX, I.EV_ARM)
    a.set(txd, I.SRC_0 if inv else I.SRC_1)
    a.setoe(txd, 1)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, idle))
    a.ucfg(I.ETX_U, I.D_DUR, stop)
    a.evt(I.DET_TX, I.EV_ARM)
    a.mov(6, 0)                                  # parity error count
    a.mov(5, 0)                                  # break count
    # TX side: nothing goes out until ETX says the stop time has passed.
    a.L("loop")
    a.jmp("rx_chk", I.NETX)
    a.jmp("brk", I.DB)
    a.jmp("rx_chk", I.TXF_EMPTY)
    # Build the word: data, then the parity bit where it goes last on the line.
    a.fget(0, I.FIFO_TX)
    if bits > 8:
        a.fget(1, I.FIFO_TX).shl(1, 8).or_(0, I.R(1))
    if parity != PAR_NONE:
        if msb:
            a.shl(0, 1)                          # room for the parity bit, which goes last
        a.par(0)                                 # C = odd number of 1s in the data
        pm = {PAR_EVEN: I.NC, PAR_ODD: I.C}.get(parity)
        if parity == PAR_MARK:
            a.or_(0, pbit)
        elif parity != PAR_SPACE:
            a.jmp("tx_go", pm).or_(0, pbit)
            a.L("tx_go")
    # Start bit, by instruction timing: set takes 2 cycles, delay(B - 3) takes B - 2 for an even B.
    a.evt(I.DET_TX, I.EV_CLR)
    a.set(txd, I.SRC_1 if inv else I.SRC_0)      # start bit: the next ustart is B cycles later
    a.delay(B - 3)
    a.ustart(I.M_TX, I.R(0))
    a.evt(I.DET_TX, I.EV_ARM)                    # deferred to after the stop edge
    # RX side: a bad stop bit goes to ferr, a bad parity bit to p_bad, and only a good frame reaches RXF.
    a.L("rx_chk")
    a.jmp("loop", I.NRXRDY)
    a.urd(0, I.RX, I.RX_DATA)
    a.jmp("ferr", I.FERR)
    if parity != PAR_NONE:
        if parity in (PAR_MARK, PAR_SPACE):
            a.mov(2, I.R(0)).and_(2, pbit)
            a.jmp("p_bad", I.Z if parity == PAR_MARK else I.NZ)
        else:
            a.par(0)                             # data + parity bit: even -> C = 0
            a.jmp("p_ok", I.NC if parity == PAR_EVEN else I.C)
            a.jmp("p_bad")
            a.L("p_ok")
        if msb:
            a.shr(0, 1)
    # Good frame: the data bits to RXF. More than 8 bits take a second byte.
    a.mov(1, I.R(0)).and_(0, mask & 0xFF)
    a.fput(0, I.FIFO_RX)
    if bits > 8:
        a.shr(1, 8).and_(1, (mask >> 8) & 0xFF).fput(1, I.FIFO_RX)
    a.jmp("loop")
    if parity != PAR_NONE:
        a.L("p_bad").add(6, 1).mov(1, 1).st(6, 1).jmp("loop")
    # a bad stop bit with all data 0 is a break; ERX needs a new edge, so it counts once
    a.L("ferr").cmp(0, 0).jmp("loop", I.NZ)
    a.add(5, 1).mov(1, 2).st(5, 1).host_irq().jmp("loop")
    # send a break, then the stop time (ETX re-arms on the idle level)
    a.L("brk").dbclr().mov(1, 3).ld(1, 1).evt(I.DET_TX, I.EV_CLR)
    a.set(txd, I.SRC_1 if inv else I.SRC_0)
    # One bit time a pass: delay(B - 4) takes B - 2 for an even B, and djnz takes 2.
    a.L("brl").delay(B - 4).djnz(1, "brl")
    a.set(txd, I.SRC_0 if inv else I.SRC_1)
    a.evt(I.DET_TX, I.EV_ARM).jmp("rx_chk")
    return a


# ------------------------------------------------------------------ SPI
SPI_PINS = dict(sck=2, mosi=3, miso=4, cs=5)


def _cs_time(a, B):
    """CS lead and lag: at least half an SCK period, after CS falls and before it rises."""
    if B // 2 > 3:
        a.delay(B // 2 - 3)
    return a


def spi_master_words(B=2, cpol=0, cpha=0, msb=1, sdly=3, bits=16, sck=2, mosi=3, miso=4, cs=5):
    """SPI master for words of 1-16 bits, DMEM-driven: dmem[0] = n words,
    then two bytes per word (low first) from dmem[1]; n replies, two bytes
    each, at dmem[0x80]. Owners: SCK, MOSI -> TX unit."""
    # Registers: r0 word out. r1 pointer to the words. r2 words left. r3 pointer to the replies. r4 scratch.
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    # SCK idles at CPOL, and CS is high until a request comes.
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    # Request: r2 = n from dmem[0], r1 now points at dmem[1]. Then select the device.
    a.mov(1, 0).ld(2, 1, inc=1).mov(3, 0x80)
    a.set(cs, I.SRC_0, dly=1)
    _cs_time(a, B)
    a.L("word")
    # One word: two bytes from DMEM, low first, out and in together. A word over 8 bits comes from a
    # register and is read with urd, because the FIFOs hold bytes. SCK pauses between words, which SPI allows.
    a.ld(0, 1, inc=1).ld(4, 1, inc=1).shl(4, 8).or_(0, I.R(4))
    a.ustart(I.M_BOTH, I.R(0))
    a.uwait(I.M_BOTH)
    a.urd(4, I.RX, I.RX_DATA)
    a.st(4, 3, inc=1).shr(4, 8).st(4, 3, inc=1)
    a.djnz(2, "word")
    # All sent: deselect, and tell the host.
    _cs_time(a, B)
    a.set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master(B=2, cpol=0, cpha=0, msb=1, sdly=3, sck=2, mosi=3, miso=4, cs=5):
    """SPI master, DMEM-driven (H1): the host writes dmem[0] = n, dmem[1..n] =
    data and rings DOORBELL; the firmware clocks n bytes with CS low and
    stores the received bytes in dmem[0x80..], then raises the host IRQ.
    Owners: SCK, MOSI -> TX unit."""
    # Registers: r0 byte out. r1 pointer to the data. r2 bytes left. r3 pointer to the replies. r4 byte in.
    a = Asm()
    # The TX unit makes SCK and MOSI. The RX unit samples MISO on the TX unit's sample tick, and SDLY covers
    # the trip through the synchroniser and the outside world.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0))          # auto-push
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    # SCK idles at CPOL, and CS is high until a request comes.
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    # Wait for the host, then select the device.
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    a.mov(1, 0).ld(2, 1, inc=1)                                 # r2 = n, r1 -> data
    a.mov(3, 0x80)
    a.set(cs, I.SRC_0, dly=1)
    _cs_time(a, B)
    # Each byte: both units start in the same cycle, and the byte received goes to DMEM.
    # SCK pauses for a few instructions between bytes, which SPI allows.
    a.L("byte")
    a.ld(0, 1, inc=1)
    a.ustart(I.M_BOTH, I.R(0))                                  # one byte out and in
    a.uwait(I.M_BOTH)
    a.fget(4, I.FIFO_RX)
    a.st(4, 3, inc=1)
    a.djnz(2, "byte")
    # All sent: deselect, and tell the host.
    _cs_time(a, B)
    a.set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_slave(cpol=0, cpha=0, msb=1, bits=0, sck=2, mosi=3, miso=4, cs=5):
    """SPI slave, all four modes, words of 1-8 bits (bits = 0: 8). CS low starts both units in
    hardware (ETX / ERX, AUTOARM re-starts them for every byte). MISO bytes
    come from the TX FIFO, MOSI bytes go to the RX FIFO. CS high: the
    firmware stops both units and waits for the next select.
    Owners: MISO -> TX unit."""
    # Registers: none. The firmware only follows CS; the units and the detectors move the data.
    lead = cpol                                         # EEDGE of the leading edge
    samp_edge = lead if cpha == 0 else 1 - lead
    # The clock comes from the master, so the units use external edges instead of their own timers.
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    # MISO changes on the edge that does not sample (CPHA 1: bit 0 on the first leading edge)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    # CS low starts each unit, and AUTOARM restarts it for the next byte while CS stays low.
    for d in (I.ETX_U, I.ERX_U):
        a.ucfg(d, I.D_COND, I.cond(cs, I.LEVEL, 0, start=1, autoarm=1))
        a.ucfg(d, I.D_DUR, 1)
    # Not selected: MISO is left to the others. OE can only change while TX is idle, so ETX
    # is armed once MISO is on, a few cycles after CS falls (spi_slave_lead_cycles).
    a.L("idle").setoe(miso, 0).wait(cs, 1)
    a.evt(I.DET_RX, I.EV_ARM)
    a.wait(cs, 0).setoe(miso, 1).evt(I.DET_TX, I.EV_ARM)
    # Selected: the hardware works by itself. Only CS going high needs the firmware.
    a.L("run").jpin(cs, 1, "end")
    a.jmp("run")
    # Deselected: stop both units in mid-byte if need be, and go back to waiting.
    a.L("end").ustop(I.M_BOTH)
    a.evt(I.DET_TX, I.EV_DISARM).evt(I.DET_RX, I.EV_DISARM)
    a.evt(I.DET_TX, I.EV_CLR).evt(I.DET_RX, I.EV_CLR)
    a.jmp("idle")
    return a


def spi_master_stream(B=4, cpol=0, cpha=0, msb=1, sdly=3, sck=2, mosi=3, miso=4, cs=5):
    """SPI master with no gap between bytes (TX and RX CHAIN), DMEM-driven: dmem[0] = n,
    dmem[1..n] = data; the replies go to dmem[0x80..]. dmem[0x7F] = 0 ok, 1 the firmware fell
    behind and SCK paused, 2 a reply was lost (RXF overran, or the host took a byte from it).
    A pass that refills TXF and empties RXF takes 18 cycles, so any length runs with no pause
    at B >= 3; at B = 2 only short streams do. Whatever happens, the firmware ends and raises the
    IRQ: once TX is idle it does not wait for a reply that is not there.
    Owners: SCK, MOSI -> TX unit."""
    # Registers: r0 byte out. r1 pointer to the data. r2 bytes left to send. r3 pointer to the replies.
    # r4 byte in. r5 replies left to take. r6 the status, stored in dmem[0x7F].
    # CHAIN makes each unit go straight on to the next byte, so the only job left is to keep the FIFOs moving.
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, chain=1))
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, chain=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    # Request, then fill TXF as far as it goes before the first clock.
    a.mov(1, 0).ld(2, 1, inc=1).mov(5, I.R(2)).mov(3, 0x80).mov(6, 0)    # r2 tx left, r5 rx left
    a.L("pre").ld(0, 1, inc=1).fput(0, I.FIFO_TX).sub(2, 1).jmp("go", I.Z)
    a.jmp("go", I.TXF_FULL).jmp("pre")
    a.L("go").set(cs, I.SRC_0, dly=1)
    _cs_time(a, B).ustart(I.M_BOTH)
    a.cmp(2, 0).jmp("drain", I.Z)
    # While bytes are left to send, the last reply cannot have come: no test of r5 or r2 here
    # Main loop: empty RXF, refill TXF. This pass is what limits the speed: 18 cycles for one of each.
    a.L("run").fget(4, I.FIFO_RX, nb=1).jmp("no_rx", I.C)
    a.st(4, 3, inc=1).sub(5, 1)
    a.L("no_rx").jmp("late", I.TXU_IDLE).jmp("run", I.TXF_FULL)
    a.ld(0, 1, inc=1).fput(0, I.FIFO_TX).djnz(2, "run").jmp("drain")
    # The FIFO ran dry, so the chain ended and SCK paused. Report it and start TX again.
    a.L("late").mov(6, 1)                                       # TX ended early: start it again
    a.ld(0, 1, inc=1).fput(0, I.FIFO_TX).ustart(I.M_TX).djnz(2, "run")
    # All sent: take the replies that are left
    a.L("drain").fget(4, I.FIFO_RX, nb=1).jmp("dry", I.C)
    a.st(4, 3, inc=1).sub(5, 1).jmp("end", I.Z).jmp("drain")
    a.L("dry").jmp("drain", I.TXU_BUSY)
    # TX is idle: a last reply is at most SDLY away, and after that none will come
    a.delay(20).fget(4, I.FIFO_RX, nb=1).jmp("lost", I.C)
    a.st(4, 3, inc=1).sub(5, 1).jmp("end", I.Z).jmp("dry")
    a.L("lost").mov(6, 2)
    # Done: let the last bits go, stop RX (a chained RX never ends by itself), deselect and report.
    a.L("end").uwait(I.M_TX).ustop(I.M_RX)
    _cs_time(a, B)
    a.set(cs, I.SRC_1)
    a.mov(1, 0x7F).st(6, 1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master_3wire(B=8, cpol=0, cpha=0, msb=1, sdly=3, sck=2, dio=3, cs=5):
    """3-wire SPI master: one data pin, DMEM-driven: dmem[0] = n_write, dmem[1] = n_read,
    dmem[2..] = write bytes; read bytes at dmem[0x80..]. All writes, then all reads, in one
    CS frame; the pin is released in between. Owners: SCK, DIO -> TX unit."""
    # Registers: r0 byte out. r1 pointer to the write data. r2 writes left. r3 pointer to the replies.
    # r4 byte in. r5 reads left.
    a = Asm()
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(dio))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    # Request. The TX unit gets the data pin only for the writes.
    a.mov(1, 0).ld(2, 1, inc=1).ld(5, 1, inc=1).mov(3, 0x80)
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio, 1, sck, 1))
    a.setoe(dio, 1)
    a.set(cs, I.SRC_0, dly=1)
    _cs_time(a, B)
    a.cmp(2, 0).jmp("rd", I.Z)
    # Write phase: the TX unit drives DIO.
    a.L("wr").ld(0, 1, inc=1).ustart(I.M_TX, I.R(0)).uwait(I.M_TX).djnz(2, "wr")
    a.L("rd").setoe(dio, 0)                                     # turnaround
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, 0, sck, 1))            # the clock only
    a.cmp(5, 0).jmp("fin", I.Z).delay(B)
    # Read phase: the TX unit only makes the clock. Its data goes nowhere, and the RX unit reads DIO.
    a.L("rl").ustart(I.M_BOTH, 0).uwait(I.M_BOTH)
    a.fget(4, I.FIFO_RX).st(4, 3, inc=1).djnz(5, "rl")
    a.L("fin")
    _cs_time(a, B).set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_slave_3wire(cpol=0, cpha=0, msb=1, sck=2, dio=3, cs=5):
    """3-wire SPI slave. The first byte of a frame is a command, MSB = 1 for a read. A write:
    the next bytes go to RXF. A read: the next bytes come from TXF on DIO (0xFF if empty).
    The command goes to RXF as well. SCK half period at least 16 cycles, as firmware turns
    the pin around between the command and the first read bit. Owners: DIO -> TX unit."""
    # Registers: r0 the byte, in or out.
    # The command decides the direction of the rest of the frame, so firmware turns the pin around.
    lead = cpol
    samp_edge = lead if cpha == 0 else 1 - lead
    a = Asm()
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(dio))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio))
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    # Not selected: DIO released. The first byte after CS falls is the command.
    a.L("idle").setoe(dio, 0).wait(cs, 1).wait(cs, 0)
    a.ustart(I.M_RX)
    a.call("rx1").jmp("end", I.C)
    a.fput(0, I.FIFO_RX)
    a.and_(0, 0x80).jmp("read", I.NZ)
    # Write: every byte goes to RXF until CS rises.
    a.L("wl").ustart(I.M_RX).call("rx1").jmp("end", I.C)
    a.fput(0, I.FIFO_RX).jmp("wl")
    # the master drives until it has seen its last bit out: take the pin only after that
    a.L("read").wait(sck, cpol).delay(10).setoe(dio, 1)
    # Read: every byte comes from TXF. An empty FIFO sends 0xFF.
    a.L("rl").mov(0, 0xFF).fget(0, I.FIFO_TX, nb=1)
    a.ustart(I.M_TX, I.R(0))
    a.L("rw").jpin(cs, 1, "end").jmp("rw", I.TXU_BUSY).jmp("rl")
    a.L("end").ustop(I.M_BOTH).jmp("idle")
    # one byte into r0, C = 1 if CS went high first
    a.L("rx1").jpin(cs, 1, "rxe").jmp("rx1", I.NRXRDY)
    a.urd(0, I.RX, I.RX_DATA).cmp(0, 0).ret()
    a.L("rxe").mov(0, 0).cmp(0, 1).ret()
    return a


def spi_slave_words(bits=12, cpol=0, cpha=0, msb=1, sck=2, mosi=3, miso=4, cs=5):
    """SPI slave for words of 1-16 bits. Each word to send is two TXF bytes, low first (all ones
    if TXF is empty), and each word received is two RXF bytes, low first. Firmware starts each
    word, with the next one ready, within 11 cycles of the last edge: SCK half period at least
    12 cycles. The master waits 16 cycles from CS low to the first edge. The host keeps
    RXF from filling up: a full RXF stalls the firmware. As in every slave here, a word
    prepared for the clock that never comes (at CS high) is gone. Owners: MISO -> TX unit."""
    # Registers: r0 the next word to send. r1, r2 scratch. r4 the word received.
    lead = cpol
    samp_edge = lead if cpha == 0 else 1 - lead
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    a.L("idle").setoe(miso, 0).call("next").wait(cs, 1).wait(cs, 0).setoe(miso, 1)
    # The next word starts at once, and the word just received is stored after that: RX
    # keeps DATA and RXRDY across a start, and the next edge is only half a period away.
    a.L("word").ustart(I.M_BOTH, I.R(0))
    a.call("store").call("next")
    a.L("w").jpin(cs, 1, "end").jmp("w", I.TXU_BUSY).jmp("word")
    a.L("end").ustop(I.M_BOTH).call("store").jmp("idle")
    # Subroutine: two RXF bytes from the word received, if there is one.
    a.L("store").jmp("sx", I.NRXRDY)
    a.urd(4, I.RX, I.RX_DATA).fput(4, I.FIFO_RX).shr(4, 8).fput(4, I.FIFO_RX)
    a.L("sx").ret()
    # the next word to send, in r0
    a.L("next").mov(0, 0xFFFF).mov(1, 0xFF)
    a.fget(2, I.FIFO_TX, nb=1).jmp("nx", I.C)
    a.fget(1, I.FIFO_TX, nb=1).shl(1, 8).or_(2, I.R(1)).mov(0, I.R(2))
    a.L("nx").ret()
    return a


# ------------------------------------------------------------------ I2C
def i2c_master(B=126, scl=6, sda=7, odly=None, sdly=2, addr10=False, arb=False, hs_B=None, master_code=0x08,
               stuck=0xFFFF):
    """I2C master, DMEM-driven. Request: dmem[0] = 7-bit address, dmem[1] = n_write, dmem[2] =
    n_read, dmem[3..] = write data. The firmware sends START, address+W and the write bytes (if
    any), then, if n_read > 0, a repeated START, address+R and reads n_read bytes (ACK all but
    the last, NACK the last), then STOP. Result: dmem[0x80..] = read bytes, dmem[0x7F] = 0 ok /
    1 address NACK / 2 data NACK / 3 arbitration lost / 4 SDA stuck low; then host IRQ.
    Bus clear (UM10204 3.1.16): if SDA is low when a request starts, the master sends up to nine
    clock pulses. When SDA is released it sends a STOP and goes on with the request. If SDA is
    still low after nine, it reports 4; the bus then needs a device reset or a power cycle.
    SCL stuck low is not cleared (the standard gives the master no way to do so): the master
    waits, as for clock stretching, until the host restarts it.
    SCL: TX unit clock (STRETCH); SDA: RX unit (drive for writes, sample for reads). Both pins
    open drain (host PIN_OD). Owners: SCL -> TX, SDA -> RX. START/STOP/ACK clocks are driven by
    the controller while the units are idle. Those phases use H = B/2, the same as a data bit, so
    every SCL low and high phase is at least floor(B/2) cycles, and each setup and hold time around
    START and STOP is too. Q = B/4 is only the short hold after SCL falls. The SCL period is B + 2.
    B for the I2C limits (i2c.check_timing): 478 = 100 kHz, 126 = 375 kHz, 48 = 960 kHz, hs_B 16 = 2.67 MHz.
    Options, chosen when the firmware is built:
    addr10   dmem[0x7E] holds address bits 9:8, dmem[0] bits 7:0 (10-bit addressing).
    arb      multi-master: waits for a free bus and checks every bit it sends. A master that
             reads 0 after sending 1 has lost: it releases both lines at once, sends no
             STOP and reports 3. Written bits are clocked by firmware, not by the units, because
             a unit would keep driving the rest of its byte: B >= 48.
             SDA low may be another master's transfer, so bus clear starts only if SDA low and SCL
             high last, with no change, for `stuck` passes of a 3-instruction loop (about 8 ms).
    hs_B     HS mode: the master code at B (F/S speed), which nobody acknowledges, then a repeated
             START, and all data bytes at hs_B. ACK clocks stay at F/S length, which is legal."""
    # Registers
    #   r0 the byte being written or read      r1 pointer to the write data (with arb, also the read bit count)
    #   r2 write bytes left                    r3 pointer to the read data
    #   r4 the address                         r7 read bytes left
    #   r5 result of wbyte: 0 ACK, 1 NACK, 2 lost. Input of rbyte: 0 ACK, 1 NACK. Also a loop counter
    #   r6 the status for dmem[0x7F]. With arb, between requests: nonzero = bus busy; hi2 also uses it
    #
    # Two builds share this code. Without arb the units clock the bytes, which is fast and takes little code.
    # With arb, firmware clocks every bit, because it has to read SDA back after each one.
    assert not (arb and hs_B), "HS mode has one master after the master code"
    H = max(1, B // 2)
    Q = max(1, B // 4)
    if odly is None:
        odly = min(15, max(1, (hs_B or B) // 8))
    a = Asm()
    # Units, when they are used: TX makes SCL and waits for stretching, RX drives or samples SDA on its ticks.
    if not arb:                                                 # with arb, firmware clocks every bit
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, den=0, cpin=scl, cen=1))
        a.ucfg(I.TX, I.TX_BIT, B)
        a.ucfg(I.TX, I.TX_FRAME, 1)
        a.ucfg(I.TX, I.TX_CLK, I.tx_clk(0, 0, stretch=1))
        a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb=1, hold=1))
        a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
        a.ucfg(I.RX, I.RX_ODLY, odly)
    a.set(scl, I.SRC_1).set(sda, I.SRC_1)
    if arb:
        # The bus is busy from a START to its STOP, whatever the lines show in between (3.1.8).
        # ERX reports both; r6 != 0 means busy, between requests.
        a.ucfg(I.ERX_U, I.D_COND, I.cond(sda, I.ANY, qpin=scl, qen=1, qlvl=1))
        a.ucfg(I.ERX_U, I.D_DUR, 1)
        a.evt(I.DET_RX, I.EV_ARM).mov(6, 0)
        a.L("idle").jmp("bev", I.ERX).jmp("idle", I.NDB)
        a.jpin(sda, 0, "low").cmp(6, 0).jmp("idle", I.NZ)
        a.jpin(scl, 0, "idle").delay(H)                          # bus free time
        a.jmp("idle", I.ERX)
    else:
        # Single master: nothing to do but wait for the host.
        a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    # A request has come: read it from DMEM.
    a.mov(1, 0).ld(4, 1, inc=1).ld(2, 1, inc=1).ld(7, 1, inc=1)    # addr, n_w, n_r
    if addr10:
        a.mov(0, 0x7E).ld(0, 0).shl(0, 8).or_(4, I.R(0))           # r4 = the 10-bit address
    a.mov(3, 0x80)
    if not arb:                                                 # with arb, r6 is 0 to get here
        # SDA low before we even started: a device is stuck in the middle of a byte.
        a.mov(6, 0).jpin(sda, 0, "clr").L("go")
    if hs_B:
        a.ucfg(I.TX, I.TX_BIT, B)
        a.call("start").mov(0, master_code).call("wbyte")           # no acknowledge expected
        a.ucfg(I.TX, I.TX_BIT, hs_B)
    # START, address + W, then the write bytes.
    a.call("start1" if arb else "start")
    if addr10:
        a.mov(0, I.R(4)).shr(0, 7).and_(0, 0x06).or_(0, 0xF0)       # 11110 A9 A8 W
        a.call("wbyte").cmp(5, 0).jmp("err_a", I.NZ)
        a.mov(0, I.R(4)).and_(0, 0xFF)
    else:
        a.mov(0, I.R(4)).shl(0, 1)
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_a", I.NZ)
    a.cmp(2, 0).jmp("rd", I.Z)
    a.L("wl").ld(0, 1, inc=1)
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_d", I.NZ)
    a.djnz(2, "wl")
    # Repeated START, address + R, then the reads. The last read byte gets a NACK, so the target lets go.
    a.L("rd").cmp(7, 0).jmp("fin", I.Z)
    a.call("start")                                             # (repeated) START
    if addr10:
        a.mov(0, I.R(4)).shr(0, 7).and_(0, 0x06).or_(0, 0xF1)       # 11110 A9 A8 R
    else:
        a.mov(0, I.R(4)).shl(0, 1).or_(0, 1)
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_a", I.NZ)
    a.L("rl").mov(5, 0).cmp(7, 1).jmp("rk", I.NZ).mov(5, 1)    # NACK the last byte
    a.L("rk").call("rbyte")
    a.st(0, 3, inc=1)
    a.djnz(7, "rl")
    # End of a request, good or bad: STOP (unless arbitration was lost), store the status, tell the host.
    a.L("fin").call("stop")
    a.mov(6, 0)
    a.jmp("rep")
    a.L("err_a").mov(6, 1).jmp("err")
    a.L("err_d").mov(6, 2)
    a.L("err")
    if arb:
        a.cmp(5, 2).jmp("lost", I.Z)
    a.call("stop").jmp("rep")
    if arb:
        a.L("lost").mov(6, 3)                                   # both lines are released already
    a.L("rep")
    if hs_B:
        a.ucfg(I.TX, I.TX_BIT, B)
    a.mov(1, 0x7F).st(6, 1)
    a.host_irq()
    if arb:
        # After a lost arbitration the bus stays busy. Otherwise our own STOP is behind us, or SDA
        # is still low: bev finds out which.
        a.cmp(6, 3).jmp("lr", I.Z)
        a.L("bev").evt(I.DET_RX, I.EV_CLR).evt(I.DET_RX, I.EV_ARM)
        a.mov(6, 1).jpin(sda, 0, "idle").mov(6, 0).jmp("idle")
        a.L("lr").evt(I.DET_RX, I.EV_CLR).evt(I.DET_RX, I.EV_ARM)
    a.jmp("idle")
    if arb:
        # SDA low with a request waiting. It is stuck only if nothing moves for this long.
        a.L("low").mov(5, stuck)
        a.L("ll").jpin(sda, 1, "idle").jpin(scl, 0, "idle").djnz(5, "ll")
    # Bus clear (3.1.16). The STOP resets a device that was in the middle of a byte.
    a.L("clr").mov(5, 9)
    a.L("cl").set(scl, I.SRC_0).delay(H).set(scl, I.SRC_1).wait(scl, 1).delay(H)
    a.jpin(sda, 1, "cr").djnz(5, "cl")
    a.L("stk").dbclr().mov(6, 4).jmp("rep")
    a.L("cr").set(scl, I.SRC_0).delay(Q).call("stop")
    if arb:
        a.jmp("bev")                                            # back to idle: the request still waits
    else:
        a.jpin(sda, 1, "go").djnz(5, "cl").jmp("stk")           # the device drove the next bit low
    # START (also repeated START): SDA falls while SCL is high
    a.L("start").set(sda, I.SRC_1).delay(H).set(scl, I.SRC_1).wait(scl, 1)
    a.delay(H)
    # With arb, a first START comes here, right after the bus was seen free: in the time of the
    # phases above, another controller could have started and be in the middle of a bit.
    a.L("start1").set(sda, I.SRC_0)
    if arb:
        a.jmp("hi2")                                            # another controller may go first
    else:
        a.delay(H).set(scl, I.SRC_0).delay(Q).ret()
    # STOP: SDA rises while SCL is high
    a.L("stop").set(sda, I.SRC_0).delay(H).set(scl, I.SRC_1).wait(scl, 1)
    a.delay(H).set(sda, I.SRC_1).delay(H).ret()
    # write r0; r5 = 0 ACK / 1 NACK / 2 arbitration lost
    if arb:
        # Subroutine wbyte, multi-master: send r0 bit by bit, reading SDA back after each. The top of r0 shifts into
        # C, which set can drive. r5 counts the 8 bits here, and holds the result on the way out.
        a.L("wbyte").shl(0, 8).mov(5, 8)
        a.L("wb").shl(0, 1).set(sda, I.SRC_C).delay(H - Q)     # C = the bit to send; low = H with the Q below
        a.call("hi1")
        a.jmp("wbk", I.NC).jpin(sda, 0, "wlost")               # sent 1, read 0: lost
        a.L("wbk").call("hi2").djnz(5, "wb")
    else:
        # Subroutine wbyte, single master: RX drives SDA and TX clocks SCL for all 8 bits. Then the ACK clock by hand.
        a.L("wbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda, sda, 1))
        a.ustart(I.M_BOTH, I.R(0))
        a.uwait(I.M_BOTH)
        a.urd(5, I.RX, I.RX_DATA).delay(odly)                   # readback (clears RXRDY); data hold
    a.set(sda, I.SRC_1).delay(H)                                # release SDA for the ACK
    if arb:
        a.call("hi1")
    else:
        a.set(scl, I.SRC_1).wait(scl, 1).delay(H)
    # Read the ACK while SCL is high: low = ACK (r5 = 0), high = NACK (r5 = 1).
    a.mov(5, 0).jpin(sda, 0, "wa").mov(5, 1)
    a.L("wa")
    if arb:
        a.jmp("hi2")
    else:
        a.set(scl, I.SRC_0).delay(Q).ret()
    if arb:
        a.L("wlost").set(sda, I.SRC_1).set(scl, I.SRC_1).mov(5, 2).ret()
        # One SCL high phase in clock synchronization (3.1.7). hi1: it starts once SCL is really
        # high. SDA is read here: another controller may end the phase at any moment. hi2: it ends
        # after H, or as soon as another controller pulls SCL low, and then SCL goes low. Uses r6.
        a.L("hi1").set(scl, I.SRC_1).wait(scl, 1).ret()
        a.L("hi2").mov(6, max(1, H // 2)).wait(scl, 0, rd=6).set(scl, I.SRC_0).delay(Q).ret()
    # read into r0; r5 = 0 -> ACK, 1 -> NACK
    if arb:
        # Subroutine rbyte, multi-master: 8 SCL clocks by firmware, SDA read at the start of each high phase.
        a.L("rbyte").set(sda, I.SRC_1).mov(0, 0).mov(1, 8)      # r1 is free here: writes are over
        a.L("rbit").delay(H - Q).call("hi1").shl(0, 1).jpin(sda, 0, "rb0").or_(0, 1)
        a.L("rb0").call("hi2").djnz(1, "rbit")
    else:
        # Subroutine rbyte, single master: TX clocks SCL for 8 bits and RX samples SDA.
        a.L("rbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda))
        a.set(sda, I.SRC_1)
        a.ustart(I.M_BOTH)
        a.uwait(I.M_BOTH)
        a.urd(0, I.RX, I.RX_DATA).delay(odly)                   # data hold
    a.cmp(5, 1).set(sda, I.SRC_NC)                             # r5 = 0 borrows: C = 1, SDA low = ACK
    a.delay(H)
    if arb:
        a.call("hi1").call("hi2")
    else:
        a.set(scl, I.SRC_1).wait(scl, 1).delay(H).set(scl, I.SRC_0).delay(Q)
    a.set(sda, I.SRC_1).ret()
    return a


I2C_GC_MEM = 0xF0   # i2c_slave: general-call data goes here


def i2c_slave(addr=0x42, scl=6, sda=7, addr10=None, general_call=False, hold=0):
    """I2C slave register device at `addr` (memory = DMEM 0x00..0xFF).
    Write: first byte = register pointer, next bytes stored at pointer++.
    Read: sends dmem[pointer++] until the master NACKs. ERX (any SDA edge
    while SCL is high) reports START / STOP; the RX unit samples on SCL
    rising edges; read bytes are shifted out by the TX unit on SCL falling
    edges. ACKs are driven by the controller. Owners: SDA -> TX unit; SCL stays with the
    controller, which holds it low (clock stretching) after each ACK, until the
    next byte is set up, so the master cannot outrun the firmware.
    Options, chosen when the firmware is built:
    addr10        the device has this 10-bit address instead of `addr`. A read needs a write
                  address phase first and a repeated START, as the I2C standard says.
    general_call  address 0 with W is acknowledged; the data bytes go to dmem[I2C_GC_MEM..],
                  and the first one raises the controller IRQ.
    hold          extra cycles of stretching, for slow processing."""
    # Registers: r0 the byte. r1 scratch. r2 = 1 while the next byte is the register pointer.
    # r3 the register pointer. r6 = 1 once a 10-bit address has matched (only with addr10).
    a = Asm()
    # RX samples SDA on SCL rising, TX shifts SDA out on SCL falling, and ERX catches any SDA edge while SCL
    # is high: that can only be a START or a STOP, so the firmware hears of them even in the middle of a byte.
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(sda))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb=1, hold=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, scl, 0, 0))
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(sda))
    a.ucfg(I.TX, I.TX_FRAME, 1)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, scl, 1))                  # shift on SCL falling
    a.ucfg(I.ERX_U, I.D_COND, I.cond(sda, I.ANY, qpin=scl, qen=1, qlvl=1))
    a.ucfg(I.ERX_U, I.D_DUR, 1)
    a.set(sda, I.SRC_1).set(scl, I.SRC_1)
    if addr10 is not None:
        a.mov(6, 0)                                             # r6 = 1: 10-bit address matched
    # Wait for a START. The SDA level after the edge tells them apart: high means it was a STOP.
    a.L("idle").set(scl, I.SRC_1).evt(I.DET_RX, I.EV_ARM)
    a.evt(I.DET_RX, I.EV_WAITC)                                 # START or STOP
    a.jpin(sda, 1, "stop")
    # Address byte. If it is not ours, ignore everything up to the next START.
    a.L("addr").evt(I.DET_RX, I.EV_ARM)
    a.call("rx8").jmp("ev", I.C)
    if general_call:
        a.cmp(0, 0).jmp("gc", I.Z)
    if addr10 is None:
        a.mov(1, I.R(0)).shr(1, 1).cmp(1, addr).jmp("idle", I.NZ)  # not us: wait for START
        a.call("ack").jmp("ev", I.C)
        a.and_(0, 1).jmp("rd", I.NZ)
    else:
        first = 0xF0 | ((addr10 >> 7) & 0x06)
        a.mov(1, I.R(0)).and_(1, 0xFE).cmp(1, first).jmp("idle", I.NZ)
        a.and_(0, 1).jmp("r10", I.NZ)
        a.call("ack").jmp("ev", I.C)
        a.call("rx8").jmp("ev", I.C)
        a.cmp(0, addr10 & 0xFF).jmp("nom", I.NZ)
        a.call("ack").jmp("ev", I.C).mov(6, 1).jmp("wstart")
        a.L("nom").mov(6, 0).jmp("idle")
        a.L("r10").cmp(6, 1).jmp("idle", I.NZ)                  # a read must follow our write
        a.call("ack").jmp("ev", I.C).jmp("rd")
        a.L("wstart")
    # Write: the first byte sets the pointer, the others are stored at pointer++.
    a.mov(2, 1)                                                 # r2 = 1: next byte is the pointer
    a.L("wr").call("rx8").jmp("ev", I.C)
    a.call("ack").jmp("ev", I.C)
    a.cmp(2, 1).jmp("wd", I.NZ)
    a.mov(3, I.R(0)).mov(2, 0).jmp("wr")
    a.L("wd").st(0, 3, inc=1).jmp("wr")
    if general_call:
        a.L("gc").call("ack").jmp("ev", I.C).mov(3, I2C_GC_MEM).mov(2, 0).host_irq().jmp("wr")
    # Read: hold SCL low while the byte is set up, then release it and let the master clock it out.
    a.L("rd").ld(0, 3, inc=1)
    a.ustart(I.M_TX, I.R(0))                                    # bit 7 now, then on SCL falls
    a.delay(60)                                                 # tr(max) + tSU;DAT = 1250 ns (Table 11 note 4)
    a.set(scl, I.SRC_1)                                         # the data is ready: release
    a.uwait(I.M_TX)
    a.set(sda, I.SRC_1)                                         # release for the master ACK
    a.wait(scl, 1).jpin(sda, 1, "nak")
    a.wait(scl, 0).set(scl, I.SRC_0).jmp("rd")                  # hold SCL while fetching
    a.L("nak").evt(I.DET_RX, I.EV_CLR).jmp("idle")
    # A START or STOP came in the middle of a byte: drop the byte and act on the event.
    a.L("ev").evt(I.DET_RX, I.EV_CLR).ustop(I.M_RX)            # START/STOP inside a byte
    a.jpin(sda, 1, "stop").jmp("addr")
    a.L("stop")
    if addr10 is not None:
        a.mov(6, 0)
    a.jmp("idle")
    # receive one byte into r0; C = 1 if a START/STOP came instead
    a.L("rx8").ustart(I.M_RX).set(scl, I.SRC_1)
    a.L("rx_w").jmp("rx_ev", I.ERX).jmp("rx_w", I.NRXRDY)
    a.urd(0, I.RX, I.RX_DATA).cmp(0, 0).ret()                   # cmp x,0 -> C = 0
    a.L("rx_ev").urd(0, I.RX, I.RX_DATA).mov(0, 0).cmp(0, 1).ret()   # 0 - 1 borrows -> C = 1
    # ACK: SDA low for the 9th clock, then hold SCL low until the next byte is set up
    # C = 1: a START or STOP came right after bit 8, so there is no ACK (r1 is free here)
    a.L("ack").jmp("ack_ev", I.ERX).jpin(scl, 1, "ack").set(sda, I.SRC_0)
    a.wait(scl, 1).wait(scl, 0).set(scl, I.SRC_0).set(sda, I.SRC_1)
    if hold:
        a.delay(hold)
    a.cmp(0, 0).ret()
    a.L("ack_ev").mov(1, 0).cmp(1, 1).ret()
    return a


# ------------------------------------------------------------------ USB
USB_SYNC = 0x80
PID_OUT, PID_IN, PID_SOF, PID_SETUP = 0xE1, 0x69, 0xA5, 0x2D
PID_DATA0, PID_DATA1 = 0xC3, 0x4B
PID_ACK, PID_NAK, PID_STALL = 0xD2, 0x5A, 0x1E
CRC5_POLY, CRC5_INIT, CRC5_RES = 0x2800, 0xF800, 0x6000
CRC16_POLY, CRC16_INIT, CRC16_RES = 0x8005, 0xFFFF, 0x800D
# usb_device DMEM map
UD_ADDR, UD_TOKEN, UD_EP, UD_RXLEN, UD_INLEN, UD_INPID, UD_OUTHS, UD_INHS, UD_EVENT = range(9)
UD_INDATA, UD_RXDATA = 0x10, 0x80
USB_IDLE = 65535        # usb_device: cycles of J that set ETX (1.37 ms at 48 MHz)
EV_OUT, EV_IN_ACK, EV_RESET, EV_IN_NOACK = 1, 2, 3, 4
# usb_host DMEM map
UH_OP, UH_TOK0, UH_TOK1, UH_TOK2, UH_DPID, UH_N, UH_NH, UH_STATUS, UH_RXLEN = range(9)
UH_RPID = 0x80          # the first byte received
UH_OK, UH_TIMEOUT, UH_BADPKT = 0, 1, 2


def _usb_setup(a, B, jp, kp):
    """Line config shared by both roles. jp: the pin that is high in J (full
    speed D+, low speed D-), kp: the other one. Owners: both -> TX unit."""
    # TX: NRZI, a stuff bit after six 1s, both pins in opposite levels, and an EOP when the chain ends.
    # The pins are released after the EOP, so the bus goes back to the pull-up.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(jp))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(chain=1))
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=kp, stuffn=6))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
    # RX: the same coding undone. It re-times itself on every edge (SRC_DPLL) and drops bits until the sync ends.
    # CHAIN keeps it receiving byte after byte until a detector stops it.
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(jp, i2pin=kp, diff=1))
    a.ucfg(I.RX, I.RX_BIT, B)
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(chain=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_DPLL))
    a.ucfg(I.RX, I.RX_ODLY, I.rx_line(nrzi=1, hunt=1, stuffn=6))
    # ERX: both lines low (SE0) means end of packet, and it stops the RX unit in the same cycle.
    a.ucfg(I.ERX_U, I.D_COND, I.cond(jp, I.LEVEL, 0, qpin=kp, qen=1, qlvl=0, stop=1))   # SE0
    # SE0 of an EOP: over 210 ns at low speed, where D+ and D- may both be low in a transition
    # (7.1.4), and at most 82 ns at full speed and 670 ns at low speed (7.1.13.2).
    a.ucfg(I.ERX_U, I.D_DUR, max(2, B // 2))
    a.set(jp, I.SRC_1).set(kp, I.SRC_0)                          # J, not driven yet


def _crc_rx(a, poly, init):
    # Check received bits. SKIP = 8: the PID is not part of the CRC.
    a.ucfg(I.CRC, I.CRC_CTRL, I.crc_ctrl(I.CRC_RX, skip=8))
    a.ucfg(I.CRC, I.CRC_POLY, poly)
    a.ucfg(I.CRC, I.CRC_INIT, init)


def _crc_tx(a, skip=16):
    # Generate over sent bits, and have the TX unit add ~CRC when the FIFO runs out.
    a.ucfg(I.CRC, I.CRC_CTRL, I.crc_ctrl(I.CRC_TX, append=1, skip=skip))   # not sync, not PID
    a.ucfg(I.CRC, I.CRC_POLY, CRC16_POLY)
    a.ucfg(I.CRC, I.CRC_INIT, CRC16_INIT)


def _rx_go(a, jp):
    """Hunt for the next packet: the bus has left SE0, ERX armed, RX started."""
    a.wait(jp, 1).evt(I.DET_RX, I.EV_ARM).ustart(I.M_RX)


def _tx_on(a, jp, kp):
    # Drive the bus and send the sync. CHAIN then takes the next bytes from TXF.
    a.setoe(jp, 1).setoe(kp, 1).ustart(I.M_TX, USB_SYNC)


def _next_crc(r, poly, bit=1):
    # One step of the CRC register, as the CRC unit does it. Used to work out the dribble values below.
    return ((r << 1) ^ (poly if ((r >> 15) ^ bit) & 1 else 0)) & 0xFFFF


# USB 2.0 7.1.9.1: a dribble bit, one more 1 before the EOP, must be accepted. It goes through the
# CRC unit as well, and then gives this value instead of the residual.
CRC5_DRIB, CRC16_DRIB = _next_crc(CRC5_RES, CRC5_POLY), _next_crc(CRC16_RES, CRC16_POLY)


def _crc_good(a, res, drib, ok, bad):
    """Compare r4 with the residual: to `ok` if good (the usual case is the short one), else `bad`."""
    a.cmp(4, res).jmp(ok, I.Z).cmp(4, drib).jmp(bad, I.NZ)


def usb_device(B=4, jp=0, kp=1, turn=None):
    """USB device at transaction level, low or full speed. The host side sets
    the address and handshakes in DMEM and owns the higher layers (enumeration,
    data toggles, endpoints). See UD_* for the map; UD_EVENT says what happened,
    with a controller IRQ. `turn`: extra cycles before answering (B = 32: the
    answer must not come before 2 bit times). A SETUP is always answered with ACK (8.5.3).
    Suspend: ETX times the idle bus. CSTAT.ETX set means J for at least USB_IDLE cycles
    (1.37 ms) since the last packet, and any packet clears it. Read it twice, 1.7 ms
    apart: still set means suspend (3 ms of idle). Resume shows as K on PIN_IN; the firmware
    keeps hunting all along, so the next packet is received normally. Remote wakeup is
    not implemented: the device must not advertise it."""
    # Registers
    #   r0 byte, event code          r1 second token byte, then the handshake to send
    #   r2 third token byte, then EP r3 DMEM pointer
    #   r4 CRC value, RX status      r5 UD_INLEN, then a time-out count
    #   r6 the PID, 0 if none came   r7 our address
    if turn is None:
        turn = 0 if B <= 8 else 3 * B
    wait_h = max(1, (33 * B // 2 - 14) // 4)                # handshake: give up before 18 bit times
    a = Asm()
    # Setup, and ETX as the suspend timer: it fires after USB_IDLE cycles of J. It only sets a flag, for the host.
    _usb_setup(a, B, jp, kp)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(jp, I.LEVEL, 1))
    a.ucfg(I.ETX_U, I.D_DUR, USB_IDLE)
    # Everything ends here, with whatever it had going stopped. The hunt comes first: the next
    # packet may start 2 bit times after the last one, and no bit reaches the FIFO or the CRC unit
    # until the hunt ends, so the rest has time.
    a.L("idle").ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).evt(I.DET_RX, I.EV_CLR)
    _rx_go(a, jp)
    a.L("dr").fget(0, I.FIFO_RX, nb=1).jmp("dr", I.NC)           # drain
    _crc_rx(a, CRC5_POLY, CRC5_INIT)
    a.evt(I.DET_TX, I.EV_CLR).evt(I.DET_TX, I.EV_ARM)            # the idle timer starts again
    a.mov(3, UD_ADDR).ld(7, 3)
    a.mov(3, UD_INLEN).ld(5, 3)
    a.mov(6, 0)
    # Receive a token, wait for its EOP, and check it: PID in r6, then the second byte, then CRC5.
    a.evt(I.DET_RX, I.EV_WAITC)                                  # SE0: RX is stopped
    a.fget(6, I.FIFO_RX, nb=1).fget(1, I.FIFO_RX, nb=1)
    a.urd(4, I.CRC, I.CRC_VAL)
    _crc_good(a, CRC5_RES, CRC5_DRIB, "t_ok", "t_bad")
    # Good token: address in the low 7 bits of the second byte. IN goes straight to the answer: it has the least time.
    a.L("t_ok").mov(4, I.R(1)).and_(4, 0x7F).cmp(4, I.R(7)).jmp("idle", I.NZ)
    a.cmp(6, PID_IN).jmp("in_tok", I.Z)                          # no time to spare here
    # OUT / SETUP: the data packet follows soon. No bit reaches the CRC unit until the hunt ends.
    # OUT or SETUP: start hunting at once for the data packet, then read the token's last byte.
    _rx_go(a, jp)
    a.ucfg(I.CRC, I.CRC_POLY, CRC16_POLY).ucfg(I.CRC, I.CRC_INIT, CRC16_INIT)
    a.fget(2, I.FIFO_RX, nb=1)
    a.cmp(6, PID_OUT).jmp("o_ok", I.Z)
    a.cmp(6, PID_SETUP).jmp("idle", I.NZ)
    # Report PID and endpoint. EP = 3 bits of the third byte, then the top bit of the second.
    a.L("o_ok").mov(3, UD_TOKEN).st(6, 3, inc=1)                 # UD_EP comes next
    a.and_(2, 7).shl(2, 1).shr(1, 7).or_(2, I.R(1)).st(2, 3)
    a.mov(3, UD_OUTHS).ld(1, 3)                                  # handshake ready in r1
    a.cmp(6, PID_SETUP).jmp("o_hs", I.NZ).mov(1, PID_ACK)
    a.L("o_hs").mov(3, UD_RXDATA)
    # The first byte must be a good DATA PID. The slow checks are done here, as bytes arrive:
    # after the EOP only the CRC and the stuff flag stand between us and the handshake.
    a.L("op").jmp("idle", I.ERX).fget(0, I.FIFO_RX, nb=1).jmp("op", I.C)
    a.st(0, 3, inc=1).cmp(0, PID_DATA0).jmp("rl", I.Z).cmp(0, PID_DATA1).jmp("idle", I.NZ).jmp("rl")
    # The EOP is looked for every 2 instructions: the time from here to the handshake is the tightest
    # in the firmware.
    a.L("rl").jmp("rle0", I.ERX).jmp("rl", I.RXF_EMPTY).fget(0, I.FIFO_RX).st(0, 3, inc=1).jmp("rl")
    # EOP seen. Answer only if CRC16 and stuffing are good; otherwise stay silent, so the host retries.
    a.L("rle0").urd(4, I.CRC, I.CRC_VAL)
    _crc_good(a, CRC16_RES, CRC16_DRIB, "rc2", "idle")
    a.L("rc2").urd(4, I.RX, I.RX_STAT).and_(4, I.RX_ST_SERR).jmp("idle", I.NZ)
    if turn:
        a.delay(turn)
    _tx_on(a, jp, kp)
    a.call("hsend")
    # At full speed the last byte can reach the FIFO just before the EOP, and the loop above leaves
    # as soon as it sees the EOP. The CRC unit has it already, so answer first and store it now.
    # At low speed the EOP takes far longer to detect than the loop takes to go round.
    if B <= 8:
        a.fget(0, I.FIFO_RX, nb=1).jmp("rle", I.C).st(0, 3, inc=1).L("rle")
    a.mov(0, I.R(3)).sub(0, UD_RXDATA).mov(3, UD_RXLEN).st(0, 3)
    a.mov(0, EV_OUT).jmp("ev")
    # Subroutine: send the handshake in r1. The CRC unit must not add anything to it.
    a.L("hsend").fput(1, I.FIFO_TX).ucfg(I.CRC, I.CRC_CTRL, 0).uwait(I.M_TX).ret()
    # IN: answer within 6.5 bit times. Sync goes out first; it is the same for both
    # answers, and it gives the time to decide and to set up the CRC.
    a.L("in_tok")
    if turn:
        a.delay(turn)
    _tx_on(a, jp, kp)
    # r5 = UD_INLEN, the PID and the data. 0 means nothing is ready: NAK.
    a.cmp(5, 0).jmp("nak", I.Z)
    # INIT is written 8 cycles after the sync started: the bits already gone do not count.
    _crc_tx(a, skip=16 - (8 // B + 1))
    # Data: PID, then the bytes. The chain must not run dry: the loop is far faster than the bytes go out.
    a.mov(3, UD_INPID).ld(0, 3).fput(0, I.FIFO_TX)
    a.mov(3, UD_INDATA).sub(5, 1).jmp("txe", I.Z)
    a.L("txl").ld(0, 3, inc=1).fput(0, I.FIFO_TX).djnz(5, "txl")
    a.L("txe").uwait(I.M_TX)
    # the host's handshake: its packet must start before 18 bit times
    a.ucfg(I.CRC, I.CRC_CTRL, 0)
    a.fget(1, I.FIFO_RX, nb=1)                                   # the last byte of the token
    _rx_go(a, jp)
    a.mov(5, wait_h)
    # Wait for K, the start of the handshake. No answer in time means the host did not get the data: it stays in DMEM.
    a.L("ha").jpin(kp, 1, "hb").djnz(5, "ha")
    a.L("nk").mov(0, EV_IN_NOACK).jmp("ev")
    # A handshake came. ACK clears UD_INLEN, and the host side can fill the next packet.
    a.L("hb").evt(I.DET_RX, I.EV_WAITC).fget(1, I.FIFO_RX, nb=1)
    a.cmp(1, PID_ACK).jmp("nk", I.NZ)
    a.mov(3, UD_INLEN).mov(0, 0).st(0, 3).mov(0, EV_IN_ACK).jmp("ev")
    # NAK or STALL, as the host side chose in UD_INHS.
    a.L("nak").mov(3, UD_INHS).ld(1, 3).call("hsend").jmp("idle")
    # A packet shorter than a token, or a bus reset: SE0 still there after 2 us (the limit is 2.5).
    a.L("t_bad").cmp(6, 0).jmp("idle", I.NZ)
    a.delay(70).jpin(jp, 1, "idle").mov(0, EV_RESET)
    # Every event ends here: tell the host what happened.
    a.L("ev").mov(3, UD_EVENT).st(0, 3).host_irq().jmp("idle")
    return a


def usb_host(B=4, jp=0, kp=1, gap=None, timeout=None):
    """USB host at transaction level, low or full speed. dmem: UH_OP (0
    transaction, 1 bus reset, 2 EOP = keep-alive, 3 resume; suspend = no transactions), UH_TOK0-2 = the
    token with its CRC5, UH_DPID = data PID for OUT / SETUP (0 for IN), UH_N = data
    bytes at 0x10. Reset and resume last n * 256 cycles, n = UH_N + 256 * dmem[UH_N + 1]: 1875 for the
    10 ms of a reset, 3750 for the 20 ms of a resume.
    Result: UH_RPID = handshake or data PID, UH_STATUS, UH_RXLEN = bytes
    received with the PID, at 0x80; controller IRQ. An IN gets an ACK when its
    PID, CRC and stuffing are good, within 7.5 bit times. Only a one-byte packet (a handshake) is
    reported with a PID that was not checked: the host software looks at UH_RPID."""
    # Registers
    #   r0 byte                       r1 the status for UH_STATUS
    #   r3 DMEM pointer               r4 CRC value, RX status
    #   r5 count, time-out            r6 UH_DPID (0 for an IN), also the high byte of a count
    if gap is None:
        gap = max(0, 4 * B - 24)        # >= 2 bit times after SE0 ends, measured from SE0
    if timeout is None:
        timeout = max(1, (35 * B // 2 - 30 - B) // 4)    # the answer must start before 18 bit times
    a = Asm()
    _usb_setup(a, B, jp, kp)
    # Wait for the doorbell, then branch on UH_OP. Each operation ends at fin0.
    a.L("idle").jmp("idle", I.NDB).dbclr()
    a.mov(3, UH_OP).ld(0, 3).cmp(0, 0).jmp("tok", I.Z)
    a.cmp(0, 3).jmp("resu", I.Z)
    a.cmp(0, 1).jmp("keep", I.NZ)
    # Bus reset: SE0 for n * 256 cycles, and then the end of an EOP.
    a.call("gn")
    a.set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)
    a.L("rl").delay(252).djnz(5, "rl")                           # 256 cycles a turn
    a.jmp("rel")
    # Keep-alive, or the end of a resume: SE0 for 2 bit times, then J for 1 bit, then release.
    a.L("keep").set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)
    a.L("eop").delay(2 * B - 6)
    a.L("rel").set(jp, I.SRC_1).delay(max(0, B - 3)).setoe(jp, 0).setoe(kp, 0).jmp("fin0")
    # Subroutine: the 16-bit count from UH_N and UH_N + 1, into r5.
    a.L("gn").mov(3, UH_N).ld(5, 3, inc=1).ld(6, 3).shl(6, 8).or_(5, I.R(6)).ret()
    # resume: K, then the EOP that "keep" ends with. J to K changes two pins: only a TX unit
    # write with DIFF changes both in one edge, with no glitch of SE0 between them.
    a.L("resu").call("gn")
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(diff=1, d2pin=kp))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(nbits=1))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_NONE)
    a.setoe(jp, 1).setoe(kp, 1)                                  # J, as the pull-ups had it
    a.ustart(I.M_TX, 0).uwait(I.M_TX)
    a.L("rk").delay(252).djnz(5, "rk")
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=kp, stuffn=6))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(chain=1))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
    a.set(kp, I.SRC_0).jmp("eop")
    # token
    a.L("tok")                      # the CRC unit is off or on RX here: neither adds to a TX packet
    _tx_on(a, jp, kp)
    # The 3 bytes of the token. The host side worked out its CRC5.
    a.mov(3, UH_TOK0).ld(0, 3, inc=1).fput(0, I.FIFO_TX).ld(0, 3, inc=1).fput(0, I.FIFO_TX)
    a.ld(0, 3).fput(0, I.FIFO_TX).uwait(I.M_TX)
    # No data PID means an IN, and the next thing is the device's answer.
    a.mov(3, UH_DPID).ld(6, 3).cmp(6, 0).jmp("rxp", I.Z)
    # data stage (OUT / SETUP)
    _crc_tx(a)
    a.delay(2 * B)
    _tx_on(a, jp, kp)
    a.fput(6, I.FIFO_TX)
    a.mov(3, UH_N).ld(5, 3).mov(3, 0x10).cmp(5, 0).jmp("dte", I.Z)
    a.L("dtl").ld(0, 3, inc=1).fput(0, I.FIFO_TX).djnz(5, "dtl")
    a.L("dte").uwait(I.M_TX)
    # receive a packet (handshake or data) into 0x80. Its start must come in time.
    a.L("rxp")
    _crc_rx(a, CRC16_POLY, CRC16_INIT)
    _rx_go(a, jp)
    # r3 stores from 0x80. The status starts as OK.
    a.mov(3, 0x80).mov(5, timeout).mov(1, UH_OK)
    # K means a packet has started. Nothing in time means a time-out.
    a.L("rw").jpin(kp, 1, "sop").djnz(5, "rw")
    a.ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).mov(1, UH_TIMEOUT).jmp("done")
    # The first byte decides. Only a good DATA PID in answer to an IN goes on to the ACK path, where
    # time is short: the other checks are made here, as bytes arrive.
    # A byte can reach the FIFO between finding it empty and seeing the EOP, so nothing is decided
    # from the FIFO being empty. A one-byte packet is read once more before it is judged (sope). On
    # the way to the ACK there is no time for that, but the CRC unit has seen the byte already: the
    # end ("hs") stores what is left, or it would pass as the next packet's PID.
    a.L("sop").fget(0, I.FIFO_RX, nb=1).jmp("s1", I.NC).jmp("sope", I.ERX).jmp("sop")
    a.L("s1").st(0, 3, inc=1).cmp(6, 0).jmp("rx", I.NZ)
    a.cmp(0, PID_DATA0).jmp("rd", I.Z).cmp(0, PID_DATA1).jmp("rd", I.Z)
    # anything else gets no ACK: with the CRC unit off the check below cannot pass, and that costs
    # the ACK no time. One byte is a handshake; more is a bad packet.
    a.L("rx").ucfg(I.CRC, I.CRC_CTRL, 0).jmp("rd")
    a.L("rs").st(0, 3, inc=1)
    a.L("rd").fget(0, I.FIFO_RX, nb=1).jmp("rs", I.NC).jmp("rd", I.NERX)
    a.urd(4, I.CRC, I.CRC_VAL)
    _crc_good(a, CRC16_RES, CRC16_DRIB, "rc2", "sope")
    # PID and CRC good. ACK it if the stuffing is good too.
    a.L("rc2").urd(4, I.RX, I.RX_STAT).and_(4, I.RX_ST_SERR).jmp("bad", I.NZ)
    if gap:
        a.delay(gap)
    _tx_on(a, jp, kp)
    a.mov(0, PID_ACK).fput(0, I.FIFO_TX).uwait(I.M_TX)
    a.jmp("hs")
    a.L("sope").fget(0, I.FIFO_RX, nb=1).jmp("s1", I.NC)
    # A packet with no ACK: one byte is a handshake, reported as OK. Anything else is a bad packet.
    a.L("rn").cmp(3, 0x81).jmp("hs", I.Z)
    a.L("bad").mov(1, UH_BADPKT)
    a.L("hs")
    if B <= 8:                      # at low speed the EOP takes far longer than a turn of the loop
        a.fget(0, I.FIFO_RX, nb=1).jmp("hse", I.C).st(0, 3, inc=1).jmp("hs").L("hse")
    # The length is how far r3 moved from 0x80.
    a.evt(I.DET_RX, I.EV_CLR).sub(3, 0x80).mov(0, I.R(3)).mov(3, UH_RXLEN).st(0, 3)
    a.L("done").mov(3, UH_STATUS).st(1, 3)
    a.L("fin0").host_irq().jmp("idle")
    return a
