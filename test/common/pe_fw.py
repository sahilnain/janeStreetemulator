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
clock cycle (21 ns at 48 MHz), too slow to time the bits of a fast protocol, so the units time them. The firmware decides what happens between packets: START, STOP, ACK, error checks.

How the host and a program talk (v3)
------------------------------------
Two places, with different jobs:
* The mailbox: eight 16-bit registers M0-M7 (host 0x30-0x3F, firmware mbr / mbw). They hold the
  request (how many bytes, which address, which operation) and the result (status, length).
  Names: M_OP, M_ADDR, M_NTX, M_NRX, M_PID, M_HS, M_STATUS, M_INFO (pe_isa.py).
* DMEM (the SRAM, 256 bytes): payload only. By convention bytes to send start at dmem[0x00]
  and received bytes at dmem[0x80].
A request-driven program waits for DOORBELL, reads the mailbox, moves the payload, writes the
result to the mailbox and raises the host IRQ. Each docstring lists the registers it uses.

  Program              Request (mailbox)               Payload (DMEM)                Result (mailbox)
  uart, spi_slave*     -                               - (FIFOs only)                -
  uart_ex              M_NTX break length              - (FIFOs only)                M_STATUS breaks, M_INFO parity errors
  spi_master(_stream)  M_NTX n                         0x00.. out, 0x80.. in         M_STATUS (stream only)
  spi_master_3wire     M_NTX writes, M_NRX reads       0x00.. out, 0x80.. in         -
  spi_master_words     M_NTX words                     0x00.. out, 0x80.. in (2 B)   -
  i2c_master           M_ADDR, M_NTX, M_NRX            0x00.. out, 0x80.. in         M_STATUS
  i2c_slave            -                               the whole DMEM is its registers
  usb_device           UD_* names (see the USB section) 0x00.. IN data, 0x80.. OUT   UD_EVENT and others
  usb_host             UH_* names (see the USB section) token 0x00, data 0x10, 0x80 in UH_STATUS, UH_RXLEN

Moving payload: the data movers
-------------------------------
Two small hardware units copy payload between DMEM and the FIFOs on their own, so the program
does not touch the bytes:
* TXM (unit 5): ucfg(TXM, MV_PTR, p); ucfg(TXM, MV_GO, n) copies dmem[p..p+n-1] into TXF, as
  fast as TXF has room (about 3 cycles a byte). The TX unit then takes them from TXF as usual.
* RXM (unit 6): the same set-up copies the next n bytes that arrive in RXF to dmem[p..].
  ucfg(RXM, MV_GO, 0) stops it; urd(r, RXM, MV_RD_CNT) gives how many it stored.
* Done-tests: jump conditions TXM_BUSY / TXM_IDLE / RXM_BUSY / RXM_IDLE. A program that needs
  the last received byte in DMEM waits for RXM_IDLE (or for RXF to be empty when RXM was given
  a large count, as USB does, where the packet length is not known in advance).
Programs that must look at every byte in a register (16-bit SPI words, I2C, the PID of a USB
packet) use ld / st instead. ld takes 2 cycles (the SRAM has a registered read); st takes 1.

Conventions
-----------
* Timing: an instruction takes 1 cycle, and delay(n) takes n + 1. ld takes 2.
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
    """Timing constants for bit period B and an RX vote of N samples, S apart.
    B      cycles per bit (48 MHz / baud: 115200 baud -> B = 417)
    N, K   the RX unit takes N samples per bit and votes on the middle N - 2K of them
    S      cycles between samples (STEP); N samples must fit inside one bit
    DUR    ERX: the line must stay low for half a bit before a falling edge counts as a start bit
    FIRST  RX: cycles from its start (when ERX fires, DUR into the start bit) to the window of data bit 0
    ADLY   ERX: cycles after the last data bit's vote to check the stop bit, in the middle of the stop bit
    TXDUR  ETX: the start bit length, less the detector-to-unit latency K_LAT"""
    if S is None:
        S = max(1, B // (N + 1))
    assert N * S + 1 <= B, "vote window must fit in a bit"
    dur = B // 2
    return dict(B=B, N=N, K=K, S=S, DUR=dur, FIRST=B - dur + 1,
                ADLY=B + B // 2 - N * S - 1, TXDUR=B - K_LAT)


def uart(txd=0, rxd=1, B=112, N=9, K=1, S=12, B_rx=None, tx=True, rx=True, stop_lvl=1):
    """Full-duplex UART, 8N1: TX on `txd`, RX on `rxd`. The host writes bytes to TXF and reads
    received ones from RXF. B_rx: a different bit time for RX. tx / rx: build one direction only.
    stop_lvl: the stop and idle level (1 for a normal line). Returns an Asm."""
    # Registers: r0 the received byte. r7 the TX state: 0 waiting for the line to be idle, 1 start bit running.
    # The loop never blocks, so TX and RX are both served by it. Each edge on the line comes from hardware.
    c = uart_consts(B if B_rx is None else B_rx, N, K, S)
    a = Asm()
    # Setup. The TX unit sends the data bits; ETX adds the stop edge the moment the last bit ends.
    if tx:
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(txd))                  # data on txd, no clock pin
        a.ucfg(I.TX, I.TX_BIT, B)                                # B cycles per bit
        a.ucfg(I.TX, I.TX_FRAME, 0)                              # 8 bits, LSB first
        a.ucfg(I.ETX_U, I.D_AFTER, I.A_DRIVE1 if stop_lvl else I.A_DRIVE0)   # after the packet: drive the stop level
    # RX runs in hardware from here on: ERX finds the start bit and starts the RX unit, the RX unit votes on
    # each bit, ERX checks the stop bit and re-arms itself (AUTOARM). RXRDY means a byte was received and checked.
    if rx:
        a.ucfg(I.RX, I.RX_PINS, I.rx_pins(rxd))                  # sample rxd
        a.ucfg(I.RX, I.RX_BIT, c["B"])                           # one window per bit time
        a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(hold=1))             # 8 bits, LSB first, hold the byte for urd
        a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(N, K))                 # N samples a bit, the middle N - 2K vote
        a.ucfg(I.RX, I.RX_STEP, c["S"])                          # S cycles between samples
        a.ucfg(I.RX, I.RX_FIRST, c["FIRST"])                     # from the start to data bit 0's window
        # Start bit: a falling edge, then low for half a bit, so a short glitch is not taken for a start.
        a.ucfg(I.ERX_U, I.D_COND, I.cond(rxd, I.FALL, start=1, autoarm=1))   # starts RX itself, re-arms after
        a.ucfg(I.ERX_U, I.D_DUR, c["DUR"])
        a.ucfg(I.ERX_U, I.D_AFTER, I.erx_after(1, 1))            # after the packet: check that the stop bit is 1
        a.ucfg(I.ERX_U, I.D_ADLY, c["ADLY"])                     # ... in the middle of the stop bit
        a.evt(I.DET_RX, I.EV_ARM)                                # RX is live from here, with no more firmware
    # Idle the line high, and have ETX report when it has been high for one bit time.
    if tx:
        a.set(txd, I.SRC_1)                                      # idle level first,
        a.setoe(txd, 1)                                          # then drive the pin, so it never glitches low
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # line idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)
    a.mov(7, 0)                                                  # TX state
    a.L("loop")
    if tx:
        # TX side, state 0: when the line has been idle and a byte is waiting, drive the start bit. ETX then times
        # it, and starts the TX unit by itself after B - k cycles, so the start bit is exact.
        a.cmp(7, 1)                                              # Z = 1: the start bit is already running
        a.jmp("tx_run", I.Z)
        a.jmp("rx_chk", I.NETX)                                  # idle >= 1 bit?
        a.jmp("rx_chk", I.TXF_EMPTY)                             # and a byte?
        a.evt(I.DET_TX, I.EV_CLR)                                # the idle flag is used up
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 0, start=1))   # ETX: once low for B - k, start TX
        a.ucfg(I.ETX_U, I.D_DUR, B - K_LAT)
        a.evt(I.DET_TX, I.EV_ARM)
        a.set(txd, I.SRC_0)                                      # start bit
        a.mov(7, 1)                                              # state 1: wait for TX to take over
        a.jmp("rx_chk")
        # TX side, state 1: ETX has fired, so the TX unit is sending. Set ETX up to time the stop bit and the idle
        # after it. The arm takes effect after the stop edge, which ETX drives.
        a.L("tx_run")
        a.jmp("rx_chk", I.NETX)                                  # TX not started yet
        a.evt(I.DET_TX, I.EV_CLR)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # stop + idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)                                # deferred to after stop edge
        a.mov(7, 0)                                              # back to state 0: ready for the next byte
    # RX side: take a finished byte. Reading DATA clears RXRDY.
    a.L("rx_chk")
    if rx:
        a.jmp("loop", I.NRXRDY)                                  # no byte yet
        a.urd(0, I.RX, I.RX_DATA)                                # r0 = the byte; clears RXRDY
        a.jmp("loop", I.FERR)                                    # bad stop bit: drop
        a.fput(0, I.FIFO_RX)                                     # good byte to the host (blocks if RXF is full)
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
    dropped; mailbox M_INFO counts parity errors.
    Break: a frame of all zeros with a bad stop bit counts in M_STATUS and raises the
    controller IRQ. A doorbell sends a break of M_NTX bit times (> 0), between frames.
    The start bit is timed by the instruction stream, because a word with parity or 9
    bits comes from a register, not the FIFO."""
    # Registers: r0 the word being sent or received. r1 scratch: second FIFO byte, break length.
    # r2 parity test. r5 break count, kept in M_STATUS. r6 parity error count, kept in M_INFO.
    # Unlike uart(), a word with parity or 9 bits comes from a register, so the start bit is timed by instructions.
    idle, active = (0, 1) if inv else (1, 0)
    c = uart_consts(B, N, K, S)
    wbits = bits + (parity != PAR_NONE)
    stop = B * stop_b2 // 2
    pbit = 1 if msb else 1 << bits              # the parity bit in the word
    mask = (1 << bits) - 1
    a = Asm()
    # Setup. NBITS is data + parity, so parity needs no framing hardware. The stop time is the idle time ETX waits for.
    # TX unit: the data bits and the parity bit, as one word; ETX drives the stop level when it ends.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(txd))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, inv, wbits))       # bit order, inverted line, data + parity bits
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_DRIVE0 if inv else I.A_DRIVE1)   # stop level after the last bit
    # RX unit: votes on each bit, as in uart(); the word (data + parity) is held for urd.
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(rxd))
    a.ucfg(I.RX, I.RX_BIT, B)
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, inv=inv, nbits=wbits))
    a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(N, K))
    a.ucfg(I.RX, I.RX_STEP, c["S"])
    a.ucfg(I.RX, I.RX_FIRST, c["FIRST"])
    # ERX: the start edge (rising on an inverted line) starts RX; the stop bit is checked at the idle level.
    a.ucfg(I.ERX_U, I.D_COND, I.cond(rxd, I.RISE if inv else I.FALL, start=1, autoarm=1))
    a.ucfg(I.ERX_U, I.D_DUR, c["DUR"])
    a.ucfg(I.ERX_U, I.D_AFTER, I.erx_after(1, idle))
    a.ucfg(I.ERX_U, I.D_ADLY, c["ADLY"])
    a.evt(I.DET_RX, I.EV_ARM)
    # The TX line idles, and ETX reports once it has been idle for the stop time (1, 1.5 or 2 bits).
    a.set(txd, I.SRC_0 if inv else I.SRC_1)
    a.setoe(txd, 1)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, idle))
    a.ucfg(I.ETX_U, I.D_DUR, stop)
    a.evt(I.DET_TX, I.EV_ARM)
    a.mov(6, 0)                                  # parity error count
    a.mov(5, 0)                                  # break count
    # TX side: nothing goes out until ETX says the stop time has passed.
    a.L("loop")
    a.jmp("rx_chk", I.NETX)                      # stop time not over yet: only RX can be served
    a.jmp("brk", I.DB)                           # the host asked for a break
    a.jmp("rx_chk", I.TXF_EMPTY)                 # nothing to send
    # Build the word: data, then the parity bit where it goes last on the line.
    a.fget(0, I.FIFO_TX)                         # r0 = the data (low byte)
    if bits > 8:
        a.fget(1, I.FIFO_TX).shl(1, 8).or_(0, I.R(1))   # high byte from a second FIFO byte
    if parity != PAR_NONE:
        if msb:
            a.shl(0, 1)                          # room for the parity bit, which goes last
        a.par(0)                                 # C = odd number of 1s in the data
        pm = {PAR_EVEN: I.NC, PAR_ODD: I.C}.get(parity)
        if parity == PAR_MARK:
            a.or_(0, pbit)                       # mark: always 1 (space: always 0, nothing to do)
        elif parity != PAR_SPACE:
            a.jmp("tx_go", pm).or_(0, pbit)      # even / odd: set the parity bit when C says so
            a.L("tx_go")
    # Start bit, by instruction timing: set takes 1 cycle, delay(B - 2) takes B - 1.
    a.evt(I.DET_TX, I.EV_CLR)
    a.set(txd, I.SRC_1 if inv else I.SRC_0)      # start bit: the next ustart is B cycles later
    a.delay(B - 2)
    a.ustart(I.M_TX, I.R(0))                     # the data and parity bits, from r0
    a.evt(I.DET_TX, I.EV_ARM)                    # deferred to after the stop edge
    # RX side: a bad stop bit goes to ferr, a bad parity bit to p_bad, and only a good frame reaches RXF.
    a.L("rx_chk")
    a.jmp("loop", I.NRXRDY)                      # no word yet
    a.urd(0, I.RX, I.RX_DATA)                    # r0 = data + parity; clears RXRDY
    a.jmp("ferr", I.FERR)                        # the stop bit was bad
    if parity != PAR_NONE:
        if parity in (PAR_MARK, PAR_SPACE):
            a.mov(2, I.R(0)).and_(2, pbit)       # Z = the parity bit was 0
            a.jmp("p_bad", I.Z if parity == PAR_MARK else I.NZ)
        else:
            a.par(0)                             # data + parity bit: even -> C = 0
            a.jmp("p_ok", I.NC if parity == PAR_EVEN else I.C)
            a.jmp("p_bad")
            a.L("p_ok")
        if msb:
            a.shr(0, 1)                          # drop the parity bit, which came last
    # Good frame: the data bits to RXF. More than 8 bits take a second byte.
    a.mov(1, I.R(0)).and_(0, mask & 0xFF)        # r1 keeps the word; r0 = the low data byte
    a.fput(0, I.FIFO_RX)
    if bits > 8:
        a.shr(1, 8).and_(1, (mask >> 8) & 0xFF).fput(1, I.FIFO_RX)   # the high data bits as a second byte
    a.jmp("loop")
    if parity != PAR_NONE:
        a.L("p_bad").add(6, 1).mbw(6, I.M_INFO).jmp("loop")     # count it in M_INFO, drop the word
    # a bad stop bit with all data 0 is a break; ERX needs a new edge, so it counts once
    a.L("ferr").cmp(0, 0).jmp("loop", I.NZ)      # bad stop bit, data not 0: a framing error, dropped
    a.add(5, 1).mbw(5, I.M_STATUS).host_irq().jmp("loop")   # a break: count it in M_STATUS, tell the host
    # send a break, then the stop time (ETX re-arms on the idle level)
    a.L("brk").dbclr().mbr(1, I.M_NTX).evt(I.DET_TX, I.EV_CLR)   # r1 = the length from M_NTX, in bit times
    a.set(txd, I.SRC_1 if inv else I.SRC_0)      # the line to the active level: the break
    # One bit time a pass: delay(B - 2) takes B - 1, and djnz takes 1.
    a.L("brl").delay(B - 2).djnz(1, "brl")
    a.set(txd, I.SRC_0 if inv else I.SRC_1)      # back to idle
    a.evt(I.DET_TX, I.EV_ARM).jmp("rx_chk")      # ETX times the stop time again before the next frame
    return a


# ------------------------------------------------------------------ SPI
SPI_PINS = dict(sck=2, mosi=3, miso=4, cs=5)


def _cs_time(a, B):
    """CS lead and lag: at least half an SCK period, after CS falls and before it rises.
    The lead: set(cs) and delay(0) take 2 cycles, and delay(n) n + 1."""
    n = (B + 1) // 2 - 3
    if n >= 0:
        a.delay(n)
    return a


def spi_master_words(B=2, cpol=0, cpha=0, msb=1, sdly=3, bits=16, sck=2, mosi=3, miso=4, cs=5):
    """SPI master for words of 1-16 bits: M_NTX = n words, two bytes per word (low first)
    from dmem[0]; n replies, two bytes each, at dmem[0x80]. A word over 8 bits does not fit
    the byte FIFOs, so this program moves its words with ld / st, not the data movers.
    Owners: SCK, MOSI -> TX unit."""
    # Registers: r0 word out. r1 pointer to the words. r2 words left. r3 pointer to the replies. r4 scratch.
    a = Asm()
    # The TX unit makes SCK and MOSI; the RX unit samples MISO on the TX unit's sample tick, SDLY cycles late.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))        # data on MOSI, clock on SCK
    a.ucfg(I.TX, I.TX_BIT, B)                                   # SCK period = B cycles
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))       # `bits` bits a word
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))                # SPI mode
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))   # hold the word for urd
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    # SCK idles at CPOL, and CS is high until a request comes.
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)                              # wait for the doorbell
    a.dbclr()
    # Request: r2 = n from M_NTX, r1 -> the words at dmem[0]. Then select the device.
    a.mbr(2, I.M_NTX).mov(1, 0).mov(3, 0x80)                    # r3 -> the replies at 0x80
    a.set(cs, I.SRC_0).delay(0)                                 # CS low, then the lead time
    _cs_time(a, B)
    a.L("word")
    # One word: two bytes from DMEM, low first, out and in together. A word over 8 bits comes from a
    # register and is read with urd, because the FIFOs hold bytes. SCK pauses between words, which SPI allows.
    a.ld(0, 1, inc=1).ld(4, 1, inc=1).shl(4, 8).or_(0, I.R(4))  # r0 = low | high << 8
    a.ustart(I.M_BOTH, I.R(0))                                  # TX sends r0 while RX receives
    a.uwait(I.M_BOTH)                                           # until both are done
    a.urd(4, I.RX, I.RX_DATA)                                   # r4 = the word received
    a.st(4, 3, inc=1).shr(4, 8).st(4, 3, inc=1)                 # stored low byte first
    a.djnz(2, "word")                                           # next word, until n are done
    # All sent: deselect, and tell the host.
    _cs_time(a, B)                                              # the lag time before CS rises
    a.set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master(B=2, cpol=0, cpha=0, msb=1, sdly=3, sck=2, mosi=3, miso=4, cs=5):
    """SPI master: the host writes M_NTX = n and dmem[0..n-1] = data and rings DOORBELL;
    the firmware clocks n bytes with CS low, the received bytes go to dmem[0x80..], then
    it raises the host IRQ. The data movers carry the bytes: TXM from DMEM to the TX FIFO,
    RXM from the RX FIFO to DMEM. Owners: SCK, MOSI -> TX unit."""
    # Registers: r2 bytes left.
    a = Asm()
    # The TX unit makes SCK and MOSI. The RX unit samples MISO on the TX unit's sample tick, and SDLY covers
    # the trip through the synchroniser and the outside world.
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))        # data on MOSI, clock on SCK
    a.ucfg(I.TX, I.TX_BIT, B)                                   # SCK period = B cycles (B = 2: 24 MHz)
    a.ucfg(I.TX, I.TX_FRAME, msb)                               # 8 bits, MSB or LSB first
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))                # SPI mode
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0))          # auto-push
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    # SCK idles at CPOL, and CS is high until a request comes.
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    # Wait for the host, then select the device.
    a.L("idle").jmp("idle", I.NDB)                              # wait for the doorbell
    a.dbclr()
    a.mbr(2, I.M_NTX)                                           # r2 = n
    a.ucfg(I.TXM, I.MV_PTR, 0).ucfg(I.TXM, I.MV_GO, I.R(2))     # TXM: dmem[0..n-1] -> TXF
    a.ucfg(I.RXM, I.MV_PTR, 0x80).ucfg(I.RXM, I.MV_GO, I.R(2))  # RXM: RXF -> dmem[0x80..]
    a.set(cs, I.SRC_0).delay(0)                                 # CS low, then the lead time
    _cs_time(a, B)
    # Each byte: both units start in the same cycle; TX takes its byte from TXF (ustart waits
    # until TXM has put one there) and RX pushes the reply to RXF, where RXM takes it.
    # SCK pauses for a few instructions between bytes, which SPI allows.
    a.L("byte").ustart(I.M_BOTH).uwait(I.M_BOTH).djnz(2, "byte")
    a.L("rxw").jmp("rxw", I.RXM_BUSY)                           # the last reply is in DMEM
    # All sent: deselect, and tell the host.
    _cs_time(a, B)                                              # the lag time before CS rises
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
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))                    # TX drives MISO, no clock of its own
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    # MISO changes on the edge that does not sample (CPHA 1: bit 0 on the first leading edge)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))                    # RX samples MOSI ...
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0, nbits=bits))   # ... and pushes each byte to RXF
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))   # on the master's sampling edge
    # CS low starts each unit, and AUTOARM restarts it for the next byte while CS stays low.
    for d in (I.ETX_U, I.ERX_U):
        a.ucfg(d, I.D_COND, I.cond(cs, I.LEVEL, 0, start=1, autoarm=1))
        a.ucfg(d, I.D_DUR, 1)
    # Not selected: MISO is left to the others. OE can only change while TX is idle, so ETX
    # is armed once MISO is on, a few cycles after CS falls (spi_slave_lead_cycles).
    a.L("idle").setoe(miso, 0).wait(cs, 1)                      # release MISO; wait for CS high (deselected)
    a.evt(I.DET_RX, I.EV_ARM)                                   # ERX starts RX as soon as CS falls
    a.wait(cs, 0).setoe(miso, 1).evt(I.DET_TX, I.EV_ARM)        # selected: drive MISO, then let ETX start TX
    # Selected: the hardware works by itself. Only CS going high needs the firmware.
    a.L("run").jpin(cs, 1, "end")
    a.jmp("run")
    # Deselected: stop both units in mid-byte if need be, and go back to waiting.
    a.L("end").ustop(I.M_BOTH)
    a.evt(I.DET_TX, I.EV_DISARM).evt(I.DET_RX, I.EV_DISARM)    # no restart while CS is high
    a.evt(I.DET_TX, I.EV_CLR).evt(I.DET_RX, I.EV_CLR)
    a.jmp("idle")
    return a


def spi_master_stream(B=4, cpol=0, cpha=0, msb=1, sdly=3, sck=2, mosi=3, miso=4, cs=5):
    """SPI master with no gap between bytes (TX and RX CHAIN): M_NTX = n, dmem[0..n-1] = data;
    the replies go to dmem[0x80..]. M_STATUS = 0 ok, 1 TX ran dry and SCK paused, 2 a reply was
    lost (RXF overran, or the host took a byte from it). The data movers keep the FIFOs going:
    TXM refills TXF in 3 cycles a byte, and a byte at B = 2 lasts 16, so any length runs with no
    pause. Whatever happens, the firmware ends and raises the IRQ: once TX is idle it does not
    wait for a reply that is not there. Owners: SCK, MOSI -> TX unit."""
    # Registers: r2 = n. r6 the status, for M_STATUS.
    # CHAIN makes each unit go straight on to the next byte; the movers keep the FIFOs moving.
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))        # data on MOSI, clock on SCK
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, chain=1))         # CHAIN: the next TXF byte follows with no gap
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, chain=1))         # CHAIN: RX takes byte after byte into RXF
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    # SCK idles at CPOL, CS high.
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)                              # wait for the doorbell
    a.dbclr()
    # Request: start both movers, and let TXF fill before the first clock.
    a.mbr(2, I.M_NTX).mov(6, 0)                                 # r2 = n; status 0
    a.ucfg(I.TXM, I.MV_PTR, 0).ucfg(I.TXM, I.MV_GO, I.R(2))     # TXM: dmem[0..n-1] -> TXF
    a.ucfg(I.RXM, I.MV_PTR, 0x80).ucfg(I.RXM, I.MV_GO, I.R(2))  # RXM: RXF -> dmem[0x80..]
    a.L("pre").jmp("go", I.TXF_FULL).jmp("pre", I.TXM_BUSY)     # TXF full, or every byte queued
    a.L("go").set(cs, I.SRC_0).delay(0)                         # select, the lead time,
    _cs_time(a, B).ustart(I.M_BOTH)                             # then both units: they take bytes from here on
    # While TXM has bytes to give, TX must not stop. If it does (the chain ran dry), SCK paused:
    # report it and start TX again (ustart waits for a byte).
    a.L("run").jmp("tend", I.TXM_IDLE).jmp("run", I.TXU_BUSY)
    a.L("late").mov(6, 1).ustart(I.M_TX).jmp("run")
    # Every byte is in TXF or gone. When TX ends, TXF must be empty too.
    a.L("tend").uwait(I.M_TX).jmp("late", I.TXF_NEMPTY)
    # The last reply is at most SDLY away; RXM stores it at once.
    a.jmp("end", I.RXM_IDLE).delay(20).jmp("end", I.RXM_IDLE)
    a.mov(6, 2)                                                 # a reply never came: report it
    # Done: stop RX (a chained RX never ends by itself) and RXM, deselect and report.
    a.L("end").ustop(I.M_RX).ucfg(I.RXM, I.MV_GO, 0)
    _cs_time(a, B)
    a.set(cs, I.SRC_1)
    a.mbw(6, I.M_STATUS)                                        # the status for the host
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master_3wire(B=8, cpol=0, cpha=0, msb=1, sdly=3, sck=2, dio=3, cs=5):
    """3-wire SPI master: one data pin. M_NTX = n_write, M_NRX = n_read, dmem[0..] = write
    bytes; read bytes go to dmem[0x80..]. All writes, then all reads, in one CS frame; the pin
    is released in between. The data movers carry the bytes. Owners: SCK, DIO -> TX unit."""
    # Registers: r2 writes left. r5 reads left.
    a = Asm()
    a.ucfg(I.TX, I.TX_BIT, B)                                   # TX_PINS is set per phase, below
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(dio))                     # RX reads the shared data pin
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb))                   # and pushes each byte to RXF
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)      # SCK idles at CPOL
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)                              # wait for the doorbell
    a.dbclr()
    # Request. The TX unit gets the data pin only for the writes.
    a.mbr(2, I.M_NTX).mbr(5, I.M_NRX)                          # r2 = writes, r5 = reads
    a.ucfg(I.TXM, I.MV_PTR, 0).ucfg(I.TXM, I.MV_GO, I.R(2))     # TXM: the write bytes -> TXF
    a.ucfg(I.RXM, I.MV_PTR, 0x80).ucfg(I.RXM, I.MV_GO, I.R(5))  # RXM: the read bytes -> dmem[0x80..]
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio, 1, sck, 1))          # write phase: TX drives DIO and SCK
    a.setoe(dio, 1)
    a.set(cs, I.SRC_0).delay(0)
    _cs_time(a, B)
    a.cmp(2, 0).jmp("rd", I.Z)                                  # no writes: straight to the reads
    # Write phase: the TX unit drives DIO.
    a.L("wr").ustart(I.M_TX).uwait(I.M_TX).djnz(2, "wr")      # each byte from TXF
    a.L("rd").setoe(dio, 0)                                     # turnaround
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, 0, sck, 1))            # the clock only
    a.cmp(5, 0).jmp("fin", I.Z).delay(B)
    # Read phase: the TX unit only makes the clock. Its data goes nowhere, and the RX unit reads DIO.
    a.L("rl").ustart(I.M_BOTH, 0).uwait(I.M_BOTH).djnz(5, "rl")   # 8 clocks; RX takes the byte, RXM stores it
    a.L("rxw").jmp("rxw", I.RXM_BUSY)                           # the last one is in DMEM
    a.L("fin")
    _cs_time(a, B).set(cs, I.SRC_1)                             # lag time, deselect
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
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(dio))                     # RX reads DIO on the master's sampling edge
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1))           # held for urd: firmware looks at each byte
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio))                     # TX drives DIO only in a read
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))   # on the other edge
    # Not selected: DIO released. The first byte after CS falls is the command.
    a.L("idle").setoe(dio, 0).wait(cs, 1).wait(cs, 0)           # wait for a fresh select
    a.ustart(I.M_RX)                                            # receive the command
    a.call("rx1").jmp("end", I.C)                               # CS rose before a full byte
    a.fput(0, I.FIFO_RX)                                        # the command goes to the host too
    a.and_(0, 0x80).jmp("read", I.NZ)                           # MSB = 1: a read
    # Write: every byte goes to RXF until CS rises.
    a.L("wl").ustart(I.M_RX).call("rx1").jmp("end", I.C)
    a.fput(0, I.FIFO_RX).jmp("wl")                              # each written byte to RXF
    # The master drives until it has seen its last bit out: take the pin only after that. About 32 cycles
    # from the command byte's RXRDY: the 9 instructions before this one, and the delay.
    a.L("read").wait(sck, cpol).delay(21).setoe(dio, 1)
    # Read: every byte comes from TXF. An empty FIFO sends 0xFF.
    a.L("rl").mov(0, 0xFF).fget(0, I.FIFO_TX, nb=1)             # r0 = the next TXF byte, or 0xFF if none
    a.ustart(I.M_TX, I.R(0))
    a.L("rw").jpin(cs, 1, "end").jmp("rw", I.TXU_BUSY).jmp("rl")   # until CS rises, one byte after another
    a.L("end").ustop(I.M_BOTH).jmp("idle")                      # CS high: stop wherever the units are
    # one byte into r0, C = 1 if CS went high first
    a.L("rx1").jpin(cs, 1, "rxe").jmp("rx1", I.NRXRDY)
    a.urd(0, I.RX, I.RX_DATA).cmp(0, 0).ret()                   # cmp x, 0 gives C = 0
    a.L("rxe").mov(0, 0).cmp(0, 1).ret()                        # 0 - 1 borrows: C = 1
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
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))                    # TX drives MISO on the master's clock
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))       # `bits` bits a word
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))   # words over 8 bits: held for urd
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    # Not selected: MISO released, the first word ready in r0. Selected: drive MISO.
    a.L("idle").setoe(miso, 0).call("next").wait(cs, 1).wait(cs, 0).setoe(miso, 1)
    # The next word starts at once, and the word just received is stored after that: RX
    # keeps DATA and RXRDY across a start, and the next edge is only half a period away.
    a.L("word").ustart(I.M_BOTH, I.R(0))
    a.call("store").call("next")                                # store the last word, fetch the next
    a.L("w").jpin(cs, 1, "end").jmp("w", I.TXU_BUSY).jmp("word")   # until this word is done, or CS rises
    a.L("end").ustop(I.M_BOTH).call("store").jmp("idle")        # a word that completed at CS high is kept
    # Subroutine: two RXF bytes from the word received, if there is one.
    a.L("store").jmp("sx", I.NRXRDY)
    a.urd(4, I.RX, I.RX_DATA).fput(4, I.FIFO_RX).shr(4, 8).fput(4, I.FIFO_RX)   # low byte, then high
    a.L("sx").ret()
    # the next word to send, in r0
    a.L("next").mov(0, 0xFFFF).mov(1, 0xFF)                     # all ones if TXF is empty
    a.fget(2, I.FIFO_TX, nb=1).jmp("nx", I.C)                   # low byte (none: keep 0xFFFF)
    a.fget(1, I.FIFO_TX, nb=1).shl(1, 8).or_(2, I.R(1)).mov(0, I.R(2))   # high byte (none: 0xFF)
    a.L("nx").ret()
    return a


# ------------------------------------------------------------------ I2C
def i2c_master(B=126, scl=6, sda=7, odly=None, sdly=2, addr10=False, arb=False, hs_B=None, master_code=0x08,
               stuck=0xFFFF):
    """I2C master. Request: M_ADDR = the address (7 bits, or 10 with addr10), M_NTX = n_write,
    M_NRX = n_read, dmem[0..] = write data. The firmware sends START, address+W and the write
    bytes (if any), then, if n_read > 0, a repeated START, address+R and reads n_read bytes
    (ACK all but the last, NACK the last), then STOP. Result: dmem[0x80..] = read bytes,
    M_STATUS = 0 ok / 1 address NACK / 2 data NACK / 3 arbitration lost / 4 SDA stuck low;
    then host IRQ. Every byte passes through a register (the firmware clocks or checks it),
    so this program moves its bytes with ld / st rather than the data movers.
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
    addr10   M_ADDR holds a 10-bit address (10-bit addressing).
    arb      multi-master: waits for a free bus and checks every bit it sends. A master that
             reads 0 after sending 1 has lost: it releases both lines at once, sends no
             STOP and reports 3. Written bits are clocked by firmware, not by the units, because
             a unit would keep driving the rest of its byte: B >= 48.
             SDA low may be another master's transfer, so bus clear starts only if SDA low and SCL
             high last, with no change, for `stuck` passes of a 6-cycle loop (about 8 ms).
    hs_B     HS mode: the master code at B (F/S speed), which nobody acknowledges, then a repeated
             START, and all data bytes at hs_B. ACK clocks stay at F/S length, which is legal."""
    # Registers
    #   r0 the byte being written or read      r1 pointer to the write data (with arb, also the read bit count)
    #   r2 write bytes left                    r3 pointer to the read data
    #   r4 the address                         r7 read bytes left
    #   r5 result of wbyte: 0 ACK, 1 NACK, 2 lost. Input of rbyte: 0 ACK, 1 NACK. Also a loop counter
    #   r6 the status for M_STATUS. With arb, between requests: nonzero = bus busy; hi2 also uses it
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
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, den=0, cpin=scl, cen=1))   # TX only makes SCL, no data pin
        a.ucfg(I.TX, I.TX_BIT, B)                               # SCL period
        a.ucfg(I.TX, I.TX_FRAME, 1)                             # 8 clocks, MSB first
        a.ucfg(I.TX, I.TX_CLK, I.tx_clk(0, 0, stretch=1))       # wait while a slave holds SCL low
        a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb=1, hold=1))     # RX: the SDA byte, held for urd
        a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))   # sampled on TX's SCL-high tick
        a.ucfg(I.RX, I.RX_ODLY, odly)                           # a write changes SDA odly after SCL falls
    a.set(scl, I.SRC_1).set(sda, I.SRC_1)                       # both lines released (open drain: high)
    if arb:
        # The bus is busy from a START to its STOP, whatever the lines show in between (3.1.8).
        # ERX reports both; r6 != 0 means busy, between requests.
        a.ucfg(I.ERX_U, I.D_COND, I.cond(sda, I.ANY, qpin=scl, qen=1, qlvl=1))
        a.ucfg(I.ERX_U, I.D_DUR, 1)
        a.evt(I.DET_RX, I.EV_ARM).mov(6, 0)
        a.L("idle").jmp("bev", I.ERX).jmp("idle", I.NDB)        # a START or STOP seen, or wait for a request
        a.jpin(sda, 0, "low").cmp(6, 0).jmp("idle", I.NZ)       # SDA low (stuck?), or the bus is busy
        a.jpin(scl, 0, "idle").delay(H)                          # bus free time
        a.jmp("idle", I.ERX)                                    # someone started meanwhile: wait again
    else:
        # Single master: nothing to do but wait for the host.
        a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    # A request has come: read it from the mailbox.
    a.mbr(4, I.M_ADDR).mbr(2, I.M_NTX).mbr(7, I.M_NRX)          # addr, n_w, n_r
    a.mov(1, 0).mov(3, 0x80)                                    # r1 -> the write data, r3 -> the read data
    if not arb:                                                 # with arb, r6 is 0 to get here
        # SDA low before we even started: a device is stuck in the middle of a byte.
        a.mov(6, 0).jpin(sda, 0, "clr").L("go")
    if hs_B:
        a.ucfg(I.TX, I.TX_BIT, B)                                   # the master code at F/S speed
        a.call("start").mov(0, master_code).call("wbyte")           # no acknowledge expected
        a.ucfg(I.TX, I.TX_BIT, hs_B)                                # everything after it at HS speed
    # START, address + W, then the write bytes.
    a.call("start1" if arb else "start")
    if addr10:
        a.mov(0, I.R(4)).shr(0, 7).and_(0, 0x06).or_(0, 0xF0)       # 11110 A9 A8 W
        a.call("wbyte").cmp(5, 0).jmp("err_a", I.NZ)            # first address byte NACKed
        a.mov(0, I.R(4)).and_(0, 0xFF)                          # second address byte: A7..A0
    else:
        a.mov(0, I.R(4)).shl(0, 1)                              # 7-bit address, W = 0
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_a", I.NZ)                              # address NACKed (or lost)
    a.cmp(2, 0).jmp("rd", I.Z)                                  # no data to write
    a.L("wl").ld(0, 1, inc=1)                                   # the next write byte
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_d", I.NZ)                              # data NACKed (or lost)
    a.djnz(2, "wl")
    # Repeated START, address + R, then the reads. The last read byte gets a NACK, so the target lets go.
    a.L("rd").cmp(7, 0).jmp("fin", I.Z)                         # no reads: finish with STOP
    a.call("start")                                             # (repeated) START
    if addr10:
        a.mov(0, I.R(4)).shr(0, 7).and_(0, 0x06).or_(0, 0xF1)       # 11110 A9 A8 R
    else:
        a.mov(0, I.R(4)).shl(0, 1).or_(0, 1)                    # 7-bit address, R = 1
    a.call("wbyte")
    a.cmp(5, 0).jmp("err_a", I.NZ)
    a.L("rl").mov(5, 0).cmp(7, 1).jmp("rk", I.NZ).mov(5, 1)    # NACK the last byte
    a.L("rk").call("rbyte")                                     # r0 = the byte; ACK or NACK per r5
    a.st(0, 3, inc=1)
    a.djnz(7, "rl")
    # End of a request, good or bad: STOP (unless arbitration was lost), store the status, tell the host.
    a.L("fin").call("stop")
    a.mov(6, 0)                                                 # status 0: ok
    a.jmp("rep")
    a.L("err_a").mov(6, 1).jmp("err")                           # status 1: address NACK
    a.L("err_d").mov(6, 2)                                      # status 2: data NACK
    a.L("err")
    if arb:
        a.cmp(5, 2).jmp("lost", I.Z)                            # lost: no STOP, the other master owns the bus
    a.call("stop").jmp("rep")
    if arb:
        a.L("lost").mov(6, 3)                                   # both lines are released already
    a.L("rep")
    if hs_B:
        a.ucfg(I.TX, I.TX_BIT, B)                               # back to F/S speed for the next master code
    a.mbw(6, I.M_STATUS)                                        # the status for the host
    a.host_irq()
    if arb:
        # After a lost arbitration the bus stays busy. Otherwise our own STOP is behind us, or SDA
        # is still low: bev finds out which.
        a.cmp(6, 3).jmp("lr", I.Z)
        a.L("bev").evt(I.DET_RX, I.EV_CLR).evt(I.DET_RX, I.EV_ARM)   # a START or STOP: watch for the next
        a.mov(6, 1).jpin(sda, 0, "idle").mov(6, 0).jmp("idle")  # SDA low after it: START (busy). High: STOP (free)
        a.L("lr").evt(I.DET_RX, I.EV_CLR).evt(I.DET_RX, I.EV_ARM)   # lost: busy (r6 = 3) until its STOP
    a.jmp("idle")
    if arb:
        # SDA low with a request waiting. It is stuck only if nothing moves for this long.
        a.L("low").mov(5, stuck)
        a.L("ll").jpin(sda, 1, "idle").jpin(scl, 0, "idle").delay(2).djnz(5, "ll")   # 6 cycles a pass
    # Bus clear (3.1.16). The STOP resets a device that was in the middle of a byte.
    a.L("clr").mov(5, 9)                                        # up to nine clock pulses
    a.L("cl").set(scl, I.SRC_0).delay(H).set(scl, I.SRC_1).wait(scl, 1).delay(H)   # one SCL pulse
    a.jpin(sda, 1, "cr").djnz(5, "cl")                          # SDA released: the device let go
    a.L("stk").dbclr().mov(6, 4).jmp("rep")                     # still low after nine: status 4
    a.L("cr").set(scl, I.SRC_0).delay(Q).call("stop")           # free: a STOP resets the device's state
    if arb:
        a.jmp("bev")                                            # back to idle: the request still waits
    else:
        a.jpin(sda, 1, "go").djnz(5, "cl").jmp("stk")           # the device drove the next bit low
    # START (also repeated START): SDA falls while SCL is high
    a.L("start").set(sda, I.SRC_1).delay(H).set(scl, I.SRC_1).wait(scl, 1)   # SDA, then SCL high (repeated START)
    a.delay(H)                                                  # START setup time
    # With arb, a first START comes here, right after the bus was seen free: in the time of the
    # phases above, another controller could have started and be in the middle of a bit.
    a.L("start1").set(sda, I.SRC_0)
    if arb:
        a.jmp("hi2")                                            # another controller may go first
    else:
        a.delay(H).set(scl, I.SRC_0).delay(Q).ret()             # START hold time, then SCL low
    # STOP: SDA rises while SCL is high
    a.L("stop").set(sda, I.SRC_0).delay(H).set(scl, I.SRC_1).wait(scl, 1)   # SDA low, then SCL high
    a.delay(H).set(sda, I.SRC_1).delay(H).ret()                 # STOP setup, SDA high, bus free time
    # write r0; r5 = 0 ACK / 1 NACK / 2 arbitration lost
    if arb:
        # Subroutine wbyte, multi-master: send r0 bit by bit, reading SDA back after each. The top of r0 shifts into
        # C, which set can drive. r5 counts the 8 bits here, and holds the result on the way out.
        a.L("wbyte").shl(0, 8).mov(5, 8)                        # the byte to the top of r0; 8 bits
        a.L("wb").shl(0, 1).set(sda, I.SRC_C).delay(H - Q)     # C = the bit to send; low = H with the Q below
        a.call("hi1")                                           # SCL high; SDA read back here
        a.jmp("wbk", I.NC).jpin(sda, 0, "wlost")               # sent 1, read 0: lost
        a.L("wbk").call("hi2").djnz(5, "wb")
    else:
        # Subroutine wbyte, single master: RX drives SDA and TX clocks SCL for all 8 bits. Then the ACK clock by hand.
        a.L("wbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda, sda, 1))   # RX drives SDA and reads it back
        a.ustart(I.M_BOTH, I.R(0))                              # 8 clocks, r0 out on SDA
        a.uwait(I.M_BOTH)
        a.urd(5, I.RX, I.RX_DATA).delay(odly)                   # readback (clears RXRDY); data hold
    a.set(sda, I.SRC_1).delay(H)                                # release SDA for the ACK
    if arb:
        a.call("hi1")
    else:
        a.set(scl, I.SRC_1).wait(scl, 1).delay(H)               # the 9th clock high (the slave may stretch)
    # Read the ACK while SCL is high: low = ACK (r5 = 0), high = NACK (r5 = 1).
    a.mov(5, 0).jpin(sda, 0, "wa").mov(5, 1)
    a.L("wa")
    if arb:
        a.jmp("hi2")                                            # hi2 returns for us
    else:
        a.set(scl, I.SRC_0).delay(Q).ret()                      # 9th clock low, a short hold
    if arb:
        a.L("wlost").set(sda, I.SRC_1).set(scl, I.SRC_1).mov(5, 2).ret()
        # One SCL high phase in clock synchronization (3.1.7). hi1: it starts once SCL is really
        # high. SDA is read here: another controller may end the phase at any moment. hi2: it ends
        # after H, or as soon as another controller pulls SCL low, and then SCL goes low. Uses r6.
        a.L("hi1").set(scl, I.SRC_1).wait(scl, 1).ret()
        # wait with a time-out: one try a cycle, so r6 = k gives k + 1 cycles
        a.L("hi2").mov(6, 2 * (H // 2) + 2).wait(scl, 0, rd=6).set(scl, I.SRC_0).delay(Q).ret()
    # read into r0; r5 = 0 -> ACK, 1 -> NACK
    if arb:
        # Subroutine rbyte, multi-master: 8 SCL clocks by firmware, SDA read at the start of each high phase.
        a.L("rbyte").set(sda, I.SRC_1).mov(0, 0).mov(1, 8)      # r1 is free here: writes are over
        a.L("rbit").delay(H - Q).call("hi1").shl(0, 1).jpin(sda, 0, "rb0").or_(0, 1)   # SCL high: bit in
        a.L("rb0").call("hi2").djnz(1, "rbit")                  # SCL low, next bit
    else:
        # Subroutine rbyte, single master: TX clocks SCL for 8 bits and RX samples SDA.
        a.L("rbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda))      # RX samples SDA, drives nothing
        a.set(sda, I.SRC_1)                                     # SDA released for the slave
        a.ustart(I.M_BOTH)                                      # 8 clocks
        a.uwait(I.M_BOTH)
        a.urd(0, I.RX, I.RX_DATA).delay(odly)                   # data hold
    a.cmp(5, 1).set(sda, I.SRC_NC)                             # r5 = 0 borrows: C = 1, SDA low = ACK
    a.delay(H)                                                  # data setup before the 9th clock
    if arb:
        a.call("hi1").call("hi2")
    else:
        a.set(scl, I.SRC_1).wait(scl, 1).delay(H).set(scl, I.SRC_0).delay(Q)   # the 9th clock
    a.set(sda, I.SRC_1).ret()                                   # release SDA
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
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb=1, hold=1))         # 8 bits MSB first, held for urd
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, scl, 0, 0))    # sample SDA on SCL rising
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(sda))                     # TX drives SDA in a read
    a.ucfg(I.TX, I.TX_FRAME, 1)                                 # 8 bits MSB first
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, scl, 1))                  # shift on SCL falling
    a.ucfg(I.ERX_U, I.D_COND, I.cond(sda, I.ANY, qpin=scl, qen=1, qlvl=1))   # any SDA edge while SCL is high
    a.ucfg(I.ERX_U, I.D_DUR, 1)
    a.set(sda, I.SRC_1).set(scl, I.SRC_1)                       # both released
    if addr10 is not None:
        a.mov(6, 0)                                             # r6 = 1: 10-bit address matched
    # Wait for a START. The SDA level after the edge tells them apart: high means it was a STOP.
    a.L("idle").set(scl, I.SRC_1).evt(I.DET_RX, I.EV_ARM)       # SCL released, watch for START / STOP
    a.evt(I.DET_RX, I.EV_WAITC)                                 # START or STOP
    a.jpin(sda, 1, "stop")                                      # SDA high now: it was a STOP
    # Address byte. If it is not ours, ignore everything up to the next START.
    a.L("addr").evt(I.DET_RX, I.EV_ARM)                         # catch a START / STOP inside the byte
    a.call("rx8").jmp("ev", I.C)                                # r0 = address byte, or C = 1 on an event
    if general_call:
        a.cmp(0, 0).jmp("gc", I.Z)                              # address 0 + W: the general call
    if addr10 is None:
        a.mov(1, I.R(0)).shr(1, 1).cmp(1, addr).jmp("idle", I.NZ)  # not us: wait for START
        a.call("ack").jmp("ev", I.C)
        a.and_(0, 1).jmp("rd", I.NZ)                            # R/W bit = 1: a read
    else:
        first = 0xF0 | ((addr10 >> 7) & 0x06)                   # 11110 A9 A8, the first byte of a 10-bit address
        a.mov(1, I.R(0)).and_(1, 0xFE).cmp(1, first).jmp("idle", I.NZ)   # not our A9 A8
        a.and_(0, 1).jmp("r10", I.NZ)                           # a read: allowed only after our write
        a.call("ack").jmp("ev", I.C)
        a.call("rx8").jmp("ev", I.C)                            # the second byte: A7..A0
        a.cmp(0, addr10 & 0xFF).jmp("nom", I.NZ)
        a.call("ack").jmp("ev", I.C).mov(6, 1).jmp("wstart")    # matched: remember it for a later read
        a.L("nom").mov(6, 0).jmp("idle")
        a.L("r10").cmp(6, 1).jmp("idle", I.NZ)                  # a read must follow our write
        a.call("ack").jmp("ev", I.C).jmp("rd")
        a.L("wstart")
    # Write: the first byte sets the pointer, the others are stored at pointer++.
    a.mov(2, 1)                                                 # r2 = 1: next byte is the pointer
    a.L("wr").call("rx8").jmp("ev", I.C)                        # until a STOP or repeated START
    a.call("ack").jmp("ev", I.C)
    a.cmp(2, 1).jmp("wd", I.NZ)
    a.mov(3, I.R(0)).mov(2, 0).jmp("wr")                        # first byte: the register pointer
    a.L("wd").st(0, 3, inc=1).jmp("wr")                         # the rest: stored at pointer++
    if general_call:
        a.L("gc").call("ack").jmp("ev", I.C).mov(3, I2C_GC_MEM).mov(2, 0).host_irq().jmp("wr")   # data to 0xF0..
    # Read: hold SCL low while the byte is set up, then release it and let the master clock it out.
    a.L("rd").ld(0, 3, inc=1)                                   # r0 = dmem[pointer++]
    a.ustart(I.M_TX, I.R(0))                                    # bit 7 now, then on SCL falls
    a.delay(62)                                                 # tr(max) + tSU;DAT = 1250 ns (Table 11 note 4)
    a.set(scl, I.SRC_1)                                         # the data is ready: release
    a.uwait(I.M_TX)                                             # the master has clocked out 8 bits
    a.set(sda, I.SRC_1)                                         # release for the master ACK
    a.wait(scl, 1).jpin(sda, 1, "nak")                          # 9th clock high: SDA high = NACK, the read ends
    a.wait(scl, 0).set(scl, I.SRC_0).jmp("rd")                  # hold SCL while fetching
    a.L("nak").evt(I.DET_RX, I.EV_CLR).jmp("idle")              # wait for the STOP
    # A START or STOP came in the middle of a byte: drop the byte and act on the event.
    a.L("ev").evt(I.DET_RX, I.EV_CLR).ustop(I.M_RX)            # START/STOP inside a byte
    a.jpin(sda, 1, "stop").jmp("addr")                          # SDA high: STOP. Low: repeated START, an address follows
    a.L("stop")
    if addr10 is not None:
        a.mov(6, 0)                                             # a STOP ends the 10-bit match
    a.jmp("idle")
    # receive one byte into r0; C = 1 if a START/STOP came instead
    a.L("rx8").ustart(I.M_RX).set(scl, I.SRC_1)                 # RX ready first, then release SCL
    a.L("rx_w").jmp("rx_ev", I.ERX).jmp("rx_w", I.NRXRDY)       # an event, or wait for the byte
    a.urd(0, I.RX, I.RX_DATA).cmp(0, 0).ret()                   # cmp x,0 -> C = 0
    a.L("rx_ev").urd(0, I.RX, I.RX_DATA).mov(0, 0).cmp(0, 1).ret()   # 0 - 1 borrows -> C = 1
    # ACK: SDA low for the 9th clock, then hold SCL low until the next byte is set up
    # C = 1: a START or STOP came right after bit 8, so there is no ACK (r1 is free here)
    a.L("ack").jmp("ack_ev", I.ERX).jpin(scl, 1, "ack").set(sda, I.SRC_0)   # wait for SCL low, then SDA low
    a.wait(scl, 1).wait(scl, 0).set(scl, I.SRC_0).set(sda, I.SRC_1)   # 9th clock; then hold SCL low, release SDA
    if hold:
        a.delay(hold)                                           # extra stretching, for slow processing
    a.cmp(0, 0).ret()                                           # C = 0: acknowledged
    a.L("ack_ev").mov(1, 0).cmp(1, 1).ret()                     # C = 1: an event instead
    return a


# ------------------------------------------------------------------ USB
USB_SYNC = 0x80
PID_OUT, PID_IN, PID_SOF, PID_SETUP = 0xE1, 0x69, 0xA5, 0x2D
PID_DATA0, PID_DATA1 = 0xC3, 0x4B
PID_ACK, PID_NAK, PID_STALL = 0xD2, 0x5A, 0x1E
CRC5_POLY, CRC5_INIT, CRC5_RES = 0x2800, 0xF800, 0x6000
CRC16_POLY, CRC16_INIT, CRC16_RES = 0x8005, 0xFFFF, 0x800D
# usb_device: the mailbox registers under their USB names.
#   host -> firmware
#     UD_ADDR   (M1) our device address, 7 bits; read again before every token
#     UD_INLEN  (M2) the IN packet waiting: its length counting the PID (0 = none, answer with UD_HS[15:8]).
#                    The firmware clears it when the host ACKs the packet.
#     UD_INPID  (M4) the DATA0 / DATA1 PID of that IN packet
#     UD_HS     (M5) the handshake PIDs to send: [7:0] for OUT data, [15:8] for an IN with no data (NAK / STALL)
#   firmware -> host
#     UD_TOKEN  (M0) the token PID of the last OUT / SETUP
#     UD_EP     (M3) its endpoint
#     UD_EVENT  (M6) what happened: EV_* below; written together with the controller IRQ
#     UD_RXLEN  (M7) bytes received in the OUT / SETUP data packet, with its PID
UD_ADDR, UD_INLEN, UD_INPID, UD_HS = I.M_ADDR, I.M_NTX, I.M_PID, I.M_HS
UD_TOKEN, UD_EP, UD_EVENT, UD_RXLEN = I.M_OP, I.M_NRX, I.M_STATUS, I.M_INFO
UD_INDATA, UD_RXDATA = 0x00, 0x80   # DMEM: the IN data (no PID), and the packet received (PID first)
USB_IDLE = 65535        # usb_device: cycles of J that set ETX (1.37 ms at 48 MHz)
EV_OUT, EV_IN_ACK, EV_RESET, EV_IN_NOACK = 1, 2, 3, 4
# usb_host: the mailbox registers under their USB names.
#   host -> firmware
#     UH_OP     (M0) 0 a transaction, 1 bus reset, 2 keep-alive EOP, 3 resume
#     UH_DPID   (M4) the DATA PID for OUT / SETUP; 0 means an IN
#     UH_N      (M2) OUT / SETUP data bytes; for a reset or resume, the length in 256-cycle turns
#   firmware -> host
#     UH_STATUS (M6) UH_OK / UH_TIMEOUT / UH_BADPKT
#     UH_RXLEN  (M7) bytes received, with the PID
UH_OP, UH_DPID, UH_N = I.M_OP, I.M_PID, I.M_NTX
UH_STATUS, UH_RXLEN = I.M_STATUS, I.M_INFO
UH_TOK, UH_DATA = 0x00, 0x10   # DMEM: the 3 token bytes (PID, address/endpoint, CRC5), and the OUT / SETUP data
UH_RPID = 0x80          # DMEM: the first byte received, then the rest of the packet
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
    a.wait(jp, 1).evt(I.DET_RX, I.EV_ARM).ustart(I.M_RX)   # J (not SE0), ERX watches for the EOP, RX hunts


def _tx_on(a, jp, kp, turn=0):
    # Drive the bus and send the sync, `turn` + 1 cycles later if turn > 0. CHAIN then takes the next bytes from TXF.
    if turn:
        a.delay(turn)
    a.setoe(jp, 1).setoe(kp, 1).ustart(I.M_TX, USB_SYNC)


def _next_crc(r, poly, bit=1):
    # One step of the CRC register, as the CRC unit does it. Used to work out the dribble values below.
    return ((r << 1) ^ (poly if ((r >> 15) ^ bit) & 1 else 0)) & 0xFFFF


# USB 2.0 7.1.9.1: a dribble bit, one more 1 before the EOP, must be accepted. It goes through the
# CRC unit as well, and then gives this value instead of the residual.
CRC5_DRIB, CRC16_DRIB = _next_crc(CRC5_RES, CRC5_POLY), _next_crc(CRC16_RES, CRC16_POLY)


def _crc_good(a, res, drib, ok, bad):
    """Compare r4 with the residual: to `ok` if good (the usual case is the short one), else `bad`."""
    a.cmp(4, res).jmp(ok, I.Z).cmp(4, drib).jmp(bad, I.NZ)   # the residual, or the residual after a dribble bit


def usb_device(B=4, jp=0, kp=1, turn=None):
    """USB device at transaction level, low or full speed. The host side sets
    the address and handshakes in the mailbox and owns the higher layers (enumeration,
    data toggles, endpoints). See UD_* for the map; UD_EVENT says what happened,
    with a controller IRQ. The IN data (dmem[UD_INDATA..]) goes out through TXM, and a
    received packet goes to dmem[UD_RXDATA..] (its PID by the firmware, the rest by RXM). `turn`: extra cycles before answering (B = 32: the
    answer must not come before 2 bit times). A SETUP is always answered with ACK (8.5.3).
    Suspend: ETX times the idle bus. CSTAT.ETX set means J for at least USB_IDLE cycles
    (1.37 ms) since the last packet, and any packet clears it. Read it twice, 1.7 ms
    apart: still set means suspend (3 ms of idle). Resume shows as K on PIN_IN; the firmware
    keeps hunting all along, so the next packet is received normally. Remote wakeup is
    not implemented: the device must not advertise it."""
    # Registers
    #   r0 byte, event code          r1 second token byte, then the handshake to send
    #   r2 third token byte, then EP r3 DMEM pointer (the PID received)
    #   r4 CRC value, RX status      r5 UD_INLEN, then a time-out count
    #   r6 the PID, 0 if none came   r7 our address
    if turn is None:
        turn = 6 if B <= 8 else 3 * B + 9                   # the answer at least 2 bit times after the EOP
    wait_h = max(1, (33 * B // 2 - 7) // 2)                 # handshake: give up before 18 bit times
    a = Asm()
    # Setup, and ETX as the suspend timer: it fires after USB_IDLE cycles of J. It only sets a flag, for the host.
    _usb_setup(a, B, jp, kp)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(jp, I.LEVEL, 1))           # ETX: the bus in J (idle) ...
    a.ucfg(I.ETX_U, I.D_DUR, USB_IDLE)                          # ... for USB_IDLE cycles
    # Everything ends here, with whatever it had going stopped. The hunt comes first: the next
    # packet may start 2 bit times after the last one, and no bit reaches the FIFO or the CRC unit
    # until the hunt ends, so the rest has time.
    a.L("idle").ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).evt(I.DET_RX, I.EV_CLR)   # stop RX, clear the EOP flag
    a.ucfg(I.RXM, I.MV_GO, 0)                                    # and RXM, if a packet was dropped half way
    _rx_go(a, jp)
    a.L("dr").fget(0, I.FIFO_RX, nb=1).jmp("dr", I.NC)           # drain
    _crc_rx(a, CRC5_POLY, CRC5_INIT)                             # a token comes next: check its CRC5
    a.evt(I.DET_TX, I.EV_CLR).evt(I.DET_TX, I.EV_ARM)            # the idle timer starts again
    a.mbr(7, UD_ADDR)                                            # r7 = our address (the host side may change it)
    a.mbr(5, UD_INLEN)                                           # r5 = the IN packet waiting, if any
    a.mov(6, 0)                                                  # r6 = 0: no PID yet
    # Receive a token, wait for its EOP, and check it: PID in r6, then the second byte, then CRC5.
    a.evt(I.DET_RX, I.EV_WAITC)                                  # SE0: RX is stopped
    a.fget(6, I.FIFO_RX, nb=1).fget(1, I.FIFO_RX, nb=1)          # r6 = PID, r1 = second byte
    a.urd(4, I.CRC, I.CRC_VAL)                                   # r4 = the CRC register after the packet
    _crc_good(a, CRC5_RES, CRC5_DRIB, "t_ok", "t_bad")
    # Good token: address in the low 7 bits of the second byte. IN goes straight to the answer: it has the least time.
    a.L("t_ok").mov(4, I.R(1)).and_(4, 0x7F).cmp(4, I.R(7)).jmp("idle", I.NZ)
    a.cmp(6, PID_IN).jmp("in_tok", I.Z)                          # no time to spare here
    # OUT / SETUP: the data packet follows soon, so start hunting for it at once, then read the token's
    # last byte. No bit reaches the CRC unit until the hunt ends, so switching it to CRC16 here is safe.
    _rx_go(a, jp)
    a.ucfg(I.CRC, I.CRC_POLY, CRC16_POLY).ucfg(I.CRC, I.CRC_INIT, CRC16_INIT)   # the data packet has CRC16
    a.fget(2, I.FIFO_RX, nb=1)                                   # r2 = third token byte
    a.cmp(6, PID_OUT).jmp("o_ok", I.Z)
    a.cmp(6, PID_SETUP).jmp("idle", I.NZ)                        # any other token: ignore
    # Report PID and endpoint. EP = 3 bits of the third byte, then the top bit of the second.
    a.L("o_ok").mbw(6, UD_TOKEN)
    a.and_(2, 7).shl(2, 1).shr(1, 7).or_(2, I.R(1)).mbw(2, UD_EP)   # EP = byte3[2:0] : byte2[7]
    a.mbr(1, UD_HS)                                              # OUT handshake in r1 (fput sends bits 7:0)
    a.cmp(6, PID_SETUP).jmp("o_hs", I.NZ).mov(1, PID_ACK)        # SETUP is always ACKed
    a.L("o_hs").mov(3, UD_RXDATA)                                # r3 -> where the data goes
    # The first byte must be a good DATA PID. The slow checks are done here, as bytes arrive:
    # after the EOP only the CRC and the stuff flag stand between us and the handshake.
    a.L("op").jmp("idle", I.ERX).fget(0, I.FIFO_RX, nb=1).jmp("op", I.C)   # an EOP before any byte: drop
    a.st(0, 3).cmp(0, PID_DATA0).jmp("rl0", I.Z).cmp(0, PID_DATA1).jmp("idle", I.NZ)   # DATA0/1 only
    # The rest of the packet goes to DMEM through RXM. The firmware only waits for the EOP:
    # the time from here to the handshake is the tightest in the firmware.
    # The length is not known in advance: RXM is given 255 (the most that fits after the PID) and is
    # stopped once the packet is over.
    a.L("rl0").ucfg(I.RXM, I.MV_PTR, UD_RXDATA + 1).ucfg(I.RXM, I.MV_GO, 255)
    a.L("rl").jmp("rl", I.NERX)
    # EOP seen. Answer only if CRC16 and stuffing are good; otherwise stay silent, so the host retries.
    a.L("rle0").urd(4, I.CRC, I.CRC_VAL)                        # r4 = the CRC register
    _crc_good(a, CRC16_RES, CRC16_DRIB, "rc2", "idle")
    a.L("rc2").urd(4, I.RX, I.RX_STAT).and_(4, I.RX_ST_SERR).jmp("idle", I.NZ)   # a stuff error: silent
    _tx_on(a, jp, kp, turn)                                      # sync out, after the turnaround
    a.call("hsend")                                              # the handshake in r1: UD_HS[7:0], or ACK for SETUP
    # The last byte can reach the FIFO just before the EOP: RXM stores whatever is left, then stops.
    # The handshake is out first, because it has a deadline; the length can wait.
    a.L("rle").jmp("rle", I.RXF_NEMPTY)
    a.urd(0, I.RXM, I.MV_RD_CNT).add(0, 1).mbw(0, UD_RXLEN)      # length: the PID and what RXM stored
    a.ucfg(I.RXM, I.MV_GO, 0)
    a.mov(0, EV_OUT).jmp("ev")
    # Subroutine: send the handshake in r1. The CRC unit must not add anything to it.
    a.L("hsend").fput(1, I.FIFO_TX).ucfg(I.CRC, I.CRC_CTRL, 0).uwait(I.M_TX).ret()
    # IN: answer within 6.5 bit times. Sync goes out first; it is the same for both
    # answers, and it gives the time to decide and to set up the CRC.
    a.L("in_tok")
    _tx_on(a, jp, kp, turn)
    # r5 = UD_INLEN, the PID and the data. 0 means nothing is ready: NAK.
    a.cmp(5, 0).jmp("nak", I.Z)
    # INIT takes effect 5 cycles after the sync started (4-6 all work): the bits already gone do not count.
    _crc_tx(a, skip=16 - (5 // B + 1))
    # Data: PID, then the bytes. The chain must not run dry: the loop is far faster than the bytes go out.
    a.mbr(0, UD_INPID).fput(0, I.FIFO_TX)                        # the DATA PID from the host side
    a.sub(5, 1).jmp("txe", I.Z)                                  # r5 = data bytes (UD_INLEN counts the PID)
    a.ucfg(I.TXM, I.MV_PTR, UD_INDATA).ucfg(I.TXM, I.MV_GO, I.R(5))   # TXM: the data -> TXF, well ahead of the line
    a.L("txe").uwait(I.M_TX)                                     # TXF empty: the CRC unit appends CRC16, EOP
    # the host's handshake: its packet must start before 18 bit times
    a.ucfg(I.CRC, I.CRC_CTRL, 0)                                 # CRC off: a handshake has none
    a.fget(1, I.FIFO_RX, nb=1)                                   # the last byte of the token
    _rx_go(a, jp)
    a.mov(5, wait_h)                                             # 2 cycles a pass of the loop below
    # Wait for K, the start of the handshake. No answer in time means the host did not get the data: it stays in DMEM.
    a.L("ha").jpin(kp, 1, "hb").djnz(5, "ha")
    a.L("nk").mov(0, EV_IN_NOACK).jmp("ev")
    # A handshake came. ACK clears UD_INLEN, and the host side can fill the next packet.
    a.L("hb").evt(I.DET_RX, I.EV_WAITC).fget(1, I.FIFO_RX, nb=1)   # wait for its EOP; r1 = its PID
    a.cmp(1, PID_ACK).jmp("nk", I.NZ)                            # not an ACK: as if none came
    a.mov(0, 0).mbw(0, UD_INLEN).mov(0, EV_IN_ACK).jmp("ev")
    # NAK or STALL, as the host side chose in UD_HS bits 15:8.
    a.L("nak").mbr(1, UD_HS).shr(1, 8).call("hsend").jmp("idle")
    # A packet shorter than a token, or a bus reset: SE0 still there after 2 us (the limit is 2.5).
    a.L("t_bad").cmp(6, 0).jmp("idle", I.NZ)                     # a PID came: just a bad token, ignore
    a.delay(81).jpin(jp, 1, "idle").mov(0, EV_RESET)             # 2 us after the SE0 began: still SE0 = reset
    # Every event ends here: tell the host what happened.
    a.L("ev").mbw(0, UD_EVENT).host_irq().jmp("idle")
    return a


def usb_host(B=4, jp=0, kp=1, gap=None, timeout=None):
    """USB host at transaction level, low or full speed. Mailbox: UH_OP (0
    transaction, 1 bus reset, 2 EOP = keep-alive, 3 resume; suspend = no transactions),
    UH_DPID = data PID for OUT / SETUP (0 for IN), UH_N = data bytes at dmem[UH_DATA]. DMEM:
    the token with its CRC5 at dmem[UH_TOK..UH_TOK+2]. Reset and resume last UH_N * 256 cycles:
    1875 for the 10 ms of a reset, 3750 for the 20 ms of a resume.
    Result: dmem[UH_RPID..] = the packet received (handshake or data PID first), UH_STATUS,
    UH_RXLEN = bytes received with the PID; controller IRQ. TXM sends the token and the data,
    RXM stores the packet after its PID. An IN gets an ACK when its
    PID, CRC and stuffing are good, within 7.5 bit times. Only a one-byte packet (a handshake) is
    reported with a PID that was not checked: the host software looks at UH_RPID."""
    # Registers
    #   r0 byte                       r1 the status for UH_STATUS
    #   r3 DMEM pointer (the PID)     r4 CRC value, RX status, RXM count
    #   r5 count, time-out            r6 UH_DPID (0 for an IN), also the high byte of a count
    if gap is None:
        gap = max(7, 4 * B - 15)        # >= 2 bit times after SE0 ends, measured from SE0
    if timeout is None:
        # the answer must start before 18 bit times. + 1: v3 reaches the loop 2 cycles sooner (mbr for mov + ld)
        timeout = max(1, (35 * B // 2 - 15 - B) // 2 + 1)
    a = Asm()
    _usb_setup(a, B, jp, kp)
    # Wait for the doorbell, then branch on UH_OP. Each operation ends at fin0.
    a.L("idle").jmp("idle", I.NDB).dbclr()
    a.mbr(0, UH_OP).cmp(0, 0).jmp("tok", I.Z)                    # 0: a transaction
    a.cmp(0, 3).jmp("resu", I.Z)                                 # 3: resume
    a.cmp(0, 1).jmp("keep", I.NZ)                                # 2: keep-alive; 1 falls through: reset
    # Bus reset: SE0 for n * 256 cycles, and then the end of an EOP.
    a.mbr(5, UH_N)                                               # r5 = the length, in 256-cycle turns
    a.set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)                 # drive both lines low: SE0
    a.L("rl").delay(254).djnz(5, "rl")                           # 256 cycles a turn
    a.jmp("rel")
    # Keep-alive, or the end of a resume: SE0 for 2 bit times, then J for 1 bit, then release.
    a.L("keep").set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)
    a.L("eop").delay(2 * B - 3)
    a.L("rel").set(jp, I.SRC_1).delay(max(0, B - 2)).setoe(jp, 0).setoe(kp, 0).jmp("fin0")
    # resume: K, then the EOP that "keep" ends with. J to K changes two pins: only a TX unit
    # write with DIFF changes both in one edge, with no glitch of SE0 between them.
    a.L("resu").mbr(5, UH_N)
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(diff=1, d2pin=kp))         # plain differential, no NRZI / EOP
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(nbits=1))                # a single bit
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_NONE)                         # and keep driving after it
    a.setoe(jp, 1).setoe(kp, 1)                                  # J, as the pull-ups had it
    a.ustart(I.M_TX, 0).uwait(I.M_TX)                            # a 0 bit: both lines flip to K together
    a.L("rk").delay(254).djnz(5, "rk")                           # K for n * 256 cycles
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=kp, stuffn=6))   # back to packet coding
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(chain=1))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
    a.set(kp, I.SRC_0).jmp("eop")                                # K -> SE0, then the EOP's J and release
    # token
    a.L("tok")                      # the CRC unit is off or on RX here: neither adds to a TX packet
    _tx_on(a, jp, kp)
    # The 3 bytes of the token, from DMEM through TXM. The host side worked out its CRC5.
    a.ucfg(I.TXM, I.MV_PTR, UH_TOK).ucfg(I.TXM, I.MV_GO, 3)
    a.uwait(I.M_TX)                                              # CHAIN sends them; EOP when TXF runs out
    # No data PID means an IN, and the next thing is the device's answer.
    a.mbr(6, UH_DPID).cmp(6, 0).jmp("rxp", I.Z)
    # data stage (OUT / SETUP)
    _crc_tx(a)                                                   # the data packet gets CRC16, appended by hardware
    a.delay(2 * B)                                               # the gap between token and data
    _tx_on(a, jp, kp)
    a.fput(6, I.FIFO_TX)                                         # the DATA PID
    a.mbr(5, UH_N).cmp(5, 0).jmp("dte", I.Z)                     # r5 = data bytes; none: zero-length
    a.ucfg(I.TXM, I.MV_PTR, UH_DATA).ucfg(I.TXM, I.MV_GO, I.R(5))   # TXM: the data -> TXF
    a.L("dte").uwait(I.M_TX)                                     # data, CRC16 and EOP sent
    # receive a packet (handshake or data) into 0x80. Its start must come in time.
    a.L("rxp")
    _crc_rx(a, CRC16_POLY, CRC16_INIT)
    _rx_go(a, jp)
    # The status starts as OK.
    a.mov(5, timeout).mov(1, UH_OK)
    # K means a packet has started. Nothing in time means a time-out.
    a.L("rw").jpin(kp, 1, "sop").djnz(5, "rw")                   # 2 cycles a pass
    a.ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).mov(1, UH_TIMEOUT).jmp("done")   # no answer: time-out
    # The first byte decides. Only a good DATA PID in answer to an IN goes on to the ACK path, where
    # time is short: the other checks are made here, as bytes arrive.
    # The PID is stored by the firmware, the rest of the packet by RXM. A byte can reach the FIFO
    # between finding it empty and seeing the EOP: a PID that came with the EOP is read once more
    # (sope); bytes after the PID are RXM's, and the end ("hs") waits until it has stored them all.
    a.L("sop").fget(0, I.FIFO_RX, nb=1).jmp("s1", I.NC).jmp("sope", I.ERX).jmp("sop")   # wait for the first byte
    a.L("s1").mov(3, UH_RPID).st(0, 3)                           # the PID
    a.ucfg(I.RXM, I.MV_PTR, UH_RPID + 1).ucfg(I.RXM, I.MV_GO, 255)   # RXM: the rest of the packet
    a.cmp(6, 0).jmp("rx", I.NZ)                                  # after an OUT / SETUP, no ACK to give
    a.cmp(0, PID_DATA0).jmp("rd", I.Z).cmp(0, PID_DATA1).jmp("rd", I.Z)   # an IN answered with data
    # anything else gets no ACK: with the CRC unit off the check below cannot pass, and that costs
    # the ACK no time. One byte is a handshake; more is a bad packet.
    a.L("rx").ucfg(I.CRC, I.CRC_CTRL, 0)
    a.L("rd").jmp("rd", I.NERX)                                  # RXM stores bytes until the EOP
    a.urd(4, I.CRC, I.CRC_VAL)                                   # r4 = the CRC register
    _crc_good(a, CRC16_RES, CRC16_DRIB, "rc2", "rn")
    # PID and CRC good. ACK it if the stuffing is good too.
    a.L("rc2").urd(4, I.RX, I.RX_STAT).and_(4, I.RX_ST_SERR).jmp("bad", I.NZ)
    if gap:
        a.delay(gap)
    _tx_on(a, jp, kp)
    a.mov(0, PID_ACK).fput(0, I.FIFO_TX).uwait(I.M_TX)           # the ACK handshake
    a.jmp("hs")
    a.L("sope").fget(0, I.FIFO_RX, nb=1).jmp("s1", I.NC)         # a PID that came with the EOP: judge it now
    a.mov(1, UH_BADPKT).mov(0, 0).mbw(0, UH_RXLEN).evt(I.DET_RX, I.EV_CLR).jmp("done")   # an EOP and no byte
    # A packet with no ACK: one byte (only the PID: RXM stored nothing) is a handshake, reported
    # as OK. Anything else is a bad packet.
    a.L("rn").jmp("rn", I.RXF_NEMPTY).urd(4, I.RXM, I.MV_RD_CNT).cmp(4, 0).jmp("hs", I.Z)
    a.L("bad").mov(1, UH_BADPKT)
    # RXM stores what is left; the length is the PID and what RXM stored.
    a.L("hs").jmp("hs", I.RXF_NEMPTY)
    a.urd(0, I.RXM, I.MV_RD_CNT).add(0, 1).mbw(0, UH_RXLEN).ucfg(I.RXM, I.MV_GO, 0)
    a.evt(I.DET_RX, I.EV_CLR)
    a.L("done").mbw(1, UH_STATUS)                                # the status for the host
    a.L("fin0").host_irq().jmp("idle")
    return a
