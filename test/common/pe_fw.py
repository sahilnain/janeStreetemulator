# SPDX-License-Identifier: Apache-2.0
"""
pe_fw: v2 firmware used by the top-level tests.

What this file does
-------------------
* `uart_consts(B, N, K, S)`: the timing constants of docs/spec/blocks.md
  Section 6 (detector DUR values, RX FIRST, ERX check delay ADLY).
* `uart(...)`: the full-duplex UART program of docs/spec/isa.md Section 5.
  The host writes TXF and reads RXF; a byte with a bad stop bit is dropped.
  Options let a test turn the TX or RX side off.

k = 2: detector-to-unit latency (pin synchroniser), blocks.md Section 5.3.
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
    c = uart_consts(B if B_rx is None else B_rx, N, K, S)
    a = Asm()
    if tx:
        a.ucfg(I.TX, I.TX_PINS, I.tx_pins(txd))
        a.ucfg(I.TX, I.TX_BIT, B)
        a.ucfg(I.TX, I.TX_FRAME, 0)
        a.ucfg(I.ETX_U, I.D_AFTER, I.A_DRIVE1 if stop_lvl else I.A_DRIVE0)
    if rx:
        a.ucfg(I.RX, I.RX_PINS, I.rx_pins(rxd))
        a.ucfg(I.RX, I.RX_BIT, c["B"])
        a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(hold=1))
        a.ucfg(I.RX, I.RX_VOTE, I.rx_vote(N, K))
        a.ucfg(I.RX, I.RX_STEP, c["S"])
        a.ucfg(I.RX, I.RX_FIRST, c["FIRST"])
        a.ucfg(I.ERX_U, I.D_COND, I.cond(rxd, I.FALL, start=1, autoarm=1))
        a.ucfg(I.ERX_U, I.D_DUR, c["DUR"])
        a.ucfg(I.ERX_U, I.D_AFTER, I.erx_after(1, 1))
        a.ucfg(I.ERX_U, I.D_ADLY, c["ADLY"])
        a.evt(I.DET_RX, I.EV_ARM)
    if tx:
        a.set(txd, I.SRC_1)
        a.setoe(txd, 1)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # line idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)
    a.mov(7, 0)                                                  # TX state
    a.L("loop")
    if tx:
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
        a.L("tx_run")
        a.jmp("rx_chk", I.NETX)                                  # TX not started yet
        a.evt(I.DET_TX, I.EV_CLR)
        a.ucfg(I.ETX_U, I.D_COND, I.cond(txd, I.LEVEL, 1))       # stop + idle >= 1 bit
        a.ucfg(I.ETX_U, I.D_DUR, B)
        a.evt(I.DET_TX, I.EV_ARM)                                # deferred to after stop edge
        a.mov(7, 0)
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
    idle, active = (0, 1) if inv else (1, 0)
    c = uart_consts(B, N, K, S)
    wbits = bits + (parity != PAR_NONE)
    stop = B * stop_b2 // 2
    pbit = 1 if msb else 1 << bits              # the parity bit in the word
    mask = (1 << bits) - 1
    a = Asm()
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
    a.L("loop")
    a.jmp("rx_chk", I.NETX)
    a.jmp("brk", I.DB)
    a.jmp("rx_chk", I.TXF_EMPTY)
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
    a.evt(I.DET_TX, I.EV_CLR)
    a.set(txd, I.SRC_1 if inv else I.SRC_0)      # start bit: the next ustart is B cycles later
    a.delay(B - 3)
    a.ustart(I.M_TX, I.R(0))
    a.evt(I.DET_TX, I.EV_ARM)                    # deferred to after the stop edge
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
    a.L("brl").delay(B - 4).djnz(1, "brl")
    a.set(txd, I.SRC_0 if inv else I.SRC_1)
    a.evt(I.DET_TX, I.EV_ARM).jmp("rx_chk")
    return a


# ------------------------------------------------------------------ SPI
SPI_PINS = dict(sck=2, mosi=3, miso=4, cs=5)


def spi_master_words(B=2, cpol=0, cpha=0, msb=1, sdly=3, bits=16, sck=2, mosi=3, miso=4, cs=5):
    """SPI master for words of 1-16 bits, DMEM-driven: dmem[0] = n words,
    then two bytes per word (low first) from dmem[1]; n replies, two bytes
    each, at dmem[0x80]. Owners: SCK, MOSI -> TX unit."""
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    a.mov(1, 0).ld(2, 1, inc=1).mov(3, 0x80)
    a.set(cs, I.SRC_0, dly=1)
    a.L("word")
    a.ld(0, 1, inc=1).ld(4, 1, inc=1).shl(4, 8).or_(0, I.R(4))
    a.ustart(I.M_BOTH, I.R(0))
    a.uwait(I.M_BOTH)
    a.urd(4, I.RX, I.RX_DATA)
    a.st(4, 3, inc=1).shr(4, 8).st(4, 3, inc=1)
    a.djnz(2, "word")
    a.set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master(B=2, cpol=0, cpha=0, msb=1, sdly=3, sck=2, mosi=3, miso=4, cs=5):
    """SPI master, DMEM-driven (H1): the host writes dmem[0] = n, dmem[1..n] =
    data and rings DOORBELL; the firmware clocks n bytes with CS low and
    stores the received bytes in dmem[0x80..], then raises the host IRQ.
    Owners: SCK, MOSI -> TX unit."""
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(mosi, 1, sck, 1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(cpol, cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(miso))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0))          # auto-push
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.set(sck, I.SRC_1 if cpol else I.SRC_0).setoe(sck, 1)
    a.set(mosi, I.SRC_0).setoe(mosi, 1)
    a.set(cs, I.SRC_1).setoe(cs, 1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    a.mov(1, 0).ld(2, 1, inc=1)                                 # r2 = n, r1 -> data
    a.mov(3, 0x80)
    a.set(cs, I.SRC_0, dly=1)
    a.L("byte")
    a.ld(0, 1, inc=1)
    a.ustart(I.M_BOTH, I.R(0))                                  # one byte out and in
    a.uwait(I.M_BOTH)
    a.fget(4, I.FIFO_RX)
    a.st(4, 3, inc=1)
    a.djnz(2, "byte")
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
    lead = cpol                                         # EEDGE of the leading edge
    samp_edge = lead if cpha == 0 else 1 - lead
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    # MISO changes on the edge that does not sample (CPHA 1: bit 0 on the first leading edge)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=0, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    for d in (I.ETX_U, I.ERX_U):
        a.ucfg(d, I.D_COND, I.cond(cs, I.LEVEL, 0, start=1, autoarm=1))
        a.ucfg(d, I.D_DUR, 1)
    a.setoe(miso, 1)
    a.L("idle").wait(cs, 1)
    a.evt(I.DET_TX, I.EV_ARM).evt(I.DET_RX, I.EV_ARM)
    a.wait(cs, 0)
    a.L("run").jpin(cs, 1, "end")
    a.jmp("run")
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
    a.mov(1, 0).ld(2, 1, inc=1).mov(5, I.R(2)).mov(3, 0x80).mov(6, 0)    # r2 tx left, r5 rx left
    a.L("pre").ld(0, 1, inc=1).fput(0, I.FIFO_TX).sub(2, 1).jmp("go", I.Z)
    a.jmp("go", I.TXF_FULL).jmp("pre")
    a.L("go").set(cs, I.SRC_0, dly=1).ustart(I.M_BOTH)
    a.cmp(2, 0).jmp("drain", I.Z)
    # While bytes are left to send, the last reply cannot have come: no test of r5 or r2 here
    a.L("run").fget(4, I.FIFO_RX, nb=1).jmp("no_rx", I.C)
    a.st(4, 3, inc=1).sub(5, 1)
    a.L("no_rx").jmp("late", I.TXU_IDLE).jmp("run", I.TXF_FULL)
    a.ld(0, 1, inc=1).fput(0, I.FIFO_TX).djnz(2, "run").jmp("drain")
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
    a.L("end").uwait(I.M_TX).ustop(I.M_RX)
    a.set(cs, I.SRC_1)
    a.mov(1, 0x7F).st(6, 1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_master_3wire(B=8, cpol=0, cpha=0, msb=1, sdly=3, sck=2, dio=3, cs=5):
    """3-wire SPI master: one data pin, DMEM-driven: dmem[0] = n_write, dmem[1] = n_read,
    dmem[2..] = write bytes; read bytes at dmem[0x80..]. All writes, then all reads, in one
    CS frame; the pin is released in between. Owners: SCK, DIO -> TX unit."""
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
    a.mov(1, 0).ld(2, 1, inc=1).ld(5, 1, inc=1).mov(3, 0x80)
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio, 1, sck, 1))
    a.setoe(dio, 1)
    a.set(cs, I.SRC_0, dly=1)
    a.cmp(2, 0).jmp("rd", I.Z)
    a.L("wr").ld(0, 1, inc=1).ustart(I.M_TX, I.R(0)).uwait(I.M_TX).djnz(2, "wr")
    a.L("rd").setoe(dio, 0)                                     # turnaround
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, 0, sck, 1))            # the clock only
    a.cmp(5, 0).jmp("fin", I.Z).delay(B)
    a.L("rl").ustart(I.M_BOTH, 0).uwait(I.M_BOTH)
    a.fget(4, I.FIFO_RX).st(4, 3, inc=1).djnz(5, "rl")
    a.L("fin").set(cs, I.SRC_1)
    a.host_irq()
    a.jmp("idle")
    return a


def spi_slave_3wire(cpol=0, cpha=0, msb=1, sck=2, dio=3, cs=5):
    """3-wire SPI slave. The first byte of a frame is a command, MSB = 1 for a read. A write:
    the next bytes go to RXF. A read: the next bytes come from TXF on DIO (0xFF if empty).
    The command goes to RXF as well. SCK half period at least 16 cycles, as firmware turns
    the pin around between the command and the first read bit. Owners: DIO -> TX unit."""
    lead = cpol
    samp_edge = lead if cpha == 0 else 1 - lead
    a = Asm()
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(dio))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(dio))
    a.ucfg(I.TX, I.TX_FRAME, msb)
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.L("idle").setoe(dio, 0).wait(cs, 1).wait(cs, 0)
    a.ustart(I.M_RX)
    a.call("rx1").jmp("end", I.C)
    a.fput(0, I.FIFO_RX)
    a.and_(0, 0x80).jmp("read", I.NZ)
    a.L("wl").ustart(I.M_RX).call("rx1").jmp("end", I.C)
    a.fput(0, I.FIFO_RX).jmp("wl")
    # the master drives until it has seen its last bit out: take the pin only after that
    a.L("read").wait(sck, cpol).delay(10).setoe(dio, 1)
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
    lead = cpol
    samp_edge = lead if cpha == 0 else 1 - lead
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(miso))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(msb, nbits=bits))
    a.ucfg(I.TX, I.TX_TICK, I.tx_tick(1, sck, 1 - samp_edge, lead=cpha))
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(mosi))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb, hold=1, nbits=bits))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_EXT, sck, samp_edge, 0))
    a.setoe(miso, 1)
    a.L("idle").call("next").wait(cs, 1).wait(cs, 0)
    # The next word starts at once, and the word just received is stored after that: RX
    # keeps DATA and RXRDY across a start, and the next edge is only half a period away.
    a.L("word").ustart(I.M_BOTH, I.R(0))
    a.call("store").call("next")
    a.L("w").jpin(cs, 1, "end").jmp("w", I.TXU_BUSY).jmp("word")
    a.L("end").ustop(I.M_BOTH).call("store").jmp("idle")
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
def i2c_master(B=126, scl=6, sda=7, odly=None, sdly=2, addr10=False, arb=False, hs_B=None, master_code=0x08):
    """I2C master, DMEM-driven. Request: dmem[0] = 7-bit address, dmem[1] = n_write, dmem[2] =
    n_read, dmem[3..] = write data. The firmware sends START, address+W and the write bytes (if
    any), then, if n_read > 0, a repeated START, address+R and reads n_read bytes (ACK all but
    the last, NACK the last), then STOP. Result: dmem[0x80..] = read bytes, dmem[0x7F] = 0 ok /
    1 address NACK / 2 data NACK / 3 arbitration lost; then host IRQ.
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
    hs_B     HS mode: the master code at B (F/S speed), which nobody acknowledges, then a repeated
             START, and all data bytes at hs_B. ACK clocks stay at F/S length, which is legal."""
    assert not (arb and hs_B), "HS mode has one master after the master code"
    H = max(1, B // 2)
    Q = max(1, B // 4)
    if odly is None:
        odly = min(15, max(1, (hs_B or B) // 8))
    a = Asm()
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(0, den=0, cpin=scl, cen=1))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, 1)
    a.ucfg(I.TX, I.TX_CLK, I.tx_clk(0, 0, stretch=1))
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(msb=1, hold=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_TXTICK, sdly=sdly))
    a.ucfg(I.RX, I.RX_ODLY, odly)
    a.set(scl, I.SRC_1).set(sda, I.SRC_1)
    a.L("idle").jmp("idle", I.NDB)
    a.dbclr()
    a.mov(1, 0).ld(4, 1, inc=1).ld(2, 1, inc=1).ld(7, 1, inc=1)    # addr, n_w, n_r
    if addr10:
        a.mov(0, 0x7E).ld(0, 0).shl(0, 8).or_(4, I.R(0))           # r4 = the 10-bit address
    a.mov(3, 0x80).mov(6, 0)
    if arb:
        a.L("free").jpin(scl, 0, "free").jpin(sda, 0, "free").delay(H)
        a.jpin(scl, 0, "free").jpin(sda, 0, "free")
    if hs_B:
        a.ucfg(I.TX, I.TX_BIT, B)
        a.call("start").mov(0, master_code).call("wbyte")           # no acknowledge expected
        a.ucfg(I.TX, I.TX_BIT, hs_B)
    a.call("start")
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
    a.L("fin").call("stop")
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
    a.jmp("idle")
    # START (also repeated START): SDA falls while SCL is high
    a.L("start").set(sda, I.SRC_1).delay(H).set(scl, I.SRC_1).wait(scl, 1)
    a.delay(H).set(sda, I.SRC_0).delay(H).set(scl, I.SRC_0).delay(Q).ret()
    # STOP: SDA rises while SCL is high
    a.L("stop").set(sda, I.SRC_0).delay(H).set(scl, I.SRC_1).wait(scl, 1)
    a.delay(H).set(sda, I.SRC_1).delay(H).ret()
    # write r0; r5 = 0 ACK / 1 NACK / 2 arbitration lost
    if arb:
        a.L("wbyte").shl(0, 8).mov(5, 8)
        a.L("wb").shl(0, 1).set(sda, I.SRC_C).delay(H - Q)     # C = the bit to send; low = H with the Q below
        a.set(scl, I.SRC_1).wait(scl, 1).delay(H)
        a.jmp("wbk", I.NC).jpin(sda, 0, "wlost")               # sent 1, read 0: lost
        a.L("wbk").set(scl, I.SRC_0).delay(Q).djnz(5, "wb")
    else:
        a.L("wbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda, sda, 1))
        a.ustart(I.M_BOTH, I.R(0))
        a.uwait(I.M_BOTH)
        a.urd(5, I.RX, I.RX_DATA)                               # readback (clears RXRDY)
    a.set(sda, I.SRC_1).delay(H)                                # release SDA for the ACK
    a.set(scl, I.SRC_1).wait(scl, 1).delay(H)
    a.mov(5, 0).jpin(sda, 0, "wa").mov(5, 1)
    a.L("wa").set(scl, I.SRC_0).delay(Q).ret()
    if arb:
        a.L("wlost").set(sda, I.SRC_1).set(scl, I.SRC_1).mov(5, 2).ret()
    # read into r0; r5 = 0 -> ACK, 1 -> NACK
    a.L("rbyte").ucfg(I.RX, I.RX_PINS, I.rx_pins(sda))
    a.set(sda, I.SRC_1)
    a.ustart(I.M_BOTH)
    a.uwait(I.M_BOTH)
    a.urd(0, I.RX, I.RX_DATA)
    a.cmp(5, 1).set(sda, I.SRC_NC)                             # Z... use C: r5 < 1 -> C=1 -> drive 0
    a.delay(H).set(scl, I.SRC_1).wait(scl, 1).delay(H)
    a.set(scl, I.SRC_0).delay(Q).set(sda, I.SRC_1).ret()
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
    a = Asm()
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
    a.L("idle").set(scl, I.SRC_1).evt(I.DET_RX, I.EV_ARM)
    a.evt(I.DET_RX, I.EV_WAITC)                                 # START or STOP
    a.jpin(sda, 1, "stop")
    a.L("addr").evt(I.DET_RX, I.EV_ARM)
    a.call("rx8").jmp("ev", I.C)
    if general_call:
        a.cmp(0, 0).jmp("gc", I.Z)
    if addr10 is None:
        a.mov(1, I.R(0)).shr(1, 1).cmp(1, addr).jmp("idle", I.NZ)  # not us: wait for START
        a.call("ack")
        a.and_(0, 1).jmp("rd", I.NZ)
    else:
        first = 0xF0 | ((addr10 >> 7) & 0x06)
        a.mov(1, I.R(0)).and_(1, 0xFE).cmp(1, first).jmp("idle", I.NZ)
        a.and_(0, 1).jmp("r10", I.NZ)
        a.call("ack")
        a.call("rx8").jmp("ev", I.C)
        a.cmp(0, addr10 & 0xFF).jmp("nom", I.NZ)
        a.call("ack").mov(6, 1).jmp("wstart")
        a.L("nom").mov(6, 0).jmp("idle")
        a.L("r10").cmp(6, 1).jmp("idle", I.NZ)                  # a read must follow our write
        a.call("ack").jmp("rd")
        a.L("wstart")
    a.mov(2, 1)                                                 # r2 = 1: next byte is the pointer
    a.L("wr").call("rx8").jmp("ev", I.C)
    a.call("ack")
    a.cmp(2, 1).jmp("wd", I.NZ)
    a.mov(3, I.R(0)).mov(2, 0).jmp("wr")
    a.L("wd").st(0, 3, inc=1).jmp("wr")
    if general_call:
        a.L("gc").call("ack").mov(3, I2C_GC_MEM).mov(2, 0).host_irq().jmp("wr")
    a.L("rd").ld(0, 3, inc=1)
    a.ustart(I.M_TX, I.R(0))                                    # bit 7 now, then on SCL falls
    a.delay(12)                                                 # data setup: 250 ns, the longest limit
    a.set(scl, I.SRC_1)                                         # the data is ready: release
    a.uwait(I.M_TX)
    a.set(sda, I.SRC_1)                                         # release for the master ACK
    a.wait(scl, 1).jpin(sda, 1, "nak")
    a.wait(scl, 0).set(scl, I.SRC_0).jmp("rd")                  # hold SCL while fetching
    a.L("nak").evt(I.DET_RX, I.EV_CLR).jmp("idle")
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
    a.L("rx_ev").mov(0, 0).cmp(0, 1).ret()                      # 0 - 1 borrows -> C = 1
    # ACK: SDA low for the 9th clock, then hold SCL low until the next byte is set up
    a.L("ack").wait(scl, 0).set(sda, I.SRC_0)
    a.wait(scl, 1).wait(scl, 0).set(scl, I.SRC_0).set(sda, I.SRC_1)
    if hold:
        a.delay(hold)
    a.ret()
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
UH_OP, UH_TOK0, UH_TOK1, UH_TOK2, UH_DPID, UH_N, UH_RPID, UH_STATUS, UH_RXLEN = range(9)
UH_OK, UH_TIMEOUT, UH_BADPKT = 0, 1, 2


def _usb_setup(a, B, jp, kp):
    """Line config shared by both roles. jp: the pin that is high in J (full
    speed D+, low speed D-), kp: the other one. Owners: both -> TX unit."""
    a.ucfg(I.TX, I.TX_PINS, I.tx_pins(jp))
    a.ucfg(I.TX, I.TX_BIT, B)
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(chain=1))
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=kp, stuffn=6))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
    a.ucfg(I.RX, I.RX_PINS, I.rx_pins(jp, i2pin=kp, diff=1))
    a.ucfg(I.RX, I.RX_BIT, B)
    a.ucfg(I.RX, I.RX_FRAME, I.rx_frame(chain=1))
    a.ucfg(I.RX, I.RX_TICK, I.rx_tick(I.SRC_DPLL))
    a.ucfg(I.RX, I.RX_ODLY, I.rx_line(nrzi=1, hunt=1, stuffn=6))
    a.ucfg(I.ERX_U, I.D_COND, I.cond(jp, I.LEVEL, 0, qpin=kp, qen=1, qlvl=0, stop=1))   # SE0
    a.ucfg(I.ERX_U, I.D_DUR, max(2, B // 4))
    a.set(jp, I.SRC_1).set(kp, I.SRC_0)                          # J, not driven yet


def _crc_rx(a, poly, init):
    a.ucfg(I.CRC, I.CRC_CTRL, I.crc_ctrl(I.CRC_RX, skip=8))
    a.ucfg(I.CRC, I.CRC_POLY, poly)
    a.ucfg(I.CRC, I.CRC_INIT, init)


def _crc_tx(a, skip=16):
    a.ucfg(I.CRC, I.CRC_CTRL, I.crc_ctrl(I.CRC_TX, append=1, skip=skip))   # not sync, not PID
    a.ucfg(I.CRC, I.CRC_POLY, CRC16_POLY)
    a.ucfg(I.CRC, I.CRC_INIT, CRC16_INIT)


def _rx_go(a, jp):
    """Hunt for the next packet: the bus has left SE0, ERX armed, RX started."""
    a.wait(jp, 1).evt(I.DET_RX, I.EV_ARM).ustart(I.M_RX)


def _tx_on(a, jp, kp):
    a.setoe(jp, 1).setoe(kp, 1).ustart(I.M_TX, USB_SYNC)


def usb_device(B=4, jp=0, kp=1, turn=None):
    """USB device at transaction level, low or full speed. The host side sets
    the address and handshakes in DMEM and owns the higher layers (enumeration,
    data toggles, endpoints). See UD_* for the map; UD_EVENT says what happened,
    with a controller IRQ. `turn`: extra cycles before answering (B = 32: the
    answer must not come before 2 bit times).
    Suspend: ETX times the idle bus. CSTAT.ETX set means J for at least USB_IDLE cycles
    (1.37 ms) since the last packet, and any packet clears it. Read it twice, 1.7 ms
    apart: still set means suspend (3 ms of idle). Resume shows as K on PIN_IN; the firmware
    keeps hunting all along, so the next packet is received normally. Remote wakeup is
    not implemented: the device must not advertise it."""
    if turn is None:
        turn = 0 if B <= 8 else 5 * B
    a = Asm()
    _usb_setup(a, B, jp, kp)
    a.ucfg(I.ETX_U, I.D_COND, I.cond(jp, I.LEVEL, 1))
    a.ucfg(I.ETX_U, I.D_DUR, USB_IDLE)
    a.L("idle").fget(0, I.FIFO_RX, nb=1).jmp("idle", I.NC)       # drain
    a.evt(I.DET_TX, I.EV_CLR).evt(I.DET_TX, I.EV_ARM)            # the idle timer starts again
    a.mov(3, UD_ADDR).ld(7, 3)
    a.mov(3, UD_INLEN).ld(5, 3)
    _crc_rx(a, CRC5_POLY, CRC5_INIT)
    _rx_go(a, jp)
    a.evt(I.DET_RX, I.EV_WAITC)                                  # SE0: RX is stopped
    a.mov(6, 0).fget(6, I.FIFO_RX, nb=1).fget(1, I.FIFO_RX, nb=1).fget(2, I.FIFO_RX, nb=1)
    a.urd(4, I.CRC, I.CRC_VAL)
    a.cmp(6, PID_IN).jmp("in_tok", I.Z)                          # no time to spare here
    a.cmp(6, 0).jmp("short", I.Z)
    # OUT / SETUP: the data packet follows soon, so hunt first and check after.
    # No bit reaches the CRC unit until the hunt ends, so its setup can follow.
    _rx_go(a, jp)
    _crc_rx(a, CRC16_POLY, CRC16_INIT)
    a.cmp(4, CRC5_RES).jmp("abort", I.NZ)
    a.mov(4, I.R(1)).and_(4, 0x7F).cmp(4, I.R(7)).jmp("abort", I.NZ)
    a.cmp(6, PID_OUT).jmp("o_ok", I.Z)
    a.cmp(6, PID_SETUP).jmp("abort", I.NZ)
    a.L("o_ok").mov(3, UD_TOKEN).st(6, 3)
    a.and_(2, 7).shl(2, 1).shr(1, 7).or_(2, I.R(1))
    a.mov(3, UD_EP).st(2, 3)
    a.mov(3, UD_OUTHS).ld(1, 3)                                  # handshake ready in r1
    a.mov(3, UD_RXDATA)
    a.L("rl").jmp("re", I.ERX).fget(0, I.FIFO_RX, nb=1).jmp("rl", I.C)
    a.st(0, 3, inc=1).jmp("rl")
    a.L("re").evt(I.DET_RX, I.EV_CLR)
    a.L("rd").fget(0, I.FIFO_RX, nb=1).jmp("rc", I.C).st(0, 3, inc=1).jmp("rd")
    a.L("rc").urd(4, I.CRC, I.CRC_VAL).cmp(4, CRC16_RES).jmp("idle", I.NZ)
    if turn:
        a.delay(turn)
    _tx_on(a, jp, kp)
    a.fput(1, I.FIFO_TX)
    a.ucfg(I.CRC, I.CRC_CTRL, 0)
    a.uwait(I.M_TX)
    a.mov(0, I.R(3)).sub(0, UD_RXDATA).mov(3, UD_RXLEN).st(0, 3)
    a.mov(0, EV_OUT).jmp("ev")
    # IN: answer within 7.5 bit times. Sync goes out first; it is the same for both
    # answers, and it gives the time to decide and to set up the CRC.
    a.L("in_tok").cmp(4, CRC5_RES).jmp("idle", I.NZ)
    a.and_(1, 0x7F).cmp(1, I.R(7)).jmp("idle", I.NZ)
    if turn:
        a.delay(turn)
    _tx_on(a, jp, kp)
    a.cmp(5, 0).jmp("nak", I.Z)
    # INIT is written 8 cycles after the sync started: the bits already gone do not count.
    _crc_tx(a, skip=16 - (8 // B + 1))
    a.mov(3, UD_INPID).ld(0, 3).fput(0, I.FIFO_TX)
    a.mov(3, UD_INDATA).sub(5, 1).jmp("txe", I.Z)
    a.L("txl").ld(0, 3, inc=1).fput(0, I.FIFO_TX).djnz(5, "txl")
    a.L("txe").uwait(I.M_TX)
    # the host's handshake, with a timeout (the host answers within 16 bit times)
    a.ucfg(I.CRC, I.CRC_CTRL, 0)
    _rx_go(a, jp)
    a.mov(5, 10 * B)                                             # 40 bit times: 16 + one handshake
    a.L("ha").jmp("hg", I.ERX).djnz(5, "ha")
    a.ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).mov(0, EV_IN_NOACK).jmp("ev")
    a.L("hg").evt(I.DET_RX, I.EV_CLR).fget(1, I.FIFO_RX, nb=1)
    a.mov(0, EV_IN_NOACK).cmp(1, PID_ACK).jmp("ev", I.NZ)
    a.mov(3, UD_INLEN).mov(0, 0).st(0, 3).mov(0, EV_IN_ACK).jmp("ev")
    a.L("nak")
    a.ucfg(I.CRC, I.CRC_CTRL, 0)
    a.mov(3, UD_INHS).ld(0, 3).fput(0, I.FIFO_TX)
    a.uwait(I.M_TX).jmp("idle")
    a.L("abort").ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).evt(I.DET_RX, I.EV_CLR).jmp("idle")
    # A packet shorter than a token, or a bus reset: SE0 still there after 2.5 us.
    a.L("short").delay(130).jpin(jp, 1, "idle").mov(0, EV_RESET)
    a.L("ev").mov(3, UD_EVENT).st(0, 3).host_irq().jmp("idle")
    return a


def usb_host(B=4, jp=0, kp=1, gap=None, timeout=None):
    """USB host at transaction level, low or full speed. dmem: UH_OP (0
    transaction, 1 bus reset for UH_N * 256 cycles, 2 EOP = keep-alive, 3 resume: K for
    UH_N * 256 cycles, then an EOP; suspend = no transactions), UH_TOK0-2 = the
    token with its CRC5, UH_DPID = data PID for OUT / SETUP (0 for IN), UH_N = data
    bytes at 0x10. Result: UH_RPID = handshake or data PID, UH_STATUS, UH_RXLEN = bytes
    received with the PID, at 0x80; controller IRQ. An IN gets an ACK when its
    CRC is good."""
    if gap is None:
        gap = max(0, 4 * B - 20)        # >= 2 bit times after SE0 ends, measured from SE0
    if timeout is None:
        timeout = max(8, 3 * B)
    a = Asm()
    _usb_setup(a, B, jp, kp)
    a.L("idle").jmp("idle", I.NDB).dbclr()
    a.mov(3, UH_OP).ld(0, 3).cmp(0, 0).jmp("tok", I.Z)
    a.cmp(0, 3).jmp("resu", I.Z)
    a.cmp(0, 1).jmp("keep", I.NZ)
    a.mov(3, UH_N).ld(5, 3)
    a.set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)
    a.L("rl").delay(250).djnz(5, "rl")
    a.jmp("rel")
    a.L("keep").set(jp, I.SRC_0).setoe(jp, 1).setoe(kp, 1)
    a.L("eop").delay(2 * B - 6)
    a.L("rel").set(jp, I.SRC_1).delay(max(0, B - 3)).setoe(jp, 0).setoe(kp, 0).jmp("fin0")
    # resume: K, then the EOP that "keep" ends with. J to K changes two pins: only a TX unit
    # write with DIFF changes both in one edge, with no glitch of SE0 between them.
    a.L("resu").mov(3, UH_N).ld(5, 3)
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(diff=1, d2pin=kp))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(nbits=1))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_NONE)
    a.setoe(jp, 1).setoe(kp, 1)                                  # J, as the pull-ups had it
    a.ustart(I.M_TX, 0).uwait(I.M_TX)
    a.L("rk").delay(250).djnz(5, "rk")
    a.ucfg(I.TX, I.TX_LINE, I.tx_line(nrzi=1, eop=1, diff=1, d2pin=kp, stuffn=6))
    a.ucfg(I.TX, I.TX_FRAME, I.tx_frame(chain=1))
    a.ucfg(I.ETX_U, I.D_AFTER, I.A_RELEASE)
    a.set(kp, I.SRC_0).jmp("eop")
    # token
    a.L("tok").ucfg(I.CRC, I.CRC_CTRL, 0)
    _tx_on(a, jp, kp)
    a.mov(3, UH_TOK0).ld(0, 3, inc=1).fput(0, I.FIFO_TX).ld(0, 3, inc=1).fput(0, I.FIFO_TX)
    a.ld(0, 3).fput(0, I.FIFO_TX).uwait(I.M_TX)
    a.mov(3, UH_DPID).ld(6, 3).cmp(6, 0).jmp("rxp", I.Z)
    # data stage (OUT / SETUP)
    _crc_tx(a)
    a.delay(2 * B)
    _tx_on(a, jp, kp)
    a.fput(6, I.FIFO_TX)
    a.mov(3, UH_N).ld(5, 3).mov(3, 0x10).cmp(5, 0).jmp("dte", I.Z)
    a.L("dtl").ld(0, 3, inc=1).fput(0, I.FIFO_TX).djnz(5, "dtl")
    a.L("dte").uwait(I.M_TX)
    # receive a packet (handshake or data) into 0x80
    a.L("rxp")
    _crc_rx(a, CRC16_POLY, CRC16_INIT)
    _rx_go(a, jp)
    a.mov(3, 0x80).mov(5, timeout)
    a.L("rw").jmp("rg", I.ERX).fget(0, I.FIFO_RX, nb=1).jmp("rn", I.C)
    a.st(0, 3, inc=1).mov(5, timeout).jmp("rw")
    a.L("rn").djnz(5, "rw")
    a.ustop(I.M_RX).evt(I.DET_RX, I.EV_DISARM).mov(1, UH_TIMEOUT).jmp("done")
    a.L("rg").evt(I.DET_RX, I.EV_CLR)
    a.L("rd").fget(0, I.FIFO_RX, nb=1).jmp("rc", I.C).st(0, 3, inc=1).jmp("rd")
    a.L("rc").urd(4, I.CRC, I.CRC_VAL).urd(2, I.RX, I.RX_STAT)
    a.mov(1, UH_OK).sub(3, 0x80).cmp(3, 1).jmp("hs", I.Z)        # one byte: a handshake
    a.cmp(4, CRC16_RES).jmp("bad", I.NZ)
    a.and_(2, I.RX_ST_SERR | I.RX_ST_NBITS).jmp("bad", I.NZ)
    a.cmp(6, 0).jmp("hs", I.NZ)                                  # data stage already done
    a.delay(gap)
    a.ucfg(I.CRC, I.CRC_CTRL, 0)
    _tx_on(a, jp, kp)
    a.mov(0, PID_ACK).fput(0, I.FIFO_TX).uwait(I.M_TX)
    a.jmp("hs")
    a.L("bad").mov(1, UH_BADPKT)
    a.L("hs").mov(0, I.R(3)).mov(3, UH_RXLEN).st(0, 3)
    a.mov(3, 0x80).ld(0, 3).mov(3, UH_RPID).st(0, 3)
    a.L("done").mov(3, UH_STATUS).st(1, 3)
    a.L("fin0").host_irq().jmp("idle")
    return a
