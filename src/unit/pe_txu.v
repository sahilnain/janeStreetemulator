/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_txu: TX unit (docs/spec/blocks.md Section 3). Sends one 8-bit packet
 * per start on a data pin, with an optional clock on a second pin.
 *
 * Config (ucfg TX, idx):
 *   0 PINS  [3:0] DPIN [4] DEN [11:8] CPIN [12] CEN
 *   1 BIT   [15:0] B, cycles per bit (0 -> 1)
 *   2 FRAME [0] MSB-first
 *   3 CLK   [0] CPOL [1] CPHA [2] STRETCH
 *   4 TICK  [0] EXT [7:4] EPIN [8] EEDGE (0 rising, 1 falling) [9] LEAD
 * Status (urd TX, 0): [0] busy [6:4] bit index
 *
 * Own timer (EXT = 0), start in cycle S, H = floor(B/2):
 *   data bit i written in S + i*B (on the pad S+1+i*B .. S+(i+1)*B)
 *   CPHA 0: SCK active written in S+i*B+H, idle in S+(i+1)*B (and in S)
 *   CPHA 1: SCK active written in S+i*B, idle in S+i*B+H
 *   stick (sample tick) in S+i*B+H; dtick (shift tick) in S+i*B, i = 1..7
 *   done in D = S+8*B; busy = S+1 .. D
 * Offset `o` runs 1..B within a bit, so cycle S+i*B+o has offset o of bit i
 * (o = B is the boundary to bit i+1).
 *
 * STRETCH: after SCK is written to 1 in cycle w, the timer holds for 2
 * cycles and then until pin_in[CPIN] reads 1 (I2C clock stretching). If
 * that write is the sampling edge (CPOL 0 / CPHA 0, I2C), stick is moved
 * to the cycle SCK is seen high, so the RX unit never samples while a slave
 * still holds SCL low.
 *
 * External clock (EXT = 1): bit 0 in S; bit i (1..7) in the cycle the
 * i-th selected EPIN edge is seen in pin_in; done on the 8th edge (SPI
 * slave CPHA 0: EEDGE = trailing edge).
 * With LEAD = 1 (SPI slave CPHA 1: EEDGE = leading edge): nothing is written
 * in S; bit i is written on selected edge i+1; done on the first opposite
 * edge after bit 7, so bit 7 stays until the master has sampled it.
 *
 * `stop` makes the unit idle in the next cycle without a done pulse; the
 * pins keep their last level.
 */

/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_txu: TX unit (docs/spec/blocks.md Section 3). Sends one packet of 1-16
 * bits per start on a data pin, with an optional clock on a second pin.
 *
 * Config (ucfg TX, idx):
 *   0 PINS  [3:0] DPIN [4] DEN [11:8] CPIN [12] CEN
 *   1 BIT   [15:0] B, cycles per bit (0 -> 1)
 *   2 FRAME [0] MSB-first [2] INV [8:4] NBITS (0 -> 8, >= 16 -> 16) [9] CHAIN
 *   3 CLK   [0] CPOL [1] CPHA [2] STRETCH
 *   4 TICK  [0] EXT [7:4] EPIN [8] EEDGE (0 rising, 1 falling) [9] LEAD
 *   5 LINE  [0] NRZI [1] EOP [2] DIFF [7:4] D2PIN [11:8] STUFFN (0 off)
 * Status (urd TX, 0): [0] busy [7:4] bit index
 *
 * Below, n = NBITS. Own timer (EXT = 0), start in cycle S, H = floor(B/2):
 *   data bit i written in S + i*B (on the pad S+1+i*B .. S+(i+1)*B)
 *   CPHA 0: SCK active written in S+i*B+H, idle in S+(i+1)*B (and in S)
 *   CPHA 1: SCK active written in S+i*B, idle in S+i*B+H
 *   stick (sample tick) in S+i*B+H; dtick (shift tick) in S+i*B, i = 1..n-1
 *   done in D = S+n*B; busy = S+1 .. D
 * Offset `o` runs 1..B within a bit, so cycle S+i*B+o has offset o of bit i
 * (o = B is the boundary to bit i+1).
 *
 * STRETCH: after SCK is written to 1 in cycle w, the timer holds for 2
 * cycles and then until pin_in[CPIN] reads 1 (I2C clock stretching). If
 * that write is the sampling edge (CPOL 0 / CPHA 0, I2C), stick is moved
 * to the cycle SCK is seen high, so the RX unit never samples while a slave
 * still holds SCL low.
 *
 * External clock (EXT = 1): bit 0 in S; bit i in the cycle the i-th selected
 * EPIN edge is seen in pin_in; done on the n-th edge (SPI slave CPHA 0:
 * EEDGE = trailing edge).
 * With LEAD = 1 (SPI slave CPHA 1: EEDGE = leading edge): nothing is written
 * in S; bit i is written on selected edge i+1; done on the first opposite
 * edge after bit n-1, so that bit stays until the master has sampled it.
 *
 * Line coding (own timer only; the clock outputs are not meaningful with it):
 *   NRZI    a 0 bit toggles the level, a 1 bit keeps it. The level is 1 at S.
 *   STUFFN  after N data 1s in a row one extra 0 is sent for one bit time,
 *           counted across chained words and also before the end of a packet.
 *   DIFF    D2PIN is written with the inverse of DPIN.
 *   CHAIN   when the last bit of a word ends and next_ok = 1, the next word
 *           starts in that cycle with no gap (`chain` pulse, no done); the
 *           word comes from next_data; next_crc makes it 16 bits MSB-first.
 *   EOP     when a packet ends: DPIN and D2PIN = 0 for 2*B, then DPIN = 1,
 *           D2PIN = 0 for B, then done (USB SE0, SE0, J).
 *
 * `stop` makes the unit idle in the next cycle without a done pulse; the
 * pins keep their last level.
 */

`default_nettype none

module pe_txu #(
    parameter NUM_PINS = 8
) (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

    input wire        cfg_we,
    input wire [ 2:0] cfg_idx,
    input wire [15:0] cfg_data,
    input wire [ 2:0] rd_idx,
    output reg [15:0] rd_data,

    input wire        start,
    input wire [15:0] start_data,
    input wire        stop,

    // gap-free next word (CHAIN) and the CRC unit's bit stream
    input  wire        next_ok,
    input  wire        next_crc,
    input  wire [15:0] next_data,
    output wire        chain,
    output wire        bit_v,
    output wire        bit_o,

    input wire [NUM_PINS-1:0] pin_in,

    output reg                 busy,
    output reg                 done,
    output reg                 stick,
    output reg                 dtick,
    output wire [NUM_PINS-1:0] pw_mask,
    output wire [NUM_PINS-1:0] pw_val,

    output wire                den,
    output wire [         3:0] dpin,
    output wire [NUM_PINS-1:0] d2_mask
);

  localparam WX = NUM_PINS + 16;

  // ------------------------------------------------------------- config
  reg [12:0] c_pins;
  reg [15:0] c_bit;
  reg        c_msb, c_cpol, c_cpha, c_stretch, c_ext, c_eedge, c_lead;
  reg        c_inv, c_chain, c_nrzi, c_eop, c_diff;
  reg [ 3:0] c_epin, c_d2pin, c_stuffn;
  reg [ 4:0] c_nbits;

  always @(posedge clk) begin
    if (rst) begin
      c_pins    <= 13'd0;
      c_bit     <= 16'd1;
      c_msb     <= 1'b0;
      c_cpol    <= 1'b0;
      c_cpha    <= 1'b0;
      c_stretch <= 1'b0;
      c_ext     <= 1'b0;
      c_epin    <= 4'd0;
      c_eedge   <= 1'b0;
      c_lead    <= 1'b0;
      c_inv     <= 1'b0;
      c_chain   <= 1'b0;
      c_nbits   <= 5'd0;
      c_nrzi    <= 1'b0;
      c_eop     <= 1'b0;
      c_diff    <= 1'b0;
      c_d2pin   <= 4'd0;
      c_stuffn  <= 4'd0;
    end else if (cfg_we) begin
      case (cfg_idx)
        3'd0: c_pins <= cfg_data[12:0];
        3'd1: c_bit <= cfg_data;
        3'd2: {c_chain, c_nbits, c_inv, c_msb} <= {cfg_data[9:4], cfg_data[2], cfg_data[0]};
        3'd3: {c_stretch, c_cpha, c_cpol} <= cfg_data[2:0];
        3'd4: {c_lead, c_eedge, c_epin, c_ext} <= {cfg_data[9:8], cfg_data[7:4], cfg_data[0]};
        3'd5: {c_stuffn, c_d2pin, c_diff, c_eop, c_nrzi} <= {cfg_data[11:4], cfg_data[2:0]};
        default: ;
      endcase
    end
  end

  assign dpin = c_pins[3:0];
  assign den  = c_pins[4];
  wire [3:0] cpin = c_pins[11:8];
  wire       cen = c_pins[12];

  wire [WX-1:0] dm_x = {{(WX - 1) {1'b0}}, den} << dpin;
  wire [WX-1:0] cm_x = {{(WX - 1) {1'b0}}, cen} << cpin;
  wire [WX-1:0] cin_x = {16'd0, pin_in} >> cpin;
  wire [WX-1:0] ein_x = {16'd0, pin_in} >> c_epin;
  wire cpin_in = cin_x[0];
  wire epin_in = ein_x[0];

  wire [15:0] beff = (c_bit == 16'd0) ? 16'd1 : c_bit;
  wire [15:0] half = beff >> 1;

  // n - 1 from the NBITS field
  wire [3:0] nm1_cfg = (c_nbits == 5'd0) ? 4'd7 : (c_nbits >= 5'd16) ? 4'd15 : (c_nbits[3:0] - 1'b1);

  wire [WX-1:0] d2_x = {{(WX - 1) {1'b0}}, c_diff & den} << c_d2pin;
  assign d2_mask = d2_x[NUM_PINS-1:0];

  // -------------------------------------------------------------- state
  reg  [15:0] sh;
  reg  [ 3:0] idx;
  reg  [15:0] o;
  reg         sw;  // stretch: waiting
  reg  [ 1:0] swc;
  reg         eprev;
  reg         sp;  // stick pending until the stretch wait ends
  reg         lf;  // LEAD: bit 0 has been written
  reg         crc_pkt;  // the word being sent is the CRC
  reg         lvl;  // NRZI: last level written
  reg  [ 3:0] ones;  // data 1s in a row
  reg  [ 1:0] eph;  // EOP: 0 none, 1 first SE0 bit, 2 second SE0 bit, 3 J bit

  wire        lead_on = c_ext & c_lead;  // LEAD only applies to the external clock
  wire [ 3:0] last = crc_pkt ? 4'd15 : nm1_cfg;

  wire hold = sw & ((swc != 2'd0) | ~cpin_in);
  wire edge_seen = c_ext & (epin_in != eprev) & (epin_in == ~c_eedge);
  wire edge_opp = c_ext & (epin_in != eprev) & (epin_in == c_eedge);
  wire at_b = (o == beff);
  wire at_h = (half != 16'd0) & (o == half);
  wire vh = c_cpha ? c_cpol : ~c_cpol;  // SCK level written at offset H
  wire rel_h = c_stretch & cen & vh;  // ... is a release: tick comes later
  wire st_end = busy & sw & (swc == 2'd0) & cpin_in;

  wire stuff_now = (c_stuffn != 4'd0) & (ones == c_stuffn);
  // The last bit of the word has run for its full time and nothing is owed.
  wire lend = busy & ~c_ext & ~hold & at_b & (eph == 2'd0) & ~stuff_now & (idx == last);
  assign chain = lend & c_chain & next_ok;
  wire eop_in = lend & ~chain & c_eop;

  // The busy-path outputs (done, ticks, writes) depend only on registered
  // state; start (only while idle) and stop act at the clock edge. This keeps
  // the unit -> RX unit -> controller paths free of combinational loops.
  reg wd_b, wc_b, vc_b;  // write data / write clock with value vc
  reg stf_w, se_w, j_w;  // the data write is: a stuffed 0, SE0, J
  reg adv;

  always @(*) begin
    wd_b  = 1'b0;
    wc_b  = 1'b0;
    vc_b  = c_cpol;
    stf_w = 1'b0;
    se_w  = 1'b0;
    j_w   = 1'b0;
    done  = 1'b0;
    stick = 1'b0;
    dtick = 1'b0;
    adv   = 1'b0;
    if (busy) begin
      if (c_ext) begin
        if (lead_on) begin
          if (edge_seen && (!lf || idx != last)) wd_b = 1'b1;
          if (edge_opp && lf && idx == last) done = 1'b1;
        end else if (edge_seen) begin
          if (idx == last) done = 1'b1;
          else wd_b = 1'b1;
        end
      end else if (!hold) begin
        adv = 1'b1;
        if (at_b) begin
          if (eph != 2'd0) begin
            if (eph == 2'd2) begin
              wd_b = 1'b1;
              j_w  = 1'b1;
            end
            if (eph == 2'd3) done = 1'b1;
          end else if (stuff_now) begin
            wd_b  = 1'b1;
            stf_w = 1'b1;
          end else begin
            wc_b = 1'b1;
            vc_b = c_cpha ? ~c_cpol : c_cpol;
            if (idx == last) begin
              wc_b = ~c_cpha;  // CPHA 0 returns SCK to idle at the end
              vc_b = c_cpol;
              if (c_eop && !(c_chain && next_ok)) begin
                wd_b = 1'b1;
                se_w = 1'b1;
              end else if (!(c_chain && next_ok)) done = 1'b1;
            end else begin
              wd_b  = 1'b1;
              dtick = 1'b1;
            end
          end
        end else if (at_h && eph == 2'd0) begin
          stick = ~rel_h;
          wc_b  = 1'b1;
          vc_b  = vh;
        end
      end
      if (sp && st_end) stick = 1'b1;
    end
  end

  wire go = start | chain;  // a word begins in this cycle
  wire wd = go ? ~lead_on : wd_b;
  wire wc = go ? ~c_ext : wc_b;
  wire vc = go ? (c_cpha ? ~c_cpol : c_cpol) : vc_b;

  // Data bit to write: bit 0 of a new word, else bit idx+1.
  wire [15:0] dsrc = start ? start_data : chain ? next_data : sh;
  wire        go_crc = chain & next_crc;
  wire [ 3:0] last_w = go ? (go_crc ? 4'd15 : nm1_cfg) : last;
  wire        msb_w = go ? (c_msb | go_crc) : (c_msb | crc_pkt);
  wire [ 3:0] bit_i = (go || (lead_on && !lf)) ? 4'd0 : (idx + 1'b1);
  wire [ 3:0] bit_s = msb_w ? (last_w - bit_i) : bit_i;
  wire        raw = stf_w ? 1'b0 : dsrc[bit_s];

  // NRZI: a 0 toggles, a 1 keeps. A new packet starts from level 1.
  wire lvl_c = start ? 1'b1 : lvl;
  wire enc = c_nrzi ? (raw ? lvl_c : ~lvl_c) : raw;
  wire dv = se_w ? 1'b0 : j_w ? 1'b1 : (enc ^ c_inv);
  wire d2v = (se_w | j_w) ? 1'b0 : ~dv;
  wire wd_data = wd & ~se_w & ~j_w;  // a data or stuffed bit

  // Data bits (not stuffed ones, not the CRC itself) feed the CRC unit.
  assign bit_v = wd_data & ~stf_w & ~go_crc & ~(crc_pkt & ~go);
  assign bit_o = raw;

  wire [WX-1:0] pw_m = (wd ? (dm_x | d2_x) : {WX{1'b0}}) | ((wc & cen) ? cm_x : {WX{1'b0}});
  wire [WX-1:0] pw_v = (wd ? ((dv ? dm_x : {WX{1'b0}}) | (d2v ? d2_x : {WX{1'b0}})) : {WX{1'b0}}) |
                       ((wc & cen & vc) ? cm_x : {WX{1'b0}});
  assign pw_mask = pw_m[NUM_PINS-1:0];
  assign pw_val  = pw_v[NUM_PINS-1:0];

  always @(posedge clk) begin
    if (rst) begin
      busy    <= 1'b0;
      sh      <= 16'd0;
      idx     <= 4'd0;
      o       <= 16'd0;
      sw      <= 1'b0;
      swc     <= 2'd0;
      eprev   <= 1'b0;
      sp      <= 1'b0;
      lf      <= 1'b0;
      crc_pkt <= 1'b0;
      lvl     <= 1'b1;
      ones    <= 4'd0;
      eph     <= 2'd0;
    end else begin
      eprev <= epin_in;
      if (start || stop || st_end) sp <= 1'b0;
      if (busy && !c_ext && !hold && !at_b && at_h && rel_h) sp <= 1'b1;
      if (wd_data) begin
        lvl  <= enc;
        ones <= raw ? (start ? 4'd1 : ones + 1'b1) : 4'd0;
      end
      if (start) begin
        busy    <= 1'b1;
        sh      <= start_data;
        idx     <= 4'd0;
        o       <= 16'd1;
        lf      <= 1'b0;
        sw      <= 1'b0;
        swc     <= 2'd0;
        crc_pkt <= 1'b0;
        eph     <= 2'd0;
        if (!wd_data) ones <= 4'd0;
      end else if (busy) begin
        if (stop || done) begin
          busy <= 1'b0;
          eph  <= 2'd0;
        end else if (chain) begin
          sh      <= next_data;
          idx     <= 4'd0;
          o       <= 16'd1;
          crc_pkt <= next_crc;
          sw      <= 1'b0;
          swc     <= 2'd0;
        end else if (c_ext) begin
          if (edge_seen) begin
            if (lead_on && !lf) lf <= 1'b1;
            else if (idx != last) idx <= idx + 1'b1;
          end
        end else if (adv) begin
          if (at_b) begin
            o <= 16'd1;
            if (eph != 2'd0) eph <= eph + 1'b1;  // eph == 3 ends in done, above
            else if (eop_in) eph <= 2'd1;
            else if (!stuff_now) idx <= idx + 1'b1;
          end else o <= o + 1'b1;
        end
        if (sw && !chain) begin
          if (swc != 2'd0) swc <= swc - 1'b1;
          else if (cpin_in) sw <= 1'b0;
        end
      end
      // STRETCH: a write of SCK = 1 starts a stretch wait
      if (!stop && c_stretch && cen && wc && vc && !done) begin
        sw  <= 1'b1;
        swc <= 2'd2;
      end
    end
  end

  always @(*) begin
    rd_data = 16'd0;
    if (rd_idx == 3'd0) rd_data = {8'd0, idx, 3'd0, busy};
  end

  wire _unused = &{1'b0, dm_x[WX-1:NUM_PINS], cm_x[WX-1:NUM_PINS], d2_x[WX-1:NUM_PINS], cin_x[WX-1:1], ein_x[WX-1:1],
                   c_pins[7:5], pw_m[WX-1:NUM_PINS], pw_v[WX-1:NUM_PINS]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Contract (assumed alone, asserted in the chip proof): no start while busy.
  // f_clean = no config write since the start: the unit's timing is only
  // specified then. Reference counters, independent of idx / o:
  // f_c cycles since S, f_nw data writes, f_e selected edges.
  // * Own timer, all B, all n = NBITS, CPOL, CPHA: D = S + nB; dtick at
  //   S + (i+1)B for i < n-1; stick at S + iB + H; offsets in range
  //   (k-induction, unbounded).
  // * All modes: data writes come out in order, each is bit f_nw of the
  //   started word (MSB / LSB); never more than n writes.
  // * External clock: done on selected edge n; LEAD: bit i on selected edge
  //   i+1, nothing at start, done on an opposite edge after bit n-1.
  // * Line coding: never more than STUFFN data 1s in a row; done / chain / EOP
  //   only after all bits of the word; EOP is SE0 for 2B, J for B; DIFF pins
  //   are complementary on data writes and both low in SE0.
  // * Stretch: the timer holds during the wait; a release-sampling tick
  //   only comes when the wait ends with SCK seen high.
  // * busy rises only from start, falls only from done / stop; writes stay
  //   inside {DPIN, CPIN}.
`ifdef PE_TXU_TOP
  `define PE_TXU_ASSUME assume
  initial assume (rst);
`else
  `define PE_TXU_ASSUME assert
`endif
`ifdef PE_CHIP_FORMAL
  wire f_rst_seen = f_seen_i;  // one flag for the whole chip proof
`else
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
`endif
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  reg        f_clean;
  reg [19:0] f_c;
  reg [ 4:0] f_nw, f_e;
  reg [15:0] f_data;
  reg [15:0] f_stf;  // stuffed bits sent in this word (saturating is not needed: <= 16)
  wire       f_lead = lead_on;

  always @(posedge clk) begin
    if (rst) begin
      f_clean <= 1'b0;
    end else if (start || (busy && chain)) begin
      f_clean <= start ? ~cfg_we : (f_clean & ~cfg_we);
      f_c     <= 20'd1;
      f_nw    <= f_lead ? 5'd0 : 5'd1;
      f_e     <= 5'd0;
      f_data  <= dsrc;
    end else begin
      if (cfg_we) f_clean <= 1'b0;
      if (busy) begin
        f_c <= f_c + 1'b1;
        if (wd_data && !stf_w) f_nw <= f_nw + 1'b1;
        if (edge_seen && f_e != 5'd31) f_e <= f_e + 1'b1;
      end
    end
  end

  wire [19:0] f_b20 = {4'd0, beff};
  wire [19:0] f_h20 = {4'd0, half};
  wire [19:0] f_i20 = {16'd0, idx};
  wire [ 4:0] f_n = {1'b0, last} + 5'd1;  // bits in this word
  wire [19:0] f_n20 = {15'd0, f_n};
  // the plain packet of Section 3.3: no line coding, one word, no EOP
  wire        f_plain = (c_stuffn == 4'd0) & ~c_eop & ~c_chain;
  wire        f_nmsb = c_msb | crc_pkt;

  always @(*) begin
    if (f_ok) begin
      if (start) `PE_TXU_ASSUME (!busy);
      assert (beff != 16'd0);
      assert ((pw_mask & ~(dm_x[NUM_PINS-1:0] | cm_x[NUM_PINS-1:0] | d2_x[NUM_PINS-1:0])) == {NUM_PINS{1'b0}});
      assert (swc <= 2'd2);
      if (!busy) begin
        assert (!done && !stick && !dtick && !chain);
        if (!start) assert (pw_mask == {NUM_PINS{1'b0}} && !bit_v);
        assert (eph == 2'd0);
      end
      if (chain) assert (busy && !done && !c_ext && !eop_in);
      if (done || chain) assert (!stf_w);
      if (busy && f_clean) begin
        assert (sh == f_data);
        assert (f_nw <= f_n);
        // Stuffing: never more than STUFFN data 1s in a row on the wire.
        if (c_stuffn != 4'd0 && !c_ext) assert (ones <= c_stuffn);
        if (eph != 2'd0) assert (idx == last && !c_ext);
        if (f_lead) begin
          assert ({1'b0, f_nw} == (lf ? {2'b00, idx} + 6'd1 : 6'd0));
          if (!lf) assert (idx == 4'd0);
          if (f_e < f_n) assert (f_nw == f_e);
          else assert (f_nw == f_n);
          if (done) assert (edge_opp && f_nw == f_n);
        end else begin
          assert (!lf);
          assert ({1'b0, f_nw} == {2'b00, idx} + 6'd1);
          if (c_ext) begin
            assert (f_e < f_n && f_nw == f_e + 5'd1);
            if (done) assert (edge_seen && f_nw == f_n);
          end
        end
        // Every data write is the right bit of the word, in order.
        if (wd_data && !stf_w && !go) begin
          assert (f_nw < f_n);
          assert (raw == (f_nmsb ? f_data[last-f_nw[3:0]] : f_data[f_nw[3:0]]));
        end
        // Done / chain only after every bit of the word has been written.
        if (done || chain || eop_in) assert (f_nw == f_n);
        if (!c_ext) begin
          assert (o >= 16'd1 && o <= beff);
          if (hold) assert (!adv && !done && !wd_b && !dtick && !chain);
          if (!c_stretch) begin
            assert (!sw && !sp);
            if (c_stuffn == 4'd0 && eph == 2'd0) begin
              assert (f_c == f_i20 * f_b20 + {4'd0, o});
              if (!c_chain) assert (dtick == (f_c == (f_i20 + 20'd1) * f_b20 && idx != last));
              assert (stick == (half != 16'd0 && f_c == f_i20 * f_b20 + f_h20));
              if (f_plain) assert (done == (f_c == f_n20 * f_b20));
            end
            // EOP: SE0 at the end of the last bit, J 2B later, done 3B later.
            if (c_stuffn == 4'd0 && eph != 2'd0) begin
              assert (f_c == f_n20 * f_b20 + {18'd0, eph - 2'd1} * f_b20 + {4'd0, o});
              if (j_w) assert (f_c == f_n20 * f_b20 + 20'd2 * f_b20);
              if (done) assert (f_c == f_n20 * f_b20 + 20'd3 * f_b20);
            end
          end else if (cen && vh) begin
            if (stick) assert (st_end && sp);
          end
        end
      end
      // SE0 and J write both pins low / J; a data write with DIFF is complementary.
      if (se_w) assert (!dv && !d2v);
      if (j_w) assert (dv && !d2v);
      if (wd_data) assert (d2v == ~dv);
`ifdef PE_TXU_TOP  // module-level covers only when this module is the proof top
      cover (f_clean && done && !c_ext && beff == 16'd2 && cen);
      cover (f_clean && done && c_lead && c_ext);
      cover (f_clean && done && !c_ext && c_nbits == 5'd9 && beff == 16'd1);
      cover (f_clean && chain && next_crc && beff == 16'd1 && c_nbits == 5'd1);
      cover (f_clean && stf_w && beff == 16'd1 && c_stuffn == 4'd1);
      cover (f_clean && done && eph == 2'd3 && beff == 16'd1 && c_nbits == 5'd1);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(start)) assert (busy);
      if (!$past(busy) && !$past(start)) assert (!busy);
      if ($past(busy) && !$past(start)) begin
        if ($past(stop) || $past(done)) assert (!busy);
        else assert (busy);
      end
      if ($past(hold) && $past(busy) && !$past(c_ext) && !$past(start) && !$past(stop)
          && !$past(done)) assert (o == $past(o) && idx == $past(idx));
    end
  end
`undef PE_TXU_ASSUME
`endif

endmodule
