/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_rxu: RX unit (docs/spec/blocks.md Section 4). Receives one packet of 1-16
 * bits per start; can also drive a pin with a word while it samples (half duplex).
 *
 * Config (ucfg RX, idx):
 *   0 PINS  [3:0] IPIN [7:4] I2PIN [11:8] OPIN [12] OEN (drive mode)
 *           [13] DIFF (no sample while IPIN and I2PIN are both low: USB SE0)
 *   1 BIT   [15:0] B, cycles per bit (own timer)
 *   2 FRAME [0] MSB-first [1] HOLD (0 push to RX FIFO, 1 hold for urd)
 *           [2] INV [3] CHAIN [8:4] NBITS (0 -> 8, >= 16 -> 16)
 *   3 VOTE  [3:0] N [7:4] K
 *   4 STEP  [15:0] cycles between vote samples
 *   5 FIRST [15:0] cycles from start to bit 0's window
 *   6 TICK  [1:0] SRC (0 own timer, 1 TX sample tick, 2 external edge,
 *                      3 own timer resynchronised on IPIN edges)
 *           [7:4] EPIN [8] EEDGE (0 rising, 1 falling) [15:12] SDLY
 *   7 ODLY  [3:0] drive delay: in drive mode with SRC 1 / 2, bit i >= 1 is
 *           written ODLY cycles after its shift event (I2C: SDA changes
 *           after SCL is low, never at the same edge)
 *           [4] NRZI [5] HUNT [11:8] STUFFN (0 off)
 * Status (urd RX, idx): 0 DATA (read clears RXRDY), 1 STAT [0] busy
 *   [1] RXRDY [2] FERR [3] overrun [4] stuff error [12:8] bits of the
 *   current word
 *
 * Sampling, start in cycle S:
 *   SRC 0: window i starts at W_i = S + FIRST + i*B and is voted by pe_vote
 *          (sample j in W_i + (j+1)*STEP); D = W_7 + N*STEP + 1.
 *   SRC 1: bit i = pin_in[IPIN] in cycle T_i + SDLY, T_i = i-th tx_stick.
 *   SRC 2: bit i = pin_in[IPIN] SDLY cycles after the i-th EPIN edge.
 *   SRC 3: a counter makes one sample per B cycles; an edge on IPIN seen in
 *          cycle e moves the next sample to e + H (H = floor(B/2)) and cancels
 *          a sample due in e, which would count the end of a long run twice.
 *   done is a pulse in D, the cycle the n-th bit is taken; busy = S+1 .. D.
 * Line decoding, in this order on every sample (`stop` ends the packet):
 *   INV     the sample is inverted.
 *   NRZI    decoded bit = 1 if the sample equals the previous one, else 0.
 *   HUNT    samples are dropped until a 1 follows two or more 0s (USB sync).
 *           That 1 counts as the first 1 for STUFFN.
 *   STUFFN  the sample after N data 1s in a row is dropped; if it is a 1,
 *           the sticky stuff error is set (cleared by the next start).
 *   CHAIN   (SRC != 0, no drive) after the last bit the unit stays busy and
 *           the next word starts at once. Each word gives a done pulse.
 * Drive mode: bit 0 is written to OPIN in S; bit i >= 1 at W_i (SRC 0), or
 *   ODLY cycles after the i-th tx_dtick (SRC 1) / opposite EPIN edge (SRC 2).
 * Result: HOLD = 0 pushes the byte in D. HOLD = 1 keeps it and sets RXRDY
 *   at `after_end` (end of the ERX after-packet step); a new byte while
 *   RXRDY is still set overwrites it and sets `overrun`.
 */

`default_nettype none

module pe_rxu #(
    parameter NUM_PINS = 8
) (
    input wire clk,
    input wire rst,
`ifdef PE_CHIP_FORMAL
    input wire f_seen_i,
`endif

    input  wire        cfg_we,
    input  wire [ 2:0] cfg_idx,
    input  wire [15:0] cfg_data,
    input  wire [ 2:0] rd_idx,
    output reg  [15:0] rd_data,
    input  wire        rd_strobe,

    input wire        start,
    input wire [15:0] start_data,
    input wire        stop,
    input wire        restart,

    input wire [NUM_PINS-1:0] pin_in,
    input wire                tx_stick,
    input wire                tx_dtick,
    input wire                after_end,
    input wire                ferr,
    input wire                clr_ovr,

    output reg                 busy,
    output wire                done,
    output wire                push,
    output wire [         7:0] push_data,
    output reg                 rxrdy,
    output reg                 overrun,
    output wire [NUM_PINS-1:0] pw_mask,
    output wire [NUM_PINS-1:0] pw_val,
    output wire                need,
    output wire                chain_on,
    output wire [         3:0] ipin,
    output wire                bit_v,
    output wire                bit_o
);

  localparam WX = NUM_PINS + 16;

  // ------------------------------------------------------------- config
  reg [13:0] c_pins;
  reg [15:0] c_bit, c_step, c_first;
  reg        c_msb, c_hold, c_eedge, c_inv, c_chain, c_nrzi, c_hunt;
  reg [3:0] c_n, c_k, c_epin, c_sdly, c_odly, c_stuffn;
  reg [4:0] c_nbits;
  reg [1:0] c_src;

  always @(posedge clk) begin
    if (rst) begin
      c_pins  <= 14'd0;
      c_bit   <= 16'd1;
      c_step  <= 16'd1;
      c_first <= 16'd0;
      c_msb   <= 1'b0;
      c_hold  <= 1'b0;
      c_n     <= 4'd1;
      c_k     <= 4'd0;
      c_src   <= 2'd0;
      c_epin  <= 4'd0;
      c_eedge <= 1'b0;
      c_sdly  <= 4'd0;
      c_odly  <= 4'd0;
      c_inv   <= 1'b0;
      c_chain <= 1'b0;
      c_nbits <= 5'd0;
      c_nrzi  <= 1'b0;
      c_hunt  <= 1'b0;
      c_stuffn <= 4'd0;
    end else if (cfg_we) begin
      case (cfg_idx)
        3'd0: c_pins <= cfg_data[13:0];
        3'd1: c_bit <= cfg_data;
        3'd2: {c_nbits, c_chain, c_inv, c_hold, c_msb} <= {cfg_data[8:4], cfg_data[3:0]};
        3'd3: {c_k, c_n} <= cfg_data[7:0];
        3'd4: c_step <= cfg_data;
        3'd5: c_first <= cfg_data;
        3'd6: {c_sdly, c_eedge, c_epin, c_src} <= {cfg_data[15:12], cfg_data[8], cfg_data[7:4], cfg_data[1:0]};
        default: {c_stuffn, c_hunt, c_nrzi, c_odly} <= {cfg_data[11:8], cfg_data[5:0]};  // 3'd7
      endcase
    end
  end

  assign ipin = c_pins[3:0];
  wire [3:0] opin = c_pins[11:8];
  wire       oen = c_pins[12];
  assign need = oen;

  wire [WX-1:0] iin_x = {16'd0, pin_in} >> ipin;
  wire [WX-1:0] ein_x = {16'd0, pin_in} >> c_epin;
  wire [WX-1:0] om_x = {{(WX - 1) {1'b0}}, oen} << opin;
  wire ipin_in = iin_x[0];
  wire [WX-1:0] i2n_x = {16'd0, pin_in} >> c_pins[7:4];
  wire se0 = c_pins[13] & ~ipin_in & ~i2n_x[0];
  wire ipin_edge = (ipin_in != iprev);
  wire epin_in = ein_x[0];

  wire [15:0] beff = (c_bit == 16'd0) ? 16'd1 : c_bit;
  wire src_own = (c_src == 2'd0);
  wire src_tick = (c_src == 2'd1);
  wire src_ext = (c_src == 2'd2);
  wire src_dp = (c_src == 2'd3);
  wire [15:0] half = beff >> 1;
  wire [15:0] dp_load = (half == 16'd0) ? 16'd0 : (half - 1'b1);

  // n - 1 from the NBITS field; n fits in 5 bits
  wire [3:0] nm1 = (c_nbits == 5'd0) ? 4'd7 : (c_nbits >= 5'd16) ? 4'd15 : (c_nbits[3:0] - 1'b1);
  wire [4:0] n5 = {1'b0, nm1} + 5'd1;
  assign chain_on = c_chain & ~src_own & ~oen;

  // -------------------------------------------------------------- state
  reg [15:0] sh, od, rbyte, data_q;
  reg [4:0] nbits;  // bits taken so far
  reg [3:0] oidx;  // last bit driven
  reg [15:0] wcnt;  // cycles until the next window (SRC 0) / sample (SRC 3)
  reg [4:0] wins;  // windows started (SRC 0)
  reg       eprev, iprev;
  reg [15:0] tq;  // delay line for tick / edge events (sampling)
  reg [15:0] oq;  // delay line for drive events
  reg       lp;  // NRZI: previous sample
  reg       hunting;  // HUNT: looking for the end of the sync pattern
  reg [1:0] hz;  // HUNT: 0s in a row (saturating)
  reg [3:0] rones;  // decoded 1s in a row
  reg       serr;  // sticky stuff error

  wire live = busy;  // stop / restart act at the clock edge

  // ------------------------------------------------- SRC 0: vote windows
  // window i starts when wcnt reaches 0 with wins = i (or in S if FIRST = 0)
  wire win_now = src_own & ((start & (c_first == 16'd0)) | (live & (wins < n5) & (wcnt == 16'd0)));
  wire vdone, vbit, vbusy;

  pe_vote #(
      .NUM_PINS(NUM_PINS),
      .N_W     (4),
      .S_W     (16)
  ) u_vote (
      .clk    (clk),
      .rst    (rst | stop | restart),
      .start  (win_now),
      .pin_sel(ipin),
      .pin_in (pin_in),
      .n_cfg  ((c_n == 4'd0) ? 4'd1 : c_n),  // spec: N = 0 means 1
      .k_cfg  (c_k),
      .s_cfg  (c_step),
      .busy   (vbusy),
      .done   (vdone),
      .bit_out(vbit)
  );

  // ------------------------------------------- SRC 1 / 2: delayed events
  wire edge_any = (epin_in != eprev);
  wire edge_smp = edge_any & (epin_in == ~c_eedge);
  wire edge_drv = edge_any & (epin_in == c_eedge);
  wire ev = live & (src_tick ? tx_stick : (src_ext & edge_smp));
  wire [15:0] tq_sh = tq >> (c_sdly - 1'b1);
  wire ev_d = (c_sdly == 4'd0) ? ev : (live & tq_sh[0]);

  // ------------------------------------------------------------ sampling
  wire samp_raw = live & ~se0 & (src_own ? vdone : src_dp ? ((wcnt == 16'd0) & ~ipin_edge) : ev_d);
  wire sval = (src_own ? vbit : ipin_in) ^ c_inv;

  // line decoding
  wire dec = c_nrzi ? ~(sval ^ lp) : sval;
  wire st_skip = (c_stuffn != 4'd0) & (rones == c_stuffn) & ~hunting;
  wire samp = samp_raw & ~hunting & ~st_skip;
  assign bit_v = samp;
  assign bit_o = dec;

  // MSB-first: shift in. LSB-first: bit k goes to position k, so that a word
  // of n < 16 bits ends up right-aligned in both orders.
  wire [15:0] one_at = 16'd1 << nbits[3:0];
  wire [15:0] sh_nx = c_msb ? {sh[14:0], dec} : (dec ? (sh | one_at) : sh);
  assign done      = samp & (nbits == {1'b0, nm1});
  assign push      = done & ~c_hold;
  assign push_data = sh_nx[7:0];

  // --------------------------------------------------------------- drive
  wire drv_raw = live & (src_tick ? tx_dtick : (src_ext & edge_drv));
  wire [15:0] oq_sh = oq >> (c_odly - 1'b1);
  wire drv_d = (c_odly == 4'd0) ? drv_raw : (live & oq_sh[0]);
  wire drv_ev = live & (oidx != nm1) & (src_own ? (win_now & (wins != 5'd0)) : drv_d);
  wire [15:0] dsrc = start ? start_data : od;
  wire [3:0] obit = start ? 4'd0 : (oidx + 1'b1);
  wire [3:0] obit_s = c_msb ? (nm1 - obit) : obit;
  wire obit_v = dsrc[obit_s] ^ c_inv;
  wire wr_o = oen & (start | drv_ev);
  assign pw_mask = wr_o ? om_x[NUM_PINS-1:0] : {NUM_PINS{1'b0}};
  assign pw_val  = (wr_o & obit_v) ? om_x[NUM_PINS-1:0] : {NUM_PINS{1'b0}};

  wire rd_data_now = rd_strobe & (rd_idx == 3'd0);
  wire [15:0] byte_end = done ? sh_nx : rbyte;

  always @(posedge clk) begin
    if (rst) begin
      busy    <= 1'b0;
      sh      <= 16'd0;
      od      <= 16'd0;
      rbyte   <= 16'd0;
      data_q  <= 16'd0;
      nbits   <= 5'd0;
      oidx    <= 4'd0;
      wcnt    <= 16'd0;
      wins    <= 5'd0;
      eprev   <= 1'b0;
      iprev   <= 1'b0;
      tq      <= 16'd0;
      oq      <= 16'd0;
      rxrdy   <= 1'b0;
      overrun <= 1'b0;
      lp      <= 1'b0;
      hunting <= 1'b0;
      hz      <= 2'd0;
      rones   <= 4'd0;
      serr    <= 1'b0;
    end else begin
      eprev <= epin_in;
      iprev <= ipin_in;
      // events still in flight from before a start (or a stop) never count
      tq    <= (stop | restart | start) ? 16'd0 : {tq[14:0], ev};
      oq    <= (stop | restart | start) ? 16'd0 : {oq[14:0], drv_raw};
      if (start) begin
        busy    <= 1'b1;
        od      <= start_data;
        sh      <= 16'd0;
        nbits   <= 5'd0;
        oidx    <= 4'd0;
        lp      <= ipin_in ^ c_inv;
        hunting <= c_hunt;
        hz      <= 2'd0;
        rones   <= 4'd0;
        serr    <= 1'b0;
        if (c_first == 16'd0) begin
          wins <= 5'd1;
          wcnt <= beff - 1'b1;
        end else begin
          wins <= 5'd0;
          wcnt <= c_first - 1'b1;
        end
      end else if (busy) begin
        if (stop || restart || (done && !chain_on)) busy <= 1'b0;
        if (samp_raw) begin
          lp <= sval;
          if (hunting) begin
            if (!dec) begin
              if (hz != 2'd2) hz <= hz + 1'b1;
            end else begin
              hz <= 2'd0;
              if (hz == 2'd2) begin
                hunting <= 1'b0;
                rones   <= 4'd1;
              end
            end
          end else if (st_skip) begin
            rones <= 4'd0;
            if (dec) serr <= 1'b1;
          end else begin
            rones <= dec ? (rones + 1'b1) : 4'd0;
          end
        end
        if (samp) begin
          sh    <= (done && chain_on) ? 16'd0 : sh_nx;
          nbits <= (done && chain_on) ? 5'd0 : (nbits + 1'b1);
        end
        if (drv_ev) oidx <= oidx + 1'b1;
        if (src_own) begin
          if (wcnt == 16'd0) begin
            if (wins < n5) begin
              wins <= wins + 1'b1;
              wcnt <= beff - 1'b1;
            end
          end else wcnt <= wcnt - 1'b1;
        end else if (src_dp) begin
          if (ipin_edge) wcnt <= dp_load;
          else if (wcnt == 16'd0) wcnt <= beff - 1'b1;
          else wcnt <= wcnt - 1'b1;
        end
      end
      if (done) rbyte <= sh_nx;

      // RXRDY / DATA (HOLD mode)
      if (restart) rxrdy <= 1'b0;
      else if (after_end && c_hold) begin
        data_q <= byte_end;
        rxrdy  <= 1'b1;
      end else if (rd_data_now) rxrdy <= 1'b0;

      if (restart) overrun <= 1'b0;
      else if (after_end && c_hold && rxrdy && !rd_data_now) overrun <= 1'b1;
      else if (clr_ovr) overrun <= 1'b0;
    end
  end

  always @(*) begin
    case (rd_idx)
      3'd0: rd_data = data_q;
      3'd1: rd_data = {3'd0, nbits, 3'd0, serr, overrun, ferr, rxrdy, busy};
      default: rd_data = 16'd0;
    endcase
  end

  wire _unused = &{1'b0, iin_x[WX-1:1], ein_x[WX-1:1], om_x[WX-1:NUM_PINS], tq_sh[15:1], oq_sh[15:1], vbusy,
                   i2n_x[WX-1:1]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Contract (assumed alone, asserted in the chip proof): no start while busy.
  // Reference: f_c cycles since S, f_b the sampled bits in arrival order, f_nb
  // their count, f_no drive writes; f_clean as in pe_txu.
  // * Own timer, all B / FIRST: window i starts exactly at S + FIRST + i*B
  //   (pe_vote's own proof then gives sample and done times).
  // * All sources, all n = NBITS: done on data bit n; the word is exactly the
  //   decoded bits in the configured order, right-aligned; push only in done
  //   with HOLD = 0, push_data = low byte.
  // * Line decoding: nothing is taken while hunting; a stuffed sample is never
  //   taken; never more than STUFFN data 1s in a row.
  // * Events from before a start or stop never count (delay lines empty).
  // * Drive: writes only OPIN, in order, each is the right bit of the byte.
  // * RXRDY: set by after_end in HOLD mode; cleared by a DATA read / restart.
  //   overrun: only when a byte ends while RXRDY stays set.
`ifdef PE_RXU_TOP
  `define PE_RXU_ASSUME assume
  initial assume (rst);
`else
  `define PE_RXU_ASSUME assert
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
  reg [20:0] f_c;
  reg [15:0] f_b, f_od;
  reg [ 4:0] f_nb, f_no;

  always @(posedge clk) begin
    if (rst) begin
      f_clean <= 1'b0;
    end else if (start) begin
      f_clean <= ~cfg_we;
      f_c     <= 21'd1;
      f_nb    <= 5'd0;
      f_b     <= 16'd0;
      f_od    <= start_data;
      f_no    <= oen ? 5'd1 : 5'd0;
    end else begin
      if (cfg_we) f_clean <= 1'b0;
      if (busy) begin
        f_c <= f_c + 1'b1;
        if (samp) begin
          if (done && chain_on) begin
            f_nb <= 5'd0;
            f_b  <= 16'd0;
          end else if (f_nb < n5) begin
            f_b[f_nb[3:0]] <= dec;
            f_nb <= f_nb + 1'b1;
          end
        end
        if (wr_o) f_no <= f_no + 1'b1;
      end
    end
  end

  wire [20:0] f_b21 = {5'd0, beff};
  wire [20:0] f_first = {5'd0, c_first};
  wire [20:0] f_w21 = {16'd0, wins};
  integer f_k;

  always @(*) begin
    if (f_ok) begin
      if (start) `PE_RXU_ASSUME (!busy);
      assert (nbits <= {1'b0, nm1} || !busy || !f_clean);
      assert ((pw_mask & ~om_x[NUM_PINS-1:0]) == {NUM_PINS{1'b0}});
      if (!busy) assert (!done && !push && !bit_v);
      assert (push == (done && !c_hold));
      if (busy && f_clean) begin
        assert (nbits == f_nb);
        assert (od == f_od);
        assert (hz <= 2'd2);
        if (c_stuffn != 4'd0 && !hunting) assert (rones <= c_stuffn);
        if (hunting) assert (nbits == 5'd0 && !done);
        if (st_skip) assert (!samp && !done);
        if (src_own) begin
          assert (wins <= n5);
          if (wins < n5) begin
            assert (f_c + {5'd0, wcnt} == f_first + f_w21 * f_b21);
            if (win_now) assert (f_c == f_first + f_w21 * f_b21);
          end
        end
        for (f_k = 0; f_k < 16; f_k = f_k + 1) begin
          if (f_k < f_nb) begin
            if (c_msb) assert (sh[f_k] == f_b[f_nb-1-f_k]);
            else assert (sh[f_k] == f_b[f_k]);
          end else if (!c_msb) assert (!sh[f_k]);
        end
        if (done) begin
          assert (f_nb == {1'b0, nm1});
          for (f_k = 0; f_k < 15; f_k = f_k + 1) begin
            if (f_k < f_nb) begin
              if (c_msb) assert (sh_nx[nm1-f_k] == f_b[f_k]);
              else assert (sh_nx[f_k] == f_b[f_k]);
            end
          end
          assert ((c_msb ? sh_nx[0] : sh_nx[nm1]) == dec);
        end
        assert (oidx <= nm1);
        if (oen) begin
          assert (f_no == {1'b0, oidx} + 5'd1);
          if (wr_o) assert (f_no < n5 && obit_v == ((c_msb ? f_od[nm1-f_no[3:0]] : f_od[f_no[3:0]]) ^ c_inv));
        end
      end
`ifdef PE_RXU_TOP  // module-level covers only when this module is the proof top
      cover (f_clean && done && src_own && c_n == 4'd3 && c_k == 4'd1);
      cover (f_clean && done && src_tick && c_sdly == 4'd3);
      cover (f_clean && done && src_ext && c_hold);
      if (se0) assert (!samp && !done);
      if (src_dp && ipin_edge) assert (!samp_raw);
      cover (f_clean && done && src_dp && c_nbits == 5'd9 && beff == 16'd1);
      cover (f_clean && done && chain_on && c_hunt && c_nbits == 5'd1 && beff == 16'd1);
      cover (f_clean && samp_raw && st_skip && dec && c_stuffn == 4'd1 && beff == 16'd1);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(start)) assert (busy && tq == 16'd0 && oq == 16'd0);
      if ($past(stop) || $past(restart)) assert (tq == 16'd0 && oq == 16'd0);
      if (!$past(busy) && !$past(start)) assert (!busy);
      if ($past(busy) && !$past(start)) begin
        if ($past(stop) || $past(restart) || ($past(done) && !$past(chain_on))) assert (!busy);
        else assert (busy);
      end
      if ($past(restart)) assert (!rxrdy && !overrun);
      else if ($past(after_end) && $past(c_hold)) begin
        assert (rxrdy);
        assert (data_q == $past(byte_end));
      end else if ($past(rd_data_now)) assert (!rxrdy);
      else assert (rxrdy == $past(rxrdy));
      if ($past(restart)) assert (!overrun);
      else if ($past(after_end) && $past(c_hold) && $past(rxrdy) && !$past(rd_data_now)) assert (overrun);
      else if ($past(clr_ovr)) assert (!overrun);
      else assert (overrun == $past(overrun));
    end
  end
`undef PE_RXU_ASSUME
`endif

endmodule
