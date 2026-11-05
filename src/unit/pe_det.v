/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_det: pin-event detector (docs/spec/blocks.md Section 5). Instantiated
 * twice: KIND 0 = ETX (paired with the TX unit), KIND 1 = ERX (RX unit).
 *
 * Config (ucfg ETX|ERX, idx):
 *   0 COND  [3:0] PIN [5:4] MODE (0 level, 1 rising, 2 falling, 3 any edge)
 *           [6] LVL [7] START [8] AUTOARM [12:9] QPIN [13] QEN [14] QLVL
 *           [15] STOP (a fire stops its unit, as `ustop`)
 *   1 DUR   [15:0] cycles the condition must hold (0 -> 1)
 *   2 AFTER ETX: [1:0] 0 none, 1 drive 0, 2 drive 1, 3 release
 *           ERX: [0] CHECK [1] check level
 *   3 ADLY  ERX: [15:0] check delay after the unit's done
 * Status (urd, 0): [0] armed [1] flag [2] arm pending [3] after-step / own
 *   packet in progress [4] FERR
 *
 * Trigger (only while armed): level mode counts consecutive cycles with
 * pin == LVL and the qualifier true; edge modes count from an edge (seen
 * with the qualifier true) while the pin keeps its new level. It fires at
 * count DUR: flag <- 1, disarm, and with START a start request to its unit
 * in the same cycle. `prev` follows the pin at all times, so arming on a
 * line that is already low never fires a falling-edge detector.
 *
 * After-packet step, on the unit's done D (whoever started the packet):
 *   ETX: drive DPIN to 0/1 (oe = 1) or release it (oe = 0) in D.
 *   ERX: with CHECK, FERR <- (pin_in[chk_pin] != level) in D + ADLY; the
 *        step is pending (and extends RX busy) until then. No check while
 *        the RX unit has CHAIN set (a done is not the end of a packet).
 *   after_end is a pulse in the cycle the step ends.
 * Arming: `arm` while the unit is busy or the step is pending sets arm
 * pending; the detector then arms in the cycle after the step ends. With
 * AUTOARM it also re-arms then, after a packet it started itself.
 */

`default_nettype none

module pe_det #(
    parameter KIND     = 0,
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
    output wire [15:0] rd_data,

    input wire clr,
    input wire arm,
    input wire disarm,
    input wire restart,

    input wire [NUM_PINS-1:0] pin_in,

    // own unit
    input  wire       unit_busy,
    input  wire       unit_done,
    input  wire       unit_stop,
    input  wire       unit_chain,  // ERX: RX CHAIN, a done is not the end of a packet: no check
    input  wire       start_ack,
    input  wire [3:0] act_pin,     // ETX: TX DPIN; ERX: RX IPIN
    input  wire [NUM_PINS-1:0] act2_mask,  // ETX release: second pin (TX DIFF)
    output wire       start_req,
    output wire       stop_req,
    output wire       after_pend,
    output wire       after_end,

    output reg  flag,
    output reg  ferr,

    // ETX drive / release (TXP port)
    output wire [NUM_PINS-1:0] pw_mask,
    output wire [NUM_PINS-1:0] pw_val,
    output wire [NUM_PINS-1:0] po_mask,
    output wire [NUM_PINS-1:0] po_val
);

  localparam WX = NUM_PINS + 16;

  // ------------------------------------------------------------- config
  reg [15:0] c_cond;
  reg [15:0] c_dur, c_adly;
  reg [ 1:0] c_after;
  wire cfg_trig = cfg_we & (cfg_idx == 3'd0 || cfg_idx == 3'd1);

  always @(posedge clk) begin
    if (rst) begin
      c_cond  <= 16'd0;
      c_dur   <= 16'd0;
      c_after <= 2'd0;
      c_adly  <= 16'd0;
    end else if (cfg_we) begin
      case (cfg_idx)
        3'd0: c_cond <= cfg_data;
        3'd1: c_dur <= cfg_data;
        3'd2: c_after <= cfg_data[1:0];
        3'd3: c_adly <= cfg_data;
        default: ;
      endcase
    end
  end

  wire [3:0] m_pin = c_cond[3:0];
  wire [1:0] m_mode = c_cond[5:4];
  wire       m_lvl = c_cond[6];
  wire       m_start = c_cond[7];
  wire       m_auto = c_cond[8];
  wire [3:0] m_qpin = c_cond[12:9];
  wire       m_qen = c_cond[13];
  wire       m_qlvl = c_cond[14];
  wire       m_stop = c_cond[15];

  wire [WX-1:0] p_x = {16'd0, pin_in} >> m_pin;
  wire [WX-1:0] q_x = {16'd0, pin_in} >> m_qpin;
  wire [WX-1:0] a_x = {16'd0, pin_in} >> act_pin;
  wire [WX-1:0] am_x = {{(WX - 1) {1'b0}}, 1'b1} << act_pin;
  wire cur = p_x[0];
  wire qual = ~m_qen | (q_x[0] == m_qlvl);

  // ------------------------------------------------------------ trigger
  reg        armed, pend, mine, prev, hold, hlvl;
  reg [15:0] cnt;

  wire [15:0] dure = (c_dur == 16'd0) ? 16'd1 : c_dur;
  wire is_lvl = (m_mode == 2'd0);
  wire dir_ok = (m_mode == 2'd3) | ((m_mode == 2'd1) & cur) | ((m_mode == 2'd2) & ~cur);
  wire edge_ev = ~is_lvl & (cur != prev) & dir_ok & qual;
  wire cont = is_lvl ? ((cur == m_lvl) & qual) : (edge_ev | (hold & (cur == hlvl)));
  wire [16:0] cnt_nx = (~is_lvl & edge_ev) ? 17'd1 : {1'b0, cnt} + 1'b1;
  wire fire = armed & cont & (cnt_nx == {1'b0, dure});

  assign start_req = fire & m_start;
  assign stop_req  = fire & m_stop;

  // ----------------------------------------------------- after-packet step
  reg        apend;
  reg [15:0] acnt;
  wire       chk_en = (KIND == 1) & c_after[0] & ~unit_chain;
  wire       chk_now = chk_en & ((unit_done & (c_adly == 16'd0)) | (apend & (acnt == 16'd1)));
  assign after_end  = (unit_done & ~(chk_en & (c_adly != 16'd0))) | (apend & (acnt == 16'd1));
  assign after_pend = apend;

  wire unit_active = unit_busy | apend;
  wire rearm = after_end & (pend | (m_auto & mine));

  // ETX: drive / release in D
  wire drv = (KIND == 0) & unit_done & (c_after != 2'd0);
  wire drv_out = drv & (c_after != 2'd3);
  assign pw_mask = drv_out ? am_x[NUM_PINS-1:0] : {NUM_PINS{1'b0}};
  assign pw_val  = (drv_out & (c_after == 2'd2)) ? am_x[NUM_PINS-1:0] : {NUM_PINS{1'b0}};
  assign po_mask = drv ? (am_x[NUM_PINS-1:0] | act2_mask) : {NUM_PINS{1'b0}};
  assign po_val  = drv_out ? (am_x[NUM_PINS-1:0] | act2_mask) : {NUM_PINS{1'b0}};

  always @(posedge clk) begin
    if (rst || restart) begin
      armed <= 1'b0;
      pend  <= 1'b0;
      mine  <= 1'b0;
      flag  <= 1'b0;
      ferr  <= 1'b0;
      hold  <= 1'b0;
      hlvl  <= 1'b0;
      cnt   <= 16'd0;
      apend <= 1'b0;
      acnt  <= 16'd0;
      prev  <= rst ? 1'b0 : cur;
    end else begin
      prev <= cur;

      // counter
      if (!armed || fire || cfg_trig || !cont) begin
        cnt  <= 16'd0;
        hold <= 1'b0;
      end else begin
        cnt <= cnt_nx[15:0];
        if (edge_ev) begin
          hold <= 1'b1;
          hlvl <= cur;
        end
      end

      // flag
      if (fire) flag <= 1'b1;
      else if (clr) flag <= 1'b0;

      // armed / pending
      if (fire || disarm) armed <= 1'b0;
      else if (rearm) armed <= 1'b1;
      else if (arm && !unit_active) armed <= 1'b1;
      else if (pend && !unit_active && !unit_done) armed <= 1'b1;

      if (disarm || rearm) pend <= 1'b0;
      else if (arm && unit_active) pend <= 1'b1;
      else if (pend && !unit_active && !unit_done) pend <= 1'b0;

      if (start_ack) mine <= 1'b1;
      else if (after_end || unit_stop) mine <= 1'b0;

      // ERX check
      if (unit_stop) apend <= 1'b0;
      else if (unit_done && chk_en && c_adly != 16'd0) begin
        apend <= 1'b1;
        acnt  <= c_adly;
      end else if (apend) begin
        if (acnt == 16'd1) apend <= 1'b0;
        else acnt <= acnt - 1'b1;
      end
      if (chk_now) ferr <= (a_x[0] != c_after[1]);
    end
  end

  assign rd_data = {11'd0, ferr, mine | apend, pend, flag, armed};

  wire _unused = &{1'b0, p_x[WX-1:1], q_x[WX-1:1], a_x[WX-1:1], am_x[WX-1:NUM_PINS]};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // Contract (assumed alone, asserted in the chip proof): done only while the
  // unit is busy; a start is acknowledged only when requested and unit idle.
  // Reference: f_run counts consecutive qualifying cycles while armed
  // (independent of cnt); f_edge = a real edge since the detector was armed.
  // * Level mode: fires exactly in the DUR-th consecutive qualifying cycle.
  // * Edge modes: never fires without an edge seen while armed.
  // * start_req only in a fire with START; flag set by fire; disarm on fire.
  // * ETX drives only in done, only on act_pin; ERX never drives.
  // * Check: after_end exactly ADLY cycles after done; FERR from that sample.
  // * Arm: armed rises only from arm with the unit idle, a re-arm at
  //   after_end, or pending released after a stop.
`ifdef PE_DET_TOP
  `define PE_DET_ASSUME assume
  initial assume (rst);
`else
  `define PE_DET_ASSUME assert
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

  reg [16:0] f_run;
  reg        f_edge;
  reg [16:0] f_el;  // cycles since done with a check pending
  reg [15:0] f_adly;
  wire f_cond_lvl = is_lvl & (cur == m_lvl) & qual;
  reg  f_parmed = 1'b0;  // the counter clears in the cycle after disarm
  always @(posedge clk) f_parmed <= armed;

  always @(posedge clk) begin
    if (rst || restart || !armed || fire || cfg_trig) begin
      f_run  <= 17'd0;
      f_edge <= 1'b0;
    end else begin
      f_run <= f_cond_lvl ? f_run + 1'b1 : 17'd0;
      if (edge_ev) f_edge <= 1'b1;
    end
    if (unit_done && chk_en && c_adly != 16'd0) begin
      f_el   <= 17'd1;
      f_adly <= c_adly;
    end else if (apend) f_el <= f_el + 1'b1;
  end

  always @(*) begin
    if (f_ok) begin
      if (unit_done) `PE_DET_ASSUME (unit_busy);
      `PE_DET_ASSUME (!(unit_busy && apend));
      if (start_ack) `PE_DET_ASSUME (start_req && !unit_busy && !apend);
      assert (start_req == (fire && m_start));
      assert (stop_req == (fire && m_stop));
      assert (dure != 16'd0);
      if (fire) assert (armed);
      if (armed && is_lvl) begin
        assert ({1'b0, cnt} == f_run);
        assert (f_run < {1'b0, dure});
        if (f_cond_lvl && f_run + 17'd1 == {1'b0, dure}) assert (fire);
        if (fire) assert (f_cond_lvl && f_run + 17'd1 == {1'b0, dure});
      end
      if (armed && !is_lvl) begin
        if (hold) assert (f_edge);
        if (fire) assert (edge_ev || f_edge);
        if (!hold) assert (cnt == 16'd0);
      end
      if (!armed && !f_parmed) assert (cnt == 16'd0 && !hold);
      if (KIND == 1) assert (pw_mask == {NUM_PINS{1'b0}} && po_mask == {NUM_PINS{1'b0}});
      else begin
        if (!unit_done) assert (pw_mask == {NUM_PINS{1'b0}} && po_mask == {NUM_PINS{1'b0}});
        assert ((pw_mask & ~am_x[NUM_PINS-1:0]) == {NUM_PINS{1'b0}});
        assert (!apend);
      end
      if (apend) begin
        assert (acnt != 16'd0);
        assert (f_el <= {1'b0, f_adly});
        assert ({1'b0, acnt} + f_el == {1'b0, f_adly} + 17'd1);
        assert (after_end == (f_el == {1'b0, f_adly}));
      end
`ifdef PE_DET_TOP  // module-level covers only when this module is the proof top
      cover (fire && m_start && dure == 16'd3 && is_lvl);
      cover (fire && m_mode == 2'd2 && dure == 16'd2);
      if (KIND == 1) cover (after_end && apend && f_adly == 16'd2);
      cover (rearm && m_auto && mine);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if ($past(restart)) assert (!armed && !pend && !flag && !ferr && !mine && !apend);
      else begin
        if ($past(fire)) assert (flag && !armed);
        else if ($past(clr)) assert (!flag);
        else assert (flag == $past(flag));
        if ($past(disarm)) assert (!armed && !pend);
        if (armed && !$past(armed))
          assert ($past(rearm) || ($past(arm) && !$past(unit_active)) ||
                  ($past(pend) && !$past(unit_active)));
        if ($past(chk_now)) assert (ferr == ($past(a_x[0]) != $past(c_after[1])));
        else assert (ferr == $past(ferr));
      end
    end
  end
`undef PE_DET_ASSUME
`endif

endmodule
