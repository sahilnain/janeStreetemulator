/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_vote: oversample one pin and take a centre-only majority vote.
 *
 * On `start` (cycle t), latches pin_sel, N, S and k. Sample j (j = 0..N-1) is
 * taken in cycle t + (j+1)*S. Only samples with k <= j < N-k are counted.
 * `done` is a 1-cycle pulse in cycle t + N*S + 1, with
 *     bit_out = ones > (N - 2k) / 2      (ties give 0, so use an odd N - 2k)
 * `busy` is high from t+1 up to and including the `done` cycle, so the
 * owning context cannot issue while the result is being written back.
 *
 * S = 0 is treated as S = 1. N = 0 is treated as N = 1.
 */

`default_nettype none

module pe_vote #(
    parameter NUM_PINS = 8,
    parameter N_W      = 4,
    parameter S_W      = 12
) (
    input  wire                clk,
    input  wire                rst,
    input  wire                start,
    input  wire [         3:0] pin_sel,
    input  wire [NUM_PINS-1:0] pin_in,
    input  wire [     N_W-1:0] n_cfg,
    input  wire [     N_W-1:0] k_cfg,
    input  wire [     S_W-1:0] s_cfg,
    output wire                busy,
    output reg                 done,
    output reg                 bit_out
);

  reg           running;
  reg [    3:0] sel;
  reg [N_W-1:0] n_l, k_l, idx, ones;
  reg [S_W-1:0] s_l, scnt;

  // Selected pin. Out-of-range selects read 0.
  wire [NUM_PINS+15:0] pin_x = {16'b0, pin_in} >> sel;
  wire sample = pin_x[0];
  wire _unused_pin_x = &{1'b0, pin_x[NUM_PINS+15:1]};

  // Comparisons are done one bit wider so N - k and N - 2k cannot wrap.
  wire [N_W:0] idx_w = {1'b0, idx};
  wire [N_W:0] n_w = {1'b0, n_l};
  wire [N_W:0] k_w = {1'b0, k_l};
  wire [N_W+1:0] two_k = {k_w, 1'b0};
  wire [N_W+1:0] votes = ({1'b0, n_w} > two_k) ? ({1'b0, n_w} - two_k) : {(N_W + 2) {1'b0}};

  wire in_win = (idx_w >= k_w) && (idx_w + k_w < n_w);
  wire [N_W-1:0] ones_nx = ones + {{(N_W - 1) {1'b0}}, in_win & sample};
  wire last = (idx_w + 1'b1 >= n_w);

  wire [S_W-1:0] s_reload = (s_l == {S_W{1'b0}}) ? {S_W{1'b0}} : s_l - 1'b1;
  wire [S_W-1:0] s_first = (s_cfg == {S_W{1'b0}}) ? {S_W{1'b0}} : s_cfg - 1'b1;

  always @(posedge clk) begin
    if (rst) begin
      running <= 1'b0;
      done    <= 1'b0;
      bit_out <= 1'b0;
      sel     <= 4'd0;
      n_l     <= {N_W{1'b0}};
      k_l     <= {N_W{1'b0}};
      s_l     <= {S_W{1'b0}};
      scnt    <= {S_W{1'b0}};
      idx     <= {N_W{1'b0}};
      ones    <= {N_W{1'b0}};
    end else begin
      done <= 1'b0;
      if (start && !running) begin
        running <= 1'b1;
        sel     <= pin_sel;
        n_l     <= n_cfg;
        k_l     <= k_cfg;
        s_l     <= s_cfg;
        scnt    <= s_first;
        idx     <= {N_W{1'b0}};
        ones    <= {N_W{1'b0}};
      end else if (running) begin
        if (scnt == {S_W{1'b0}}) begin
          ones <= ones_nx;
          scnt <= s_reload;
          if (last) begin
            running <= 1'b0;
            done    <= 1'b1;
            bit_out <= ({2'b0, ones_nx, 1'b0} > {1'b0, votes});
          end else begin
            idx <= idx + 1'b1;
          end
        end else begin
          scnt <= scnt - 1'b1;
        end
      end
    end
  end

  assign busy = running | done;

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // f_cyc counts cycles since `start` (1 in the first cycle after it).
  // * Timing: while running, f_cyc == idx*S + (S - scnt), so sample j is
  //   taken in cycle (j+1)*S; `done` comes exactly when f_cyc reached N*S
  //   (N, S treated as 1 when 0). Proved for ALL N, S, k (k-induction).
  // * busy = running | done; done is a 1-cycle pulse right after running
  //   ends; config is frozen while running (start is ignored while busy).
  // * Result: if every centre sample was 1 (and the window is not empty)
  //   bit_out = 1; if every centre sample was 0, bit_out = 0.
`ifdef PE_VOTE_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  wire [N_W-1:0] f_n_eff = (n_l == {N_W{1'b0}}) ? {{(N_W - 1) {1'b0}}, 1'b1} : n_l;
  wire [S_W-1:0] f_s_eff = (s_l == {S_W{1'b0}}) ? {{(S_W - 1) {1'b0}}, 1'b1} : s_l;
  reg [N_W+S_W:0] f_cyc;
  reg f_all1, f_all0;

  always @(posedge clk) begin
    if (rst) f_cyc <= 0;
    else if (start && !running) begin
      f_cyc  <= 1;
      f_all1 <= 1'b1;
      f_all0 <= 1'b1;
    end else if (running) begin
      f_cyc <= f_cyc + 1'b1;
      if (scnt == {S_W{1'b0}} && in_win) begin
        if (sample) f_all0 <= 1'b0;
        else f_all1 <= 1'b0;
      end
    end
  end

`ifdef PE_VOTE_MAJORITY
  // Independent reference count (enabled with -DPE_VOTE_MAJORITY): samples
  // are taken where f_cyc is a multiple of S, sample index j = f_cyc/S - 1,
  // counted if k <= j < N-k. bit_out must be exactly the majority of these
  // counted samples, for EVERY pin waveform. (Uses / and %, so it runs on
  // reduced widths; the logic is width-independent.)
  wire [N_W+S_W:0] f_q = f_cyc / f_s_eff;
  wire [N_W+S_W:0] f_r = f_cyc % f_s_eff;
  wire [N_W+S_W:0] f_j = f_q - 1'b1;
  wire f_inwin = (f_j >= k_l) && (f_j + k_l < n_l);
  wire [NUM_PINS+15:0] f_pinx = {16'b0, pin_in} >> sel;
  reg [N_W:0] f_ones;
  always @(posedge clk) begin
    if (start && !running) f_ones <= 0;
    else if (running && f_r == 0 && f_inwin && f_pinx[0]) f_ones <= f_ones + 1'b1;
  end
`endif

  // number of centre-window indices below idx
  wire [N_W:0] f_hi_w = (n_w > k_w) ? n_w - k_w : {(N_W + 1) {1'b0}};
  wire [N_W:0] f_hi = (idx_w < f_hi_w) ? idx_w : f_hi_w;
  wire [N_W:0] f_wcnt = (f_hi > k_w) ? f_hi - k_w : {(N_W + 1) {1'b0}};

  always @(*) begin
    if (f_ok) begin
      assert (busy == (running | done));
      if (running) begin
        assert (scnt < f_s_eff);
        assert (idx < f_n_eff);
        assert (f_cyc == idx * f_s_eff + (f_s_eff - scnt));
        if (f_all1) assert ({1'b0, ones} == f_wcnt);
        if (f_all0) assert (ones == {N_W{1'b0}});
        assert ({1'b0, ones} <= f_wcnt);
      end
      if (done) begin
        if (f_all1 && votes != 0) assert (bit_out);
        if (f_all0) assert (!bit_out);
      end
`ifdef PE_VOTE_MAJORITY
      if (running) assert ({1'b0, ones} == f_ones);
      if (done) assert (bit_out == ({1'b0, f_ones, 1'b0} > {1'b0, votes}));
`endif
`ifdef PE_VOTE_TOP  // module-level covers only when this module is the proof top
      cover (done && bit_out);
      cover (done && !bit_out && n_l == 9 && k_l == 1);
`endif
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok) begin
      if (done) begin
        assert ($past(running) && !running);
        assert ($past(f_cyc) == f_n_eff * f_s_eff);
      end
      if ($past(running) && !running) assert (done);
      if ($past(running) && running)
        assert (n_l == $past(n_l) && k_l == $past(k_l) && s_l == $past(s_l) && sel == $past(sel));
      if ($past(start && !running))
        assert (running && idx == 0 && ones == 0 && sel == $past(pin_sel) && n_l == $past(n_cfg)
                && k_l == $past(k_cfg) && s_l == $past(s_cfg));
    end
  end
`endif

endmodule
