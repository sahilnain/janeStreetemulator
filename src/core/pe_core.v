/*
 * Copyright (c) 2026 Sahil
 * SPDX-License-Identifier: Apache-2.0
 *
 * pe_core: slot scheduler + contexts + shared decode/execute + vote units.
 *
 * Fixed time-slot round-robin over NUM_SLOTS = max(2, NUM_CTX) slots.
 * Slot s belongs to context s (slots >= NUM_CTX are idle). Each cycle:
 *   - fetch: im_raddr = PC of the context that owns the NEXT slot
 *            (pe_imem has a registered read, so the word arrives next cycle)
 *   - execute: im_rdata is the instruction of the context owning THIS slot.
 *            If that context is ready, it issues: pe_exec computes its next
 *            state, which is written back at the clock edge, together with
 *            pin, FIFO and vote side effects.
 * An instruction therefore occupies one slot = NUM_SLOTS clock cycles, plus
 * any stall (rounded up to whole slots).
 *
 * Parameter limits: IMEM_DEPTH <= 256, NUM_PINS <= 16, DATA_W < SR_W <= 16,
 * CTR_W <= 16.
 */

`default_nettype none

module pe_core #(
    parameter NUM_CTX       = 2,
    parameter IMEM_DEPTH    = 64,
    parameter IW            = 24,
    parameter NUM_PINS      = 8,
    parameter SR_W          = 16,
    parameter DATA_W        = 8,
    parameter CTR_W         = 16,
    parameter VOTE_CTX_MASK = {NUM_CTX{1'b1}},
    parameter VOTE_N_W      = 4,
    parameter VOTE_S_W      = 12,
    // derived, do not override
    parameter AW            = (IMEM_DEPTH > 1) ? $clog2(IMEM_DEPTH) : 1,
    parameter CTX_W         = (NUM_CTX > 1) ? $clog2(NUM_CTX) : 1
) (
    input wire clk,
    input wire rst,

    // instruction memory read port
    output wire [AW-1:0] im_raddr,
    input  wire [IW-1:0] im_rdata,

    // pin fabric
    output wire                pin_wr_en,
    output wire [ CTX_W-1:0]   pin_wr_ctx,
    output wire [NUM_PINS-1:0] pin_wr_mask,
    output wire [NUM_PINS-1:0] pin_wr_val,
    output wire                pin_oe_en,
    output wire [NUM_PINS-1:0] pin_oe_mask,
    output wire [NUM_PINS-1:0] pin_oe_val,
    input  wire [NUM_PINS-1:0] pin_in,

    // FIFOs (firmware side)
    output wire              txf_rd,
    input  wire [DATA_W-1:0] txf_rdata,
    input  wire              txf_empty,
    output wire              rxf_wr,
    output wire [DATA_W-1:0] rxf_wdata,
    input  wire              rxf_full,

    // configuration from pe_regs
    input wire [        NUM_CTX-1:0] ctx_run,
    input wire [        NUM_CTX-1:0] ctx_restart,
    input wire [     NUM_CTX*AW-1:0] ctx_start_pc,
    input wire [        NUM_CTX-1:0] ctx_shift_msb,
    input wire [NUM_CTX*VOTE_N_W-1:0] vote_n,
    input wire [NUM_CTX*VOTE_N_W-1:0] vote_k,
    input wire [NUM_CTX*VOTE_S_W-1:0] vote_s,

    // status to pe_regs
    output wire [NUM_CTX*AW-1:0] ctx_pc,
    output wire [   NUM_CTX-1:0] ctx_ready,
    output wire [   NUM_CTX-1:0] ctx_vote_busy,
    output wire [   NUM_CTX-1:0] ctx_c,
    output wire [   NUM_CTX-1:0] ctx_to
);

  localparam NUM_SLOTS = (NUM_CTX < 2) ? 2 : NUM_CTX;
  localparam SLOT_W = $clog2(NUM_SLOTS);
  localparam CNT_W = $clog2(SR_W + 1);
  localparam STALL_W = 17;  // 16-bit delay immediate + 3-bit delay field
  localparam integer LAST_SLOT_I = NUM_SLOTS - 1;
  localparam [SLOT_W-1:0] LAST_SLOT = LAST_SLOT_I[SLOT_W-1:0];
  localparam integer NUM_CTX_I = NUM_CTX;
  localparam [SLOT_W:0] NUM_CTX_S = NUM_CTX_I[SLOT_W:0];
  // VOTE_CTX_MASK may arrive sized or as a 32-bit integer; normalise it.
  /* verilator lint_off WIDTH */
  localparam [NUM_CTX-1:0] VMASK = VOTE_CTX_MASK;
  /* verilator lint_on WIDTH */

  // ---------------------------------------------------------------- slots
  reg  [SLOT_W-1:0] slot;
  wire [SLOT_W-1:0] slot_nx = (slot == LAST_SLOT) ? {SLOT_W{1'b0}} : slot + 1'b1;

  always @(posedge clk) begin
    if (rst) slot <= {SLOT_W{1'b0}};
    else slot <= slot_nx;
  end

  wire slot_valid = ({1'b0, slot} < NUM_CTX_S);
  wire nx_valid = ({1'b0, slot_nx} < NUM_CTX_S);
  wire [CTX_W-1:0] cur = slot_valid ? slot[CTX_W-1:0] : {CTX_W{1'b0}};
  wire [CTX_W-1:0] nxt = nx_valid ? slot_nx[CTX_W-1:0] : {CTX_W{1'b0}};

  // ---------------------------------------------------- per-context state
  wire [  NUM_CTX*AW-1:0] pc_all;
  wire [NUM_CTX*CTR_W-1:0] ctr0_all, ctr1_all;
  wire [ NUM_CTX*SR_W-1:0] osr_all, isr_all;
  wire [NUM_CTX*CNT_W-1:0] osr_cnt_all;
  wire [     NUM_CTX-1:0] c_all, to_all, ready_all;
  wire [     NUM_CTX-1:0] vbusy, vdone, vbit;

  wire issue = slot_valid & ready_all[cur];

  assign im_raddr = pc_all[nxt*AW+:AW];

  // ------------------------------------------------------------- decode
  wire op_set, op_setoe, op_out, op_in, op_wait, op_jpin, op_jmp, op_djnz;
  wire op_mov, op_delay, op_pull, op_push, op_vote, op_illegal;
  wire [2:0] d_dly, d_cond;
  wire [3:0] d_pin;
  wire [4:0] d_n;
  wire d_lvl, d_to_en, d_to_ctr, d_ctr_sel, d_nb;
  wire [1:0] d_src;
  wire [AW-1:0] d_target;
  wire [15:0] d_imm;

  pe_decode #(
      .IW(IW),
      .AW(AW)
  ) u_decode (
      .ir        (im_rdata),
      .op_set    (op_set),
      .op_setoe  (op_setoe),
      .op_out    (op_out),
      .op_in     (op_in),
      .op_wait   (op_wait),
      .op_jpin   (op_jpin),
      .op_jmp    (op_jmp),
      .op_djnz   (op_djnz),
      .op_mov    (op_mov),
      .op_delay  (op_delay),
      .op_pull   (op_pull),
      .op_push   (op_push),
      .op_vote   (op_vote),
      .op_illegal(op_illegal),
      .dly       (d_dly),
      .pin       (d_pin),
      .n         (d_n),
      .lvl       (d_lvl),
      .to_en     (d_to_en),
      .to_ctr    (d_to_ctr),
      .cond      (d_cond),
      .ctr_sel   (d_ctr_sel),
      .src       (d_src),
      .nb        (d_nb),
      .target    (d_target),
      .imm       (d_imm)
  );

  // ------------------------------------------------------------ execute
  wire              x_done;
  wire [    AW-1:0] x_pc;
  wire [ CTR_W-1:0] x_ctr0, x_ctr1;
  wire [  SR_W-1:0] x_osr, x_isr;
  wire [ CNT_W-1:0] x_osr_cnt;
  wire              x_c, x_to;
  wire [STALL_W-1:0] x_stall;
  wire              x_pin_wr_en, x_pin_oe_en, x_txf_rd, x_rxf_wr, x_vote_start;
  wire [       3:0] x_vote_pin;

  pe_exec #(
      .AW      (AW),
      .NUM_PINS(NUM_PINS),
      .SR_W    (SR_W),
      .DATA_W  (DATA_W),
      .CTR_W   (CTR_W),
      .STALL_W (STALL_W)
  ) u_exec (
      .op_set     (op_set),
      .op_setoe   (op_setoe),
      .op_out     (op_out),
      .op_in      (op_in),
      .op_wait    (op_wait),
      .op_jpin    (op_jpin),
      .op_jmp     (op_jmp),
      .op_djnz    (op_djnz),
      .op_mov     (op_mov),
      .op_delay   (op_delay),
      .op_pull    (op_pull),
      .op_push    (op_push),
      .op_vote    (op_vote),
      .dly        (d_dly),
      .pin        (d_pin),
      .n          (d_n),
      .lvl        (d_lvl),
      .to_en      (d_to_en),
      .to_ctr     (d_to_ctr),
      .cond       (d_cond),
      .ctr_sel    (d_ctr_sel),
      .src        (d_src),
      .nb         (d_nb),
      .target     (d_target),
      .imm        (d_imm),
      .pc         (pc_all[cur*AW+:AW]),
      .ctr0       (ctr0_all[cur*CTR_W+:CTR_W]),
      .ctr1       (ctr1_all[cur*CTR_W+:CTR_W]),
      .osr        (osr_all[cur*SR_W+:SR_W]),
      .osr_cnt    (osr_cnt_all[cur*CNT_W+:CNT_W]),
      .isr        (isr_all[cur*SR_W+:SR_W]),
      .c          (c_all[cur]),
      .to         (to_all[cur]),
      .shift_msb  (ctx_shift_msb[cur]),
      .vote_avail (VMASK[cur]),
      .pin_in     (pin_in),
      .txf_rdata  (txf_rdata),
      .txf_empty  (txf_empty),
      .rxf_full   (rxf_full),
      .done       (x_done),
      .pc_n       (x_pc),
      .ctr0_n     (x_ctr0),
      .ctr1_n     (x_ctr1),
      .osr_n      (x_osr),
      .osr_cnt_n  (x_osr_cnt),
      .isr_n      (x_isr),
      .c_n        (x_c),
      .to_n       (x_to),
      .stall_n    (x_stall),
      .pin_wr_en  (x_pin_wr_en),
      .pin_wr_mask(pin_wr_mask),
      .pin_wr_val (pin_wr_val),
      .pin_oe_en  (x_pin_oe_en),
      .pin_oe_mask(pin_oe_mask),
      .pin_oe_val (pin_oe_val),
      .txf_rd     (x_txf_rd),
      .rxf_wr     (x_rxf_wr),
      .rxf_wdata  (rxf_wdata),
      .vote_start (x_vote_start),
      .vote_pin   (x_vote_pin)
  );

  // Side effects only happen when the slot's context actually issues.
  assign pin_wr_en  = issue & x_pin_wr_en;
  assign pin_oe_en  = issue & x_pin_oe_en;
  assign pin_wr_ctx = cur;
  assign txf_rd     = issue & x_txf_rd;
  assign rxf_wr     = issue & x_rxf_wr;

  // ------------------------------------------- contexts and vote units
  genvar g;
  generate
    for (g = 0; g < NUM_CTX; g = g + 1) begin : g_ctx
      wire this_issue = issue & (cur == g[CTX_W-1:0]);

      pe_ctx #(
          .AW     (AW),
          .SR_W   (SR_W),
          .CTR_W  (CTR_W),
          .STALL_W(STALL_W)
      ) u_ctx (
          .clk      (clk),
          .rst      (rst),
          .run      (ctx_run[g]),
          .restart  (ctx_restart[g]),
          .start_pc (ctx_start_pc[g*AW+:AW]),
          .shift_msb(ctx_shift_msb[g]),
          .we       (this_issue),
          .pc_n     (x_pc),
          .ctr0_n   (x_ctr0),
          .ctr1_n   (x_ctr1),
          .osr_n    (x_osr),
          .osr_cnt_n(x_osr_cnt),
          .isr_n    (x_isr),
          .c_n      (x_c),
          .to_n     (x_to),
          .stall_n  (x_stall),
          .vote_busy(vbusy[g]),
          .vote_done(vdone[g]),
          .vote_bit (vbit[g]),
          .pc       (pc_all[g*AW+:AW]),
          .ctr0     (ctr0_all[g*CTR_W+:CTR_W]),
          .ctr1     (ctr1_all[g*CTR_W+:CTR_W]),
          .osr      (osr_all[g*SR_W+:SR_W]),
          .osr_cnt  (osr_cnt_all[g*CNT_W+:CNT_W]),
          .isr      (isr_all[g*SR_W+:SR_W]),
          .c        (c_all[g]),
          .to       (to_all[g]),
          .ready    (ready_all[g])
      );

      if (VMASK[g]) begin : g_vote
        pe_vote #(
            .NUM_PINS(NUM_PINS),
            .N_W     (VOTE_N_W),
            .S_W     (VOTE_S_W)
        ) u_vote (
            .clk    (clk),
            .rst    (rst),
            .start  (this_issue & x_vote_start),
            .pin_sel(x_vote_pin),
            .pin_in (pin_in),
            .n_cfg  (vote_n[g*VOTE_N_W+:VOTE_N_W]),
            .k_cfg  (vote_k[g*VOTE_N_W+:VOTE_N_W]),
            .s_cfg  (vote_s[g*VOTE_S_W+:VOTE_S_W]),
            .busy   (vbusy[g]),
            .done   (vdone[g]),
            .bit_out(vbit[g])
        );
      end else begin : g_no_vote
        assign vbusy[g] = 1'b0;
        assign vdone[g] = 1'b0;
        assign vbit[g]  = 1'b0;
        wire _unused = &{1'b0, vote_n[g*VOTE_N_W+:VOTE_N_W], vote_k[g*VOTE_N_W+:VOTE_N_W],
                         vote_s[g*VOTE_S_W+:VOTE_S_W]};
      end
    end
  endgenerate

  // ------------------------------------------------------------- status
  assign ctx_pc        = pc_all;
  assign ctx_ready     = ready_all;
  assign ctx_vote_busy = vbusy;
  assign ctx_c         = c_all;
  assign ctx_to        = to_all;

  wire _unused = &{1'b0, x_done, op_illegal, x_vote_start, x_vote_pin};

`ifdef FORMAL
  // ------------------------------------------------------------ formal
  // The instruction stream (im_rdata), pins, FIFO flags and all config are
  // FREE inputs, so these hold for ANY program. Sub-module contracts
  // (pe_ctx: writeback only when ready; pe_exec: one-hot op) are asserted
  // here, i.e. proved.
  // * Slot counter runs 0..NUM_SLOTS-1 in order.
  // * Every side effect (pins, FIFOs) happens only when the slot's context
  //   issues, and it is attributed to that context.
  // * Never reads an empty TX FIFO; writes a full RX FIFO only for push nb.
  // * Bounded service: a context that stays ready issues within NUM_SLOTS
  //   cycles, whatever the other contexts do (no starvation).
`ifdef PE_CORE_TOP
  initial assume (rst);
`endif
  reg f_rst_seen = 1'b0;
  always @(posedge clk) if (rst) f_rst_seen <= 1'b1;
  wire f_ok = f_rst_seen & ~rst;
  reg  f_past_ok = 1'b0;
  always @(posedge clk) f_past_ok <= f_ok;

  always @(*) begin
    if (f_ok) begin
      assert (slot <= LAST_SLOT);
      if (pin_wr_en || pin_oe_en || txf_rd || rxf_wr) assert (issue);
      if (issue) assert (slot_valid && ready_all[cur]);
      assert (pin_wr_ctx == cur);
      if (txf_rd) assert (!txf_empty);
      if (rxf_wr && rxf_full) assert (op_push && d_nb);
    end
  end

  always @(posedge clk) begin
    if (f_past_ok && f_ok)
      assert (slot == (($past(slot) == LAST_SLOT) ? {SLOT_W{1'b0}} : $past(slot) + 1'b1));
  end

  genvar fg;
  generate
    for (fg = 0; fg < NUM_CTX; fg = fg + 1) begin : g_f_service
      wire f_issue = issue & (cur == fg[CTX_W-1:0]);
      reg [7:0] f_wait;  // cycles this context has been ready without issuing
      always @(posedge clk) begin
        if (rst || f_issue || !ready_all[fg]) f_wait <= 8'd0;
        else f_wait <= f_wait + 1'b1;
      end
      wire [7:0] f_dist = (slot > fg) ? (slot - fg - 1) : (slot + NUM_SLOTS - fg - 1);
      always @(*) begin
        if (f_ok) begin
          assert (f_wait <= f_dist);
          assert (f_wait < NUM_SLOTS);
        end
      end
    end
  endgenerate
`endif

endmodule
