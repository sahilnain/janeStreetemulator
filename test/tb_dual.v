`default_nettype none
`timescale 1ns / 1ps

/* Two Protocol Emulators on one clock (test_dual.py): emulator 1 (e1_*) and
   emulator 2 (e2_*). The test wires their protocol pins to each other. */
module tb_dual ();

  initial begin
    $dumpfile("tb_dual.fst");
    $dumpvars(0, tb_dual);
    #1;
  end

  reg clk;
  reg e1_rst_n, e2_rst_n, e1_ena, e2_ena;
  reg [7:0] e1_ui_in, e2_ui_in, e1_uio_in, e2_uio_in;
  wire [7:0] e1_uo_out, e2_uo_out, e1_uio_out, e2_uio_out, e1_uio_oe, e2_uio_oe;

  tt_um_sahil_protoemu emu1 (
      .ui_in  (e1_ui_in),
      .uo_out (e1_uo_out),
      .uio_in (e1_uio_in),
      .uio_out(e1_uio_out),
      .uio_oe (e1_uio_oe),
      .ena    (e1_ena),
      .clk    (clk),
      .rst_n  (e1_rst_n)
  );

  tt_um_sahil_protoemu emu2 (
      .ui_in  (e2_ui_in),
      .uo_out (e2_uo_out),
      .uio_in (e2_uio_in),
      .uio_out(e2_uio_out),
      .uio_oe (e2_uio_oe),
      .ena    (e2_ena),
      .clk    (clk),
      .rst_n  (e2_rst_n)
  );

endmodule
