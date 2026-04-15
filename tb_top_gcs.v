// =============================================================================
// FILE      : tb_top_gcs.v
// PROJECT   : FPGA Drone GCS Motion Controller
// PURPOSE   : Basic simulation testbench for top_gcs.v
//
// RUN WITH  : ModelSim / QuestaSim
//   vlog rtl/uart_common.v rtl/stepper_axes.v rtl/servo_axes.v \
//        rtl/top_gcs.v sim/tb_top_gcs.v
//   vsim tb_top_gcs -voptargs=+acc
//   run 200ms
//
// NOTES
//   • All buttons are held inactive (high) by default.
//   • ADC SDO is tied low (reads 0 → motors stopped; centre = 0 during calib).
//   • After 5 s calib window, Axis 1 joystick goes high (forward motion).
//   • Servo A FWD button is pulsed at t = 6 s to verify packet transmission.
//   • Expected: m1_pul_out / m2_pul_out toggle after calib_done assertion.
// =============================================================================

`timescale 1ns / 1ps

module tb_top_gcs;

    // ─── Clock ───────────────────────────────────────────────────────────────
    reg clk = 0;
    always #10 clk = ~clk;              // 50 MHz (20 ns period)

    // ─── DUT ports ───────────────────────────────────────────────────────────
    // Stepper
    wire adc_convst_out, adc_sck_out, adc_sdi_out;
    reg  adc_sdo_in = 0;
    reg  btn_ax1_speed_in = 1;
    reg  btn_ax2_speed_in = 1;
    wire m1_dir_out, m1_pul_out;
    wire m2_dir_out, m2_pul_out;
    wire m3_dir_out, m3_pul_out;
    wire m4_dir_out, m4_pul_out;

    // Servo
    reg  btn_sa_step_in = 1, btn_sa_fwd_in = 1, btn_sa_bwd_in = 1;
    reg  btn_sb_step_in = 1, btn_sb_fwd_in = 1, btn_sb_bwd_in = 1;
    reg  btn_sc_step_in = 1, btn_sc_fwd_in = 1, btn_sc_bwd_in = 1;
    reg  uart_rx_sa_in = 1, uart_rx_sb_in = 1, uart_rx_sc_in = 1;
    wire uart_tx_sa_out, uart_tx_sb_out, uart_tx_sc_out;

    // ─── DUT ─────────────────────────────────────────────────────────────────
    top_gcs dut (
        .clk_50mhz_in      (clk),
        .adc_convst_out    (adc_convst_out),
        .adc_sck_out       (adc_sck_out),
        .adc_sdi_out       (adc_sdi_out),
        .adc_sdo_in        (adc_sdo_in),
        .btn_ax1_speed_in  (btn_ax1_speed_in),
        .btn_ax2_speed_in  (btn_ax2_speed_in),
        .m1_dir_out        (m1_dir_out), .m1_pul_out (m1_pul_out),
        .m2_dir_out        (m2_dir_out), .m2_pul_out (m2_pul_out),
        .m3_dir_out        (m3_dir_out), .m3_pul_out (m3_pul_out),
        .m4_dir_out        (m4_dir_out), .m4_pul_out (m4_pul_out),
        .btn_sa_step_in    (btn_sa_step_in),
        .btn_sa_fwd_in     (btn_sa_fwd_in),
        .btn_sa_bwd_in     (btn_sa_bwd_in),
        .uart_rx_sa_in     (uart_rx_sa_in),
        .uart_tx_sa_out    (uart_tx_sa_out),
        .btn_sb_step_in    (btn_sb_step_in),
        .btn_sb_fwd_in     (btn_sb_fwd_in),
        .btn_sb_bwd_in     (btn_sb_bwd_in),
        .uart_rx_sb_in     (uart_rx_sb_in),
        .uart_tx_sb_out    (uart_tx_sb_out),
        .btn_sc_step_in    (btn_sc_step_in),
        .btn_sc_fwd_in     (btn_sc_fwd_in),
        .btn_sc_bwd_in     (btn_sc_bwd_in),
        .uart_rx_sc_in     (uart_rx_sc_in),
        .uart_tx_sc_out    (uart_tx_sc_out)
    );

    // ─── Test sequence ────────────────────────────────────────────────────────
    initial begin
        $dumpfile("tb_top_gcs.vcd");
        $dumpvars(0, tb_top_gcs);

        $display("[TB] Simulation start — calibration window 5 s");
        #(5_100_000_000);               // 5.1 s — past calibration window

        // Simulate Axis 1 joystick pushed forward (ADC returns 0xFFF high value)
        // Drive SDO high so shift register captures all-1s → ~4095
        adc_sdo_in = 1;
        $display("[TB] t=%0t  Joystick Axis1 pushed (SDO=1)", $time);
        #(500_000_000);                 // 500 ms — observe m1/m2 pulse toggling

        // Test Axis 1 speed button press
        btn_ax1_speed_in = 0;
        #(25_000_000);                  // 25 ms (past debounce)
        btn_ax1_speed_in = 1;
        $display("[TB] t=%0t  Axis1 speed button pressed → Mode 1 (375 Hz)", $time);
        #(200_000_000);

        // Test Servo A FWD button
        btn_sa_fwd_in = 0;
        #(25_000_000);
        btn_sa_fwd_in = 1;
        $display("[TB] t=%0t  Servo A FWD pressed → TX packet expected", $time);
        #(200_000_000);                 // observe uart_tx_sa_out

        // Test Servo A STEP then BWD
        btn_sa_step_in = 0;
        #(25_000_000);
        btn_sa_step_in = 1;
        #(50_000_000);
        btn_sa_bwd_in = 0;
        #(25_000_000);
        btn_sa_bwd_in = 1;
        $display("[TB] t=%0t  Servo A: stepped to Medium, then BWD", $time);
        #(200_000_000);

        $display("[TB] Simulation complete");
        $finish;
    end

    // ─── Pulse counters for pass/fail check ──────────────────────────────────
    integer m1_pul_count = 0;
    integer sa_tx_edges  = 0;

    always @(posedge m1_pul_out)  m1_pul_count = m1_pul_count + 1;
    always @(negedge uart_tx_sa_out) sa_tx_edges = sa_tx_edges + 1;

    always @(posedge clk) begin
        // Simple sanity monitor: report every 10,000 M1 pulses
        if (m1_pul_count % 10000 == 0 && m1_pul_count > 0)
            $display("[TB] t=%0t  M1 pulse count = %0d", $time, m1_pul_count);
    end

endmodule
