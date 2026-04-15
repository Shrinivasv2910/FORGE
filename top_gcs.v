// =============================================================================
// FILE      : top_gcs.v
// PROJECT   : FPGA Drone GCS Motion Controller
// TARGET    : Terasic DE10-Nano (Intel Cyclone V)
// CLOCK     : 50 MHz (FPGA_CLK1_50 — PIN_V11)
//
// DESCRIPTION
//   Top-level module for the FPGA-based Drone Ground Control Station.
//   Combines two independent motion subsystems:
//
//   ┌──────────────────────────────────────────────────────────────────┐
//   │  STEPPER SUBSYSTEM  (stepper_axes)                               │
//   │  • 4 × TB6600 stepper drivers  →  2 synchronized axes           │
//   │  • Axis 1 (Pan / Azimuth)  : M1 + M2  ← ADC ch0 joystick       │
//   │  • Axis 2 (Tilt/Elevation) : M3 + M4  ← ADC ch1 joystick       │
//   │  • 2 push-buttons for speed selection (Slow / Med / Fast)       │
//   │  • 12-bit SPI ADC for joystick reading                          │
//   └──────────────────────────────────────────────────────────────────┘
//   ┌──────────────────────────────────────────────────────────────────┐
//   │  SERVO SUBSYSTEM  (servo_axes)                                   │
//   │  • 3 × Waveshare ST3215 serial-bus servos — dedicated UART each │
//   │  • Servo A (Roll,  ID 0x01)                                     │
//   │  • Servo B (Pitch, ID 0x02)                                     │
//   │  • Servo C (Yaw,   ID 0x03)                                     │
//   │  • 9 push-buttons (step/fwd/bwd per servo)                      │
//   └──────────────────────────────────────────────────────────────────┘
//
// PIN ASSIGNMENT
//   See constraints/de10_nano_pins.qsf
// =============================================================================

module top_gcs (
    input wire clk_50mhz_in,          // FPGA_CLK1_50  (PIN_V11)

    // =========================================================================
    // STEPPER SUBSYSTEM PORTS
    // =========================================================================

    // ADC SPI (e.g., MCP3204-CI/SL or compatible 12-bit SPI ADC)
    output wire adc_convst_out,        // /CS or CONVST
    output wire adc_sck_out,           // SPI clock
    output wire adc_sdi_out,           // MOSI
    input  wire adc_sdo_in,            // MISO

    // Speed buttons (active-low, pull-ups enabled in stepper_axes.v)
    input wire btn_ax1_speed_in,       // Axis 1 speed cycle
    input wire btn_ax2_speed_in,       // Axis 2 speed cycle

    // Axis 1: M1 + M2 (Pan / Azimuth)
    output wire m1_dir_out, output wire m1_pul_out,
    output wire m2_dir_out, output wire m2_pul_out,

    // Axis 2: M3 + M4 (Tilt / Elevation)
    output wire m3_dir_out, output wire m3_pul_out,
    output wire m4_dir_out, output wire m4_pul_out,

    // =========================================================================
    // SERVO SUBSYSTEM PORTS
    // =========================================================================

    // Servo A — Roll axis (ID 0x01)
    input wire btn_sa_step_in,
    input wire btn_sa_fwd_in,
    input wire btn_sa_bwd_in,
    input  wire uart_rx_sa_in,
    output wire uart_tx_sa_out,

    // Servo B — Pitch axis (ID 0x02)
    input wire btn_sb_step_in,
    input wire btn_sb_fwd_in,
    input wire btn_sb_bwd_in,
    input  wire uart_rx_sb_in,
    output wire uart_tx_sb_out,

    // Servo C — Yaw axis (ID 0x03)
    input wire btn_sc_step_in,
    input wire btn_sc_fwd_in,
    input wire btn_sc_bwd_in,
    input  wire uart_rx_sc_in,
    output wire uart_tx_sc_out
);

    // =========================================================================
    // STEPPER SUBSYSTEM INSTANCE
    // =========================================================================
    stepper_axes u_stepper_axes (
        .clk_50mhz_in      (clk_50mhz_in),

        // ADC
        .adc_convst_out    (adc_convst_out),
        .adc_sck_out       (adc_sck_out),
        .adc_sdi_out       (adc_sdi_out),
        .adc_sdo_in        (adc_sdo_in),

        // Speed buttons
        .btn_ax1_speed_in  (btn_ax1_speed_in),
        .btn_ax2_speed_in  (btn_ax2_speed_in),

        // Axis 1
        .m1_dir_out        (m1_dir_out),
        .m1_pul_out        (m1_pul_out),
        .m2_dir_out        (m2_dir_out),
        .m2_pul_out        (m2_pul_out),

        // Axis 2
        .m3_dir_out        (m3_dir_out),
        .m3_pul_out        (m3_pul_out),
        .m4_dir_out        (m4_dir_out),
        .m4_pul_out        (m4_pul_out)
    );

    // =========================================================================
    // SERVO SUBSYSTEM INSTANCE
    // =========================================================================
    servo_axes u_servo_axes (
        .clk_50mhz_in      (clk_50mhz_in),

        // Servo A
        .btn_sa_step_in    (btn_sa_step_in),
        .btn_sa_fwd_in     (btn_sa_fwd_in),
        .btn_sa_bwd_in     (btn_sa_bwd_in),
        .uart_rx_sa_in     (uart_rx_sa_in),
        .uart_tx_sa_out    (uart_tx_sa_out),

        // Servo B
        .btn_sb_step_in    (btn_sb_step_in),
        .btn_sb_fwd_in     (btn_sb_fwd_in),
        .btn_sb_bwd_in     (btn_sb_bwd_in),
        .uart_rx_sb_in     (uart_rx_sb_in),
        .uart_tx_sb_out    (uart_tx_sb_out),

        // Servo C
        .btn_sc_step_in    (btn_sc_step_in),
        .btn_sc_fwd_in     (btn_sc_fwd_in),
        .btn_sc_bwd_in     (btn_sc_bwd_in),
        .uart_rx_sc_in     (uart_rx_sc_in),
        .uart_tx_sc_out    (uart_tx_sc_out)
    );

endmodule
