// =============================================================================
// FILE      : stepper_axes.v
// PROJECT   : FPGA Drone GCS Motion Controller
// TARGET    : Terasic DE10-Nano (Intel Cyclone V)
// CLOCK     : 50 MHz
//
// PURPOSE
//   Controls 4 TB6600 stepper motor drivers organised as 2 physical axes.
//
//   Axis 1 (Pan / Azimuth)  : M1 + M2 driven in lock-step from Joystick ch0
//   Axis 2 (Tilt/Elevation) : M3 + M4 driven in lock-step from Joystick ch1
//
//   If M2 must spin opposite to M1 (motors face each other on the same shaft)
//   invert m2_dir_out at the top-level:  assign m2_dir_out = ~ax1_dir_sig;
//
// ADC
//   External 12-bit SPI ADC (e.g. MCP3204 / AD7994).
//   4 channels scanned in round-robin.  Only ch0 and ch1 are used for motion;
//   ch2/ch3 are sampled and available for future use (e.g. pot feedback).
//
// SPEED MODES (cycled by push-button, active-low)
//   Mode 0  →  250 Hz pulse rate  (~4.7 RPM @ 1/16-step, 200-step motor)
//   Mode 1  →  375 Hz pulse rate  (~7.0 RPM)
//   Mode 2  →  500 Hz pulse rate  (~9.4 RPM)
//
// CALIBRATION
//   On power-up the ADC is sampled for 5 seconds; the mean value becomes the
//   deadband centre for each joystick axis.  Deadband = ±200 counts.
// =============================================================================

module stepper_axes (
    input wire clk_50mhz_in,

    // ── ADC SPI interface ────────────────────────────────────────────────────
    output reg  adc_convst_out,
    output reg  adc_sck_out,
    output reg  adc_sdi_out,
    input  wire adc_sdo_in,

    // ── Speed-select buttons (active-low, internal pull-up recommended) ──────
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_ax1_speed_in,     // Axis 1 speed cycle
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_ax2_speed_in,     // Axis 2 speed cycle

    // ── Axis 1 stepper outputs (M1 + M2 synchronized) ───────────────────────
    output reg m1_dir_out, output reg m1_pul_out,   // primary
    output reg m2_dir_out, output reg m2_pul_out,   // mirror of M1

    // ── Axis 2 stepper outputs (M3 + M4 synchronized) ───────────────────────
    output reg m3_dir_out, output reg m3_pul_out,   // primary
    output reg m4_dir_out, output reg m4_pul_out    // mirror of M3
);

    // =========================================================================
    // 1. ADC SPI STATE MACHINE
    //    Generates a 1 MHz SPI clock from 50 MHz (÷50 tick enable).
    //    Scans 4 channels in round-robin.
    // =========================================================================

    // --- Tick divider: fires once every 50 clock cycles (1 µs) ---------------
    reg [5:0] tick_div_reg = 0;
    reg       spi_en_reg   = 0;

    always @(posedge clk_50mhz_in) begin
        if (tick_div_reg >= 6'd49) begin
            tick_div_reg <= 0;
            spi_en_reg   <= 1'b1;
        end else begin
            tick_div_reg <= tick_div_reg + 1'b1;
            spi_en_reg   <= 1'b0;
        end
    end

    // --- SPI sequencer --------------------------------------------------------
    reg [4:0]  spi_state_reg = 0;
    reg [3:0]  bit_idx_reg   = 0;
    reg [2:0]  ch_idx_reg    = 0;
    reg [11:0] shift_in_reg  = 0;
    reg [11:0] cmd_out_reg   = 0;

    reg [11:0] ch0_val_reg = 12'd2048;   // initialised to mid-scale
    reg [11:0] ch1_val_reg = 12'd2048;
    reg [11:0] ch2_val_reg = 12'd2048;   // spare — available for feedback
    reg [11:0] ch3_val_reg = 12'd2048;   // spare

    always @(posedge clk_50mhz_in) begin
        if (spi_en_reg) begin
            case (spi_state_reg)

                // Assert CONVST to start conversion
                0: begin
                    adc_convst_out <= 1'b1;
                    adc_sck_out    <= 1'b0;
                    spi_state_reg  <= 1;
                end

                // Deassert CONVST, load channel command word
                1: begin
                    adc_convst_out <= 1'b0;
                    bit_idx_reg    <= 0;
                    case (ch_idx_reg)
                        0: cmd_out_reg <= 12'b100010_000000;  // CH0
                        1: cmd_out_reg <= 12'b110010_000000;  // CH1
                        2: cmd_out_reg <= 12'b100110_000000;  // CH2
                        3: cmd_out_reg <= 12'b110110_000000;  // CH3
                        default: cmd_out_reg <= 12'b100010_000000;
                    endcase
                    spi_state_reg <= 2;
                end

                // SCK low — drive SDI
                2: begin
                    adc_sck_out   <= 1'b0;
                    adc_sdi_out   <= cmd_out_reg[11 - bit_idx_reg];
                    spi_state_reg <= 3;
                end

                // SCK high — sample SDO
                3: begin
                    adc_sck_out                  <= 1'b1;
                    shift_in_reg[11 - bit_idx_reg] <= adc_sdo_in;
                    if (bit_idx_reg == 11)
                        spi_state_reg <= 4;
                    else begin
                        bit_idx_reg   <= bit_idx_reg + 1'b1;
                        spi_state_reg <= 2;
                    end
                end

                // Store result (pipeline offset: result lags channel by 1)
                4: begin
                    adc_sck_out <= 1'b0;
                    case (ch_idx_reg)
                        0: ch3_val_reg <= shift_in_reg;
                        1: ch0_val_reg <= shift_in_reg;
                        2: ch1_val_reg <= shift_in_reg;
                        3: ch2_val_reg <= shift_in_reg;
                    endcase
                    ch_idx_reg    <= (ch_idx_reg == 3) ? 3'd0 : ch_idx_reg + 1'b1;
                    spi_state_reg <= 0;
                end

                default: spi_state_reg <= 0;
            endcase
        end
    end

    // =========================================================================
    // 2. JOYSTICK CALIBRATION  (5 second window at power-up)
    //    Continuously updates centre registers for the first 250 M cycles.
    // =========================================================================

    reg [31:0] calib_timer_reg = 0;
    reg        calib_done_reg  = 0;

    reg [11:0] ax1_center_reg = 12'd2048;
    reg [11:0] ax2_center_reg = 12'd2048;

    always @(posedge clk_50mhz_in) begin
        if (calib_timer_reg < 32'd250_000_000) begin
            calib_timer_reg <= calib_timer_reg + 1'b1;
            calib_done_reg  <= 1'b0;
            ax1_center_reg  <= ch0_val_reg;   // track ch0 during calibration
            ax2_center_reg  <= ch1_val_reg;   // track ch1 during calibration
        end else begin
            calib_done_reg  <= 1'b1;
        end
    end

    // =========================================================================
    // 3. BUTTON DEBOUNCERS  (20 ms, active-low, one-shot pulse on falling edge)
    // =========================================================================

    reg [19:0] db_timer_ax1_reg = 0;
    reg [19:0] db_timer_ax2_reg = 0;
    reg        state_ax1_reg    = 1;
    reg        state_ax2_reg    = 1;
    reg        pulse_ax1_reg    = 0;
    reg        pulse_ax2_reg    = 0;

    always @(posedge clk_50mhz_in) begin
        pulse_ax1_reg <= 0;
        pulse_ax2_reg <= 0;

        // Axis 1 speed button
        if (btn_ax1_speed_in != state_ax1_reg) begin
            db_timer_ax1_reg <= db_timer_ax1_reg + 1'b1;
            if (db_timer_ax1_reg >= 20'd1_000_000) begin
                state_ax1_reg    <= btn_ax1_speed_in;
                db_timer_ax1_reg <= 0;
                if (btn_ax1_speed_in == 1'b0) pulse_ax1_reg <= 1'b1;
            end
        end else
            db_timer_ax1_reg <= 0;

        // Axis 2 speed button
        if (btn_ax2_speed_in != state_ax2_reg) begin
            db_timer_ax2_reg <= db_timer_ax2_reg + 1'b1;
            if (db_timer_ax2_reg >= 20'd1_000_000) begin
                state_ax2_reg    <= btn_ax2_speed_in;
                db_timer_ax2_reg <= 0;
                if (btn_ax2_speed_in == 1'b0) pulse_ax2_reg <= 1'b1;
            end
        end else
            db_timer_ax2_reg <= 0;
    end

    // =========================================================================
    // 4. SPEED MODE STATE MACHINES  (0=Slow, 1=Med, 2=Fast)
    //
    //    delay (cycles) = (50 MHz / (2 × pulse_frequency)) − 1
    //    Mode 0: 250 Hz  → delay = 99999  cycles
    //    Mode 1: 375 Hz  → delay = 66666  cycles
    //    Mode 2: 500 Hz  → delay = 49999  cycles
    // =========================================================================

    reg [1:0]  speed_mode_ax1_reg = 0;
    reg [1:0]  speed_mode_ax2_reg = 0;
    reg [31:0] delay_ax1_reg      = 32'd99999;
    reg [31:0] delay_ax2_reg      = 32'd99999;

    always @(posedge clk_50mhz_in) begin

        // Cycle speed mode on each button press
        if (pulse_ax1_reg)
            speed_mode_ax1_reg <= (speed_mode_ax1_reg == 2'd2) ? 2'd0
                                                                 : speed_mode_ax1_reg + 1'b1;
        if (pulse_ax2_reg)
            speed_mode_ax2_reg <= (speed_mode_ax2_reg == 2'd2) ? 2'd0
                                                                 : speed_mode_ax2_reg + 1'b1;

        // Combinatorially update delay register
        case (speed_mode_ax1_reg)
            2'd0: delay_ax1_reg <= 32'd99999;   // 250 Hz
            2'd1: delay_ax1_reg <= 32'd66666;   // 375 Hz
            2'd2: delay_ax1_reg <= 32'd49999;   // 500 Hz
        endcase

        case (speed_mode_ax2_reg)
            2'd0: delay_ax2_reg <= 32'd99999;
            2'd1: delay_ax2_reg <= 32'd66666;
            2'd2: delay_ax2_reg <= 32'd49999;
        endcase
    end

    // =========================================================================
    // 5. MOTOR CONTROL  (runs only after calibration window)
    //
    //    Joystick deadband: ±200 ADC counts from centre.
    //    Within deadband → pulse output held LOW (motor stopped).
    //    M1 = M2 (Axis 1)   M3 = M4 (Axis 2) — mirror direction & pulse.
    // =========================================================================

    reg [31:0] timer_ax1_reg = 0;
    reg [31:0] timer_ax2_reg = 0;

    initial begin
        m1_pul_out = 0; m2_pul_out = 0;
        m3_pul_out = 0; m4_pul_out = 0;
        m1_dir_out = 1; m2_dir_out = 1;
        m3_dir_out = 1; m4_dir_out = 1;
    end

    always @(posedge clk_50mhz_in) begin
        if (calib_done_reg) begin

            // --- Axis 1 : M1 + M2 -------------------------------------------
            if (ch0_val_reg > (ax1_center_reg + 12'd200)) begin
                // Joystick pushed — forward direction
                m1_dir_out <= 1'b1;
                m2_dir_out <= 1'b1;
                if (timer_ax1_reg >= delay_ax1_reg) begin
                    timer_ax1_reg <= 0;
                    m1_pul_out    <= ~m1_pul_out;
                    m2_pul_out    <= ~m2_pul_out;
                end else
                    timer_ax1_reg <= timer_ax1_reg + 1'b1;

            end else if (ch0_val_reg < (ax1_center_reg - 12'd200)) begin
                // Joystick pulled — reverse direction
                m1_dir_out <= 1'b0;
                m2_dir_out <= 1'b0;
                if (timer_ax1_reg >= delay_ax1_reg) begin
                    timer_ax1_reg <= 0;
                    m1_pul_out    <= ~m1_pul_out;
                    m2_pul_out    <= ~m2_pul_out;
                end else
                    timer_ax1_reg <= timer_ax1_reg + 1'b1;

            end else begin
                // Inside deadband — stop
                m1_pul_out    <= 1'b0;
                m2_pul_out    <= 1'b0;
                timer_ax1_reg <= 0;
            end

            // --- Axis 2 : M3 + M4 -------------------------------------------
            if (ch1_val_reg > (ax2_center_reg + 12'd200)) begin
                m3_dir_out <= 1'b1;
                m4_dir_out <= 1'b1;
                if (timer_ax2_reg >= delay_ax2_reg) begin
                    timer_ax2_reg <= 0;
                    m3_pul_out    <= ~m3_pul_out;
                    m4_pul_out    <= ~m4_pul_out;
                end else
                    timer_ax2_reg <= timer_ax2_reg + 1'b1;

            end else if (ch1_val_reg < (ax2_center_reg - 12'd200)) begin
                m3_dir_out <= 1'b0;
                m4_dir_out <= 1'b0;
                if (timer_ax2_reg >= delay_ax2_reg) begin
                    timer_ax2_reg <= 0;
                    m3_pul_out    <= ~m3_pul_out;
                    m4_pul_out    <= ~m4_pul_out;
                end else
                    timer_ax2_reg <= timer_ax2_reg + 1'b1;

            end else begin
                m3_pul_out    <= 1'b0;
                m4_pul_out    <= 1'b0;
                timer_ax2_reg <= 0;
            end

        end // calib_done_reg
    end

endmodule
