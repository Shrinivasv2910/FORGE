// =============================================================================
// FILE      : servo_axes.v
// PROJECT   : FPGA Drone GCS Motion Controller
// TARGET    : Terasic DE10-Nano (Intel Cyclone V)
// CLOCK     : 50 MHz
//
// PURPOSE
//   Controls 3 Waveshare ST3215 serial-bus smart servos via half-duplex UART.
//   Each servo has its own dedicated UART channel (TX + RX pin).
//
//   Servo A (ID 0x01) — Roll  axis
//   Servo B (ID 0x02) — Pitch axis
//   Servo C (ID 0x03) — Yaw   axis
//
// PROTOCOL  (Waveshare ST3215 / STS series, Mode 3 — relative speed stepping)
//   Packet: FF FF ID LEN CMD ADDR POS_L POS_H TIME_L TIME_H SPD_L SPD_H CHK
//   CMD  = 0x03  (write)
//   ADDR = 0x2A  (goal position register)
//   Mode 3: TIME is ignored; SPD must be > 0.  Position is relative delta.
//
// STEP SIZES  (in raw position units; 32767 = one full revolution)
//   Step 0 (fine)   :    51 units  ≈  0.56°
//   Step 1 (medium) :   512 units  ≈  5.62°
//   Step 2 (coarse) :  5120 units  ≈ 56.25°
//
// BOOT SEQUENCE
//   Each servo_node sends 4 × "move 0 steps" packets at 500 ms intervals to
//   wake and initialise the servo before accepting user commands.
//
// BUG FIX
//   Original code used SERVO_ID = 8'h01 for all three nodes.
//   Fixed: motor A = 0x01, motor B = 0x02, motor C = 0x03.
// =============================================================================

// -----------------------------------------------------------------------------
// Top-level wrapper — instantiates three servo_node sub-modules
// -----------------------------------------------------------------------------
module servo_axes (
    input wire clk_50mhz_in,

    // ── Servo A (Roll, ID 0x01) buttons ─────────────────────────────────────
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sa_step_in,           // cycle step size
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sa_fwd_in,            // step forward
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sa_bwd_in,            // step backward

    // ── Servo B (Pitch, ID 0x02) buttons ────────────────────────────────────
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sb_step_in,
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sb_fwd_in,
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sb_bwd_in,

    // ── Servo C (Yaw, ID 0x03) buttons ──────────────────────────────────────
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sc_step_in,
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sc_fwd_in,
    (* altera_attribute = "-name WEAK_PULL_UP_RESISTOR ON" *)
    input wire btn_sc_bwd_in,

    // ── UART channels (one per servo) ────────────────────────────────────────
    input  wire uart_rx_sa_in,  output wire uart_tx_sa_out,   // Servo A
    input  wire uart_rx_sb_in,  output wire uart_tx_sb_out,   // Servo B
    input  wire uart_rx_sc_in,  output wire uart_tx_sc_out    // Servo C
);

    servo_node #(.SERVO_ID(8'h01)) servo_a (
        .clk_50mhz_in   (clk_50mhz_in),
        .btn_step_in     (btn_sa_step_in),
        .btn_fwd_in      (btn_sa_fwd_in),
        .btn_bwd_in      (btn_sa_bwd_in),
        .rx_serial_in    (uart_rx_sa_in),
        .tx_serial_out   (uart_tx_sa_out)
    );

    servo_node #(.SERVO_ID(8'h02)) servo_b (
        .clk_50mhz_in   (clk_50mhz_in),
        .btn_step_in     (btn_sb_step_in),
        .btn_fwd_in      (btn_sb_fwd_in),
        .btn_bwd_in      (btn_sb_bwd_in),
        .rx_serial_in    (uart_rx_sb_in),
        .tx_serial_out   (uart_tx_sb_out)
    );

    servo_node #(.SERVO_ID(8'h03)) servo_c (
        .clk_50mhz_in   (clk_50mhz_in),
        .btn_step_in     (btn_sc_step_in),
        .btn_fwd_in      (btn_sc_fwd_in),
        .btn_bwd_in      (btn_sc_bwd_in),
        .rx_serial_in    (uart_rx_sc_in),
        .tx_serial_out   (uart_tx_sc_out)
    );

endmodule


// =============================================================================
// SERVO NODE SUB-MODULE
//   Independent controller for one ST3215 servo axis.
//   Parameterised by SERVO_ID so three instances are distinguishable.
// =============================================================================
module servo_node #(
    parameter [7:0] SERVO_ID = 8'h01     // Unique ID programmed into servo
)(
    input wire clk_50mhz_in,
    input wire btn_step_in,              // step-size cycle (active-low)
    input wire btn_fwd_in,               // forward step   (active-low)
    input wire btn_bwd_in,               // backward step  (active-low)
    input wire rx_serial_in,
    output wire tx_serial_out
);

    // =========================================================================
    // A. BUTTON DEBOUNCERS  (20 ms, one-shot on falling edge)
    // =========================================================================

    reg [19:0] db_timer_step_reg = 0;  reg state_step_reg = 1;  reg pulse_step_reg = 0;
    reg [19:0] db_timer_fwd_reg  = 0;  reg state_fwd_reg  = 1;  reg pulse_fwd_reg  = 0;
    reg [19:0] db_timer_bwd_reg  = 0;  reg state_bwd_reg  = 1;  reg pulse_bwd_reg  = 0;

    always @(posedge clk_50mhz_in) begin
        pulse_step_reg <= 0;
        pulse_fwd_reg  <= 0;
        pulse_bwd_reg  <= 0;

        // STEP button
        if (btn_step_in != state_step_reg) begin
            db_timer_step_reg <= db_timer_step_reg + 1'b1;
            if (db_timer_step_reg >= 20'd1_000_000) begin
                state_step_reg    <= btn_step_in;
                db_timer_step_reg <= 0;
                if (btn_step_in == 1'b0) pulse_step_reg <= 1'b1;
            end
        end else db_timer_step_reg <= 0;

        // FWD button
        if (btn_fwd_in != state_fwd_reg) begin
            db_timer_fwd_reg <= db_timer_fwd_reg + 1'b1;
            if (db_timer_fwd_reg >= 20'd1_000_000) begin
                state_fwd_reg    <= btn_fwd_in;
                db_timer_fwd_reg <= 0;
                if (btn_fwd_in == 1'b0) pulse_fwd_reg <= 1'b1;
            end
        end else db_timer_fwd_reg <= 0;

        // BWD button
        if (btn_bwd_in != state_bwd_reg) begin
            db_timer_bwd_reg <= db_timer_bwd_reg + 1'b1;
            if (db_timer_bwd_reg >= 20'd1_000_000) begin
                state_bwd_reg    <= btn_bwd_in;
                db_timer_bwd_reg <= 0;
                if (btn_bwd_in == 1'b0) pulse_bwd_reg <= 1'b1;
            end
        end else db_timer_bwd_reg <= 0;
    end

    // =========================================================================
    // B. STEP SIZE SELECTION  (cycles Fine → Medium → Coarse → Fine …)
    //    Units: raw ST3215 position ticks  (32767 ticks = 360°)
    //    Fine   :    51 ticks  ≈  0.56°
    //    Medium :   512 ticks  ≈  5.62°
    //    Coarse :  5120 ticks  ≈ 56.25°
    // =========================================================================

    reg [1:0]  step_mode_reg       = 0;
    reg [15:0] current_step_size_reg = 16'd51;   // default: fine

    always @(posedge clk_50mhz_in) begin
        if (pulse_step_reg) begin
            case (step_mode_reg)
                2'd0: begin step_mode_reg <= 2'd1; current_step_size_reg <= 16'd512;  end
                2'd1: begin step_mode_reg <= 2'd2; current_step_size_reg <= 16'd5120; end
                2'd2: begin step_mode_reg <= 2'd0; current_step_size_reg <= 16'd51;   end
                default: begin step_mode_reg <= 2'd0; current_step_size_reg <= 16'd51; end
            endcase
        end
    end

    // =========================================================================
    // C. BOOT SEQUENCE + POSITION TRIGGER
    //    Sends 4 "move 0 steps" packets spaced 500 ms apart to wake the servo.
    //    After boot, FWD/BWD buttons trigger relative step commands.
    //
    //    Backward direction: MSB of target_pos_reg = 1 (ST3215 sign bit).
    // =========================================================================

    reg [2:0]  startup_burst_count_reg = 0;
    reg [27:0] startup_timer_reg       = 0;
    reg        startup_done_reg        = 0;

    reg [15:0] target_pos_reg = 0;
    reg        trigger_tx_reg = 0;

    always @(posedge clk_50mhz_in) begin
        trigger_tx_reg <= 1'b0;

        if (!startup_done_reg) begin
            // Boot: fire 0-step packet every 500 ms (25,000,000 cycles)
            startup_timer_reg <= startup_timer_reg + 1'b1;
            if (startup_timer_reg >= 28'd25_000_000) begin
                startup_timer_reg       <= 0;
                target_pos_reg          <= 16'h0000;
                trigger_tx_reg          <= 1'b1;
                startup_burst_count_reg <= startup_burst_count_reg + 1'b1;
                if (startup_burst_count_reg == 3'd3)
                    startup_done_reg <= 1'b1;
            end

        end else begin
            // Normal operation
            if (pulse_fwd_reg) begin
                target_pos_reg <= current_step_size_reg;              // positive delta
                trigger_tx_reg <= 1'b1;
            end else if (pulse_bwd_reg) begin
                target_pos_reg <= current_step_size_reg | 16'h8000;   // negative (MSB=1)
                trigger_tx_reg <= 1'b1;
            end
        end
    end

    // =========================================================================
    // D. PACKET FIELD WIRES  (ST3215 Mode 3 relative-step write)
    //
    //    Speed hard-coded to 2000 steps/sec (0x07D0) — adequate for GCS use.
    //    To make speed configurable, promote spd_l/h to registers driven by
    //    an additional speed control state machine.
    // =========================================================================

    wire [7:0] id_wire   = SERVO_ID;
    wire [7:0] len_wire  = 8'h09;        // payload length
    wire [7:0] cmd_wire  = 8'h03;        // write command
    wire [7:0] addr_wire = 8'h2A;        // goal position register

    wire [7:0] pos_l_wire  = target_pos_reg[7:0];
    wire [7:0] pos_h_wire  = target_pos_reg[15:8];
    wire [7:0] time_l_wire = 8'h00;      // Mode 3: time ignored
    wire [7:0] time_h_wire = 8'h00;
    wire [7:0] spd_l_wire  = 8'hD0;     // 2000 steps/sec LSB
    wire [7:0] spd_h_wire  = 8'h07;     // 2000 steps/sec MSB

    wire [15:0] checksum_sum_wire =
        id_wire + len_wire + cmd_wire + addr_wire +
        pos_l_wire + pos_h_wire +
        time_l_wire + time_h_wire +
        spd_l_wire + spd_h_wire;

    wire [7:0] calculated_checksum_wire = ~(checksum_sum_wire[7:0]);

    // =========================================================================
    // E. TX STATE MACHINE  (byte-by-byte packet transmitter)
    //    13 bytes: FF FF ID LEN CMD ADDR POS_L POS_H
    //              TIME_L TIME_H SPD_L SPD_H CHK
    // =========================================================================

    localparam TX_IDLE      = 2'd0;
    localparam TX_SEND      = 2'd1;
    localparam TX_WAIT_HIGH = 2'd2;
    localparam TX_WAIT_LOW  = 2'd3;

    reg [3:0] byte_index_reg = 0;
    reg [7:0] tx_byte_reg    = 0;
    reg       tx_start_reg   = 0;
    wire      tx_busy_wire;
    reg [1:0] tx_state_reg   = TX_IDLE;

    always @(posedge clk_50mhz_in) begin
        case (tx_state_reg)

            TX_IDLE: begin
                tx_start_reg <= 0;
                if (trigger_tx_reg) begin
                    byte_index_reg <= 0;
                    tx_state_reg   <= TX_SEND;
                end
            end

            TX_SEND: begin
                tx_start_reg <= 1'b1;
                case (byte_index_reg)
                    4'd0:  tx_byte_reg <= 8'hFF;
                    4'd1:  tx_byte_reg <= 8'hFF;
                    4'd2:  tx_byte_reg <= id_wire;
                    4'd3:  tx_byte_reg <= len_wire;
                    4'd4:  tx_byte_reg <= cmd_wire;
                    4'd5:  tx_byte_reg <= addr_wire;
                    4'd6:  tx_byte_reg <= pos_l_wire;
                    4'd7:  tx_byte_reg <= pos_h_wire;
                    4'd8:  tx_byte_reg <= time_l_wire;
                    4'd9:  tx_byte_reg <= time_h_wire;
                    4'd10: tx_byte_reg <= spd_l_wire;
                    4'd11: tx_byte_reg <= spd_h_wire;
                    4'd12: tx_byte_reg <= calculated_checksum_wire;
                    default: tx_byte_reg <= 8'h00;
                endcase
                tx_state_reg <= TX_WAIT_HIGH;
            end

            TX_WAIT_HIGH: begin
                tx_start_reg <= 0;
                if (tx_busy_wire) tx_state_reg <= TX_WAIT_LOW;
            end

            TX_WAIT_LOW: begin
                if (!tx_busy_wire) begin
                    if (byte_index_reg < 4'd12) begin
                        byte_index_reg <= byte_index_reg + 4'd1;
                        tx_state_reg   <= TX_SEND;
                    end else
                        tx_state_reg <= TX_IDLE;
                end
            end

            default: tx_state_reg <= TX_IDLE;
        endcase
    end

    simple_uart_tx #(.CLKS_PER_BIT(50)) tx_inst (
        .clk_50mhz_in    (clk_50mhz_in),
        .send_trigger_in (tx_start_reg),
        .tx_byte_in      (tx_byte_reg),
        .tx_serial_out   (tx_serial_out),
        .is_busy_out     (tx_busy_wire)
    );

    // =========================================================================
    // F. RX LISTENER  (parses status reply to extract actual hardware position)
    //    Reply format: FF FF ID LEN ERR POS_L POS_H SPD_L SPD_H LOAD_L …
    //    Only POS_L (byte index 5) and POS_H (byte index 6) are stored.
    // =========================================================================

    wire rx_ready_wire;
    wire [7:0] rx_byte_wire;

    reg [3:0]  rx_index_reg         = 0;
    reg [15:0] actual_hardware_pos_reg = 0;  // readable via SignalTap / JTAG

    simple_uart_rx #(.CLKS_PER_BIT(50)) rx_inst (
        .clk_50mhz_in  (clk_50mhz_in),
        .rx_serial_in  (rx_serial_in),
        .rx_byte_out   (rx_byte_wire),
        .rx_ready_out  (rx_ready_wire)
    );

    always @(posedge clk_50mhz_in) begin
        if (rx_ready_wire) begin
            if      (rx_byte_wire == 8'hFF && rx_index_reg == 0) rx_index_reg <= 1;
            else if (rx_byte_wire == 8'hFF && rx_index_reg == 1) rx_index_reg <= 2;
            else if (rx_index_reg >= 2) begin
                if (rx_index_reg == 5) actual_hardware_pos_reg[7:0]  <= rx_byte_wire;
                if (rx_index_reg == 6) actual_hardware_pos_reg[15:8] <= rx_byte_wire;
                rx_index_reg <= (rx_index_reg == 7) ? 4'd0 : rx_index_reg + 1'b1;
            end else
                rx_index_reg <= 0;
        end
    end

endmodule
