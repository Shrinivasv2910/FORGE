// =============================================================================
// FILE      : uart_common.v
// PROJECT   : FPGA Drone GCS Motion Controller
// TARGET    : Terasic DE10-Nano (Intel Cyclone V)
// CLOCK     : 50 MHz
// BAUD RATE : 1,000,000 bps  (CLKS_PER_BIT = 50)
//
// MODULES
//   simple_uart_rx  - 8-N-1 UART receiver
//   simple_uart_tx  - 8-N-1 UART transmitter
//
// NOTES
//   These are shared primitives instantiated by both stepper_axes.v and
//   servo_axes.v.  Keep this file in the same Quartus project source list.
// =============================================================================

// -----------------------------------------------------------------------------
// UART Receiver
// -----------------------------------------------------------------------------
module simple_uart_rx #(
    parameter CLKS_PER_BIT = 50          // 50 MHz / 1 MBaud
)(
    input  wire       clk_50mhz_in,
    input  wire       rx_serial_in,
    output reg  [7:0] rx_byte_out,
    output reg        rx_ready_out       // 1-cycle pulse when byte is ready
);

    localparam IDLE      = 3'b000;
    localparam START_BIT = 3'b001;
    localparam DATA_BITS = 3'b010;
    localparam STOP_BIT  = 3'b011;

    reg [2:0] state_reg       = IDLE;
    reg [6:0] clock_count_reg = 0;
    reg [2:0] bit_index_reg   = 0;

    initial begin
        rx_ready_out = 1'b0;
        rx_byte_out  = 8'd0;
    end

    always @(posedge clk_50mhz_in) begin
        case (state_reg)

            IDLE: begin
                rx_ready_out    <= 1'b0;
                clock_count_reg <= 0;
                bit_index_reg   <= 0;
                if (rx_serial_in == 1'b0)          // start bit detected
                    state_reg <= START_BIT;
            end

            START_BIT: begin
                if (clock_count_reg == (CLKS_PER_BIT / 2)) begin
                    if (rx_serial_in == 1'b0) begin  // still low at mid-point
                        clock_count_reg <= 0;
                        state_reg       <= DATA_BITS;
                    end else
                        state_reg <= IDLE;           // glitch — discard
                end else
                    clock_count_reg <= clock_count_reg + 1'b1;
            end

            DATA_BITS: begin
                if (clock_count_reg < CLKS_PER_BIT - 1)
                    clock_count_reg <= clock_count_reg + 1'b1;
                else begin
                    clock_count_reg             <= 0;
                    rx_byte_out[bit_index_reg]  <= rx_serial_in;
                    if (bit_index_reg < 7)
                        bit_index_reg <= bit_index_reg + 1'b1;
                    else
                        state_reg <= STOP_BIT;
                end
            end

            STOP_BIT: begin
                if (clock_count_reg < CLKS_PER_BIT - 1)
                    clock_count_reg <= clock_count_reg + 1'b1;
                else begin
                    rx_ready_out    <= 1'b1;
                    clock_count_reg <= 0;
                    state_reg       <= IDLE;
                end
            end

            default: state_reg <= IDLE;
        endcase
    end
endmodule


// -----------------------------------------------------------------------------
// UART Transmitter
// -----------------------------------------------------------------------------
module simple_uart_tx #(
    parameter CLKS_PER_BIT = 50          // 50 MHz / 1 MBaud
)(
    input  wire       clk_50mhz_in,
    input  wire       send_trigger_in,   // 1-cycle pulse to start transmission
    input  wire [7:0] tx_byte_in,
    output reg        tx_serial_out,
    output reg        is_busy_out
);

    localparam IDLE      = 2'b00;
    localparam START_BIT = 2'b01;
    localparam DATA_BITS = 2'b10;
    localparam STOP_BIT  = 2'b11;

    reg [1:0] state_reg       = IDLE;
    reg [7:0] clock_count_reg = 0;
    reg [2:0] bit_index_reg   = 0;
    reg [7:0] data_reg        = 0;

    initial begin
        tx_serial_out = 1'b1;            // idle high
        is_busy_out   = 1'b0;
    end

    always @(posedge clk_50mhz_in) begin
        case (state_reg)

            IDLE: begin
                tx_serial_out   <= 1'b1;
                clock_count_reg <= 0;
                bit_index_reg   <= 0;
                if (send_trigger_in) begin
                    is_busy_out <= 1'b1;
                    data_reg    <= tx_byte_in;
                    state_reg   <= START_BIT;
                end else
                    is_busy_out <= 1'b0;
            end

            START_BIT: begin
                tx_serial_out <= 1'b0;
                if (clock_count_reg < CLKS_PER_BIT - 1)
                    clock_count_reg <= clock_count_reg + 1'b1;
                else begin
                    clock_count_reg <= 0;
                    state_reg       <= DATA_BITS;
                end
            end

            DATA_BITS: begin
                tx_serial_out <= data_reg[bit_index_reg];
                if (clock_count_reg < CLKS_PER_BIT - 1)
                    clock_count_reg <= clock_count_reg + 1'b1;
                else begin
                    clock_count_reg <= 0;
                    if (bit_index_reg < 7)
                        bit_index_reg <= bit_index_reg + 1'b1;
                    else
                        state_reg <= STOP_BIT;
                end
            end

            STOP_BIT: begin
                tx_serial_out <= 1'b1;
                if (clock_count_reg < CLKS_PER_BIT - 1)
                    clock_count_reg <= clock_count_reg + 1'b1;
                else begin
                    clock_count_reg <= 0;
                    state_reg       <= IDLE;
                end
            end

            default: state_reg <= IDLE;
        endcase
    end
endmodule
