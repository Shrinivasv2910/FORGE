# =============================================================================
# FILE    : de10_nano_timing.sdc
# PROJECT : FPGA Drone GCS Motion Controller
# =============================================================================

# 50 MHz system clock
create_clock -period 20.000 -name clk_50mhz [get_ports clk_50mhz_in]

# Relax all input/output delays relative to the system clock
set_input_delay  -clock clk_50mhz -max 3.0 [all_inputs]
set_output_delay -clock clk_50mhz -max 3.0 [all_outputs]

# False paths: asynchronous button inputs (debounce handles synchronisation)
set_false_path -from [get_ports btn_ax1_speed_in]
set_false_path -from [get_ports btn_ax2_speed_in]
set_false_path -from [get_ports btn_sa_step_in]
set_false_path -from [get_ports btn_sa_fwd_in]
set_false_path -from [get_ports btn_sa_bwd_in]
set_false_path -from [get_ports btn_sb_step_in]
set_false_path -from [get_ports btn_sb_fwd_in]
set_false_path -from [get_ports btn_sb_bwd_in]
set_false_path -from [get_ports btn_sc_step_in]
set_false_path -from [get_ports btn_sc_fwd_in]
set_false_path -from [get_ports btn_sc_bwd_in]

# False paths: UART RX inputs (asynchronous serial)
set_false_path -from [get_ports uart_rx_sa_in]
set_false_path -from [get_ports uart_rx_sb_in]
set_false_path -from [get_ports uart_rx_sc_in]
set_false_path -from [get_ports adc_sdo_in]
