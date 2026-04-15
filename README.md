#FORGE — FPGA-Orchestrated Robot Ground-station Engine

> **FPGA-based Ground Control Station motion controller** — drives 4 stepper motors across 2 synchronized axes (TB6600) and 3 smart servos across 3 axes (Waveshare ST3215) from a single Terasic DE10-Nano board.  
> Developed as part of the **ASCEND / IRoC-U 2026** autonomous drone project, NIT Patna Robotics Lab.

---

## Overview

| Subsystem | Hardware | Axes | Control Input |
|---|---|---|---|
| Stepper | 4 × TB6600 drivers | 2 (M1+M2 / M3+M4 synchronized) | 12-bit SPI ADC joystick |
| Servo | 3 × Waveshare ST3215 | 3 (Roll / Pitch / Yaw) | Push-buttons (step/fwd/bwd per axis) |

All logic runs on the **Intel Cyclone V FPGA** of the DE10-Nano at 50 MHz. No HPS (ARM) involvement — pure RTL control.

---

## Repository Structure

```
de10-nano-drone-gcs/
├── rtl/
│   ├── top_gcs.v          ← Top-level module (instantiates both subsystems)
│   ├── stepper_axes.v     ← 4 steppers in 2 synchronized axes + ADC SPI
│   ├── servo_axes.v       ← 3 ST3215 servos via UART (servo_node × 3)
│   └── uart_common.v      ← Shared simple_uart_tx / simple_uart_rx primitives
├── sim/
│   └── tb_top_gcs.v       ← ModelSim testbench (joystick + button stimulus)
├── constraints/
│   ├── de10_nano_pins.qsf ← Quartus pin assignments + source file list
│   └── de10_nano_timing.sdc ← SDC timing constraints
└── README.md
```

---

## Hardware

### Stepper Subsystem

| Component | Part |
|---|---|
| FPGA Board | Terasic DE10-Nano |
| Driver | TB6600 (×4) |
| Motor | NEMA 17 / NEMA 23 (1.8°/step) |
| ADC | MCP3204 or compatible 12-bit SPI ADC |
| Joystick | Dual-axis analogue (0–3.3 V output) |

**Axis mapping**

| Axis | Motors | Joystick Channel | Application |
|---|---|---|---|
| Axis 1 (Pan) | M1 + M2 | ADC ch0 | Azimuth / Pan |
| Axis 2 (Tilt) | M3 + M4 | ADC ch1 | Elevation / Tilt |

M1 and M2 are driven with **identical DIR and PUL signals** — both step in lock-step.  
If your mechanical arrangement requires M2 to spin in the opposite physical direction (motors facing each other), invert DIR at the connector: `m2_dir_out → /M2_DIR`.

**Speed modes** (cycled per-axis by push-button)

| Mode | Pulse Rate | ~RPM (1/16 step, 200-step motor) |
|---|---|---|
| 0 — Slow | 250 Hz | 4.7 RPM |
| 1 — Medium | 375 Hz | 7.0 RPM |
| 2 — Fast | 500 Hz | 9.4 RPM |

**Calibration:** On power-up, the ADC is sampled for **5 seconds** with the joystick at rest. The mean value becomes the deadband centre. Deadband = ±200 ADC counts (≈ ±5% of full scale).

---

### Servo Subsystem

| Component | Part |
|---|---|
| Servo | Waveshare ST3215 (STS series) |
| Interface | Half-duplex UART @ 1 Mbaud, 8-N-1 |
| Protocol | ST3215 packet protocol, **Mode 3** (relative speed stepping) |
| Speed | Hardcoded 2000 steps/sec |

**Axis mapping**

| Instance | Servo ID | Axis |
|---|---|---|
| `servo_a` | `0x01` | Roll |
| `servo_b` | `0x02` | Pitch |
| `servo_c` | `0x03` | Yaw |

> ⚠️ **Bug fix from original code:** All three servo_node instances previously used `SERVO_ID = 8'h01`. Fixed to `0x01`, `0x02`, `0x03` respectively.

**Step sizes** (cycled by STEP button per axis)

| Mode | Step (ticks) | Angle |
|---|---|---|
| 0 — Fine | 51 | ≈ 0.56° |
| 1 — Medium | 512 | ≈ 5.62° |
| 2 — Coarse | 5120 | ≈ 56.25° |

**Boot sequence:** Each servo receives **4 × "move 0 steps"** packets (500 ms apart) to wake and initialise the servo before accepting user commands.

**Packet structure (Mode 3 relative write)**

```
FF FF  ID  09  03  2A  POS_L POS_H  00 00  D0 07  CHK
       |   |   |   |   |           |      |
       ID  LEN CMD ADDR POSITION   TIME   SPEED(2000)
```

Backward direction: MSB of POS = 1 (ST3215 sign convention).

---

## Pin Assignments (DE10-Nano)

Full assignments are in `constraints/de10_nano_pins.qsf`. Key allocations:

| Signal | GPIO Header | Pin |
|---|---|---|
| `adc_convst_out` | GPIO_0[0] | W12 |
| `adc_sck_out` | GPIO_0[2] | Y8 |
| `adc_sdi_out` | GPIO_0[4] | W11 |
| `adc_sdo_in` | GPIO_0[6] | Y7 |
| `btn_ax1_speed_in` | GPIO_0[8] | AA15 |
| `btn_ax2_speed_in` | GPIO_0[10] | AB15 |
| `m1_dir_out` | GPIO_0[12] | AA14 |
| `m1_pul_out` | GPIO_0[14] | AB14 |
| `m2_dir_out` | GPIO_0[16] | AA13 |
| `m2_pul_out` | GPIO_0[18] | AB13 |
| `m3_dir_out` | GPIO_0[20] | AA12 |
| `m3_pul_out` | GPIO_0[22] | AB12 |
| `m4_dir_out` | GPIO_0[24] | AA11 |
| `m4_pul_out` | GPIO_0[26] | AB11 |
| `uart_tx_sa_out` | GPIO_1[1] | Y16 |
| `uart_rx_sa_in` | GPIO_1[3] | AA23 |
| `uart_tx_sb_out` | GPIO_1[7] | AA21 |
| `uart_rx_sb_in` | GPIO_1[9] | AB21 |
| `uart_tx_sc_out` | GPIO_1[13] | AB19 |
| `uart_rx_sc_in` | GPIO_1[15] | AA18 |

> JP7 and JP8 jumpers on the DE10-Nano must be set to **3.3 V** for GPIO headers.

---

## Build Instructions (Quartus Prime)

### 1. Clone and open project

```bash
git clone https://github.com/<your-username>/de10-nano-drone-gcs.git
cd de10-nano-drone-gcs
```

Open **Quartus Prime** (Lite Edition works; tested on 20.1 and 22.1).

### 2. Create project

```
File → New Project Wizard
  Name:      gcs_motion_ctrl
  Top-level: top_gcs
  Device:    5CSEBA6U23I7  (Cyclone V)
```

### 3. Add source files

```
Project → Add/Remove Files
  rtl/uart_common.v
  rtl/stepper_axes.v
  rtl/servo_axes.v
  rtl/top_gcs.v
```

Or import the .qsf directly:

```
Assignments → Import Assignments → constraints/de10_nano_pins.qsf
```

### 4. Compile and program

```
Processing → Start Compilation   (Ctrl+L)
Tools → Programmer → Start       (USB-Blaster)
```

---

## Simulation (ModelSim)

```bash
cd sim

vlog ../rtl/uart_common.v \
     ../rtl/stepper_axes.v \
     ../rtl/servo_axes.v \
     ../rtl/top_gcs.v \
     tb_top_gcs.v

vsim tb_top_gcs -voptargs=+acc

# In ModelSim console:
add wave -r /*
run 200ms
```

The testbench:
1. Waits 5.1 s (past calibration window)
2. Drives ADC SDO high to simulate joystick deflection → observe M1/M2 pulse toggling
3. Presses Axis 1 speed button → speed mode increments
4. Presses Servo A FWD → observe 13-byte UART packet on `uart_tx_sa_out`
5. Presses STEP then BWD on Servo A → medium-step backward packet

---

## Wiring Notes

### TB6600 Stepper Driver

| TB6600 terminal | FPGA signal |
|---|---|
| PUL+ | `mN_pul_out` (via 330 Ω) |
| PUL− | GND |
| DIR+ | `mN_dir_out` (via 330 Ω) |
| DIR− | GND |
| ENA+ | 5 V (always enabled) |
| ENA− | GND |

Use a **3.3 V → 5 V level shifter** or 330 Ω series resistors to protect FPGA pins.

### ST3215 Servo UART

The ST3215 uses a **single-wire half-duplex** bus. Connect TX and RX to the same servo data line via a **1 kΩ resistor** (TX series) to avoid bus contention:

```
FPGA TX ──[1kΩ]──┬── Servo DATA
FPGA RX ─────────┘
```

---

## Known Limitations / TODO

- [ ] Servo speed is hardcoded to 2000 steps/sec; expose as a parameter or runtime register
- [ ] ADC ch2/ch3 data is read but unused — wire to PID feedback or limit switches
- [ ] No software end-stop / limit switch logic yet
- [ ] `actual_hardware_pos_reg` in each servo_node is only accessible via SignalTap JTAG — add a readback mux for display
- [ ] M2/M4 DIR inversion flag (parameter) for mechanically inverted motor pairs

---

## License

MIT License — see `LICENSE` for details.

---

## Acknowledgements

Developed at **NIT Patna Mechatronics & Automation Engineering** lab as part of the ASCEND autonomous GPS-denied quadrotor project for the IRoC-U ISRO Robotics Challenge 2026.

**Faculty Mentor:** Dr. Golak Bihari Mahanta, Assistant Professor, Dept. of Mechatronics & Automation Engineering, NIT Patna
