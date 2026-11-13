# Tacoma-ATC700-Standalone (v0.1 — DRAFT, NOT DRIVE-READY)

Standalone transfer-case controller for running a **BMW E70 ATC700** electronic
transfer case in a **2016 Toyota Tacoma** swap. An ESP32 reads the Tacoma's
factory 4WD selector dial and CAN bus, and closed-loop drives the ATC700's
DC servo motor (via BTS7960) using its Hall encoder — no BMW modules required.

> ⚠️ **This is a v0.1 rough draft.** Every value tagged `[CALIBRATE]` in
> `include/config.h` is a placeholder. Bench-test everything before the servo
> is ever connected to a driveline, and never trust placeholder CAN IDs.

## How it works

```
 Tacoma 4WD dial ──► voltage divider ──► ADC (D0)          ┌──────────────┐
 Tacoma CAN bus ───► MCP2515 (SPI) ────► CORE 0 task  ──► │  FreeRTOS    │
 ACS712 current ───────────────────────► ADC (D1)          │  ESP32-S3    │
                                                           │              │
 ATC700 Hall A/B ◄─────────────────────► CORE 1 task       │              │
 ATC700 servo   ◄──── BTS7960 (R/L EN + PWM) ◄── 100 Hz PID└──────────────┘
```

- **CORE 0** sniffs CAN (front/rear wheel speed, TPS) and bands the dial ADC
  into 2WD / 4HI / 4LO. SPI and Serial latency can never touch the control loop.
- **CORE 1** runs the 100 Hz PID that positions the clutch actuator,
  quadrature-counts the Hall encoder in an ISR, and enforces current/encoder
  watchdogs.
- **Boot homing:** drive OPEN (reverse) at low duty until the ACS712 sees a
  stall spike → back off a known distance → that position becomes tick 0.

### Dial states
| Dial | Meaning | Servo target |
|------|-----------------|--------------------------------------|
| 2WD | clutch open | 0 ticks |
| 4HI | sport AWD | **dynamic slip logic** (below) |
| 4LO | locked | `ENC_MAX_TICKS` (100 % clamp) |

### 4HI dynamic slip logic
| Rule | Behavior |
|------|----------|
| Base preload | always 10 % clamp |
| Predictive | TPS > 60 % → command 100 % |
| Proportional | slip = rear − front; over 8 % of front speed, +15 % clamp per 1 mph |
| Derivative panic | slip jumps > 5 mph in one 10 ms loop → 100 %, held ~1.2 s |
| Decay | ramp back to 10 % when traction returns (~1 s from full) |

### PID safety
- ±15-tick deadband (zero PWM, pack rests)
- Integral clamp + conditional integration (anti-windup)
- Output hard-clamped to ±255 (8-bit LEDC can't overflow; enforced again by
  `static_assert` at compile time)

## Hardware / wiring

**Board: Seeed Studio XIAO ESP32-S3** — must be the **S3** (dual core).
The XIAO ESP32-C3/C6 are single-core and cannot run the pinned task split.

| XIAO pad | GPIO | Function | Connects to |
|----------|------|-------------------|-------------|
| D0 | 1 | Dial ADC (ADC1) | Tacoma 4WD-switch tap via divider |
| D1 | 2 | Current ADC (ADC1) | ACS712 OUT |
| D2 | 3 | Hall A | ATC700 encoder ch A |
| D3 | 4 | Hall B | ATC700 encoder ch B |
| D4 | 5 | MCP2515 CS | CAN module SLAVE SELECT |
| D5 | 6 | MCP2515 INT *(optional)* | CAN module INT |
| D6 | 43 | BTS R_PWM | BTS7960 R_PWM |
| D7 | 44 | BTS L_PWM | BTS7960 L_PWM |
| D8 | 7 | SPI SCK | CAN module SCK |
| D9 | 8 | SPI MISO | CAN module MISO |
| D10 | 9 | SPI MOSI | CAN module MOSI |
| D11 | 42 | BTS R_EN | BTS7960 R_EN |
| D12 | 41 | BTS L_EN | BTS7960 L_EN |
| 3V3 | — | power out | ACS712 VCC (3.3 V!), MCP2515 VCC |
| GND | — | common ground | BTS signal GND, ACS GND, CAN GND |

### Non-negotiable wiring rules
1. **10 kΩ pulldowns on R_EN and L_EN** at the BTS board. GPIO43/44 are ROM
   UART pins that can twitch at reset; the driver must stay disabled unless
   the firmware actively holds the EN lines high.
2. **Signal ground must be common** between ESP32, BTS7960 logic side, and
   the MCP2515 module. The BTS motor power (battery +) stays separate from
   everything 3.3 V.
3. **ACS712 VCC to 3V3**, not 5 V — its 0-A midpoint then sits mid-scale of
   the ESP32 ADC, and its output can never over-volts the pin. Sense loop in
   series with ONE servo motor lead.
4. **Opto-isolate or verify logic levels on the BTS R_PWM/L_PWM/EN** — many
   BTS7960 breakout boards are fine with 3.3 V logic; cheap bare-module
   clones sometimes aren't. Check your board's datasheet.
5. **MCP2515 modules must be the 16 MHz crystal variant** (the common blue
   TJA1050 boards are). An 8 MHz module needs `CAN_CRYSTAL_MHZ` set to 8.
6. **One 120 Ω termination** across CAN-H/CAN-L at the ESP32 end. The
   Tacoma's own terminations stay in place; do NOT add a second one if your
   tap is mid-harness — measure ~60 Ω between H and L with the truck off.

### Powering the servo
The ATC700 motor draws stall currents that can exceed 30 A briefly. Feed the
BTS7960 directly from the battery through a **40 A fuse**, keep the motor
leads short/twisted, and add a big cap (≥1000 µF) across the BTS power input.
The ESP32 must be powered from a clean, regulated 5 V (DC-DC from battery,
NOT tapped off ignition-sensitive loads) — brownouts mid-homing are how
encoders lose their mind.

## Build & flash

```bash
pio run                    # compile
pio run -t upload          # flash over USB-C
pio device monitor         # 115200 baud telemetry
```

Serial telemetry (4 Hz):
```
[T] dial=1 F=34.2 R=36.9 TPS= 42% lock= 36.4% tgt=  437 enc=  441 I= 0.8A
```

## Calibration checklist (before ANY road use)

Work these in order. Values go into `include/config.h`.

1. **ACS712** — confirm `ACS_SENSITIVITY_MV_A` matches your module
   (5 A→185, 20 A→100, 30 A→66). Watch the boot line
   `[I] Zero offset: ... ok`. Measure the 3V3 rail and set `ADC_VREF_CAL_MV`.
2. **Encoder direction** — rotate the servo by hand toward LOCK; encoder
   count must INCREASE on serial. If not, flip the sign in `hallA_isr()`
   (`src/motor_task.cpp`).
3. **Homing** — bench the servo with free travel. Confirm it backs off to
   the OPEN stop, detects the spike, and reports
   `Tick 0 = OPEN stop + N`. Tune `HOME_PWM` / `HOME_STALL_AMPS` until it's
   reliable and gentle.
4. **`ENC_MAX_TICKS`** — command 4LO on the bench, read the tick count at
   the closed stop, enter it. Then verify 2WD/4LO targets land inside the
   stops with the ±15 deadband never pressing on one.
5. **Dial bands** — probe D0 with a meter in each dial detent; set
   `DIAL_*_MV`. Serial prints `[DIAL] x -> y` on every transition.
6. **CAN IDs** — the most important step. The IDs in `config.h` are
   **placeholders**. With the debug log on, every unrecognized frame prints
   at ~2 Hz (`[CAN?] id=0x... data=...`). Drive slowly next to a GPS
   reference and find which 16-bit fields track speed; find the TPS byte by
   watching it move with throttle. Fill in the IDs, byte offsets, and
   `WHEEL_KPH_PER_LSB`.
7. **Stall current on-vehicle** — re-verify `HOME_STALL_AMPS` with the servo
   bolted to the transfer case linkage, not free-spinning.

## File map

```
platformio.ini          build config (espressif32 @ 6.9.0, XIAO ESP32S3)
include/config.h        ALL pins, gains, slip constants — calibrate here
include/globals.h       shared structs + externs between tasks
src/main.cpp            globals, current sensing, homing, task spawn
src/can_task.cpp        CORE 0: CAN sniffer + dial bander + telemetry
src/motor_task.cpp      CORE 1: hall ISR, BTS7960, PID, slip logic
```

## Known v0.1 limitations (roadmap)
- Open-loop current during homing beyond the stall check (no torque control).
- No persistence — every boot re-homes (good for safety, slow by ~3 s).
- Slip logic ignores yaw rate and front-slip (brake-induced); rear-positive
  slip only.
- No watchdog over the FreeRTOS tasks themselves (add esp_task_wdt next).
- Hall A only interrupts at RISING edge (2× decoding possible).

## Safety
This controls a locking differential path in a moving vehicle. Failsafe
default is **less lockup**, and the driver is un-armed at boot and on every
fault. Still: test in a field, not traffic. The Tacoma's factory 4WD ECU
should remain connected to its own harness for dash lights; this project
listens on the bus but does not answer for it.
