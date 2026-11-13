/**
 * =====================================================================
 *  Tacoma-ATC700-Standalone — config.h   (v0.1 DRAFT)
 *  ALL pin definitions, tuning constants, and AWD slip-logic knobs.
 *
 *  Everything marked  [CALIBRATE]  is a placeholder that you MUST verify
 *  against the real truck / real ATC700 before driving it under load.
 *  Nothing here should be trusted on a bench-motor-only build either.
 * =====================================================================
 */
#ifndef TACS_CONFIG_H
#define TACS_CONFIG_H

#include <Arduino.h>

/* =====================================================================
 * 1. PIN MAP — Seeed XIAO ESP32-S3
 * =====================================================================
 *  XIAO pad -> GPIO (verified against the Seeed wiki pin map):
 *    D0=GPIO1  D1=GPIO2  D2=GPIO3  D3=GPIO4  D4=GPIO5  D5=GPIO6
 *    D6=GPIO43 D7=GPIO44 D8=GPIO7  D9=GPIO8  D10=GPIO9
 *    D11=GPIO42 (back pad)  D12=GPIO41 (back pad)
 *
 *  ADC note: on the ESP32-S3, ADC1 = GPIO1..GPIO10 (usable while the
 *  radio is on). ADC2 (GPIO41/42 are NOT ADC1; D11/D12 have no ADC at
 *  all per Seeed) — so BOTH analog inputs live on D0/D1 (ADC1).
 *
 *  SAFETY NOTE: GPIO43/44 (D6/D7) are ROM UART0 pins. The bootloader may
 *  toggle TX0 at reset. The BTS7960 enable lines therefore carry external
 *  10k pulldowns and the driver stays DISABLED until firmware explicitly
 *  arms it (see README "Wiring").
 * ===================================================================== */

/* --- Mode selector: Tacoma 4WD dial -> voltage divider -> ADC1 ------ */
#define PIN_DIAL_ADC            1     /* D0  (ADC1_CH0)                  */

/* --- Failsafe: ACS712 current sensor output -> ADC1 ------------------ */
#define PIN_CURRENT_ADC         2     /* D1  (ADC1_CH1)                  */

/* --- ATC700 Hall encoder (quadrature A/B) ---------------------------- */
#define PIN_HALL_A              3     /* D2  (interrupt, both edges)     */
#define PIN_HALL_B              4     /* D3  (direction read in ISR)     */

/* --- MCP2515 CAN module on HSPI-style SPI ---------------------------- */
#define PIN_CAN_CS              5     /* D4                              */
#define PIN_CAN_INT             6     /* D5  (optional; polled fallback) */
#define PIN_CAN_SCK             7     /* D8  (XIAO's native SPI SCK)     */
#define PIN_CAN_MISO            8     /* D9                              */
#define PIN_CAN_MOSI            9     /* D10                             */
#define CAN_CRYSTAL_MHZ         16    /* 16 MHz module (blue PCB)        */
#define CAN_BUS_KBPS            500   /* 2016 Tacoma bus speed           */

/* --- BTS7960 full-bridge driver -------------------------------------- */
#define PIN_BTS_R_EN            42    /* D11 (back pad, add 10k pulldown)*/
#define PIN_BTS_L_EN            41    /* D12 (back pad, add 10k pulldown)*/
#define PIN_BTS_R_PWM           43    /* D6                              */
#define PIN_BTS_L_PWM           44    /* D7                              */

/* --- Status LED (onboard, handy for state machine heartbeat) --------- */
#define PIN_STATUS_LED          21    /* XIAO S3 USER_LED                */

/* =====================================================================
 * 2. MOTOR DRIVE (LEDC PWM -> BTS7960)
 * ===================================================================== */
#define MOTOR_PWM_FREQ_HZ       20000  /* 20 kHz: above audible range   */
#define MOTOR_PWM_RES_BITS      8      /* 8-bit timer, per spec         */
#define MOTOR_PWM_MAX           255    /* HARD CLAMP: 8-bit overflow!!  */
                                       /*   output is always |out|<=255 */
#define MOTOR_PWM_MIN_ON        55     /* [CALIBRATE] min duty that      */
                                       /*   actually breaks the servo    */
                                       /*   gear mesh stiction over.     */
                                       /*   Below this the motor just    */
                                       /*   buzzes — treat as 0.         */

/* =====================================================================
 * 3. RTOS TASKS
 * ===================================================================== */
#define CORE_DATA               0      /* CAN + dial task               */
#define CORE_MOTOR              1      /* PID + encoder task (keep       */
                                       /*   Wi-Fi/Arduino loop off it)  */

#define PID_LOOP_HZ             100    /* 10 ms control period          */
#define PID_LOOP_MS             (1000 / PID_LOOP_HZ)

#define CAN_TASK_STACK          4096
#define MOTOR_TASK_STACK        4096
#define CAN_TASK_PRIO           4
#define MOTOR_TASK_PRIO         6      /* motor task is the highest      */

/* CAN frames are polled (INT wire optional); dial sampled slower       */
#define CAN_POLL_MS             5
#define DIAL_SAMPLE_MS          20     /* 50 Hz dial scan               */

/* If no CAN wheel-speed frame arrives for this long the vehicle data is
 * stale: slip logic falls back to base preload, telemetry reads 0.     */
#define CAN_STALE_TIMEOUT_MS    500

/* =====================================================================
 * 4. ENCODER / TRAVEL LIMITS  [CALIBRATE]
 * =====================================================================
 *  The homing routine finds "clutch fully OPEN" at tick 0. MAX_TICKS is
 *  the encoder count at "clutch fully LOCKED" (far mechanical stop).
 *  Record it from the serial log after a full open->closed sweep.
 */
#define ENC_MAX_TICKS           1200   /* [CALIBRATE] placeholder       */
#define ENC_TICKS_PER_PERCENT   (ENC_MAX_TICKS / 100.0f)

/* =====================================================================
 * 5. HOMING SEQUENCE (boot: back off to the OPEN stop, zero the encoder)
 * ===================================================================== */
#define HOME_PWM                70     /* [CALIBRATE] ~27% duty, slow   */
                                       /*   and gentle. Never slam the  */
                                       /*   clutch pack into the stop.  */
#define HOME_DIR_REVERSE        true   /* homing direction (toward the  */
                                       /*   OPEN stop)                  */
#define HOME_STALL_AMPS         10.0f  /* [CALIBRATE] current spike at  */
                                       /*   mechanical stop             */
#define HOME_STALL_CONFIRM_MS   80     /* spike must persist 80 ms      */
#define HOME_TIMEOUT_MS         15000  /* never home forever            */
#define HOME_BACKOFF_TICKS      60     /* creep away from the stop once */
                                       /*   found, before zeroing       */
#define HOME_RETRY_MAX          3      /* retry the whole sweep, then   */
                                       /*   fault (do not drive)        */

/* =====================================================================
 * 6. ACS712 CURRENT SENSE  [CALIBRATE]
 * =====================================================================
 *  Wire the ACS712 VCC to 3V3 so its 0-A midpoint sits mid-scale of the
 *  ADC range. 20 A module = 100 mV/A. 5 A module = 185 mV/A.
 *  Zero offset is auto-calibrated at boot with the motor commanded OFF.
 */
#define ACS_MODULE_AMPS         20.0f  /* 5 / 20 / 30 A variant         */
#define ACS_SENSITIVITY_MV_A    100.0f /* mV per amp for chosen module  */
#define ACS_ZERO_CAL_SAMPLES    64     /* averaged at boot, motor off   */

#define CURRENT_FILTER_TAPS     8      /* moving average window         */
#define OC_SHUTDOWN_AMPS        25.0f  /* [CALIBRATE] hard trip: cut EN */
#define OC_SHUTDOWN_MS          50     /* ...if held this long          */

/* ADC conversion: XIAO analog uses 12-bit @ 11 dB attenuation.
 * 3.3 V rail ~ 3100 raw counts with default attenuation on core 2.x.   */
#define ADC_VREF_CAL_MV         3300   /* [CALIBRATE] measure 3V3!      */
#define ADC_FULLSCALE           4095
#define ADC_COUNTS_PER_MV       (ADC_FULLSCALE / (float)ADC_VREF_CAL_MV)

/* =====================================================================
 * 7. 4WD DIAL — ADC BANDS  [CALIBRATE]
 * =====================================================================
 *  The Tacoma dial is a resistor ladder switch. Probe D0 with a meter in
 *  each detent, then set the bands. Hysteresis stops chatter at edges.
 *  Values below are placeholders assuming a ~0.6/1.8/3.0 V ladder.
 */
#define DIAL_2WD_MAX_MV         1200   /* below -> 2WD (clutch open)    */
#define DIAL_4HI_MIN_MV         1500   /* window  -> 4HI (sport AWD)    */
#define DIAL_4HI_MAX_MV         2400
#define DIAL_4LO_MIN_MV         2700   /* above -> 4LO (locked/Baja)    */
#define DIAL_HYSTERESIS_MV      150
#define DIAL_FILTER_SAMPLES     5      /* median-of-N                   */
#define DIAL_INVALID_TIMEOUT_MS 300    /* in-between reading this long  */
                                       /*   -> last valid state wins    */

typedef enum {
    DIAL_2WD = 0,
    DIAL_4HI = 1,
    DIAL_4LO = 2,
} DialState_t;

/* =====================================================================
 * 8. CAN IDS + DECODING  [CALIBRATE — VERIFY ON TRUCK FIRST!]
 * =====================================================================
 *  These are PLACEHOLDER IDs. Before trusting them, confirm with
 *  candump / a scan: log while rolling, watch which frame counts move.
 *  2016 Tacoma: wheel speeds live on the ABS/skid frames; TPS is on a
 *  powertrain frame (drive-by-wire).
 */
#define CAN_ID_FRONT_WHEEL      0x0B4  /* [CALIBRATE] front axle speed  */
#define CAN_ID_REAR_WHEEL       0x0B0  /* [CALIBRATE] rear axle speed   */
#define CAN_ID_TPS              0x2C1  /* [CALIBRATE] throttle pos      */

#define CAN_FRONT_RX_BYTE       0      /* big-endian 16-bit start byte  */
#define CAN_REAR_RX_BYTE        0
#define CAN_TPS_RX_BYTE         0      /* 8-bit raw 0..255              */

/* Raw -> physical. Toyota wheel-speed LSBs vary by frame; this is the
 * common ~0.043 km/h per count. CONFIRM against GPS at 30 mph.         */
#define WHEEL_KPH_PER_LSB       0.0432f
#define KPH_TO_MPH              0.621371f

/* TPS raw (0..255) -> percent (0..100). Confirm idle ~12-16%, WOT ~99%.*/
#define TPS_PCT_PER_RAW         (100.0f / 255.0f)
#define TPS_IDLE_MAX_PCT        20.0f  /* below this = coasting         */

/* =====================================================================
 * 9. PID — CLUTCH POSITION LOOP (100 Hz, runs on CORE 1)
 * =====================================================================
 *  Input : position error in encoder ticks
 *  Output: signed PWM, -255..+255  (sign -> direction via L/R lines)
 */
#define PID_KP                  4.0f   /* [TUNE] ticks -> duty          */
#define PID_KI                  0.08f
#define PID_KD                  0.6f

#define PID_DEADBAND_TICKS      15     /* |err| <= 15 -> 0 output       */

/* Anti-windup: the I-term contribution is clamped so a long stall can't
 * build a slingshot of stored integral.                                */
#define PID_I_MAX               120.0f /* max |I| contribution (duty)   */

/* Conditional integration: freeze the integrator when the output is
 * saturated OR inside the deadband (in addition to the clamp above).   */
#define PID_FREEZE_INTEGRAL_ON_SAT   1

#define PID_OUTPUT_CLAMP        MOTOR_PWM_MAX   /* never exceed 255 !! */

/* =====================================================================
 * 10. DYNAMIC SLIP LOGIC — 4HI "Sport AWD" mode
 * =====================================================================
 *  Target lock %  =  clamp( base + proportional + predictive/panic )
 *  Target ticks   =  lock% * ENC_TICKS_PER_PERCENT
 *
 *  slip_mph   = rear_mph - front_mph        (rear spinning up = slip)
 *  threshold  = 8% of front speed
 *  proportional: +15% clamp force per 1 mph over threshold
 *  derivative  : +5 mph jump in ONE loop (10 ms) -> 100% panic lock
 *  predictive  : TPS > 60% -> 100% lock
 *  decay       : when slip < threshold, step down toward base preload
 */
#define SLIP_BASE_PRELOAD_PCT   10.0f  /* always a little clamped       */
#define SLIP_MAX_PCT            100.0f

#define SLIP_TPS_PREDICTIVE     60.0f  /* TPS% -> full lock             */
#define SLIP_THRESH_PCT_OF_SPD  0.08f  /* 8% of front speed             */
#define SLIP_GAIN_PCT_PER_MPH   15.0f

/* Slip is meaningless below a crawl and noisy at 0 speed.             */
#define SLIP_MIN_SPEED_MPH      3.0f

/* Reactive derivative (panic): slip delta jump per SINGLE 10 ms loop.  */
#define SLIP_DERIV_PANIC_MPH    5.0f
#define PANIC_HOLD_MS           1200   /* stay clamped after panic,     */
                                       /*   then release via decay      */

/* Decay back toward base preload when traction returns:
 *   lock% -= SLIP_DECAY_PCT_PER_LOOP every loop (rate-limited ramp).
 * 1.5%/loop @ 100 Hz -> 100% down to 10% in ~1 s. Smooth, not abrupt.  */
#define SLIP_DECAY_PCT_PER_LOOP 1.5f

/* Also rate-limit the UP ramp so a big proportional step can't slam
 * the packs faster than the servo can do so cleanly.                  */
#define SLIP_RAMP_UP_PCT_PER_LOOP 3.0f

/* =====================================================================
 * 11. SYSTEM FAULT HANDLING
 * ===================================================================== */
typedef enum {
    SYS_OK = 0,
    SYS_FAULT_HOMING,        /* couldn't find the stop -> limp: no drive */
    SYS_FAULT_OVERCURRENT,   /* OC trip; auto-recover after cooldown     */
    SYS_FAULT_CAN_STALE,     /* no bus data; sport mode -> base preload  */
    SYS_FAULT_ENCODER,       /* implausible tick jump                    */
} SystemFault_t;

#define OC_RECOVER_MS           2000   /* wait after an OC trip          */

/* Encoder plausibility: one 10 ms loop should never see this many edge
 * counts from a healthy hall signal; more => bad sensor/wire.          */
#define ENC_JUMP_FAULT_TICKS    500

/* =====================================================================
 * 12. DEBUG / TELEMETRY
 * ===================================================================== */
#define LOG_BAUD                115200
#define DEBUG_LOG_ENABLED       1      /* 0 = silent build              */
#define TELEMETRY_PERIOD_MS     250    /* periodic state print          */

#if DEBUG_LOG_ENABLED
  #define LOGF(...)  do { Serial.printf(__VA_ARGS__); } while (0)
#else
  #define LOGF(...)  do {} while (0)
#endif

/* =====================================================================
 * 13. COMPILE-TIME SANITY CHECKS
 * ===================================================================== */
static_assert(PID_OUTPUT_CLAMP <= 255, "PWM clamp must fit 8-bit timer");
static_assert(PID_DEADBAND_TICKS < ENC_MAX_TICKS, "deadband >= full travel?");
static_assert(SLIP_BASE_PRELOAD_PCT < SLIP_MAX_PCT, "base preload >= max?");
static_assert(PID_LOOP_HZ > 0 && PID_LOOP_HZ <= 1000, "sane PID rate?");

#endif /* TACS_CONFIG_H */
