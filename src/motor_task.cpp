/**
 * =====================================================================
 *  Tacoma-ATC700-Standalone — motor_task.cpp   (v0.1 DRAFT)
 *  CORE 1: Hall ISR, BTS7960 primitives, 100 Hz PID loop, and the
 *  4HI dynamic-slip ("Sport AWD") clamping strategy.
 *
 *  NOTHING here touches SPI, CAN, Serial, or long critical sections —
 *  that is the whole point of the core split. Telemetry printing is
 *  done from the can task's snapshot cadence, not here.
 * =====================================================================
 */
#include "globals.h"
#include <math.h>

/* =====================================================================
 * LEDC API compat shim (Arduino core 2.x vs 3.x)
 * ===================================================================== */
#include "esp_arduino_version.h"
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define TACS_LEDC_ATTACH(pin, freq, res) ledcAttach((pin), (freq), (res))
  #define TACS_LEDC_WRITE(pin, duty)       ledcWrite((pin), (duty))
#else
  static const uint8_t LEDC_CH_FWD = 0;
  static const uint8_t LEDC_CH_REV = 1;
  #define TACS_LEDC_ATTACH(pin, freq, res)                 \
      do { ledcSetup((pin) == PIN_BTS_R_PWM ? LEDC_CH_FWD : LEDC_CH_REV, \
                     (freq), (res));                       \
           ledcAttachPin((pin), (pin) == PIN_BTS_R_PWM ? LEDC_CH_FWD : LEDC_CH_REV); \
     } while (0)
  #define TACS_LEDC_WRITE(pin, duty) \
      ledcWrite((pin) == PIN_BTS_R_PWM ? LEDC_CH_FWD : LEDC_CH_REV, (duty))
#endif

/* =====================================================================
 * Hall ISR (attaches on core 1, fires on any core — ESP32 GPIO ISRs
 * are IRAM-locked and tiny by design)
 * =====================================================================
 *  Quadrature: sample B on every A edge. Sign convention below assumes
 *  "forward" (clockwise when looking at the servo shaft) => increasing
 *  ticks => clutch CLOSES. If wiring swaps it, flip the sign HERE once
 *  (or swap the A/B connector). [CALIBRATE]
 */
static void IRAM_ATTR hallA_isr(void)
{
    const int8_t dir = digitalRead(PIN_HALL_B) ? 1 : -1;
    g_encoderTicks += dir;
}

/* =====================================================================
 * BTS7960 primitives (also used by the homing sequence in main.cpp)
 * ===================================================================== */
static volatile bool s_enabled = false;

void motorHwInit(void)
{
    pinMode(PIN_BTS_R_EN, OUTPUT);
    pinMode(PIN_BTS_L_EN, OUTPUT);
    digitalWrite(PIN_BTS_R_EN, LOW);
    digitalWrite(PIN_BTS_L_EN, LOW);

    pinMode(PIN_BTS_R_PWM, OUTPUT);
    pinMode(PIN_BTS_L_PWM, OUTPUT);
    TACS_LEDC_ATTACH(PIN_BTS_R_PWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    TACS_LEDC_ATTACH(PIN_BTS_L_PWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    TACS_LEDC_WRITE(PIN_BTS_R_PWM, 0);
    TACS_LEDC_WRITE(PIN_BTS_L_PWM, 0);

    pinMode(PIN_HALL_A, INPUT_PULLUP);
    pinMode(PIN_HALL_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_A), hallA_isr, RISING);
}

void motorEnable(bool en)
{
    if (!en) {
        TACS_LEDC_WRITE(PIN_BTS_R_PWM, 0);
        TACS_LEDC_WRITE(PIN_BTS_L_PWM, 0);
    }
    digitalWrite(PIN_BTS_R_EN, en ? HIGH : LOW);
    digitalWrite(PIN_BTS_L_EN, en ? HIGH : LOW);
    s_enabled = en;
}

bool motorIsEnabled(void) { return s_enabled; }

/*
 * Signed PWM: positive = CLOSE (lock direction), negative = OPEN.
 * BTS7960 truth table with both EN high:
 *   R_PWM=duty, L_PWM=0  -> direction A
 *   R_PWM=0, L_PWM=duty  -> direction B
 *   both 0               -> coast
 * Never drive both PWM high (brake mode — leave for a future kickdown).
 */
void motorDrive(float signedPwm)
{
    if (!s_enabled) return;

    /* Hard clamp — the 8-bit LEDC register must never see > 255. */
    if (signedPwm >  (float)MOTOR_PWM_MAX) signedPwm =  (float)MOTOR_PWM_MAX;
    if (signedPwm < -(float)MOTOR_PWM_MAX) signedPwm = -(float)MOTOR_PWM_MAX;

    int duty = (int)fabsf(signedPwm);
    if (duty > 0 && duty < MOTOR_PWM_MIN_ON) duty = MOTOR_PWM_MIN_ON;
    duty = (duty > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : duty;

    if (signedPwm >= 0.0f) {
        TACS_LEDC_WRITE(PIN_BTS_R_PWM, duty);
        TACS_LEDC_WRITE(PIN_BTS_L_PWM, 0);
    } else {
        TACS_LEDC_WRITE(PIN_BTS_R_PWM, 0);
        TACS_LEDC_WRITE(PIN_BTS_L_PWM, duty);
    }
}

/* =====================================================================
 * Dynamic slip logic — 4HI "Sport AWD"
 * =====================================================================
 *  Desired lock% each loop:
 *    base            = 10%
 *    proportional    = base + 15%/mph over an 8%-of-front-speed band
 *    predictive      = TPS > 60%           -> 100%
 *    derivative      = +5 mph slip in one 10 ms loop -> 100% + hold
 *  The ACTUATED lock% then ramps toward desired (fast up, smooth down).
 * ===================================================================== */
typedef struct {
    float    lockPct;        /* actuated lock demand, %  */
    float    lastSlipMph;
    uint32_t panicUntilMs;
    bool     primed;
} SlipState_t;

static float slipCompute(float frontMph, float rearMph, float tpsPct,
                         bool stale, uint32_t nowMs, SlipState_t *st)
{
    /* No bus / crawling: hold the pack at base preload. */
    if (stale || frontMph < SLIP_MIN_SPEED_MPH) {
        st->lastSlipMph = 0.0f;
        return SLIP_BASE_PRELOAD_PCT;
    }

    float slip = rearMph - frontMph;          /* + = rear spinning up */
    if (slip < 0.0f) slip = 0.0f;             /* front slip handled by brakes */

    /* Derivative / panic check runs BEFORE any priming guard: it uses
     * the true previous-loop slip whenever we have one.                */
    if (st->primed && (slip - st->lastSlipMph) > SLIP_DERIV_PANIC_MPH) {
        st->panicUntilMs = nowMs + PANIC_HOLD_MS;
        LOGF("[SLIP] PANIC: dSlip=%.1f mph -> 100%%\n", slip - st->lastSlipMph);
    }
    st->lastSlipMph = slip;
    st->primed = true;

    float desired;
    if ((int32_t)(nowMs - st->panicUntilMs) < 0) {
        desired = SLIP_MAX_PCT;                       /* panic hold      */
    } else if (tpsPct > SLIP_TPS_PREDICTIVE) {
        desired = SLIP_MAX_PCT;                       /* predictive      */
    } else {
        const float thresh = frontMph * SLIP_THRESH_PCT_OF_SPD;
        const float over   = slip - thresh;
        desired = (over > 0.0f)
                ? SLIP_BASE_PRELOAD_PCT + over * SLIP_GAIN_PCT_PER_MPH
                : SLIP_BASE_PRELOAD_PCT;              /* decay target    */
    }
    if (desired > SLIP_MAX_PCT) desired = SLIP_MAX_PCT;

    /* Rate limits: quick to clamp, smooth to release.                  */
    if (desired > st->lockPct) {
        float step = desired - st->lockPct;
        if (step > SLIP_RAMP_UP_PCT_PER_LOOP) step = SLIP_RAMP_UP_PCT_PER_LOOP;
        st->lockPct += step;
    } else if (desired < st->lockPct) {
        float step = st->lockPct - desired;
        if (step > SLIP_DECAY_PCT_PER_LOOP) step = SLIP_DECAY_PCT_PER_LOOP;
        st->lockPct -= step;
    }
    return st->lockPct;
}

/* =====================================================================
 * PID — clutch position loop (100 Hz)
 * ===================================================================== */
typedef struct {
    float iTerm;
    float lastErr;
    bool  primed;
} PidState_t;

static PidState_t   pid;
static SlipState_t  slip;

volatile float g_slipLockPct = 0.0f;
volatile float g_targetTicks = 0.0f;

/* dt is folded into the gains: Ki applies per 10 ms loop. */
static float pidCompute(float targetTicks, float actualTicks)
{
    const float err = targetTicks - actualTicks;

    if (fabsf(err) <= (float)PID_DEADBAND_TICKS) {
        /* In the deadband: coast and let the pack rest. Integral is
         * frozen (not zeroed) so we don't lose preload authority, but
         * it cannot grow here.                                         */
        pid.lastErr = err;
        pid.primed  = true;
        return 0.0f;
    }

    /* --- P --- */
    const float p = PID_KP * err;

    /* --- I with anti-windup (clamp + conditional integration) --- */
    float iCandidate = pid.iTerm + PID_KI * err;
    if (iCandidate >  PID_I_MAX) iCandidate =  PID_I_MAX;
    if (iCandidate < -PID_I_MAX) iCandidate = -PID_I_MAX;

    float out = p + iCandidate + (pid.primed ? PID_KD * (err - pid.lastErr) : 0.0f);

#if PID_FREEZE_INTEGRAL_ON_SAT
    /* If the *proposed* output saturates, don't bank integral in that
     * direction this loop (classic conditional integration).          */
    if (fabsf(out) > (float)PID_OUTPUT_CLAMP) {
        const bool pushing = (err > 0.0f) == (iCandidate > pid.iTerm);
        if (pushing) iCandidate = pid.iTerm;   /* freeze */
        out = p + iCandidate +
              (pid.primed ? PID_KD * (err - pid.lastErr) : 0.0f);
    }
#endif
    pid.iTerm = iCandidate;
    pid.lastErr = err;
    pid.primed  = true;

    if (out >  (float)PID_OUTPUT_CLAMP) out =  (float)PID_OUTPUT_CLAMP;
    if (out < -(float)PID_OUTPUT_CLAMP) out = -(float)PID_OUTPUT_CLAMP;
    return out;
}

/* =====================================================================
 * Faults raised here
 * ===================================================================== */
static uint32_t ocTripUntilMs = 0;

static bool overcurrentCheck(uint32_t nowMs)
{
    static uint32_t ocSince = 0;
    const float amps = fabsf(currentReadFilteredAmps());
    if (amps > OC_SHUTDOWN_AMPS) {
        if (ocSince == 0) ocSince = nowMs;
        if (nowMs - ocSince >= OC_SHUTDOWN_MS) {
            motorEnable(false);
            ocTripUntilMs = nowMs + OC_RECOVER_MS;
            g_fault = SYS_FAULT_OVERCURRENT;
            LOGF("[OC] %.1f A trip — driver off %d ms\n",
                 amps, OC_RECOVER_MS);
            ocSince = 0;
            return true;
        }
    } else {
        ocSince = 0;
    }
    return false;
}

/* =====================================================================
 * The 100 Hz task
 * ===================================================================== */
void motorTask(void *arg)
{
    (void)arg;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PID_LOOP_MS);

    int32_t lastTicks = g_encoderTicks;

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        /* Never touch the driver unless homing succeeded. */
        if (!g_homed) { motorEnable(false); continue; }

        const uint32_t nowMs = millis();

        /* --- Snapshot shared state (µs critical section) -------------- */
        VehicleData_t v;
        portENTER_CRITICAL(&g_dataMux);
        v = g_veh;
        portEXIT_CRITICAL(&g_dataMux);

        const int32_t ticks = g_encoderTicks;   /* 32-bit atomic read */

        /* --- Encoder plausibility ------------------------------------- */
        if (abs((int)(ticks - lastTicks)) > ENC_JUMP_FAULT_TICKS) {
            g_fault = SYS_FAULT_ENCODER;
            motorDrive(0);
            lastTicks = ticks;
            continue;   /* sensor is lying — don't act on it this loop */
        }
        lastTicks = ticks;

        /* --- Overcurrent watchdog ------------------------------------- */
        if (overcurrentCheck(nowMs)) continue;
        if ((int32_t)(nowMs - ocTripUntilMs) < 0) continue;  /* cooldown */

        /* --- Dial state -> target position ---------------------------- */
        float targetPct;
        switch (g_dialState) {
            case DIAL_2WD: targetPct = 0.0f;            break;
            case DIAL_4LO: targetPct = SLIP_MAX_PCT;    break;
            case DIAL_4HI:
            default:
                targetPct = slipCompute(v.frontMph, v.rearMph, v.tpsPct,
                                        v.stale, nowMs, &slip);
                break;
        }
        const float targetTicks = targetPct * ENC_TICKS_PER_PERCENT;
        g_targetTicks = targetTicks;
        g_slipLockPct = targetPct;

        /* --- PID -> signed PWM -> BTS7960 ----------------------------- */
        const float out = pidCompute(targetTicks, (float)ticks);

        /* Arm the driver the moment the loop actually wants torque;
         * motorDrive() is a no-op while disarmed, so this is safe.    */
        if (out != 0.0f && !motorIsEnabled()) motorEnable(true);
        motorDrive(out);
    }
}
