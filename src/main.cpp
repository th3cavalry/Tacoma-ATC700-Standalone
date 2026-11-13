/**
 * =====================================================================
 *  Tacoma-ATC700-Standalone — main.cpp   (v0.1 DRAFT)
 *  - global variable definitions
 *  - ACS712 current sensing (shared primitive)
 *  - boot homing sequence (find OPEN stop, zero the encoder)
 *  - FreeRTOS task spawn pinned to core 0 / core 1
 * =====================================================================
 */
#include "globals.h"

/* =====================================================================
 * 1. Global definitions
 * ===================================================================== */
VehicleData_t          g_veh            = {};
portMUX_TYPE           g_dataMux        = portMUX_INITIALIZER_UNLOCKED;
volatile DialState_t   g_dialState      = DIAL_2WD;
volatile bool          g_homed          = false;
volatile SystemFault_t g_fault          = SYS_OK;
volatile int32_t       g_encoderTicks   = 0;

/* ACS712 0-A midpoint in ADC counts, auto-calibrated in setup(). */
static uint16_t g_currentZeroCounts = ADC_FULLSCALE / 2;

/* =====================================================================
 * 2. ACS712 current sensing
 * =====================================================================
 *  Sensor loop polarity is arbitrary (either motor lead can pass
 *  through), so all consumers use fabsf() on these values.
 * ===================================================================== */
float currentReadAmps(void)
{
    const float deltaCounts = (float)analogRead(PIN_CURRENT_ADC)
                            - (float)g_currentZeroCounts;
    const float mv = deltaCounts / ADC_COUNTS_PER_MV;
    return mv / ACS_SENSITIVITY_MV_A;
}

float currentReadFilteredAmps(void)
{
    static float          ring[CURRENT_FILTER_TAPS];
    static uint8_t        idx = 0, filled = 0;
    static portMUX_TYPE   filterMux = portMUX_INITIALIZER_UNLOCKED;

    float sum = 0.0f;
    uint8_t n;

    portENTER_CRITICAL(&filterMux);
    ring[idx] = currentReadAmps();
    idx = (uint8_t)((idx + 1) % CURRENT_FILTER_TAPS);
    if (filled < CURRENT_FILTER_TAPS) filled++;
    for (uint8_t i = 0; i < filled; i++) sum += ring[i];
    n = filled;
    portEXIT_CRITICAL(&filterMux);

    return sum / (float)n;
}

void currentCalibrateZero(void)
{
    /* Driver OFF: whatever the sensor says is its zero offset + noise. */
    motorEnable(false);
    delay(20);

    uint32_t acc = 0;
    for (uint16_t i = 0; i < ACS_ZERO_CAL_SAMPLES; i++) {
        acc += (uint32_t)analogRead(PIN_CURRENT_ADC);
        delayMicroseconds(200);
    }
    g_currentZeroCounts = (uint16_t)(acc / ACS_ZERO_CAL_SAMPLES);

    /* A healthy ACS712 on a 3V3 rail sits near mid-scale. */
    const int32_t offCenter = labs((int32_t)g_currentZeroCounts
                                   - (int32_t)(ADC_FULLSCALE / 2));
    LOGF("[I]  Zero offset: %u counts (%s)\n", g_currentZeroCounts,
         offCenter > 600 ? "OFF-NORMAL - check ACS712 wiring!" : "ok");
}

/* =====================================================================
 * 3. Homing sequence
 * =====================================================================
 *  Drive REVERSE at low duty toward the clutch-OPEN stop until the
 *  current spike confirms a mechanical stall. Then back off a known
 *  distance and define that position as tick 0, so the resting pack
 *  never presses against the stop. If the encoder doesn't move during
 *  back-off, the sweep fails -> also catches dead encoder wiring.
 * ===================================================================== */
static bool homeAttempt(void)
{
    LOGF("[HOME] Sweep reverse @ %d duty; stall > %.1f A for %u ms\n",
         HOME_PWM, HOME_STALL_AMPS, HOME_STALL_CONFIRM_MS);

    const uint32_t startMs    = millis();
    uint32_t       stallSince = 0;
    bool           hitStop    = false;

    g_encoderTicks = 0;
    motorEnable(true);
    motorDrive(-(float)HOME_PWM);

    while (millis() - startMs < HOME_TIMEOUT_MS) {
        const float amps = fabsf(currentReadFilteredAmps());
        if (amps > HOME_STALL_AMPS) {
            if (stallSince == 0) stallSince = millis();
            if (millis() - stallSince >= HOME_STALL_CONFIRM_MS) {
                hitStop = true;
                break;
            }
        } else {
            stallSince = 0;          /* spike must be sustained */
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    motorDrive(0);
    motorEnable(false);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!hitStop) {
        LOGF("[HOME] TIMEOUT after %d ms (no confirmed stall)\n",
             HOME_TIMEOUT_MS);
        return false;
    }

    /* --- At the OPEN stop: back off, then zero here. --- */
    g_encoderTicks = 0;
    const uint32_t backoffStartMs = millis();
    motorEnable(true);
    motorDrive(HOME_PWM * 0.8f);     /* gentle creep forward (CLOSE)   */

    while (millis() - backoffStartMs < 5000 &&
           g_encoderTicks < HOME_BACKOFF_TICKS) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    motorDrive(0);
    motorEnable(false);

    const int32_t moved = g_encoderTicks;
    if (moved < HOME_BACKOFF_TICKS) {
        /* Stalled again, or encoder not counting — hardware problem. */
        LOGF("[HOME] Back-off moved only %ld ticks - check encoder/jam!\n",
             (long)moved);
        return false;
    }

    g_encoderTicks = 0;              /* this position is now tick 0    */
    LOGF("[HOME] Done. Tick 0 = OPEN stop + %d ticks.\n", HOME_BACKOFF_TICKS);
    return true;
}

static void homeSequence(void)
{
    for (uint8_t retry = 1; retry <= HOME_RETRY_MAX; retry++) {
        if (homeAttempt()) {
            g_homed = true;
            return;
        }
        LOGF("[HOME] attempt %d/%d failed\n", retry, HOME_RETRY_MAX);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    g_fault = SYS_FAULT_HOMING;      /* permanent limp: never drive    */
    LOGF("[HOME] !! ALL ATTEMPTS FAILED - LIMP MODE, motor disabled !!\n");
}

/* =====================================================================
 * 4. setup() / loop()
 * ===================================================================== */
void setup()
{
    Serial.begin(LOG_BAUD);
    Serial.setTxTimeoutMs(0);        /* USB CDC writes never stall a core */

    LOGF("\n\n=== Tacoma-ATC700-Standalone v0.1 (%s %s) ===\n",
         __DATE__, __TIME__);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    pinMode(PIN_DIAL_ADC, INPUT);
    pinMode(PIN_CURRENT_ADC, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_DIAL_ADC,    ADC_11db);
    analogSetPinAttenuation(PIN_CURRENT_ADC, ADC_11db);

    currentCalibrateZero();

    /* Motor PWM + EN + hall ISR must be live BEFORE homing — the homing
     * routine needs the encoder count and the driver primitives.      */
    motorHwInit();

    homeSequence();

    /* Pin the two workers to their cores. Arduino loop() also lives on
     * core 1 but at priority 1, so it can never preempt the 100 Hz
     * motor loop — it just sleeps.                                     */
    xTaskCreatePinnedToCore(canTask,   "can",   CAN_TASK_STACK,
                            NULL, CAN_TASK_PRIO,   NULL, CORE_DATA);
    xTaskCreatePinnedToCore(motorTask, "motor", MOTOR_TASK_STACK,
                            NULL, MOTOR_TASK_PRIO, NULL, CORE_MOTOR);

    LOGF("[SYS] Tasks spawned (can@core%d p%d, motor@core%d p%d) homed=%d\n",
         CORE_DATA, CAN_TASK_PRIO, CORE_MOTOR, MOTOR_TASK_PRIO, (int)g_homed);
}

void loop()
{
    /* Nothing on the critical path; the pinned tasks do the real work. */
    vTaskDelay(pdMS_TO_TICKS(1000));
}
