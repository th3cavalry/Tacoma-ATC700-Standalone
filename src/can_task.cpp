/**
 * =====================================================================
 *  Tacoma-ATC700-Standalone — can_task.cpp   (v0.1 DRAFT)
 *  CORE 0: CAN bus sniffer (wheel speeds + TPS) and 4WD dial ADC scan.
 *
 *  Runs on CORE_DATA so SPI/CAN latency can never stutter the 100 Hz
 *  PID loop pinned to CORE_MOTOR. All vehicle data hands off through
 *  g_veh under g_dataMux; Serial printing lives HERE, not in the
 *  motor task, for the same reason.
 * =====================================================================
 */
#include "globals.h"

#include <SPI.h>
#include <mcp_can.h>

/* CS on D4; SPI pins handed to SPI.begin() below. */
MCP_CAN CAN0(PIN_CAN_CS);

/* =====================================================================
 * CAN decoding
 * ===================================================================== */
static inline uint16_t be16(const unsigned char *b)
{
    return (uint16_t)((b[0] << 8) | b[1]);
}

static void handleFrame(uint32_t id, unsigned char len, unsigned char *buf)
{
    if (id == CAN_ID_FRONT_WHEEL && len >= CAN_FRONT_RX_BYTE + 2) {
        const float mph = be16(&buf[CAN_FRONT_RX_BYTE])
                        * WHEEL_KPH_PER_LSB * KPH_TO_MPH;
        portENTER_CRITICAL(&g_dataMux);
        g_veh.frontMph        = mph;
        g_veh.lastWheelFrameMs = millis();
        g_veh.stale           = false;
        portEXIT_CRITICAL(&g_dataMux);

    } else if (id == CAN_ID_REAR_WHEEL && len >= CAN_REAR_RX_BYTE + 2) {
        const float mph = be16(&buf[CAN_REAR_RX_BYTE])
                        * WHEEL_KPH_PER_LSB * KPH_TO_MPH;
        portENTER_CRITICAL(&g_dataMux);
        g_veh.rearMph          = mph;
        g_veh.lastWheelFrameMs = millis();
        g_veh.stale            = false;
        portEXIT_CRITICAL(&g_dataMux);

    } else if (id == CAN_ID_TPS && len > CAN_TPS_RX_BYTE) {
        const float tps = buf[CAN_TPS_RX_BYTE] * TPS_PCT_PER_RAW;
        portENTER_CRITICAL(&g_dataMux);
        g_veh.tpsPct = tps;
        portEXIT_CRITICAL(&g_dataMux);

    } else if (DEBUG_LOG_ENABLED) {
        /* Bus-learn aid: every unrecognized ID is printed at ~2 Hz so you
         * can find the REAL wheel-speed/TPS frames while rolling.        */
        static uint32_t lastUnknownLog = 0;
        if (millis() - lastUnknownLog > 500) {
            lastUnknownLog = millis();
            LOGF("[CAN?] id=0x%03X len=%d data=", id, len);
            for (unsigned char i = 0; i < len; i++) LOGF("%02X ", buf[i]);
            LOGF("\n");
        }
    }
}

/* =====================================================================
 * 4WD dial — median-of-N banding with hysteresis
 * ===================================================================== */
static int16_t dialReadMv(void)
{
    int16_t s[DIAL_FILTER_SAMPLES];
    for (uint8_t i = 0; i < DIAL_FILTER_SAMPLES; i++) {
        s[i] = (int16_t)analogRead(PIN_DIAL_ADC);
        delayMicroseconds(150);
    }
    /* insertion sort (tiny N) */
    for (uint8_t i = 1; i < DIAL_FILTER_SAMPLES; i++) {
        int16_t v = s[i];
        int8_t  j = i - 1;
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    const int16_t med = s[DIAL_FILTER_SAMPLES / 2];
    return (int16_t)((float)med / ADC_COUNTS_PER_MV);
}

/* Bands are laid out as:
 *      < 2WD_MAX            -> 2WD
 *      > 4LO_MIN            -> 4LO
 *      4HI_MIN..4HI_MAX     -> 4HI
 * Hysteresis: to LEAVE the current band you must clear it by
 * DIAL_HYSTERESIS_MV. Anything in a no-man's-land gap keeps the last
 * valid state (bounded by DIAL_INVALID_TIMEOUT_MS).                    */
static DialState_t dialBand(float mv, DialState_t last)
{
    switch (last) {
        case DIAL_2WD:
            if (mv > DIAL_2WD_MAX_MV + DIAL_HYSTERESIS_MV) {
                if (mv >= DIAL_4LO_MIN_MV)                    return DIAL_4LO;
                if (mv >= DIAL_4HI_MIN_MV && mv <= DIAL_4HI_MAX_MV) return DIAL_4HI;
            } else {
                return DIAL_2WD;
            }
            break;
        case DIAL_4HI:
            if (mv < DIAL_4HI_MIN_MV - DIAL_HYSTERESIS_MV) {
                if (mv < DIAL_2WD_MAX_MV) return DIAL_2WD;
            } else if (mv > DIAL_4HI_MAX_MV + DIAL_HYSTERESIS_MV) {
                if (mv > DIAL_4LO_MIN_MV) return DIAL_4LO;
            } else {
                return DIAL_4HI;
            }
            break;
        case DIAL_4LO:
            if (mv < DIAL_4LO_MIN_MV - DIAL_HYSTERESIS_MV) {
                if (mv <= DIAL_4HI_MAX_MV && mv >= DIAL_4HI_MIN_MV) return DIAL_4HI;
                if (mv < DIAL_2WD_MAX_MV)                    return DIAL_2WD;
            } else {
                return DIAL_4LO;
            }
            break;
    }
    return last;   /* ambiguous gap or hysteresis not cleared */
}

static void dialScan(void)
{
    static DialState_t lastState = DIAL_2WD;
    const DialState_t  s = dialBand((float)dialReadMv(), lastState);
    if (s != lastState) {
        LOGF("[DIAL] %d -> %d\n", (int)lastState, (int)s);
        lastState = s;
    }
    g_dialState = lastState;   /* ambiguous bands hold the last valid state */
}

/* =====================================================================
 * CAN bring-up + main loop
 * ===================================================================== */
static bool canInit(void)
{
    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
    pinMode(PIN_CAN_INT, INPUT_PULLUP);   /* optional wire; polled anyway */

    for (uint8_t tryNum = 1; tryNum <= 5; tryNum++) {
        if (CAN0.begin(MCP_ANY, CAN_500KBPS,
                       CAN_CRYSTAL_MHZ == 8 ? MCP_8MHZ : MCP_16MHZ) == CAN_OK) {
            CAN0.setMode(MCP_NORMAL);        /* listen + ACK on the bus   */
            LOGF("[CAN] MCP2515 up @ %ld kbit/s (try %d)\n",
                 (long)CAN_BUS_KBPS, tryNum);
            return true;
        }
        LOGF("[CAN] init failed (try %d/5) — check wiring/crystal\n", tryNum);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

void canTask(void *arg)
{
    (void)arg;

    const bool canUp = canInit();
    if (!canUp) {
        portENTER_CRITICAL(&g_dataMux);
        g_veh.stale = true;
        portEXIT_CRITICAL(&g_dataMux);
        g_fault = SYS_FAULT_CAN_STALE;
        LOGF("[CAN] !! bus offline — slip logic disabled, dial still live !!\n");
    }

    uint32_t lastCanPoll = 0, lastDial = 0, lastStaleCheck = 0, lastBlink = 0;
    uint8_t  blinkOn = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));

        /* --- 1) Drain CAN RX FIFO (INT just means "try now", poll is truth) */
        if (canUp && millis() - lastCanPoll >= CAN_POLL_MS) {
            lastCanPoll = millis();
            uint8_t guard = 0;
            while (CAN0.checkReceive() == CAN_MSGAVAIL && guard++ < 32) {
                unsigned long id  = 0;          /* library wants ulong* */
                unsigned char ext = 0;
                unsigned char len = 0;
                unsigned char buf[8];
                if (CAN0.readMsgBuf(&id, &ext, &len, buf) == CAN_OK) {
                    handleFrame(id, len, buf);
                }
            }
        }

        /* --- 2) Staleness watchdog ------------------------------------- */
        if (millis() - lastStaleCheck >= 100) {
            lastStaleCheck = millis();
            portENTER_CRITICAL(&g_dataMux);
            if (g_veh.lastWheelFrameMs != 0 &&
                millis() - g_veh.lastWheelFrameMs > CAN_STALE_TIMEOUT_MS) {
                if (!g_veh.stale) {
                    g_veh.stale = true;
                    LOGF("[CAN] stale (> %d ms silent) — sport logic -> preload\n",
                         CAN_STALE_TIMEOUT_MS);
                }
            }
            portEXIT_CRITICAL(&g_dataMux);
        }

        /* --- 3) Dial scan @ 50 Hz -------------------------------------- */
        if (millis() - lastDial >= DIAL_SAMPLE_MS) {
            lastDial = millis();
            dialScan();
        }

        /* --- 4) Heartbeat / fault blink (LED on core 0, never motor loop) */
        if (millis() - lastBlink >= (g_fault == SYS_OK ? 1000 : 150)) {
            lastBlink = millis();
            blinkOn ^= 1;
            digitalWrite(PIN_STATUS_LED, blinkOn ? HIGH : LOW);
        }

        /* --- 5) Telemetry print — Serial work stays off core 1 -------- */
        static uint32_t lastTelem = 0;
        if (DEBUG_LOG_ENABLED && millis() - lastTelem >= TELEMETRY_PERIOD_MS) {
            lastTelem = millis();
            VehicleData_t v;
            portENTER_CRITICAL(&g_dataMux);
            v = g_veh;
            portEXIT_CRITICAL(&g_dataMux);
            LOGF("[T] dial=%d F=%4.1f R=%4.1f TPS=%3.0f%% lock=%5.1f%% "
                 "tgt=%5.0f enc=%ld I=%4.1fA%s%s\n",
                 (int)g_dialState, v.frontMph, v.rearMph, v.tpsPct,
                 (double)g_slipLockPct, (double)g_targetTicks,
                 (long)g_encoderTicks,
                 (double)fabsf(currentReadFilteredAmps()),
                 (v.stale ? " STALE" : ""),
                 (g_fault == SYS_OK ? "" : " FAULT!"));
        }
    }
}
