/**
 * =====================================================================
 *  Tacoma-ATC700-Standalone — globals.h   (v0.1 DRAFT)
 *  Shared state across translation units: the CAN task (core 0) writes
 *  vehicle telemetry, the motor task (core 1) consumes it.
 * =====================================================================
 */
#ifndef TACS_GLOBALS_H
#define TACS_GLOBALS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"

/* =====================================================================
 *  Vehicle snapshot — written by can_task (CORE_DATA), read by
 *  motor_task (CORE_MOTOR). Every access goes through g_dataMux with
 *  portENTER_CRITICAL(). Keep critical sections microseconds long —
 *  NEVER Serial.print() inside one!
 * ===================================================================== */
typedef struct {
    float    frontMph;          /* front axle speed                     */
    float    rearMph;           /* rear axle speed                      */
    float    tpsPct;            /* throttle position, 0..100            */
    uint32_t lastWheelFrameMs;  /* millis() of last wheel-speed frame   */
    bool     stale;             /* no frames for CAN_STALE_TIMEOUT_MS   */
} VehicleData_t;

extern VehicleData_t g_veh;
extern portMUX_TYPE  g_dataMux;

/* =====================================================================
 *  State written by can_task / ISR, read anywhere
 * ===================================================================== */
extern volatile DialState_t   g_dialState;   /* banded dial position    */
extern volatile bool          g_homed;       /* valid zero point exists */
extern volatile SystemFault_t g_fault;       /* last latched fault      */

/* Encoder edge count. Updated inside the hall ISR; 32-bit aligned
 * loads/stores are atomic on Xtensa, so a plain volatile gives
 * lock-free reads from the other core.                                 */
extern volatile int32_t       g_encoderTicks;

/* Control-loop status mirrors, written by CORE_MOTOR every PID loop,
 * read by the can task's telemetry print.                              */
extern volatile float         g_slipLockPct;  /* sport lock demand, %   */
extern volatile float         g_targetTicks;  /* where the PID wants to be */

/* =====================================================================
 *  Motor primitives — implemented in src/motor_task.cpp.
 *  The homing sequence (main.cpp) calls these before the task starts,
 *  so motorHwInit() must run first.
 * ===================================================================== */
void  motorHwInit(void);            /* PWM channels, EN pins, hall ISR  */
void  motorEnable(bool en);         /* BTS7960 R_EN/L_EN master kill    */
bool  motorIsEnabled(void);
void  motorDrive(float signedPwm);  /* -255..255; + = CLOSE (lock side) */
void  motorTask(void *arg);         /* core 1: 100 Hz PID + slip logic  */

/* =====================================================================
 *  CAN/dial task — implemented in src/can_task.cpp
 * ===================================================================== */
void  canTask(void *arg);           /* core 0: bus sniffer + dial scan  */

/* =====================================================================
 *  Current sensing — implemented in src/main.cpp (shared by homing and
 *  the overcurrent watchdog in the motor task).
 * ===================================================================== */
void  currentCalibrateZero(void);   /* run once with motor OFF          */
float currentReadAmps(void);        /* one sample, signed; polarity is arbitrary */
float currentReadFilteredAmps(void);/* moving average CURRENT_FILTER_TAPS */

#endif /* TACS_GLOBALS_H */
