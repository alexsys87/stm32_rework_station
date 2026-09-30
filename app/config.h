/**
 * @file    config.h
 * @brief   User settings: temperature limits, PID, air flow, safety.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Setpoint (temperature potentiometer) ----------------------------- */
#define SETPOINT_MIN_C          50.0f   /* Potentiometer fully CCW */
#define SETPOINT_MAX_C          480.0f  /* Potentiometer fully CW */
#define SETPOINT_STEP_C         5.0f    /* Setpoint resolution */

/* ---- Air flow (pump potentiometer) ------------------------------------ */
#define PUMP_MIN_PERCENT        25U     /* Minimum air flow while heating */
#define PUMP_COOLING_PERCENT    80U     /* Air flow while cooling down */
#define PUMP_FAULT_PERCENT      100U    /* Air flow on a fault */

/* ---- Cool-down in standby --------------------------------------------- */
#define COOLING_TARGET_C        50.0f   /* Pump stops below this */
#define COOLING_HYSTERESIS_C    5.0f    /* Pump restarts above target + hysteresis */

/* ---- PID (output 0..100 % heater power) ------------------------------- */
#define CONTROL_PERIOD_MS       50U
#define PID_KP                  2.5f    /* % per C */
#define PID_KI                  3.0f    /* % per C per second */
#define PID_KD                  0.004f  /* % per C/s */
#define PID_D_FILTER            0.3f    /* Derivative low-pass factor, 0..1 */

/* ---- Safety ------------------------------------------------------------ */
#define OVERHEAT_C              510.0f  /* Hard over-temperature limit */
#define OVERHEAT_CLEAR_C        450.0f  /* Fault may be cleared below this */

/* Open thermocouple: with a pull-up on the thermocouple input the
 * amplifier saturates and the ADC reads near full scale. */
#define TC_OPEN_ADC_THRESHOLD   4000U

/* Thermal runaway: heater at >= RUNAWAY_POWER_PERCENT and the temperature
 * at least RUNAWAY_MIN_ERROR_C below the setpoint, yet it rose less than
 * RUNAWAY_MIN_RISE_C within RUNAWAY_TIME_MS -> heater or sensor failure. */
#define RUNAWAY_POWER_PERCENT   90U
#define RUNAWAY_MIN_ERROR_C     50.0f
#define RUNAWAY_MIN_RISE_C      10.0f
#define RUNAWAY_TIME_MS         30000U

/* Work switch debounce: number of equal consecutive samples (every
 * CONTROL_PERIOD_MS) before the state changes. */
#define SWITCH_DEBOUNCE_SAMPLES 4U

/* ---- Display ------------------------------------------------------------ */
#define DISPLAY_PERIOD_MS       100U
#define DISPLAY_REINIT_MS       2000U   /* Periodic MAX7219 re-configuration */
#define DISPLAY_TEMP_FILTER     0.3f    /* Displayed temperature smoothing, 0..1 */

#endif /* CONFIG_H */
