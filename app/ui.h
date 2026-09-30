/**
 * @file    ui.h
 * @brief   User interface: 6-digit display and status LED.
 *
 * Display layout (left to right): [actual temperature][.][setpoint]
 *   WORK     - "250.260"  actual 250 C, setpoint 260 C
 *   COOLING  - "COL" alternating with the temperature every second
 *   STANDBY  - "COL" (cooled down)
 *   FAULT    - "Err  N" alternating with the temperature every 0.5 s
 * LED: blinks 1 Hz in WORK, 4 Hz on a fault, off otherwise.
 */
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "station.h"

void ui_init(void);

/* Call every DISPLAY_PERIOD_MS */
void ui_update(const station_status_t *st, uint32_t now_ms);

#endif /* UI_H */
