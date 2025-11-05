/*
 * watchdog.h
 *
 *  Created on: nov 2016
 *      Author: stoledo
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

void watchdog_init(uint32_t period_ms, uint32_t initial_wait_ms);
void watchdog_pacify();

// Sivan Oct 2025 for external TPL5010; marks the end of the GPIO pacification pulse
void watchdog_pacify_stop();

#endif /* WATCHDOG_H_ */
