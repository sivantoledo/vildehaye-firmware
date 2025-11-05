/*
 * leds.h
 *
 *  Created on: 2016
 *      Author: stoledo
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <ti/drivers/PIN.h>

#include "config.h"

#define LEDS_ANY 0

//#define LEDS_RED   0
//#define LEDS_GREEN 1

extern void leds_init();

#define LEDS_STATE_OFF  0
#define LEDS_STATE_SLOW 1
#define LEDS_STATE_FAST 2
#define LEDS_STATE_ON   3

extern void leds_setState(int led, int state); // clients should not use this directly!
#define leds_off(led)  leds_setState((led),LEDS_STATE_OFF)
#define leds_slow(led) leds_setState((led),LEDS_STATE_SLOW)
#define leds_fast(led) leds_setState((led),LEDS_STATE_FAST)
#define leds_on(led)   leds_setState((led),LEDS_STATE_ON)

#ifdef OBSOLETE
extern void leds_on(uint8_t led);
extern void leds_off(uint8_t led);

// new Aug 2018 for Clock-based implemenation
extern void leds_slow(uint8_t led);
extern void leds_fast(uint8_t led);
#endif

// removed toggle Aug 2018
//extern void leds_toggle(uint8_t led);
extern void leds_blink(int led, uint8_t howmanytimes);
extern void leds_incrementBlinkCounter();
extern void leds_setBlinkCounter(uint8_t count);

#endif /* LEDS_H_ */
