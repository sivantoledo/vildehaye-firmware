/* This file was called leds2.c in tags-cc1310 */

#include <stdlib.h>
#include <time.h>

#include <xdc/std.h>
//#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/hal/Timer.h>

#include <ti/drivers/PIN.h>

#include "config.h"
//#include "Board.h"

#include "leds.h"

/*
 * We do not want to wake tags up every 250ms, so no periodic states.
 *
 * Also, we do not support multiple blinks, to simplify.
 *
 */
#if defined(USE_TAG)
#define NO_PERIODIC_CLOCK
#endif


/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State  ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */

#define LEDS_COUNT 0

const PIN_Config pinTable[] = {
#ifdef BOARD_LED0
#if BOARD_LED0_ON==1
    BOARD_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#else
    BOARD_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#define LEDS_COUNT 1
#endif

#ifdef BOARD_LED1
    BOARD_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 2
#endif

#ifdef BOARD_LED2
    BOARD_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 3
#endif

#ifdef BOARD_LED3
    BOARD_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 4
#endif

#ifdef BOARD_LED4
    BOARD_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 5
#endif

#ifdef BOARD_LED5
    BOARD_LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 6
#endif

#ifdef BOARD_LED6
    BOARD_LED6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 7
#endif

#ifdef BOARD_LED7
    BOARD_LED7 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#define LEDS_COUNT 8
#endif
    PIN_TERMINATE
};

#ifndef LEDS_INITIAL_BLINK_COUNTER
#define LEDS_INITIAL_BLINK_COUNTER 0
#endif

#ifndef LEDS_BLINK_FAST_RATE_US
#define LEDS_BLINK_FAST_RATE_US 250000
#endif

#ifndef LEDS_BLINK_SLOW_RATE_MULTIPLIER
#define LEDS_BLINK_SLOW_RATE_MULTIPLIER 6
#endif

#ifndef LEDS_BLINK_DURATION_US
#define LEDS_BLINK_DURATION_US 10000
#endif

static Clock_Handle clockOff;

/***********************************************************************/
/* API                                                                 */
/***********************************************************************/

static const PIN_Id ledsMap[] = {
#ifdef BOARD_LED0
    BOARD_LED0
#endif

#ifdef BOARD_LED1
    ,BOARD_LED1
#endif

#ifdef BOARD_LED2
    ,BOARD_LED2
#endif

#ifdef BOARD_LED3
    ,BOARD_LED3
#endif

#ifdef BOARD_LED4
    ,BOARD_LED4
#endif

#ifdef BOARD_LED5
    ,BOARD_LED5
#endif

#ifdef BOARD_LED6
    ,BOARD_LED6
#endif

#ifdef BOARD_LED7
    ,BOARD_LED7
#endif
};

static const uint8_t ledsOffValue[] = {
#ifdef BOARD_LED0
    BOARD_LED0_OFF
#endif

#ifdef BOARD_LED1
    ,BOARD_LED1_OFF
#endif

#ifdef BOARD_LED2
    ,BOARD_LED2_OFF
#endif

#ifdef BOARD_LED3
    ,BOARD_LED3_OFF
#endif

#ifdef BOARD_LED4
    ,BOARD_LED4_OFF
#endif

#ifdef BOARD_LED5
    ,BOARD_LED5_OFF
#endif

#ifdef BOARD_LED6
    ,BOARD_LED6_OFF
#endif

#ifdef BOARD_LED7
    ,BOARD_LED7_OFF
#endif
};

static const uint8_t ledsOnValue[] = {
#ifdef BOARD_LED0
    BOARD_LED0_ON
#endif

#ifdef BOARD_LED1
    ,BOARD_LED1_ON
#endif

#ifdef BOARD_LED2
    ,BOARD_LED2_ON
#endif

#ifdef BOARD_LED3
    ,BOARD_LED3_ON
#endif

#ifdef BOARD_LED4
    ,BOARD_LED4_ON
#endif

#ifdef BOARD_LED5
    ,BOARD_LED5_ON
#endif

#ifdef BOARD_LED6
    ,BOARD_LED6_ON
#endif

#ifdef BOARD_LED7
    ,BOARD_LED7_ON
#endif
};

static uint32_t ledsBlink[LEDS_COUNT]; // we make it large hoping for atomicity in set/get

/*
 * The setting of each led is specified by 2 bits,
 *
 * 00: off
 * 01: slow blinking
 * 10: fast blinking
 * 11: on
 */
static uint32_t ledsSetting = 0;

void leds_setState(int led, int value) {
  if (led >= LEDS_COUNT) return;

#ifdef NO_PERIODIC_CLOCK
  // implify and execute immediately
  if (value <= 1) { // off or slow blinking, simply turn off
    value = 0; // so that the clock turns it off
    PIN_setOutputValue(ledPinHandle, ledsMap[led], ledsOffValue[led]);
  } else {
    value = 1; // so that the clock keeps it on
    PIN_setOutputValue(ledPinHandle, ledsMap[led], ledsOnValue[led]);
  }
#endif

  uint32_t bits;
  uint32_t shift = led << 1; // 2 positions per led

  bits = 3;
  bits = bits << shift; // shift by 2 bits per led
  ledsSetting &= ~bits;    // clear bits in the array
  bits = ((value&3) << shift); //
  ledsSetting |= bits;    // copy value to bit array

  //System_printf("leds %d %d shift %d bits %08x setState %08x\n",led,value,shift,bits,ledsSetting); System_flush();
  ledsBlink[led] = 0;
}

static int leds_getState(int led) {
  if (led >= LEDS_COUNT) return 0;

  uint32_t bits;
  uint32_t shift = led << 1; // 2 positions per led

  bits = ledsSetting;
  bits = bits >> shift; // shift by 2 bits per led
  return (bits & 3);
}

uint8_t leds_counter = LEDS_INITIAL_BLINK_COUNTER;
void leds_setBlinkCounter(uint8_t c) {
	leds_counter = c;
	System_printf("LEDS setting blink counter to %d\n",leds_counter);
}

void leds_incrementBlinkCounter() {
  if (leds_counter!=0xFF) leds_counter++;
	System_printf("LEDS incrementing blink counter to %d\n",leds_counter);
}


#ifndef NO_PERIODIC_CLOCK
void leds_blink(int led, uint8_t howmanytimes) {
  if (led >= LEDS_COUNT) return;
  if (leds_counter==0 || leds_getState(led) != 0) return; // blink only if we still have blink budget and the led it is off.
  ledsBlink[led] = howmanytimes;
}
#else
void leds_blink(int led, uint8_t howmanytimes) {
  if (led >= LEDS_COUNT) return;
  if (leds_counter==0 || leds_getState(led) != 0) return; // blink only if we still have blink budget and the led it is off.

  if (leds_counter!=0xFF) leds_counter--;

  PIN_setOutputValue(ledPinHandle, ledsMap[led], ledsOnValue[led]);

  // schedule a turn-off, if not already scheduled
  if (!Clock_isActive(clockOff)) {
    Clock_setTimeout(clockOff, LEDS_BLINK_DURATION_US / Clock_tickPeriod); // turn everything off in 10ms
    Clock_start(clockOff);
  }
}
#endif

/***********************************************************************/
/* mechanism!                                                          */
/***********************************************************************/

#ifndef NO_PERIODIC_CLOCK
//static Clock_Handle clockPeriodic;

static uint8_t counter = 0;
// how often to blink slowly (in units of fast wakeups)
// #define SLOW_COUNT 4

static void ledsPeriodicFunction(UArg dummy) {
  int i;

  uint32_t setting = ledsSetting;

  for (i=0; i<LEDS_COUNT; i++) {
    switch (setting & 3) { // extract lower 2 bits
    case LEDS_STATE_OFF:
      if (ledsBlink[i] > 0 && leds_counter!=0) {
        PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOnValue[i]);
        ledsBlink[i]--;
        if (leds_counter!=0xFF) leds_counter--;
      } else
        PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
      break;
    case LEDS_STATE_SLOW:
      if (counter==0) PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOnValue[i] );
      else            PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
      break;
    case LEDS_STATE_FAST:
    case LEDS_STATE_ON:
      PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOnValue[i]);
      break;
    }
    setting = setting >> 2;
  }
  //  PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);

  counter++;
  if (counter == LEDS_BLINK_SLOW_RATE_MULTIPLIER) counter = 0;

  Clock_setTimeout(clockOff, LEDS_BLINK_DURATION_US / Clock_tickPeriod); // turn everything off in 10ms
  Clock_start(clockOff);
}
#endif // periodic clock

static void ledsOffFunction(UArg dummy) {
  int i;


  uint32_t setting = ledsSetting;

  for (i=0; i<LEDS_COUNT; i++) {
    //PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
    switch (setting & 3) { // extract lower 2 bits
    case 0:
      // it should be already off, but maybe it's on a limited blink, so turn off
      PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
      break;
    case 1:
    case 2:
      // it might be blinking, turn off
      PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
      break;
    case 3:
      // it should be already on, don't do anything
      break;
    }
    setting = setting >> 2;
  }
}

void leds_init() {
  //System_printf("leds init!\n");
  //System_flush();

  ledPinHandle = PIN_open(&ledPinState, pinTable);
  if(!ledPinHandle) {
    System_abort("Error initializing board LED pins\n");
  }

  int i;
  for (i=0; i<LEDS_COUNT; i++) PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOffValue[i]);
  //for (i=0; i<LEDS_COUNT; i++) PIN_setOutputValue(ledPinHandle, ledsMap[i], ledsOnValue[i]);

  Clock_Params clockParams;

#ifndef NO_PERIODIC_CLOCK
  Clock_Params_init(&clockParams);
  clockParams.period    = LEDS_BLINK_FAST_RATE_US / Clock_tickPeriod; // 500ms
  clockParams.startFlag = true;
  Clock_create(ledsPeriodicFunction, LEDS_BLINK_FAST_RATE_US / Clock_tickPeriod, &clockParams, NULL);
  //clockPeriodic = Clock_create(ledsPeriodicFunction, LEDS_BLINK_FAST_RATE_US / Clock_tickPeriod, &clockParams, NULL);
#endif

  Clock_Params_init(&clockParams);
  clockParams.period    = 0; // one-shot timer
  clockParams.startFlag = false; // don't start; the periodic function will start it to turn off the leds.
  clockOff      = Clock_create(ledsOffFunction,      0,                         &clockParams, NULL);

  //System_printf("leds init handles = %08x %08x\n",clockPeriodic, clockOff);
}


