/*
 * hooks_release_mechanism.c
 *
 *  Created on: 13 ???? 2020
 *      Author: stoledo
 */

#include "config.h"

#ifdef USE_HOOKS
#ifdef USE_HOOKS_RELEASE_MECHANISM

#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <time.h>

#include <ti/drivers/PIN.h>
#include <ti/sysbios/hal/Seconds.h>

#include "hooks.h"

//#include <xdc/std.h>
//#include <xdc/runtime/Timestamp.h>
//#include <xdc/runtime/Types.h>
//#include <xdc/runtime/System.h>

//#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/knl/Semaphore.h>

static PIN_Handle pinsHandle = NULL;

static PIN_Config BoardPinsTable[] = {
    HOOKS_GPIO_1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

static int8_t   previousConfiguration = -1;
static uint32_t timestamp = 0;

void hooksInit() {
  PIN_State pinState;
  pinsHandle = PIN_open(&pinState, BoardPinsTable);
}

extern uint8_t configurationsCount; // defined in tag.c

void hookEndOfSlot(int8_t currentConfiguration,uint16_t slotInFrameIndex) {
  if (currentConfiguration == (configurationsCount-1)) {
    if (previousConfiguration != (configurationsCount-1)) {
      // configuration change triggers GPIO!
      PIN_setOutputValue(pinsHandle,HOOKS_GPIO_1,1);
      timestamp = Seconds_get();
    }
  } else {
    PIN_setOutputValue(pinsHandle,HOOKS_GPIO_1,0);
  }

  if ((timestamp != 0) && (Seconds_get() - timestamp > 10)) PIN_setOutputValue(pinsHandle,HOOKS_GPIO_1,0);

  previousConfiguration = currentConfiguration;
}

#endif
#endif

