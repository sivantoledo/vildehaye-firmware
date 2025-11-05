
#include <stdlib.h>
#include <time.h>

#include <xdc/std.h>
//#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/hal/Timer.h>

#include <ti/drivers/PIN.h>

#include "config.h"

#include "watchdog.h"

/* Pin driver handle */
static PIN_Handle watchdogPinHandle;
static PIN_State  watchdogPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */

static const PIN_Config pinTable[] = {
#ifdef WATCHDOG_PACIFIER_PIN
  WATCHDOG_PACIFIER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
#endif
    PIN_TERMINATE
};

/***********************************************************************/
/* API                                                                 */
/***********************************************************************/

void watchdog_pacify() {
#ifdef WATCHDOG_PACIFIER_PIN
    PIN_setOutputValue(watchdogPinHandle, WATCHDOG_PACIFIER_PIN, 1);
#endif
}

void watchdog_pacify_stop() {
#ifdef WATCHDOG_PACIFIER_PIN
    PIN_setOutputValue(watchdogPinHandle, WATCHDOG_PACIFIER_PIN, 0);
#endif
}

// in this implementation we ignore the arguments
void watchdog_init(uint32_t period_ms, uint32_t initial_wait_ms) {

  watchdogPinHandle = PIN_open(&watchdogPinState, pinTable);
  if(!watchdogPinHandle) {
    System_abort("Error initializing board watchdog pin\n");
  }

}






#ifdef NO_LONGER_USED

/*
 * watchdog.c
 *
 *  A watchdog implementation. We do not use the built-in watchdog because
 *  on CC13XX and CC26XX the watchdog is only active when the CPU is active
 *  (a known and recognized flaw).
 */

#include <stdint.h>
#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>

#include "config.h"
#include "leds.h"

static Semaphore_Handle watchdogSemaphore;
static uint8_t watchdogInitialized = 0;

static void watchdog_taskFunction(UArg arg0, UArg arg1) {

	//uint32_t period_ticks       = (((uint32_t) arg0) * 1000 ) / Clock_tickPeriod;
	//uint32_t initial_wait_ticks = (((uint32_t) arg1) * 1000 ) / Clock_tickPeriod;

	uint32_t reset_source = SysCtrlResetSourceGet(); // driverlib

	switch (reset_source) {
	case RSTSRC_WARMRESET:
	case RSTSRC_SYSRESET:
		System_printf("RESET SOURCE system/warm reset\n");
		leds_blink(LEDS_TX,2);
		break;
	case RSTSRC_VDDS_LOSS:
	case RSTSRC_VDDR_LOSS:
		System_printf("RESET SOURCE VDD(R/S) loss\n");
		leds_blink(LEDS_TX,1);
		break;
	case RSTSRC_PWR_ON:
		System_printf("RESET SOURCE power on\n");
		leds_blink(LEDS_TX,3);
		break;
	case RSTSRC_WAKEUP_FROM_SHUTDOWN:
	default:
		System_printf("RESET SOURCE wakeup from shutdown (or other)\n");
		leds_blink(LEDS_TX,4);
		break;
	}

	Task_sleep( 10000000 / Clock_tickPeriod ); // wait 10s to let the system boot in peace

	while (1) {
		Semaphore_post(watchdogSemaphore);

		Task_sleep( 10000000 / Clock_tickPeriod );

		if (Semaphore_getCount(watchdogSemaphore) > 0) {
			System_printf("WATCHDOG reset\n");
			SysCtrlSystemReset(); // driverlib reset
		} else {
			Semaphore_post(watchdogSemaphore);
		}
	}
}

void watchdog_pacify() {
	//System_printf("WATCHDOG pacify\n");
	if (watchdogInitialized)
		Semaphore_reset(watchdogSemaphore, 0);
}

//#define WATCHDOG_TASK_STACK_SIZE 512
#define WATCHDOG_TASK_STACK_SIZE 256 // this may not be enough! potential but introduced July 2018

static Task_Params watchdogTaskParams;
Task_Struct watchdogTask;    /* not static so you can see in ROV */
static uint8_t watchdogTaskStack[WATCHDOG_TASK_STACK_SIZE];

void watchdog_init(uint32_t period_ms, uint32_t initial_wait_ms) {
	System_printf("WATCHDOG init %d %d\n",period_ms, initial_wait_ms);
	watchdogInitialized = 1;

	Semaphore_Params semParams;
	semParams.mode = Semaphore_Mode_BINARY;
  Semaphore_Params_init(&semParams);
  watchdogSemaphore = Semaphore_create(0, &semParams, NULL);

  Task_Params_init(&watchdogTaskParams);
	watchdogTaskParams.stackSize = WATCHDOG_TASK_STACK_SIZE;
	watchdogTaskParams.priority  = TAG_TASK_PRIORITY;
	watchdogTaskParams.stack     = &watchdogTaskStack;
	watchdogTaskParams.arg0      = (UInt) period_ms;
	watchdogTaskParams.arg1      = (UInt) initial_wait_ms;

	Task_construct(&watchdogTask, watchdog_taskFunction, &watchdogTaskParams, NULL);
}

#endif
