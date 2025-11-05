#include "config.h"

#ifdef USE_TAG

#include <stdint.h>
#include <math.h>
#include SensorControllerDirectory(scif.h)

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)


#include "config.h"
#include "leds.h"
//#include "board.h"

#include "scif-low-voltage-suspender.h"

#define BV(n)               (1 << (n))

// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_TIRTOS_H
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'TI-RTOS' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
//#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
#if defined(DeviceFamily_CC13X0) && !defined(SCIF_TARGET_CHIP_PACKAGE_QFN32_4X4_RSM)
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

Semaphore_Handle scStartSemaphore;
//Semaphore_Handle scStopSemaphore;

uint16_t state;
//uint16_t compb;
//uint16_t adc;

void scCtrlReadyCallback(void) {
}

void scTaskAlertCallback(void) {
	// Wake up the LED OS task
	//Semaphore_post(Semaphore_handle(&semScTaskAlert));

	// Clear the ALERT interrupt source
  scifClearAlertIntSource();
  // Acknowledge the ALERT event
  scifAckAlertEvents();


  state=scifTaskData.lowVoltageSuspender.output.state;
  //compb  =scifTaskData.lowVoltageSuspender.output.compb;
  //adc    =scifTaskData.lowVoltageSuspender.output.adc;
  Semaphore_post(scStartSemaphore); // wake up the tag task
}

/*
 * Wait a given number of seconds, but also until voltage
 * rises above a given threshold. Thresholds below 2.5v are understood as 2.5v.
 */

static float maxDrop = NAN;

uint16_t lvsWait(float wait, float voltageThreshold) {

  if (isnan(voltageThreshold)) {
    scifTaskData.lowVoltageSuspender.input.policy = 0;
  } else {
    if (voltageThreshold >= 2.56f)
      scifTaskData.lowVoltageSuspender.input.policy = 2; // wait for 2.5-2.56 with COMPB, then with ADC
    else
      scifTaskData.lowVoltageSuspender.input.policy = 1; // threshold is too low for COMPB to be useful
  }

  scifTaskData.lowVoltageSuspender.input.adcThreshold = (uint16_t) (voltageThreshold/4.3f*4096.0f);

  uint32_t waitFixedPoint = (uint32_t) (wait*65536); // 16.16 format

  scifTaskData.lowVoltageSuspender.input.wait3  = 0; // currently not supported
  scifTaskData.lowVoltageSuspender.input.wait2  = ((waitFixedPoint >> 16) & 0xFF); // seconds
  scifTaskData.lowVoltageSuspender.input.wait1  = ((waitFixedPoint >>  8) & 0xFF); // seconds
  scifTaskData.lowVoltageSuspender.input.wait0  = ((waitFixedPoint      ) & 0xFF); // seconds

  //System_printf("lvs %f %08x %d %d %d\n",wait,waitFixedPoint, scifTaskData.lowVoltageSuspender.input.wait2,scifTaskData.lowVoltageSuspender.input.wait1,scifTaskData.lowVoltageSuspender.input.wait0);

#if OLD
  uint32_t waitInTicks = (uint32_t) (wait*4096.0);
  uint16_t exponent = 0;

  //System_printf("lvs time %f ticks %d\n",wait,waitInTicks);

  while ((waitInTicks > 65535) && (exponent <= 7)) {
    exponent++;
    waitInTicks = waitInTicks >> 1;
  }

 // System_printf("lvs ticks %d e %d\n",waitInTicks,exponent);

  while (((waitInTicks & 1) == 0) && (exponent <= 7)) {
    exponent++;
    waitInTicks = waitInTicks >> 1;
  }

  //System_printf("lvs ticks %d e %d\n",waitInTicks,exponent);

  scifTaskData.lowVoltageSuspender.input.waitLowBits  = (uint16_t) ((waitInTicks     ) & 0xFF);
  scifTaskData.lowVoltageSuspender.input.waitHighBits = (uint16_t) ((waitInTicks >> 8) & 0xFF);
  scifTaskData.lowVoltageSuspender.input.exponent = (uint16_t) exponent;
#endif

#if 0
  System_printf("lvs %f %f threshold %d wait %d:%d,%d exponent %d\n",wait,voltageThreshold,
      scifTaskData.lowVoltageSuspender.input.adcThreshold,waitInTicks,
      scifTaskData.lowVoltageSuspender.input.waitLowBits,scifTaskData.lowVoltageSuspender.input.waitHighBits,
      scifTaskData.lowVoltageSuspender.input.exponent
  );
#endif

#ifdef DeviceFamily_CC13X0
  scifSwTriggerEventHandlerCode();
#endif
#ifdef DeviceFamily_CC13X2
  scifSwTriggerEventHandlerCode(0,0); //task id, triggger index
#endif


  Semaphore_pend(scStartSemaphore, BIOS_WAIT_FOREVER);

  float s = 4.3f * (float) (scifTaskData.lowVoltageSuspender.output.startVoltage) / 4096.0;
  float e = 4.3f * (float) (scifTaskData.lowVoltageSuspender.output.endVoltage) / 4096.0;

  if (!isnan(voltageThreshold) && s!=0.0 && e!=0.0) {
    if (isnan(maxDrop) || (s-e) > maxDrop) maxDrop = (s-e);
    System_printf("V %f -> %f drop %f\n",s,e,s-e);
  }

  return state;
}

float lvsMaxDrop() { return maxDrop; }

void lvsInit() {

	Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
  scStartSemaphore = Semaphore_create(0, &semParams, NULL);
  //scStopSemaphore = Semaphore_create(0, &semParams, NULL);

  // Initialize and start the Sensor Controller
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);
  // no need because we don't use the RTC
  scifStartRtcTicksNow(0x0000FFFF); // upper 16 bits in seconds
	//System_printf("starting sensor controller init complete\n");
	//System_printf("starting sensor controller task\n");

  scifStartTasksNbl(BV(SCIF_LOW_VOLTAGE_SUSPENDER_TASK_ID)); // asynchronous
}

#endif // USE_TAG
