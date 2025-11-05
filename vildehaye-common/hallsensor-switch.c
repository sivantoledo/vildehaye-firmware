#include "config.h"

#ifdef USE_HALLSENSOR_SWITCH

#include <stdint.h>
#include "scif.h"

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/devices/cc13x0/driverlib/rf_mailbox.h>
#include <ti/devices/cc13x0/driverlib/rf_prop_mailbox.h>
#include <ti/devices/cc13x0/driverlib/rf_common_cmd.h>
#include <ti/devices/cc13x0/driverlib/rf_prop_cmd.h>
#include <ti/drivers/rf/RF.h>

#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>

#include "config.h"
#include "leds.h"
//#include "board.h"

#define BV(n)               (1 << (n))

// Display error message if the SCIF driver has been generated with incorrect operating system setting
#ifndef SCIF_OSAL_TIRTOS_H
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'TI-RTOS' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
//#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN32_4X4_RSM
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

/*
// Task data
Task_Struct myLedTask;
Char myLedTaskStack[1024];
Task_Struct myButtonTask;
Char myButtonTaskStack[1024];


// Semaphore used to wait for Sensor Controller task ALERT event
static Semaphore_Struct semScTaskAlert;

// Semaphore used to wait for buttons to be pressed
static Semaphore_Struct semButtonPressed;




PIN_Config pLedPinTable[] = {
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_State ledPinState;
PIN_Handle hLedPins;

void ledTaskFxn(UArg a0, UArg a1) {

    // Enable LED pins
    hLedPins = PIN_open(&ledPinState, pLedPinTable);

    // Main loop
    while (1) {

        // Wait for an ALERT callback
        Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

        // Clear the ALERT interrupt source
        scifClearAlertIntSource();

        // If the LED blinker task has not been stopped ...
        if (scifGetActiveTaskIds() & BV(SCIF_LED_BLINKER_TASK_ID)) {

            // Indicate value of the output counter variable on the green LED
            PIN_setOutputValue(hLedPins, Board_GLED, (scifTaskData.ledBlinker.output.counter & 0x0001) != 0);
        }

        // Acknowledge the ALERT event
        scifAckAlertEvents();
    }

} // ledTaskFxn




PIN_Config pButtonPinTable[] = {
    Board_BTN1 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
    Board_BTN2 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
    PIN_TERMINATE
};
PIN_State buttonPinState;
PIN_Handle hButtonPins;


void buttonPressedCallback(PIN_Handle handle, PIN_Id pinId) {

    // Disable button pin interrupt
    PIN_setConfig(hButtonPins, PIN_BM_IRQ, Board_BTN1 | PIN_IRQ_DIS);
    PIN_setConfig(hButtonPins, PIN_BM_IRQ, Board_BTN2 | PIN_IRQ_DIS);

    // Resume the button OS task
    Semaphore_post(Semaphore_handle(&semButtonPressed));

} // buttonPressedCallback




void buttonTaskFxn(UArg a0, UArg a1) {

    // Enable button pins, with interrupt handler buttonPressedCallback()
    hButtonPins = PIN_open(&buttonPinState, pButtonPinTable);
    PIN_registerIntCb(hButtonPins, buttonPressedCallback);


    // Main loop
    while (1) {

        // Setup interrupt on the left button and wait for the user to press it
        PIN_setConfig(hButtonPins, PIN_BM_IRQ, Board_BTN1 | PIN_IRQ_NEGEDGE);
        Semaphore_pend(Semaphore_handle(&semButtonPressed), BIOS_WAIT_FOREVER);

        // Start the Sensor Controller LED blinker task
        while (scifWaitOnNbl(0) != SCIF_SUCCESS);
        scifStartTasksNbl(BV(SCIF_LED_BLINKER_TASK_ID));

        // Setup interrupt on the right button and wait for the user to press it
        PIN_setConfig(hButtonPins, PIN_BM_IRQ, Board_BTN2 | PIN_IRQ_NEGEDGE);
        Semaphore_pend(Semaphore_handle(&semButtonPressed), BIOS_WAIT_FOREVER);

        // Stop the Sensor Controller LED blinker task
        while (scifWaitOnNbl(0) != SCIF_SUCCESS);
        scifStopTasksNbl(BV(SCIF_LED_BLINKER_TASK_ID));

        // Reset the output data structure in addition to the state data structure
        scifResetTaskStructs(BV(SCIF_LED_BLINKER_TASK_ID), BV(SCIF_STRUCT_OUTPUT));
    }

} // buttonTaskFxn
*/

Semaphore_Handle scStartSemaphore;
Semaphore_Handle scStopSemaphore;


void scCtrlReadyCallback(void) {
}

void scTaskAlertCallback(void) {
	// Wake up the LED OS task
	//Semaphore_post(Semaphore_handle(&semScTaskAlert));

	// Clear the ALERT interrupt source
  scifClearAlertIntSource();
  // Acknowledge the ALERT event
  scifAckAlertEvents();

  //System_printf(">sc> %d\n",scifTaskData.hallSensor.output.state);

#ifdef HALL_SENSOR
  switch (scifTaskData.hallSensor.output.state) {
  case 2: // magnet attached, stop the tag
    // set the semaphore to zero, telling the tag task to suspend and to wait until it is one again
    Semaphore_post(scStopSemaphore);
    break;
  case 5:
    Semaphore_post(scStartSemaphore); // wake up the tag task
  	break;
  }
#else
  switch (scifTaskData.lowVoltageSuspender.output.state) {
  case 2: // magnet attached, stop the tag
    // set the semaphore to zero, telling the tag task to suspend and to wait until it is one again
    Semaphore_post(scStopSemaphore);
    break;
  case 5:
    Semaphore_post(scStartSemaphore); // wake up the tag task
    break;
  }
#endif
}

void sensorcontroller_ack() {
  scifSwTriggerEventHandlerCode();
}


void sensorcontroller_init() {

	Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
  scStartSemaphore = Semaphore_create(0, &semParams, NULL);
  scStopSemaphore = Semaphore_create(0, &semParams, NULL);

  // Initialize and start the Sensor Controller
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);
  // no need because we don't use the RTC
  //ifStartRtcTicksNow(0x00010000); // upper 16 bits in seconds
	//System_printf("starting sensor controller init complete\n");
	//System_printf("starting sensor controller task\n");
#ifdef HALL_SENSOR
  scifStartTasksNbl(BV(SCIF_HALL_SENSOR_TASK_ID)); // asynchronous
#else
  scifStartTasksNbl(BV(SCIF_LOW_VOLTAGE_SUSPENDER_TASK_ID)); // asynchronous
#endif
}

#endif // USE_HALLSENSOR_SWITCH
