#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>

#include "config.h"

#include "buffers.h"
#include "receive.h"

//#include "Board.h"
#include "console.h"
#include "uart.h"
//#include "uart_printf.h"

#include "radio_setup.h"
#include "tag.h"
#include "basestation.h"
#include "hallsensor-switch.h"
//#include "controller.h"
#include "leds.h"
#include "watchdog.h"
#include "i2c_sensors.h"

//#include "logger.h"
/*
 *  ======== main ========
 */

//void UartPrintf_init(UART_Handle handle);

/*******************************************************************************************/
/* Initial Pin States (originally in CC1310_LAUNCHXL.c                                     */
/*******************************************************************************************/

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
 */

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(BoardGpioInitTable, ".const:BoardGpioInitTable")
#endif

const PIN_Config BoardGpioInitTable[] = {

    // LEDs

#ifdef WATCHDOG_PACIFIER_PIN
  WATCHDOG_PACIFIER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
#endif

#ifdef BOARD_LED0
#if BOARD_LED0_ON==1
    BOARD_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

#ifdef BOARD_LED1
#if BOARD_LED1_ON==1
    BOARD_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

#ifdef BOARD_LED2
#if BOARD_LED2_ON==1
    BOARD_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

#ifdef BOARD_LED3
#if BOARD_LED3_ON==1
    BOARD_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

#ifdef BOARD_LED4
#if BOARD_LED4_ON==1
    BOARD_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

#ifdef BOARD_LED5
#if BOARD_LED5_ON==1
    BOARD_LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,
#else
    BOARD_LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif
#endif

    // BUTTONS

#ifdef BOARD_BUTTON0
#if BOARD_BUTTON0_PRESSED==0
    BOARD_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    BOARD_BUTTON0 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS,
#endif
#endif

#ifdef BOARD_BUTTON1
#if BOARD_BUTTON1_PRESSED==0
    BOARD_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    BOARD_BUTTON1 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS,
#endif
#endif

#ifdef BOARD_BUTTON2
#if BOARD_BUTTON2_PRESSED==0
    BOARD_BUTTON2 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    BOARD_BUTTON2 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS,
#endif
#endif

#ifdef BOARD_BUTTON3
#if BOARD_BUTTON3_PRESSED==0
    BOARD_BUTTON3 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    // sivan 2020 was pulldown, a bug
    // Sivan 2022, back to pulldown for buttons that are 1 when pressed....
    BOARD_BUTTON3 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#endif
#endif

#ifdef BOARD_BUTTON4
#if BOARD_BUTTON4_PRESSED==0
    BOARD_BUTTON4 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    BOARD_BUTTON4 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS,
#endif
#endif

#ifdef BOARD_BUTTON5
#if BOARD_BUTTON5_PRESSED==0
    BOARD_BUTTON5 | PIN_INPUT_EN | PIN_PULLUP | PIN_HYSTERESIS,
#else
    BOARD_BUTTON5 | PIN_INPUT_EN | PIN_PULLDOWN | PIN_HYSTERESIS,
#endif
#endif

    // Specialized GPIOs

//#ifdef LAUNCHPAD_RF_SWITCH_POWER
//    LAUNCHPAD_RF_SWITCH_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//#endif

//#ifdef LAUNCHPAD_RF_SWITCH
//    LAUNCHPAD_RF_SWITCH       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL, // high is sub1GHz
//#endif

//#ifdef LAUNCHPAD_RF_SWITCH_0
//    LAUNCHPAD_RF_SWITCH_0 | PIN_GPIO_OUTPUT_EN | LAUNCHPAD_RF_SWITCH_0_SUB1GHZ | PIN_PUSHPULL,
//#endif
//#ifdef LAUNCHPAD_RF_SWITCH_1
//    LAUNCHPAD_RF_SWITCH_1 | PIN_GPIO_OUTPUT_EN | LAUNCHPAD_RF_SWITCH_1_SUB1GHZ | PIN_PUSHPULL,
//#endif
//#ifdef LAUNCHPAD_RF_SWITCH_2
//    LAUNCHPAD_RF_SWITCH_2 | PIN_GPIO_OUTPUT_EN | LAUNCHPAD_RF_SWITCH_2_SUB1GHZ | PIN_PUSHPULL,
//#endif

#ifdef RF_SWITCH_0
    RF_SWITCH_0 | PIN_GPIO_OUTPUT_EN | RF_SWITCH_0_INITIAL | PIN_PUSHPULL,
#endif
#ifdef RF_SWITCH_1
    RF_SWITCH_1 | PIN_GPIO_OUTPUT_EN | RF_SWITCH_1_INITIAL | PIN_PUSHPULL,
#endif
#ifdef RF_SWITCH_2
    RF_SWITCH_2 | PIN_GPIO_OUTPUT_EN | RF_SWITCH_2_INITIAL | PIN_PUSHPULL,
#endif


    // UART

#ifdef BOARD_UART0_TX
    BOARD_UART0_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif

#ifdef BOARD_UART0_RX
    BOARD_UART0_RX | PIN_INPUT_EN | PIN_PULLUP,
#endif

#ifdef BOARD_UART1_TX
    BOARD_UART1_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif

#ifdef BOARD_UART1_RX
    BOARD_UART1_RX | PIN_INPUT_EN | PIN_PULLUP,
#endif

    // I2C; these were not defined in CC1310-LAUNCHXL; strange

#ifdef BOARD_I2C0_SCL
    BOARD_I2C0_SCL | PIN_INPUT_EN | PIN_PULLUP,
#endif

#ifdef BOARD_I2C0_SDA
    BOARD_I2C0_SDA | PIN_INPUT_EN | PIN_PULLUP,
#endif

    // SPI

#ifdef BOARD_SPI0_SCLK
    BOARD_SPI0_SCLK | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif

#ifdef BOARD_SPI0_MOSI
    BOARD_SPI0_MOSI | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif

#ifdef BOARD_SPI0_MISO
    BOARD_SPI0_MISO | PIN_INPUT_EN | PIN_PULLUP,
#endif

    /*
     * Boosterpack devices
     */

#ifdef BOOSTERPACK_SDCARD_CS
    BOOSTERPACK_SDCARD_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif

#ifdef BOOSTERPACK_GPS_PPS
    BOOSTERPACK_GPS_PPS | PIN_INPUT_EN | PIN_PULLDOWN,
#endif

#ifdef BOOSTERPACK_DISPLAY_POWER
    BOOSTERPACK_DISPLAY_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,
#endif

    PIN_TERMINATE
};


/*******************************************************************************************/
/* main                                                                                    */
/*******************************************************************************************/

int main(void) {

  Power_init();

  if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) System_abort("PIN_init failed\n");

	//Board_initGeneral();

#if defined(USE_HALLSENSOR_SWITCH)
	sensorcontroller_init();
#endif

#ifndef VESPER_COMPATIBILITY
  UART_init(); // this needs to move to the uart module initialization file
  //Board_initUART();
#endif

#ifdef USE_CONSOLE
    consoleInit();
#endif

	i2cInit();

	//uartPrintf_init();

	leds_init();
	//leds_on(LEDS_TX);

	System_printf("***BUILD TIME DATE " __TIME__ " " __DATE__ " ***\n");

	buffers_init();

#ifdef USE_TAG
//#if defined(CC1310_V3)
//    loggerTask_init();
//    //tagTask_init();
//    //i2cSensors_init();
//#else
    //watchdog_init( 2*tagPeriodMs, 8*tagPeriodMs );
    watchdog_init( 0, 0 ); // Sivan Oct 2025
    //SPI_init();        // remove from non sensor/mem tags
    //spiFlash_init();   // same
    //i2cSensors_init(); // same
  receive_init();

  System_printf("going to radio setup\n");
  radioSetup_init();

  tagTask_init();
#endif

#ifdef USE_BASESTATION
  receive_init();

  System_printf("going to radio setup\n");
  radioSetup_init();

  System_printf("goint to uart tasks\n");

	System_printf("goint to bs tasks\n");
	basestationTask_init();
#endif
//#endif

#ifdef USE_FLASHCONTROLLER
	flashcontrollerTask_init();
#endif

	BIOS_start();

	return (0);
}
