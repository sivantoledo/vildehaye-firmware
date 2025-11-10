#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

/* BUILD_CONFIG_ARCH cc13x2 */
/* BUILD_CONFIG_SC lvs-cc13x2-7x7 */

/*
 * I used by mistake the same SVN project for both E72 and E79 boards.
 */
#include "../../vildehaye-common/board_rfstar_ti1352p1.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-lvs-cc13x2-7x7/FILE>

// force high VDDR, to allow 14dBm if so configured
#define CCFG_FORCE_VDDR_HH 0x1

#ifdef KRUCHIA
#define RECEIVE_TIME_LIMIT_US 15000

#define HOOKS_GPIO_1          IOID_5 // edge of board...
#define USE_HOOKS
#define USE_HOOKS_RELEASE_MECHANISM
#endif

// uart16 implies sensor-controller uart
#define HOST_UART 0

#define BOARD_UART0_TX        IOID_13
#define BOARD_UART0_RX        IOID_12

#define LEDS_TX 0
#define BOARD_LED0            IOID_7
#define BOARD_LED0_ON         1
#define BOARD_LED0_OFF        0

#define BOARD_BOOTLOADER_BACKDOOR  IOID_15

//#define LEDS_RX 1
//#define BOARD_LED1            LAUNCHPAD_GREEN_LED
//#define BOARD_LED1_ON         LAUNCHPAD_GREEN_LED_ON
//#define BOARD_LED1_OFF        LAUNCHPAD_GREEN_LED_OFF

/********************************************************************/
/* module configurations                                            */
/********************************************************************/

//#define USE_BASESTATION
#define USE_TAG
//#define USE_FLASHCONTROLLER
#define USE_CONSOLE

#define LEDS_INITIAL_BLINK_COUNTER 0xFF
//#define LEDS_INITIAL_BLINK_COUNTER 0

#define BUFFER_COUNT 12
#define BUFFER_SIZE  256

/********************************************************************/
/* task priorities                                                  */
/********************************************************************/

#define LEDS_GRN_TASK_PRIORITY         1
#define LEDS_RED_TASK_PRIORITY         2

#define UART_RX_TASK_PRIORITY          3
#define UART_TX_TASK_PRIORITY          4

// We either have a logger or a UART tasks in a base station

#define LOGGER_TASK_PRIORITY           5
#define BASESTATION_UART_TASK_PRIORITY 5

#define BASESTATION_PACKET_HANDLER_TASK_PRIORITY   6

#define TAG_TASK_PRIORITY              7
#define BASESTATION_RECEIVE_TASK_PRIORITY      7

// Sivan July 2019: I don't think we use the watchdog task,
// so I used the priority for the scuart, but the priority should
// really be low, not high. XXX needs to verify & fix.

#define WATCHDOG_TASK_PRIORITY         8
#define SCUART_TASK_PRIORITY           8

#endif // config.h
