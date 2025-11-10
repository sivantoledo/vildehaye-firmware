#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

/* BUILD_CONFIG_ARCH cc13x0 */
/* BUILD_CONFIG_SC lvs-4x4 */

//#include "../../vildehaye-common/board_cc1350_launchpad.h"
//#include "../../vildehaye-common/board_ax5031_boosterpack.h"
#include "../../vildehaye-common/board_vildehaye_v3_0_ax5031.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-lvs-cc13x0-4x4/FILE>

#define LEDS_TX 0
//#define BOARD_LED0            TAG_RED_LED
//#define BOARD_LED0_ON         TAG_RED_LED_ON
//#define BOARD_LED0_OFF        TAG_RED_LED_OFF

//#define BOARD_LED0            LAUNCHPAD_RED_LED
//#define BOARD_LED0_ON         LAUNCHPAD_RED_LED_ON
//#define BOARD_LED0_OFF        LAUNCHPAD_RED_LED_OFF

#define SPI_AX5031_INDEX 0

// The SPI pins are defined in the board files
//#define BOARD_SPI0_CSN             BOOSTERPACK_SDCARD_CS
// AX5031_CS is already defined

/********************************************************************/
/* module configurations                                            */
/********************************************************************/

#define USE_UART_PRINTF
#define BOARD_UART0_TX VH_CONNECTOR_7
#define BOARD_UART0_RX VH_CONNECTOR_9

//#define USE_BASESTATION
#define USE_TAG
//#define USE_FLASHCONTROLLER
#define USE_AX5031

//#define USE_VH_LOGGER
//#define USE_SPI_SDCARD
//#define USE_SPI_FLASH
// for v2.9 tags
//#define USE_HALLSENSOR_SWITCH

//#define EXT_REG_1_8V
//#define HALL_SENSOR
//#define BOOTLOADER_BACKDOOR_PIN  6

//#define LEDS_INITIAL_BLINK_COUNTER 0xFF
#define LEDS_INITIAL_BLINK_COUNTER 0

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




#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

#include "../../vildehaye-common/board_vildehaye_ax5031.h"
//#include "../../vildehaye-common/board_vildehaye_sensors_memory.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-lvs-cc13x0-4x4/FILE>

#define LEDS_TX 0
#define BOARD_LED0            TAG_RED_LED
#define BOARD_LED0_ON         TAG_RED_LED_ON
#define BOARD_LED0_OFF        TAG_RED_LED_OFF

#define SPI_AX5031_INDEX 0
//#define BOARD_SPI0_CSN             BOOSTERPACK_SDCARD_CS
// AX5031_CS is already defined
#define BOARD_SPI0_SCLK            AX5031_SPI_SCLK
#define BOARD_SPI0_MISO            AX5031_SPI_MISO
#define BOARD_SPI0_MOSI            AX5031_SPI_MOSI


/********************************************************************/
/* module configurations                                            */
/********************************************************************/

//#define USE_BASESTATION
#define USE_TAG
//#define USE_FLASHCONTROLLER
#define USE_AX5031

//#define USE_VH_LOGGER
//#define USE_SPI_SDCARD
//#define USE_SPI_FLASH
// for v2.9 tags
//#define USE_HALLSENSOR_SWITCH

//#define EXT_REG_1_8V
//#define HALL_SENSOR
//#define BOOTLOADER_BACKDOOR_PIN  6

//#define LEDS_INITIAL_BLINK_COUNTER 0xFF
#define LEDS_INITIAL_BLINK_COUNTER 0

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
