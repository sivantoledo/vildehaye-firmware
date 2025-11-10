#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

// Sivan Nov 2025: this was not committed to SVN for some reason

/* BUILD_CONFIG_ARCH cc13x0 */
/* BUILD_CONFIG_SC lvs-4x4 */

// test

#include "../../vildehaye-common/board_vildehaye_v2_6.h"
//#include "../../vildehaye-common/board_vildehaye_sensors_memory.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-lvs-cc13x0-4x4/FILE>

//#define SPI_FLASH_INDEX 0
//#define SPI_FLASH_CS               ADDON_FLASH_CS
//#define SPI_FLASH_CS_OFF 1
//#define SPI_FLASH_CS_ON  0
//#define BOARD_SPI0_SCLK            ADDON_SPI_SCLK
//#define BOARD_SPI0_MISO            ADDON_SPI_MISO
//#define BOARD_SPI0_MOSI            ADDON_SPI_MOSI
//#define BOARD_I2C0_SCL             ADDON_I2C_SCL
//#define BOARD_I2C0_SDA             ADDON_I2C_SDA

#define BOARD_I2C0_SCL             VH_CONNECTOR_3
#define BOARD_I2C0_SDA             VH_CONNECTOR_5


#define LEDS_TX 0
#define BOARD_LED0            TAG_RED_LED
#define BOARD_LED0_ON         TAG_RED_LED_ON
#define BOARD_LED0_OFF        TAG_RED_LED_OFF

/********************************************************************/
/* module configurations                                            */
/********************************************************************/

//#define USE_BASESTATION
#define USE_TAG
//#define USE_FLASHCONTROLLER

#define USE_VH_SENSORS
#define USE_VH_SENSOR_BME280


#define USE_NVM
#define USE_VH_LOGGER
#define USE_NVM_CC13XX_FLASH
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
