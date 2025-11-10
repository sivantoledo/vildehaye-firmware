#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

/* BUILD_CONFIG_ARCH cc13x0 */
/* BUILD_CONFIG_SC uart-cc13x0-7x7 */

#include "../../vildehaye-common/board_cc1350_launchpad.h"
#include "../../vildehaye-common/board_logger_boosterpack.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-uart-cc13x0-7x7/FILE>

//#define BOARD_BUTTON4             BOOSTERPACK_J1_10 /* pin 27 on cc1350 launchpads */
//#define BOARD_BUTTON4_PRESSED     0

#define BOOSTERPACK_SENSE            IOID_27
#define BOOSTERPACK_SENSE_GND        IOID_28   /* ground point for tether signal (to allow a jumper to set it) */

// uart16 implies sensor-controller uart
#define HOST_UART 0
#define GPS_UART  16

#define BOARD_UART0_TX        LAUNCHPAD_UART_TX
#define BOARD_UART0_RX        LAUNCHPAD_UART_RX

// in CC13X0, there is no UART1 but we use the sensor-controller's UART instead
#define BOARD_UART1_TX        BOOSTERPACK_GPS_TX
#define BOARD_UART1_RX        BOOSTERPACK_GPS_RX

#define SPI_SDCARD_INDEX 0
//#define BOARD_SPI0_CSN             BOOSTERPACK_SDCARD_CS
#define SDCARD_CS                  BOOSTERPACK_SDCARD_CS
#define BOARD_SPI0_SCLK            BOOSTERPACK_SPI_SCLK
#define BOARD_SPI0_MISO            BOOSTERPACK_SPI_MISO
#define BOARD_SPI0_MOSI            BOOSTERPACK_SPI_MOSI
#define BOARD_I2C0_SCL             BOOSTERPACK_I2C_SCL
#define BOARD_I2C0_SDA             BOOSTERPACK_I2C_SDA

#define LEDS_TX 0
#define BOARD_LED0            LAUNCHPAD_RED_LED
#define BOARD_LED0_ON         LAUNCHPAD_RED_LED_ON
#define BOARD_LED0_OFF        LAUNCHPAD_RED_LED_OFF

#define LEDS_RX 1
#define BOARD_LED1            LAUNCHPAD_GREEN_LED
#define BOARD_LED1_ON         LAUNCHPAD_GREEN_LED_ON
#define BOARD_LED1_OFF        LAUNCHPAD_GREEN_LED_OFF

#define LEDS_SDCARD  2
#define BOARD_LED2     BOOSTERPACK_LED_SDCARD
#define BOARD_LED2_ON  BOOSTERPACK_LED_SDCARD_ON
#define BOARD_LED2_OFF BOOSTERPACK_LED_SDCARD_OFF

#define LEDS_CLOCK   3
#define BOARD_LED3     BOOSTERPACK_LED_CLOCK
#define BOARD_LED3_ON  BOOSTERPACK_LED_CLOCK_ON
#define BOARD_LED3_OFF BOOSTERPACK_LED_CLOCK_OFF

#define LEDS_TX2     4
#define BOARD_LED4     BOOSTERPACK_LED_TX
#define BOARD_LED4_ON  BOOSTERPACK_LED_TX_ON
#define BOARD_LED4_OFF BOOSTERPACK_LED_TX_OFF

#define LEDS_RX2     5
#define BOARD_LED5     BOOSTERPACK_LED_RX
#define BOARD_LED5_ON  BOOSTERPACK_LED_RX_ON
#define BOARD_LED5_OFF BOOSTERPACK_LED_RX_OFF

// force high VDDR, to allow 14dBm if so configured
#define CCFG_FORCE_VDDR_HH 0x1

/********************************************************************/
/* module configurations                                            */
/********************************************************************/

#define USE_BASESTATION
//#define USE_TAG
//#define USE_FLASHCONTROLLER
#define USE_CONSOLE

#define USE_NVM
#define USE_SPI_SDCARD
#define USE_VH_LOGGER
//#define USE_SPI_FLASH
// for v2.9 tags
//#define USE_HALLSENSOR_SWITCH

//#define EXT_REG_1_8V
//#define HALL_SENSOR
//#define BOOTLOADER_BACKDOOR_PIN  6

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
