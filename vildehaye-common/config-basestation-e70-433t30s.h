#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

/* BUILD_CONFIG_ARCH cc13x0 */
/* BUILD_CONFIG_SC uart-cc13x0-7x7 */

#include <ti/devices/cc13x0/driverlib/ioc.h>
#define MCU_CC13X0
#define MCU_CC1350

//#include "../../vildehaye-common/board_cc1350_launchpad.h"
//#include "../../vildehaye-common/board_logger_boosterpack.h"

#define SensorControllerDirectory(FILE) <../vildehaye-sc-uart-cc13x0-7x7/FILE>

// as on cc1350 launchpads (BTN1)
#define BOARD_BOOTLOADER_BACKDOOR    IOID_13

//#define BOARD_BUTTON4             BOOSTERPACK_J1_10 /* pin 27 on cc1350 launchpads */
//#define BOARD_BUTTON4_PRESSED     0

#define BOOSTERPACK_SENSE            IOID_27
#define BOOSTERPACK_SENSE_GND        IOID_28   /* ground point for tether signal (to allow a jumper to set it) */

// uart16 implies sensor-controller uart
#define HOST_UART 0
#define GPS_UART  16

#define BOARD_UART0_TX        IOID_3
#define BOARD_UART0_RX        IOID_2

// in CC13X0, there is no UART1 but we use the sensor-controller's UART instead
#define BOARD_UART1_TX        IOID_25
#define BOARD_UART1_RX        IOID_26

#define SPI_SDCARD_INDEX 0
//#define BOARD_SPI0_CSN             BOOSTERPACK_SDCARD_CS
#define SDCARD_CS                  IOID_11
#define BOARD_SPI0_SCLK            IOID_10
#define BOARD_SPI0_MISO            IOID_8
#define BOARD_SPI0_MOSI            IOID_9
#define BOARD_I2C0_SCL             IOID_4
#define BOARD_I2C0_SDA             IOID_5

#define LEDS_TX 0
#define BOARD_LED0            IOID_6
#define BOARD_LED0_ON         1
#define BOARD_LED0_OFF        0

#define LEDS_RX 1
#define BOARD_LED1            IOID_7
#define BOARD_LED1_ON         1
#define BOARD_LED1_OFF        0

#define LEDS_SDCARD  2
#define BOARD_LED2     IOID_18
#define BOARD_LED2_ON  1
#define BOARD_LED2_OFF 0

#define LEDS_CLOCK   3
#define BOARD_LED3     IOID_19
#define BOARD_LED3_ON  1
#define BOARD_LED3_OFF 0

#define LEDS_TX2       4
#define BOARD_LED4     IOID_20
#define BOARD_LED4_ON  1
#define BOARD_LED4_OFF 0

#define LEDS_RX2     5
#define BOARD_LED5     IOID_21
#define BOARD_LED5_ON  1
#define BOARD_LED5_OFF 0

#define RF_SWITCH_0       IOID_1  // pa enable
#define RF_SWITCH_1       IOID_22 // lna enable

#define RF_SWITCH_0_INITIAL PIN_GPIO_LOW
#define RF_SWITCH_1_INITIAL PIN_GPIO_LOW

#define RF_SWITCH_0_OFF        0
#define RF_SWITCH_1_OFF        0

#define RF_SWITCH_0_SUB1GHZ_TX 1
#define RF_SWITCH_1_SUB1GHZ_TX 0

#define RF_SWITCH_0_SUB1GHZ_RX 0
#define RF_SWITCH_1_SUB1GHZ_RX 1

// this board cannot communicate on 2.4GHz at all

#define RF_SWITCH_0_2_4GHZ_RX 0
#define RF_SWITCH_1_2_4GHZ_RX 0

#define RF_SWITCH_0_2_4GHZ_TX 0
#define RF_SWITCH_1_2_4GHZ_TX 0


#define LAUNCHPAD_BTN1            IOID_13
#define LAUNCHPAD_BTN2            IOID_14
#define BOOSTERPACK_BUTTON              IOID_15
#define BOOSTERPACK_CARD_DETECT         IOID_16
#define BOOSTERPACK_DISPLAY_POWER       IOID_17

#define LAUNCHPAD_BTN1_PRESSED          0
#define LAUNCHPAD_BTN2_PRESSED          0
#define BOOSTERPACK_BUTTON_PRESSED      0
#define BOOSTERPACK_CARD_DETECT_PRESSED 0

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
