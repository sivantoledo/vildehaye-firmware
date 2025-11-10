#ifndef CONFIG_H
#define CONFIG_H

/********************************************************************/
/* board definitions and mappings                                 */
/********************************************************************/

// Sivan Nov 2025: this project was not committed to SVN, and it does refer to a
// sensor-controller project (even though the macro is defined below).

/* BUILD_CONFIG_ARCH cc13x2 */
/* BUILD_CONFIG_SC  */

//#include "../../vildehaye-common/board_cc1352P_launchpad.h"
#include "../../vildehaye-common/board_ebyte_E79_400DM2005S.h"
//#include "../../vildehaye-common/board_logger_boosterpack.h"

//#define SensorControllerDirectory(FILE) <../vildehaye-sc-uart-cc13x0-7x7/FILE>
#define SensorControllerDirectory(FILE) <../vildehaye-sc-lvs-cc13x2-7x7/FILE>

#define BASESTATION_TYPE_FORCE_TETHERED

#define BOOSTERPACK_SENSE            IOID_7
#define BOOSTERPACK_SENSE_GND        IOID_8   /* ground point for tether signal (to allow a jumper to set it) */

// uart16 implies sensor-controller uart
#define HOST_UART 0
#define GPS_UART  1

#define BOARD_UART0_TX        IOID_13 // the only real thing here ...
#define BOARD_UART0_RX        IOID_12 // the only real thing here ...


#define BOARD_BOOTLOADER_BACKDOOR  IOID_15

// in CC13X0, there is no UART1 but we use the sensor-controller's UART instead
#define BOARD_UART1_TX        IOID_26
#define BOARD_UART1_RX        IOID_25

#define SPI_SDCARD_INDEX 0
#define SDCARD_CS                  IOID_16
#define BOARD_SPI0_SCLK            IOID_17
#define BOARD_SPI0_MISO            IOID_18
#define BOARD_SPI0_MOSI            IOID_19
#define BOARD_I2C0_SCL             IOID_20
#define BOARD_I2C0_SDA             IOID_21

#define LEDS_TX 0
#define BOARD_LED0            IOID_22 // actually present
#define BOARD_LED0_ON         1
#define BOARD_LED0_OFF        0

#define LEDS_RX 1
#define BOARD_LED1            IOID_23
#define BOARD_LED1_ON         1
#define BOARD_LED1_OFF        0

#define LEDS_SDCARD  2
#define BOARD_LED2     IOID_24
#define BOARD_LED2_ON  1
#define BOARD_LED2_OFF 0

#define LEDS_CLOCK   3
#define BOARD_LED3     IOID_25
#define BOARD_LED3_ON  1
#define BOARD_LED3_OFF 0

#define LEDS_TX2     4
#define BOARD_LED4     NC
#define BOARD_LED4_ON  1
#define BOARD_LED4_OFF 0

#define LEDS_RX2     5
#define BOARD_LED5     NC
#define BOARD_LED5_ON  1
#define BOARD_LED5_OFF 0

#define LAUNCHPAD_BTN1            IOID_29
#define LAUNCHPAD_BTN2            IOID_23
#define LAUNCHPAD_BTN1_PRESSED    0
#define LAUNCHPAD_BTN2_PRESSED    0

#define BOARD_BUTTON2             IOID_9  /* pin 18 on cc1350 launchpads */
#define BOARD_BUTTON2_PRESSED     0
#define BOARD_BUTTON3             IOID_10  /* pin 22 on cc1350 launchpads */
// pressed is 0 in my boosterpck, 1 in the sparkfun breakout board
#define BOARD_BUTTON3_PRESSED     0
//#define BOARD_BUTTON3_PRESSED     1

#define BOOSTERPACK_BUTTON              BOARD_BUTTON2
#define BOOSTERPACK_BUTTON_PRESSED      BOARD_BUTTON2_PRESSED
#define BOOSTERPACK_CARD_DETECT         BOARD_BUTTON3
#define BOOSTERPACK_CARD_DETECT_PRESSED BOARD_BUTTON3_PRESSED

#define BOOSTERPACK_DISPLAY_POWER       IOID_14    /* TPS2553 switch for display                                   */
#define BOOSTERPACK_DISPLAY_POWER_ON    1



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
