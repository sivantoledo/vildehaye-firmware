#ifndef BOARD_VILDEHAYE_V3_0_H
#define BOARD_VILDEHAYE_V3_0_H

#define MCU_CC13X0

#define LINEAR_REGULATOR

#include <ti/devices/cc13x0/driverlib/ioc.h>
//#include <ti/drivers/PIN.h>

/*
 * Definition of the tag's connector pinout
 */

#define VH_CONNECTOR_1      RESET
#define VH_CONNECTOR_3      IOID_3  // I2C SCL
#define VH_CONNECTOR_5      IOID_4  // I2C SDA
#define VH_CONNECTOR_7      IOID_2  // MCU TXD - host RXD
#define VH_CONNECTOR_9      IOID_1  // MCU RXD - host TXD
#define VH_CONNECTOR_11     NC      // SPI-CS3
#define VH_CONNECTOR_13     GND

#define VH_CONNECTOR_2      VDDS
#define VH_CONNECTOR_4      IOID_9  // SPI CS1
#define VH_CONNECTOR_6      IOID_8  // SPI SCLK
#define VH_CONNECTOR_8      IOID_7  // SPI MISO
#define VH_CONNECTOR_10     IOID_5  // SPI MOSI
#define VH_CONNECTOR_12     IOID_6  // bootloader trapdoor
#define VH_CONNECTOR_14     GND

/*
 * Definition of the Lauchpad's devices
 */

// The LED is installed backwards, unfotunately, so it does not work.
//#define TAG_RED_LED_ON  0
//#define TAG_RED_LED_OFF 1

//#define TAG_RED_LED         IOID_2
//#define TAG_UART_TX         IOID_2
//#define TAG_UART_RX         IOID_1

#define AX5031_CS                  IOID_0
#define BOARD_SPI0_SCLK            IOID_8
#define BOARD_SPI0_MISO            IOID_7
#define BOARD_SPI0_MOSI            IOID_5

#define AX5031_XTAL_FREQ           24000000
//#define AX5031_XTAL_FREQ           16000000

#define BOARD_BOOTLOADER_BACKDOOR  VH_CONNECTOR_12

#endif /* no reload */
