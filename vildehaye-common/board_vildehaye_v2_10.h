#ifndef BOARD_VILDEHAYE_V2_10_H
#define BOARD_VILDEHAYE_V2_10_H

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

#define TAG_RED_LED         IOID_0
#define TAG_UART_TX         IOID_2
#define TAG_UART_RX         IOID_1

#define TAG_RED_LED_ON  1
#define TAG_RED_LED_OFF 0

#define BOARD_BOOTLOADER_BACKDOOR  VH_CONNECTOR_12

#endif /* no reload */
