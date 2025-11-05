#ifndef BOARD_VILDEHAYE_AX5031_H
#define BOARD_VILDEHAYE_AX5031_H

//#define MCU_CC13X0

#include <ti/devices/cc13x0/driverlib/ioc.h>
//#include <ti/drivers/PIN.h>

/*
 * Definitions for Vildehaye V3.0 tags that contain a CC1350 MCU and an AX5031 transmitter
 */

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
 * Definition of the tag's devices.
 *
 * The LED pin is shared with the UART TX, and it is active low.
 */

#define TAG_RED_LED         IOID_2
#define TAG_UART_TX         IOID_2
#define TAG_UART_RX         IOID_1

#define TAG_RED_LED_ON  0
#define TAG_RED_LED_OFF 1

#define AX5031_CS                       IOID_2
#define AX5031_SPI_SCLK                 IOID_8
#define AX5031_SPI_MISO                 IOID_7
#define AX5031_SPI_MOSI                 IOID_5

#define BOARD_BOOTLOADER_BACKDOOR  VH_CONNECTOR_12

#endif /* no reload */
