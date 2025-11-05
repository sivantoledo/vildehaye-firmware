#ifndef BOARD_CC1350_LAUNCHPAD_H
#define BOARD_CC1350_LAUNCHPAD_H

#include <ti/devices/cc13x0/driverlib/ioc.h>
//#include <ti/drivers/PIN.h>

#define MCU_CC13X0
#define MCU_CC1350

/*
 * Definition of the Lauchpad's boosterpack pinout
 */

#define BOOSTERPACK_J1_1      VDDS
#define BOOSTERPACK_J1_3      IOID_23
#define BOOSTERPACK_J1_5      IOID_3
#define BOOSTERPACK_J1_7      IOID_2
#define BOOSTERPACK_J1_9      IOID_22
#define BOOSTERPACK_J1_11     IOID_24
#define BOOSTERPACK_J1_13     IOID_10
#define BOOSTERPACK_J1_15     IOID_21
#define BOOSTERPACK_J1_17     IOID_4
#define BOOSTERPACK_J1_19     IOID_5

#define BOOSTERPACK_J1_2      VUSB
#define BOOSTERPACK_J1_4      GND
#define BOOSTERPACK_J1_6      IOID_25
#define BOOSTERPACK_J1_8      IOID_26
#define BOOSTERPACK_J1_10     IOID_27
#define BOOSTERPACK_J1_12     IOID_28
#define BOOSTERPACK_J1_14     IOID_29
#define BOOSTERPACK_J1_16     IOID_30
#define BOOSTERPACK_J1_18     NC
#define BOOSTERPACK_J1_20     IOID_1

#define BOOSTERPACK_J2_1      IOID_7
#define BOOSTERPACK_J2_3      IOID_6
#define BOOSTERPACK_J2_5      IOID_20
#define BOOSTERPACK_J2_7      IOID_19
#define BOOSTERPACK_J2_9      IOID_18
#define BOOSTERPACK_J2_11     BOOSTERPACK_RESET
#define BOOSTERPACK_J2_13     JTAG_TMS
#define BOOSTERPACK_J2_15     JTAG_TCK
#define BOOSTERPACK_J2_17     IOID_16
#define BOOSTERPACK_J2_19     IOID_17

#define BOOSTERPACK_J2_2      GND
#define BOOSTERPACK_J2_4      IOID_12
#define BOOSTERPACK_J2_6      IOID_11
#define BOOSTERPACK_J2_8      NC
#define BOOSTERPACK_J2_10     LAUNCHPAD_RESET
#define BOOSTERPACK_J2_12     IOID_9
#define BOOSTERPACK_J2_14     IOID_8
#define BOOSTERPACK_J2_16     IOID_13
#define BOOSTERPACK_J2_18     IOID_14
#define BOOSTERPACK_J2_20     IOID_15

/*
 * Definition of the Lauchpad's devices
 */

#define LAUNCHPAD_RED_LED         IOID_6
#define LAUNCHPAD_GREEN_LED       IOID_7
#define LAUNCHPAD_BTN1            IOID_13
#define LAUNCHPAD_BTN2            IOID_14
#define LAUNCHPAD_UART_TX         IOID_3
#define LAUNCHPAD_UART_RX         IOID_2
#define LAUNCHPAD_FLASH_CS        IOID_20 /* We normlaly do not use the on-board flash */
#define LAUNCHPAD_FLASH_SCLK      IOID_10
#define LAUNCHPAD_FLASH_MISO      IOID_8
#define LAUNCHPAD_FLASH_MOSI      IOID_9

#define LAUNCHPAD_BTN1_PRESSED    0
#define LAUNCHPAD_BTN2_PRESSED    0

#define LAUNCHPAD_RED_LED_ON  1
#define LAUNCHPAD_RED_LED_OFF 0

#define LAUNCHPAD_GREEN_LED_ON  1
#define LAUNCHPAD_GREEN_LED_OFF 0

#define BOARD_BUTTON0         LAUNCHPAD_BTN1
#define BOARD_BUTTON0_PRESSED LAUNCHPAD_BTN1_PRESSED
#define BOARD_BUTTON1         LAUNCHPAD_BTN2
#define BOARD_BUTTON1_PRESSED LAUNCHPAD_BTN2_PRESSED

#define BOARD_BOOTLOADER_BACKDOOR  LAUNCHPAD_BTN1

//#define LAUNCHPAD_RF_SWITCH       IOID_1
//#define LAUNCHPAD_RF_SWITCH_POWER IOID_30

//#define LAUNCHPAD_RF_SWITCH_SUB1GHZ 1
//#define LAUNCHPAD_RF_SWITCH_2_4GHZ  0

#define RF_SWITCH_0       IOID_1  // 1 for sub-GHz, 0 for 2.4GHz
#define RF_SWITCH_1       IOID_30 // switch enable

// there is no PA but we need to define the constants
// these switch settings are correct for the LAUNCHXL-CC1350-4

#define RF_SWITCH_0_INITIAL PIN_GPIO_LOW
#define RF_SWITCH_1_INITIAL PIN_GPIO_LOW

#define RF_SWITCH_0_OFF        0
#define RF_SWITCH_1_OFF        0

#define RF_SWITCH_0_SUB1GHZ_TX 1
#define RF_SWITCH_1_SUB1GHZ_TX 1
#define RF_SWITCH_0_SUB1GHZ_TX_PA 1
#define RF_SWITCH_1_SUB1GHZ_TX_PA 1

#define RF_SWITCH_0_SUB1GHZ_RX 1
#define RF_SWITCH_1_SUB1GHZ_RX 1

#define RF_SWITCH_0_2_4GHZ_RX 0
#define RF_SWITCH_1_2_4GHZ_RX 1

#define RF_SWITCH_0_2_4GHZ_TX 0
#define RF_SWITCH_1_2_4GHZ_TX 1
#define RF_SWITCH_0_2_4GHZ_TX_PA 0
#define RF_SWITCH_1_2_4GHZ_TX_PA 1

#endif /* BOARD_CC1350_LAUNCHPAD_H */
