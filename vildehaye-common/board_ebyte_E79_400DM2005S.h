#ifndef BOARD_EBYTE_E79_CC1352P_H
#define BOARD_EBYTE_E79_CC1352P_H

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>

#define NC PIN_UNASSIGNED

/*
 *  CC1352 launchpads have a different pinout than CC1350 launchpads.
 *
 *  Pins 28, 29, 30 control the RF switch.
 *
 */

//#include <ti/drivers/PIN.h>

/*
 * Definition of the Lauchpad's boosterpack pinout
 */


/*
 * Definition of the Lauchpad's devices
 */

//#define LAUNCHPAD_RED_LED         IOID_6
//#define LAUNCHPAD_GREEN_LED       IOID_7
//#define LAUNCHPAD_BTN1            IOID_15
//#define LAUNCHPAD_BTN2            IOID_14
//#define RF_SWITCH       IOID_1
//#define RF_SWITCH_POWER IOID_30
//#define LAUNCHPAD_UART_TX         IOID_13
//#define LAUNCHPAD_UART_RX         IOID_12
//#define LAUNCHPAD_FLASH_CS        IOID_20 /* We normlaly do not use the on-board flash */
//#define LAUNCHPAD_FLASH_SCLK      IOID_10
//#define LAUNCHPAD_FLASH_MISO      IOID_8
//#define LAUNCHPAD_FLASH_MOSI      IOID_9

//#define RF_SWITCH_SUB1GHZ 1
//#define RF_SWITCH_2_4GHZ  0

//#define LAUNCHPAD_BTN1_PRESSED    0
//#define LAUNCHPAD_BTN2_PRESSED    0

//#define LAUNCHPAD_RED_LED_ON  1
//#define LAUNCHPAD_RED_LED_OFF 0

//#define LAUNCHPAD_GREEN_LED_ON  1
//#define LAUNCHPAD_GREEN_LED_OFF 0

//#define BOARD_BUTTON0         LAUNCHPAD_BTN1
//#define BOARD_BUTTON0_PRESSED LAUNCHPAD_BTN1_PRESSED
//#define BOARD_BUTTON1         LAUNCHPAD_BTN2
//#define BOARD_BUTTON1_PRESSED LAUNCHPAD_BTN2_PRESSED

//#define BOARD_BOOTLOADER_BACKDOOR  LAUNCHPAD_BTN1

// The RF switch settings are for LAUNCHXL-CC1352P1-4
// in which 433 is always no PA dn 2.4 is either with PA
// or no PA.

#define RF_SWITCH
#define PA_PATH_DURING_TX
//#define FORCE_PA

#define RF_SWITCH_0       IOID_5 // sub-GHz TX high-power TX
#define RF_SWITCH_1       IOID_6 // sub-GHz RX (and probably low-power TX)

#define RF_SWITCH_0_INITIAL PIN_GPIO_LOW
#define RF_SWITCH_1_INITIAL PIN_GPIO_HIGH

#define RF_SWITCH_0_OFF        0
#define RF_SWITCH_1_OFF        1

// should be high-low for PA

// should be high-low for PA
#define RF_SWITCH_0_SUB1GHZ_TX 0
#define RF_SWITCH_1_SUB1GHZ_TX 1

#define RF_SWITCH_0_SUB1GHZ_TX_PA 1
#define RF_SWITCH_1_SUB1GHZ_TX_PA 0

#define RF_SWITCH_0_SUB1GHZ_RX 0
#define RF_SWITCH_1_SUB1GHZ_RX 1

#define RF_SWITCH_0_2_4GHZ_RX 0
#define RF_SWITCH_1_2_4GHZ_RX 1

#define RF_SWITCH_0_2_4GHZ_TX 0
#define RF_SWITCH_1_2_4GHZ_TX 1

#define RF_SWITCH_0_2_4GHZ_TX_PA 0
#define RF_SWITCH_1_2_4GHZ_TX_PA 1


#endif /* BOARD_CC1352P_LAUNCHPAD_H */
