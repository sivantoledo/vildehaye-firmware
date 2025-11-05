#include "config.h"

#ifdef EXT_REG_1_8V
#define SET_CCFG_MODE_CONF_DCDC_RECHARGE             0x1        // Do not use the DC/DC during recharge in powerdown
#define SET_CCFG_MODE_CONF_DCDC_ACTIVE               0x1        // Do not use the DC/DC during active mode
#endif

#ifdef LINEAR_REGULATOR
#define SET_CCFG_MODE_CONF_DCDC_RECHARGE             0x1        // Do not use the DC/DC during recharge in powerdown
#define SET_CCFG_MODE_CONF_DCDC_ACTIVE               0x1        // Do not use the DC/DC during active mode
#endif

#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE         0xC5       // Enable ROM boot loader
#define SET_CCFG_BL_CONFIG_BL_LEVEL                  0x0        // Active low, has an internal pullup
#define SET_CCFG_BL_CONFIG_BL_ENABLE                 0xC5       // Enabled boot loader backdoor
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER             BOARD_BOOTLOADER_BACKDOOR // DIO number for boot loader backdoor

// Sivan Nov 2021 commented 0x0 out, since if not defined it's defined in ccfg.c
//#define CCFG_FORCE_VDDR_HH 0x0        // Use default VDDR trim
//#define CCFG_FORCE_VDDR_HH 0x1 // Use default VDDR trim

// now include the built-in ccfg.c file, which uses the macros defined above to configure the chip

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(startup_files/ccfg.c)
