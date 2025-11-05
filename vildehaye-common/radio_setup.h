#ifndef RADIO_SETTINGS_H_
#define RADIO_SETTINGS_H_
#include <stdint.h>
#include <limits.h>

#include "config.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)

//#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_sl_longrange.h)
//#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_sl_longrange.h)
//#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_sl_longrange.h)

#include <ti/drivers/rf/RF.h>

// Sivan Sep 2019 I don't see why these are needed in the h file
//#include <rf_patches/rf_patch_cpe_genfsk.h>
//#include <rf_patches/rf_patch_rfe_genfsk.h>
//#include "rf_patch_cpe_genfsk.h"
//#include "rf_patch_rfe_genfsk.h"

#ifndef MAX_RADIO_SETUPS
#define MAX_RADIO_SETUPS 3 // changed from 4, Sep 2019, to make memory fit
#endif

#define MODULATION_TYPE_FSK  0
#define MODULATION_TYPE_GFSK 1
#define DEVIATION_STEP_SIZE  250

extern RF_Mode RF_prop;
//extern uint32_t pOverrides[];
extern RF_Mode                        radio_mode              [MAX_RADIO_SETUPS];
#ifdef DeviceFamily_CC13X2
extern rfc_CMD_PROP_RADIO_DIV_SETUP_PA_t radio_cmd_prop_div_setup[MAX_RADIO_SETUPS];
#else
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t radio_cmd_prop_div_setup[MAX_RADIO_SETUPS];
#endif
extern rfc_CMD_FS_t                   radio_cmd_fs            [MAX_RADIO_SETUPS];
extern rfc_CMD_PROP_TX_t              radio_cmd_prop_tx       [MAX_RADIO_SETUPS];
extern rfc_CMD_PROP_TX_ADV_t          radio_cmd_prop_tx_adv   [MAX_RADIO_SETUPS];
extern rfc_CMD_PROP_RX_t              radio_cmd_prop_rx       [MAX_RADIO_SETUPS];
extern rfc_CMD_NOP_t                  radio_cmd_nop                             ;

extern void radioSetup_init();

extern void radioSetup_frequency(uint32_t index, uint32_t f);
extern void radioSetup_modulation(uint32_t index,
		                              uint8_t modulation_type,
                              		uint32_t symbolrate,
		                              uint32_t deviation,
																	uint32_t rxbw);
extern void radioSetup_txPower(uint32_t index, int8_t dbm);

#define DATA_ENCODING_CC1310_MODE2LRM 4
// fastlrm for raw bitrates higher than 100kb/s
#define DATA_ENCODING_CC1310_FASTLRM 3
#define DATA_ENCODING_CC1310_LRM     2
#define DATA_ENCODING_CC1101_FEC     1
#define DATA_ENCODING_PLAIN          0

extern void radioSetup_packetFormat(uint32_t index, int8_t packetFormat);

extern void radioSetup_setFastLRM(uint32_t index);

#define RPE(prot,mod,txp,msb,swb,enc) ((prot) | (((mod<<6)|(txp+32)) << 8) | ((((msb)<<7)|swb) << 16)  | (enc<<24))
#define RADIO_SETUP_BUFFER_SIZE(n) ((n)*6 * sizeof(uint32_t))

//static const uint32_t radioData[] = {
//	RPE(DATA_PROTOCOL_ATLAS,            MODULATION_TYPE_FSK, 10,1,32,DATA_ENCODING_PLAIN  ), 0x333C3C33, 915000000, 380859,  999756, 2200000,
//	RPE(DATA_PROTOCOL_VILDEHAYE_BEACON, MODULATION_TYPE_FSK, 10,1,32,DATA_ENCODING_CC1310_FASTLRM), 0x930B51DE, 915000000, 175000,  500000, 1410000,
//  RPE(DATA_PROTOCOL_VILDEHAYE_SESSION,MODULATION_TYPE_GFSK,10,1,32,DATA_ENCODING_PLAIN  ), 0x333C3C33, 915000000, 253906,  499878, 1100000,
//};

extern uint8_t radioSetupsCount;
extern uint8_t  radioSetupDataProtocol[ MAX_RADIO_SETUPS ];
//void radioSetup_configureFromBuffer(const uint32_t* radioData, uint16_t radioDataLength);
void radioSetup_configureFromBuffer(const uint8_t* radioData, uint16_t radioDataLength);

extern uint32_t ratDiffToUs(uint32_t now, uint32_t rat_time);

#endif /* RADIO_SETTINGS_H_ */
