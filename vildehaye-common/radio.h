#ifndef RADIO_H_
#define RADIO_H_
#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <limits.h>

#define RADIO_SUCCESS  0

#define RADIO_NO_RADIO 1
#define RADIO_ERROR    2
#define RADIO_SYNTHESIZER_FAILURE 3
#define RADIO_PAST_START 3
#define RADIO_RX_TIMEOUT 3

//#define RADIO_FOREVER 0xFFFFFFFF
#define RADIO_SCHEDULE_NOW     1
#define RADIO_SCHEDULE_REL_TS  2
#define RADIO_SCHEDULE_REL_NOW 3

#define RADIO_IDENTITY_BITS 0
#define RADIO_INVERT_BITS   1

#ifndef MAX_RADIO_SETUPS
#define MAX_RADIO_SETUPS 3 // changed from 4, Sep 2019, to make memory fit
#endif

#define MODULATION_TYPE_FSK  0
#define MODULATION_TYPE_GFSK 1

#define RADIO_DBM_DEFAULT 127 // invalid value
#define RADIO_DBM_KEEP    126 // invalid value

extern void radioSetup_init();

extern void radioSetup_frequency(uint32_t index, uint32_t f);
extern void radioSetup_modulation(uint32_t index,
		                              uint8_t modulation_type,
                              		uint32_t symbolrate,
		                              uint32_t deviation,
																	uint32_t rxbw);
extern void radioSetup_txPower(uint32_t index, int8_t dbm);

/* packet formats */
#define DATA_ENCODING_CC1310_MODE2LRM 4
// fastlrm for raw bitrates higher than 100kb/s
#define DATA_ENCODING_CC1310_FASTLRM 3
#define DATA_ENCODING_CC1310_LRM     2
#define DATA_ENCODING_CC1101_FEC     1
#define DATA_ENCODING_PLAIN          0

#define RADIO_MODULATION_FSK  0
#define RADIO_MODULATION_GFSK 1
#define RADIO_MODULATION_PSK  2
#define RADIO_MODULATION_DPSK 3

extern void radioSetup_packetFormat(uint32_t index, int8_t packetFormat);

extern uint8_t radioSetupsCount;
extern uint8_t  radioSetupDataProtocol[ MAX_RADIO_SETUPS ];
//void radioSetup_configureFromBuffer(const uint32_t* radioData, uint16_t radioDataLength);

extern int radioInit();
extern int radioPrepare(uint8_t setupIndex, int bitInversion, int tx, int8_t dbm);
extern int radioTransmit(uint8_t* packet, int length, int scheduleType, uint32_t timestamp, int32_t offset);

//extern int  radioTransmit(uint8_t* data, size_t length);
extern int  radioReceiveMessage(uint32_t howLongUs);
extern int  radioReceiveStart();
extern void radioReceiveStop();
extern void radioShutdown();

#endif /* RADIO_H_ */
