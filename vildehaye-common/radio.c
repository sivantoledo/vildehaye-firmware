/*
 * Generic radio handling C code
 */

#include "config.h"

#include "radio.h"

#include <xdc/runtime/System.h>

uint8_t  radioSetupDataProtocol[ MAX_RADIO_SETUPS ];
uint8_t radioSetupsCount;
//void radioSetup_configureFromBuffer(const uint32_t* radioData, uint16_t radioDataLength) {
//	 uint8_t* p = (uint8_t*) radioData;
void radioSetup_configureFromBuffer(const uint8_t* radioData, uint16_t radioDataLength) {
	 const uint8_t* p = radioData;
/*
	 uint8_t dataProtocol = DATA_PROTOCOL_ATLAS; // 8 bits
	 uint8_t modulation   = MODULATION_TYPE_FSK; // 2 bits
	 uint8_t txPower      = 10;                  // 6 bits signed
	 uint8_t msbFirst     = 1;                   // 1 bit
	 uint8_t swBits       = 32;                  // 6 bits unsigned
	 uint8_t fecWhiten    = DATA_ENCODING_PLAIN;    // 8 bits
*/
	 //uint8_t i = 0;
	 radioSetupsCount = 0;

	 while ((p - (uint8_t*) radioData) < radioDataLength) {
		 uint8_t b;
		 uint8_t dataProtocol = *(p++); // 8 bits
		 b = *(p++);
		 uint8_t modulation   = (b >> 6) & 0x03; // 2 bits
		 uint8_t txPower      = (b & 0x3F) - 32; // 6 bits signed
		 b = *(p++);
		 uint8_t msbFirst     = (b >> 7);                   // 1 bit
		 uint8_t swBits       = (b & 0x3F);                 // 6 bits unsigned
		 uint8_t fecWhiten    = *(p++);    // 8 bits

		 uint32_t syncWord    = *( (uint32_t*) p ); p+=4;
		 uint32_t freq        = *( (uint32_t*) p ); p+=4;
		 uint32_t dev         = *( (uint32_t*) p ); p+=4;
		 uint32_t symbRate    = *( (uint32_t*) p ); p+=4;
		 uint32_t rxBw        = *( (uint32_t*) p ); p+=4;

		 //System_printf("%d>> %08x %d %d %d %d -- %d %d %d %d %d %d\n", radioSetupsCount, syncWord, freq, dev, symbRate, rxBw, dataProtocol,modulation,txPower,msbFirst,swBits,fecWhiten);

		 radioSetup_modulation  (radioSetupsCount, modulation,symbRate,dev,rxBw);
         radioSetup_frequency   (radioSetupsCount, freq); // Sivan Aug 2020, moved up because overrides depends on divider
		 radioSetup_packetFormat(radioSetupsCount, fecWhiten); // this will override the number of preamble bytes in lrm mode 2
		 System_printf("cfb %d: %d dBm (%d)\n",radioSetupsCount,txPower,(p - (uint8_t*) radioData));

		 // will be set in radioPrepare; July  2022
         //radioSetup_txPower     (radioSetupsCount, txPower); // Sivan Nov 2020 moved down to allow freq-speficic settings

		 radioSetupDataProtocol[ radioSetupsCount ] = dataProtocol;
		 // sync word missing still

		 radioSetupsCount++;
	 }
}


