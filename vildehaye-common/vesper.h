/*
 * I2C interface to Vesper tags
 */

#include "config.h"

#ifdef USE_VESPER

typedef uint32_t nvm_t;

extern nvm_t nvmSize;

#include "logger.h"

//#define VESPER_SUCCESS 0
//#define VESPER_INVALID_RESPONSE 1
//#define VESPER_NO_RESPONSE 2

uint8_t vesperWakeup();

//void vesperInit();
//void vesperAck(nvm_t address);
//uint32_t vesperInteract(uint8_t* length, uint8_t* type,
//                        uint8_t* data,
//                        nvm_t*   returnedItemAddress,
//                        uint8_t* wakeup);

#endif



