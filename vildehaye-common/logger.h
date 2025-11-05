/*
 * logger.h
 *
 *  Created on: 2018
 *      Author: stoledo
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "config.h"

#if defined(USE_VH_LOGGER) || defined(USE_VESPER)

#include "nvm.h"
//#ifdef BASESTATION_FIRMWARE
//#include "spi_sdcard.h"
//#else
//#include "spi_flash.h"
//#endif

#define LOGGER_SUCCESS            0
#define LOGGER_ERR_OUT_OF_SPACE   1
#define LOGGER_ERR_NO_MORE_ITEMS  2
#define LOGGER_ERR_UNFORMATTED    3
#define LOGGER_ERR_MISMATCH       4
#define LOGGER_ERR_COMM           5
#define LOGGER_ERR_MISBEHAVIOR    6

// maximum data item length
#define LOGGER_DATA_ITEM_MAX 224

#define LOGGER_STATE_UNKNOWN  0
#define LOGGER_NO_FLASH       1
#define LOGGER_LOGGING        2
#define LOGGER_UART_CTRL      3
#define LOGGER_UNFORMATTED    4
#define LOGGER_FLASH_IS_FULL  5

#define LOGGER_MEMORY_TYPE_NOR             1
#define LOGGER_MEMORY_TYPE_NAND            2
#define LOGGER_MEMORY_TYPE_SDCARD          3
#define LOGGER_MEMORY_TYPE_FRAM            4

#define LOGGER_DATATYPE_LOG_HEADER         8
#define LOGGER_DATATYPE_LOG_SECTOR_HEADER  9
#define LOGGER_DATATYPE_LOG_BOOT_MARKER   10
#define LOGGER_DATATYPE_LOG_CATALOG_ID    11
#define LOGGER_DATATYPE_LOG_FILLER        12
#define LOGGER_DATATYPE_LOG_ITERATOR      13
#define LOGGER_DATATYPE_LOG_MESSAGE       14

#define LOGGER_FLAG_SKIP_TO_NEXT_PAGE     0xFE
#define LOGGER_FLAG_SKIP_TO_NEXT_SECTOR   0xFD

extern uint8_t  loggerState;
extern nvm_t loggerCurrentItemAddress;
extern nvm_t loggerFreeFlashPointer;
extern nvm_t loggerFlashSize;

extern uint32_t loggerLogCreationTime;

void loggerInit();
void loggerClose();
//void loggerTask_init();

// next functions return an error code
uint32_t loggerLog(uint8_t type, uint8_t* data, uint8_t length);

uint32_t loggerNext(uint8_t* length, uint8_t* type,
                    uint8_t* data,
                    nvm_t* returnedItemAddress);
void loggerAck(nvm_t itemAddress);
nvm_t loggerQueueSize();

void loggerIteratorInit(nvm_t* address);
uint32_t loggerIteratorNext(uint8_t*  length, uint8_t* type,
                            uint8_t*  data,
                            nvm_t* nextItemAddress,
                            nvm_t* returnedItemAddress);

void flashcontrollerTask_init();

#endif // VH_LOGGER

#endif /* LOGGER_H_ */
