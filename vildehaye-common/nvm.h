/*
 * nvm.h
 *
 * Created on: 24 August 2018
 * Author: Sivan Toledo
 *
 * An abstract interface for non-volatile memories for logging.
 * Implementations include SPI NOR flash and SPI SD cards.
 *
 * The implementations are allowed to assume that implementations that perform
 * buffered writes will not interleave them with reads, to save a buffer.
 *
 * This file should be is used only by the NVM implementation header file,
 * which in turn are used only by the logger.
 *
 * The NVM implemetation header should define the pointer type nvm_t
 * and macros that are used to manipulate addresses.
 *
 * Generic algorithms are implemented in nvm.c; the rest in implementations like
 * spi_sdcard.c and spi_flash.c.
 *
 */

#ifndef NVM_H_
#define NVM_H_

#include "config.h"

#include <stdint.h>

#if defined(USE_SPI_SDCARD)
#include "spi_sdcard.h"
#define NVM_IMPLEMENTAION SDCARD
#endif

#if defined(USE_SPI_FLASH)
#include "spi_flash.h"
#define NVM_IMPLEMENTAION FLASH
#endif

#if defined(USE_NVM_CC13XX_FLASH)
#include "nvm_cc13xx_flash.h"
#define NVM_IMPLEMENTAION CC13XX_FLASH
#endif

//#ifdef NVM_64_BIT_ADDRESSING
//typedef uint64_t nvm_t;
//#else
//typedef uint32_t nvm_t;
//#endif

#if defined(NVM_IMPLEMENTAION)
#define pageOffset(addr)    ((addr) & NVM_PAGE_OFFSET_MASK   )
#define sectorOffset(addr)  ((addr) & NVM_SECTOR_OFFSET_MASK )

#define pageAddress(addr)   ((addr) & NVM_PAGE_ADDRESS_MASK  )
#define sectorAddress(addr) ((addr) & NVM_SECTOR_ADDRESS_MASK)

#include <stdbool.h>

extern nvm_t nvmSize;
extern uint8_t nvmPageBuffer[NVM_PAGE_SIZE];


/*
 * returns 1 if nvm is found, 0 otherwise.
 */
bool nvmInit();

void nvmWakeup();
void nvmSleep();

bool spiFlashRead(nvm_t addr, uint32_t length, uint8_t* data);

/*
 * Functions to read and write a page into/from nvmPageBuffer
 */
bool nvmPageRead(nvm_t address);
bool nvmPageWrite(nvm_t address);

/*
 * Buffered writes are for aligned pages.
 */
void nvmBufferInvalidate();
void nvmBufferClear();
void nvmBufferWrite(uint32_t offset, uint32_t length, uint8_t* data);
bool nvmBufferProgram(nvm_t addr);

/*
 * Reads may not be aligned so the nvm driver needs do to the buffering.
 */

uint8_t nvmBufferedReadByte (nvm_t addr);
bool    nvmBufferedReadRange(nvm_t addr, uint32_t length, uint8_t* data);

#ifdef NVM_UNBUFFERED_READS
uint8_t nvmReadByte (nvm_t addr);
bool    nvmReadRange(nvm_t addr, uint32_t length, uint8_t* data);
#endif

//bool spiFlashWritePage(uint32_t page, uint8_t* data);
//bool spiFlashReset();
//bool spiFlashEnter4ByteMode();
//bool spiFlashDeepPowerDown();
//bool spiFlashReleaseFromDeepPowerDown();
//void spiFlashUseInternalFlash(); // on launchpads, default is external

/*
 * Chip erase. Only supported on raw flash.
 * On SD cards it's better to erase on a PC, much faster.
 */
bool nvmEraseChip();
#endif // nvm implementation

#endif /* NVM_H_ */
