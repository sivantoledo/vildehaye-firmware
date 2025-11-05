/*
 * spi-flash.h
 *
 *  Created on: Jan 30, 2018
 *      Author: Nir Zaidman
 */

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

#define NVM_UNBUFFERED_READS

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t nvm_t;

#define NVM_PAGE_SIZE            256
#define NVM_SECTOR_SIZE         4096

#define NVM_PAGE_OFFSET_MASK              0x000000FF
#define NVM_PAGE_ADDRESS_MASK     0xFFFFFFFFFFFFFF00L
#define NVM_SECTOR_OFFSET_MASK            0x00000FFF
#define NVM_SECTOR_ADDRESS_MASK   0xFFFFFFFFFFFFF000L

int spiFlashWriteInProgress();
bool spiFlashEraseChip();
//#include "nvm.h"

#ifdef OBSOLETE
#include "config.h"

#if defined(VH_LOGGER) && defined(USE_TAG)

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t nvm_t;

#define NVM_PAGE_SIZE            256
#define NVM_SECTOR_SIZE         4096

#define NVM_PAGE_OFFSET_MASK              0x000000FF
#define NVM_PAGE_ADDRESS_MASK     0xFFFFFFFFFFFFF00L
#define NVM_SECTOR_OFFSET_MASK            0x00000FFF
#define NVM_SECTOR_ADDRESS_MASK   0xFFFFFFFFFFFFF000L

#include "nvm.h"

#ifdef FLASH_512
#define MAX_PAGE 262144
#endif
#ifdef FLASH_256
#define MAX_PAGE 131072
#endif

#define PAGE_SIZE 256
#define HEADER_SIZE 5
#define EMPTY_PAGE 0xFF

#define CONFIG_PAGE 0x0
#define BMI_160_PAGE 0x1
#define DONE_READ_PAGE 0x2

bool spiFlashRDID(uint8_t* data, uint32_t length);
bool spiFlashReadPage(uint32_t page, uint8_t* data);
bool spiFlashRead(uint32_t addr, uint32_t length, uint8_t* data);

void spiFlashBufferClear();
void spiFlashBufferWrite(uint32_t offset, uint32_t length, uint8_t* data);
bool spiFlashBufferProgram(uint32_t addr);

bool spiFlashBufferRead(uint32_t addr);
uint8_t spiFlashBufferGetByte(uint32_t offset);
void spiFlashBufferGetRange(uint32_t offset, uint32_t length, uint8_t* data);


bool spiFlashWritePage(uint32_t page, uint8_t* data);

bool spiFlashEraseChip();
bool spiFlashReset();
bool spiFlashEnter4ByteMode();
bool spiFlashDeepPowerDown();
bool spiFlashReleaseFromDeepPowerDown();
void spiFlashUseInternalFlash(); // on launchpads, default is external
void spiFlash_init();

#endif // VH_LOGGER

#endif // obsolete

#endif /* SPI_FLASH_H_ */
