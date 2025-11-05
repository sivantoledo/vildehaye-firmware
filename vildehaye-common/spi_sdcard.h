#ifndef SPI_SDCARD_H_
#define SPI_SDCARD_H_

//#define NVM_64_BIT_ADDRESSING

#include <stdint.h>

typedef uint64_t nvm_t;

#define NVM_PAGE_SIZE            512
#define NVM_SECTOR_SIZE         4096

#define NVM_PAGE_OFFSET_MASK              0x000001FF
#define NVM_PAGE_ADDRESS_MASK     0xFFFFFFFFFFFFFE00L
#define NVM_SECTOR_OFFSET_MASK            0x00000FFF
#define NVM_SECTOR_ADDRESS_MASK   0xFFFFFFFFFFFFF000L

//#include "nvm.h"

//#define BLOCK_SIZE				512 //in bytes.
//#include <stdbool.h>
//#include <stdint.h>

#if obsolete
//Initialize SD
bool initializeSD();

//Other
void printCID();

//Data transfer
bool writeBlock(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t writeDataBuffer[BLOCK_SIZE]);
bool readBlock(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t readDataBuffer[BLOCK_SIZE]);
bool readUntil(uint8_t giveup, uint8_t value);
bool multipleRead(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint32_t amount, uint8_t *readDataBuffer);
bool multipleWrite(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint32_t amount, uint8_t *writeDataBuffer);
#endif

#endif /* SPI_SDCARD_H_ */

