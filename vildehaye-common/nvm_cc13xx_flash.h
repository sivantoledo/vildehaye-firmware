/*
 * spi-flash.h
 *
 *  Created on: Jan 30, 2018
 *      Author: Nir Zaidman
 */

#ifndef NVM_CC13XX_FLASH_H_
#define NVM_CC13XX_FLASH_H_

#define NVM_UNBUFFERED_READS

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t nvm_t;

#define NVM_PAGE_SIZE            64
#define NVM_SECTOR_SIZE         128

#define NVM_PAGE_OFFSET_MASK              0x0000003F
#define NVM_PAGE_ADDRESS_MASK     0xFFFFFFFFFFFFFFC0L
#define NVM_SECTOR_OFFSET_MASK            0x0000007F
#define NVM_SECTOR_ADDRESS_MASK   0xFFFFFFFFFFFFFF80L

#endif /* SPI_FLASH_H_ */
