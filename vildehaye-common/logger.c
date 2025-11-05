/*
 * logger.c
 *
 *  Created on: 13 July 2018
 *      Author: stoledo
 */

#include "config.h"

#ifdef USE_VH_LOGGER

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
//#include <boolean.h>
#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>

#include <ti/sysbios/hal/Seconds.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include "config.h"

//#include "Board.h"
#include "buffers.h"
#include "uart.h"
#include "tag.h"
#include "i2c_sensors.h"
#include "vildehaye.h"

#include "logger.h"
#include "logger_catalog.h"
#include "leds.h"

#include "nvm.h"

uint32_t timestampFreq;

uint8_t loggerState = LOGGER_STATE_UNKNOWN;
uint32_t loggerLogCreationTime = 3456;

//uint32_t loggerFlashSize = 0;

/****************************************************/
/* COMMANDS                                         */
/****************************************************/

#define LOGGER_CMD_ID             1

#define LOGGER_CMD_SECTOR_ERASE  16
#define LOGGER_CMD_CHIP_ERASE    17
#define LOGGER_CMD_PROGRAM       18
#define LOGGER_CMD_READ          19
#define LOGGER_CMD_BUFFER_CLEAR  20
#define LOGGER_CMD_BUFFER_WRITE  21
#define LOGGER_CMD_BUFFER_PRGRM  22

#define LOGGER_CMD_FIND_FIRST    27

#define LOGGER_CMD_LOG            2
#define LOGGER_CMD_NEXT           3
#define LOGGER_CMD_ACK_ADVANCE    4
#define LOGGER_CMD_ITERATOR_INIT  5
#define LOGGER_CMD_ITERATOR_NEXT  6

#define LOGGER_CMD_RESET          7
#define LOGGER_CMD_COMMIT         8
//#define LOGGER_CMD_SELECT_CHIP    9

#define LOGGER_ACK                33
#define LOGGER_NACK_UNKNOWN_CMD   34
#define LOGGER_NACK_SPI_FAILED    35
#define LOGGER_NACK_NO_MORE_ITEMS 36

#define LOGGER_KEEPALIVE          85

/****************************************************/
/* Variables for cyclic retrieval                   */
/****************************************************/

       nvm_t loggerCurrentItemAddress = 0;
//static nvm_t loggerNextItemAddress = 0; // XXX also an argument with this name?

/****************************************************/
/* API: LOGGING                                     */
/****************************************************/

nvm_t loggerFreeFlashPointer = 0; // pointer to the next item
//static uint8_t  pageBuffer[256];

#if OBSOLETE
#define pageOffset(addr)    ((addr) & 0x000000FF)
#define sectorOffset(addr)  ((addr) & 0x00000FFF)

#define pageAddress(addr)   ((addr) & 0xFFFFFF00)
#define sectorAddress(addr) ((addr) & 0xFFFFF000)
#endif

static void flush() {
  nvmWakeup();
  nvmBufferProgram( pageAddress(loggerFreeFlashPointer) );
  nvmSleep();
  nvmBufferClear();
}

// The invariant of this function is that the buffer has space when it is called
static void putByte(uint8_t b) {
  nvmBufferWrite(pageOffset(loggerFreeFlashPointer), 1, &b);
  if (pageOffset(loggerFreeFlashPointer)==NVM_PAGE_SIZE-1) flush();

  loggerFreeFlashPointer++;
}

uint32_t loggerLog(uint8_t type, uint8_t* data, uint8_t length) {
    //System_printf("loggerLog type %d\n",type);

  if (loggerFreeFlashPointer >= nvmSize) {
    System_printf("logger out of space\n");
    return LOGGER_ERR_OUT_OF_SPACE;
  }

    uint32_t n = NVM_SECTOR_SIZE - sectorOffset(loggerFreeFlashPointer);
    if (n < (1+1+length)) {
        flush();
        // skip to next page
        loggerFreeFlashPointer += n; // should be on a sector boundary
        // check if we have enough space; when we move to cyclic logs, wrap around and check for erased sector
        if (loggerFreeFlashPointer >= nvmSize) {
          System_printf("logger out of space\n");
          return LOGGER_ERR_OUT_OF_SPACE;
        }
        // Write sector header ....
        putByte(1);
        putByte(LOGGER_DATATYPE_LOG_SECTOR_HEADER);
        putByte(0); // number of times this sector has been erased while log was in use (formatted)
        putByte(4);
        putByte(LOGGER_DATATYPE_LOG_ITERATOR);
        putByte( loggerCurrentItemAddress        & 0xFF); // vildehaye or little endian order
        putByte((loggerCurrentItemAddress >>  8) & 0xFF);
        putByte((loggerCurrentItemAddress >> 16) & 0xFF);
        putByte((loggerCurrentItemAddress >> 24) & 0xFF);
    }

    // now we have enough space in the sector
    putByte(length);
    putByte(type);
    uint32_t i;
    for (i=0; i<length; i++)
        putByte(data[i]);

    return LOGGER_SUCCESS;
}

/*
 * This function fills the rest of the page with 0xFE.
 * The last call to putByte causes the page to be programmed.
 */
uint32_t loggerCommit() {
  if (loggerFreeFlashPointer >= nvmSize) {
    System_printf("logger out of space\n");
    return LOGGER_ERR_OUT_OF_SPACE;
  }

  while (pageOffset(loggerFreeFlashPointer) != 0)
    putByte(LOGGER_FLAG_SKIP_TO_NEXT_PAGE);

  return LOGGER_SUCCESS;
}

// here this is empty, defined for Vesper
void loggerAck(nvm_t address) {}

/****************************************************/
/* API: VOLATILE ITERATION                          */
/****************************************************/

void loggerIteratorInit(nvm_t* address) {
    *address = 0;
}

uint32_t loggerIteratorNext(uint8_t*  length, uint8_t* type,
                            uint8_t*  data,
                            nvm_t* nextItemAddress, // this is the iterator; can point to empty space too
                            nvm_t* returnedItemAddress) {
  nvmWakeup();

  *length = nvmBufferedReadByte(*nextItemAddress);

  //System_printf("+++ %d %d\n",(int) (*nextItemAddress), *length);

  if (*length==LOGGER_FLAG_SKIP_TO_NEXT_PAGE) { // 0xFE, not a valid length
    // advance pointer to first byte in next page
    while (pageOffset(*nextItemAddress) != 0) (*nextItemAddress)++;

    //spiFlashBufferRead(*nextItemAddress);
    // get the length of the first item on the next page
    *length = nvmBufferedReadByte(*nextItemAddress);
  }

  /*
   * If the length is 0xFF, then either there was not enough space for the
   * item on the last page in the sector and we skipped to the next sector,
   * or we got to the erased part of the flash.
   */
  if (*length==0xff) {
    uint32_t n;
    n = NVM_SECTOR_SIZE - sectorOffset(*nextItemAddress); // bytes left in sector
    if ( n < LOGGER_DATA_ITEM_MAX + 2) {
      // maybe there was not enough space in the sector, so skip to next sector and check again
      *nextItemAddress += n;
      if (*nextItemAddress == nvmSize) { // at the end of flash; revise for cyclic use
        *length = 0;
        nvmSleep();
        return LOGGER_ERR_NO_MORE_ITEMS;
      }
      //spiFlashBufferRead(*nextItemAddress);
      *length = nvmBufferedReadByte(*nextItemAddress);
      // if the first (length) byte at the next sector is 0xff, then there are no more items
    }
    // otherwise, there is plenty of space in the sector, but it's empty; this is the end of the log
  }

  if (*length==0xff) {
    *length = 0; // XXXXXX Sivan seems like a bug April 2019
    nvmSleep();
    return LOGGER_ERR_NO_MORE_ITEMS;
  }

  //System_printf("--- %d %d\n",(int) (*nextItemAddress), *length);

  *type = nvmBufferedReadByte((*nextItemAddress)+1);
  if (data != NULL) nvmBufferedReadRange((*nextItemAddress)+2, (*length), data);
  if (returnedItemAddress != NULL) *returnedItemAddress = *nextItemAddress;
  (*nextItemAddress) = (*nextItemAddress) + 2 + (*length);

  //System_printf("*** %d %d\n",(int) (*nextItemAddress), *length);

  nvmSleep();
  return LOGGER_SUCCESS;
}

#ifdef OBSOLETE
static void loggerIteratorInit(nvm_t* i) {
    *i = 0;
}

static uint32_t loggerIteratorNext(uint8_t*  length, uint8_t* type,
                            uint8_t*  data,
                            nvm_t* nextItemAddress, // this is the iterator; can point to empty space too
                            nvm_t* returnedItemAddress) {
    int p; // pointer to next byte in buffer

    nvmWakeup();

    /*
     * We moved to an API with explicit address in every call, so this becomes
     * advisory and not needed.
     */
    spiFlashBufferRead(*nextItemAddress);
    p = 0;
    *length = spiFlashBufferGetByte(p++);

    if (*length==LOGGER_FLAG_SKIP_TO_NEXT_PAGE) { // 0xFE, not a valid length
        while (pageOffset(*nextItemAddress) != 0) (*nextItemAddress)++;

        spiFlashBufferRead(*nextItemAddress);
        p = 0;
        *length = spiFlashBufferGetByte(p++);
    }

    if (*length==0xff) {
        uint32_t n;
        n = 4096 - sectorOffset(*nextItemAddress);
        if ( n < LOGGER_DATA_ITEM_MAX + 2) {
            // maybe there was not enough space in the sector, so skip to next sector and check again
            *nextItemAddress += n;
            if (*nextItemAddress == nvmSize) { // at the end of flash; revise for cyclic use
                *length = 0;
                nvmSleep();
                return LOGGER_ERR_NO_MORE_ITEMS;
            }
            spiFlashBufferRead(*nextItemAddress);
            p = 0;
            *length = spiFlashBufferGetByte(p++);
            // if the first (length) byte at the next sector is 0xff, then there are no more items
        }
        // otherwise, there is plenty of space in the sector, but it's empty; this is the end of the log
    }

    if (*length==0xff) {
        *length = 0;
        nvmSleep();
        return LOGGER_ERR_NO_MORE_ITEMS;
    }

    *type   = spiFlashBufferGetByte(p++);
    if (data != NULL) spiFlashBufferGetRange(p, (*length), data);
    if (returnedItemAddress != NULL) *returnedItemAddress = *nextItemAddress;
    (*nextItemAddress) = (*nextItemAddress) + p + (*length);

    nvmSleep();
    return LOGGER_SUCCESS;
}
#endif
/****************************************************/
/* API: NON-VOLATILE ITERATION                      */
/****************************************************/

nvm_t loggerQueueSize() {
  if (loggerCurrentItemAddress >= loggerFreeFlashPointer) return 0;
  return (loggerFreeFlashPointer-loggerCurrentItemAddress);
}

uint32_t loggerNext(uint8_t* length, uint8_t* type,
                    uint8_t* data,
                    nvm_t* returnedItemAddress) {
  // read the current item, get a pointer to the next item, and return the address of the current item
#ifdef BUGGY
  loggerNextItemAddress = loggerCurrentItemAddress;
  uint32_t rc = loggerIteratorNext(length, type, data, &loggerNextItemAddress, &loggerCurrentItemAddress);
  *returnedItemAddress = loggerCurrentItemAddress;
#else
  uint32_t rc = loggerIteratorNext(length, type, data, &loggerCurrentItemAddress, returnedItemAddress);
  // Sivan Aug 2019: next line is a bug, commented out. The returned address is set by the call above.
  //*returnedItemAddress = loggerCurrentItemAddress;
  return rc;
#endif
}

#ifdef OBSOLETE
uint32_t loggerAck(nvm_t itemAddress) {
  // the address must be of the last returned item
  if (itemAddress != loggerCurrentItemAddress) return LOGGER_ERR_MISMATCH;
  // otherwise, the client consumed the item at currentItemAddress, so we skip to the next address
  loggerCurrentItemAddress = loggerNextItemAddress; // this advances the iterator; April 2019: not really
  return LOGGER_SUCCESS;
}
#endif

/****************************************************/
/* API: INITIALIZATION                              */
/****************************************************/

/*
 * We assume that during initialization, the flash is not in deep power down mode,
 * but it will get into this mode during iteration to find the last iterator.
 *
 * We use unbuffered byte reads here. This is an optimization for NOR flash.
 * For SD cards, it calls the buffered byte read function.
 */

nvm_t loggerFindFirstErasedSector() {
  uint8_t b;
  nvm_t lb, ub, test;

  b = nvmReadByte(0);
  if (b==0xff) {
    System_printf("loggerFindFirstSector empty\n");
    return 0;
  } else
    System_printf("logger: flash first byte is %02x\n",b);

  b = nvmReadByte(nvmSize - NVM_SECTOR_SIZE);
  if (b!=0xff) {
    System_printf("logger: fisrt byte on last sector is %02x\n",b);
    return nvmSize; // even the last sector is in use
  }

  lb = 0;
  ub = (nvmSize/NVM_SECTOR_SIZE) - 1;

  while (lb != ub) {
    test = lb + (ub-lb)/2;
    b = nvmReadByte(test * NVM_SECTOR_SIZE);
    //step = step >> 1; // divide by 2
    System_printf("loggerFindFirstSector sector=%d lb=%d ub=%d b=%02x\n",(int)test,(int)lb,(int)ub,b);
    if (b==0xff) ub = test;
    else         lb = test+1;
  }

  System_printf("loggerFindFirstSector END lb=%d\n",(int)lb);

  return lb * NVM_SECTOR_SIZE;

#if 0

  nvm_t addr = (nvmSize >> 1);
  nvm_t step = (nvmSize >> 1);
  uint8_t  b;

  b = nvmReadByte(0); // spiFlashRead(0, 1, &b);
  if (b==0xff) {
    System_printf("loggerFindFirstSector empty\n");
    return 0;
  } else
    System_printf("SDCard first byte is %02x\n",b);

  while (step > NVM_SECTOR_SIZE) {
    b = nvmReadByte(addr);
    step = step >> 1; // divide by 2
    System_printf("loggerFindFirstSector sector=%d step=%d b=%02x\n",(uint32_t) (addr/4096),step,b);
    if (b==0xff) addr = addr - step;
    else         addr = addr + step;
  }

  // now at a sector granularity
  b = nvmReadByte(addr);
  System_printf("loggerFindFirstSector sector=%d step=%d b=%02x END\n",(uint32_t) (addr/4096),step,b);
  if (b==0xff) return addr;
  else         return addr + step;
#endif
}

static void initFreeFlashPointer() {
  loggerFreeFlashPointer = loggerFindFirstErasedSector();

  if (loggerFreeFlashPointer==0) {
    // we do not even have a log header
    loggerCurrentItemAddress = 0; // nothing retrieved yet
    //loggerNextItemAddress = 0;
    return;
  }

  //if (freeFlashPointer==flashSize) {
  //  // flash is full or almost full
  //  return;
  //}

  // if we got here, the first free sector is somewhere in the middle
  loggerFreeFlashPointer -= NVM_SECTOR_SIZE; // go back one sector

  System_printf("initFreeFlashPointer (went back 1 sector) freeFlashPointer=%08x\n",loggerFreeFlashPointer);

  // now search for the first free address, and also for the most recent loggerCurrentItemAddress

  buffer_descriptor d;
  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

  uint8_t length, type;
  // skip data items until we find no more; this leaves the pointer pointing
  // to the next free space.
  while (loggerIteratorNext(&length, &type,
                            buffers[ d.id ], // we do not want the data
                            &loggerFreeFlashPointer,NULL) == LOGGER_SUCCESS) {
    if (type==LOGGER_DATATYPE_LOG_ITERATOR) { // there must be one in every non-erased sector
      loggerCurrentItemAddress = vildehayeGetUint32(buffers[ d.id ], 4);
    }
    System_printf("  >> freeFlashPointer=%08x len=%d type=%d\n",(int) loggerFreeFlashPointer,length,type);
    if (loggerFreeFlashPointer >= nvmSize) {
      loggerFreeFlashPointer = nvmSize;
      loggerState = LOGGER_FLASH_IS_FULL;
      System_printf("  >> flash is full\n");
      break;
    }
  }

  System_printf("  >> freeFlashPointer=%08x END\n",(int) loggerFreeFlashPointer);

  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
}

#ifdef OBSOLETE
nvm_t loggerFindFirstErasedSector() {
  nvm_t addr = (nvmSize >> 1);
  nvm_t step = (nvmSize >> 1);
  uint8_t  b;

  spiFlashRead(0, 1, &b);
  if (b==0xff) {
    System_printf("loggerFindFirstSector empty\n");
    return 0;
  }

  while (step > NVM_SECTOR_SIZE) {
    spiFlashRead(addr, 1, &b);
    step = step >> 1; // divide by 2
    System_printf("loggerFindFirstSector addr=%08x step=%d b=%02x\n",addr,step,b);
    if (b==0xff) addr = addr - step;
    else         addr = addr + step;
  }

  // now at a sector granularity
  spiFlashRead(addr, 1, &b);
  System_printf("loggerFindFirstSector addr=%08x step=%d b=%02x END\n",addr,step,b);
  if (b==0xff) return addr;
  else         return addr + step;
}

static void initFreeFlashPointer() {
  loggerFreeFlashPointer = loggerFindFirstErasedSector();

  if (loggerFreeFlashPointer==0) {
    // we do not even have a log header
    loggerCurrentItemAddress = 0; // nothing retrieved yet
    nextItemAddress = 0;
    return;
  }

  //if (freeFlashPointer==flashSize) {
  //  // flash is full or almost full
  //  return;
  //}

  // if we got here, the first free sector is somewhere in the middle
  loggerFreeFlashPointer -= NVM_SECTOR_SIZE; // go back one sector

  System_printf("initFreeFlashPointer freeFlashPointer=%08x\n",loggerFreeFlashPointer);

  // now search for the first free address, and also for the most recent loggerCurrentItemAddress

  buffer_descriptor d;
  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

  uint8_t length, type;
  // skip data items until we find no more; this leaves the pointer pointing
  // to the next free space.
  while (loggerIteratorNext(&length, &type,
                            buffers[ d.id ], // we do not want the data
                            &loggerFreeFlashPointer,NULL) == LOGGER_SUCCESS) {
    if (type==LOGGER_DATATYPE_LOG_ITERATOR) { // there must be one in every non-erased sector
      loggerCurrentItemAddress = vildehayeGetUint32(buffers[ d.id ], 4);
    }
    System_printf("  >> freeFlashPointer=%08x type=%d\n",loggerFreeFlashPointer,type);
  }

  System_printf("  >> freeFlashPointer=%08x END\n",loggerFreeFlashPointer);

  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
}
#endif

#ifdef USE_FLASHCONTROLLER
/****************************************************/
/* HOST COMMAND PROCESSOR                           */
/****************************************************/

/*
 * We assume that during host communication, the flash is not in deep power down mode.
 * But it might get into this mode by a logging or reading command ...
 */

static uint32_t hostIterator;
extern uint8_t flashId[3]; // in spi_flash.c

void loggerParseAndExecute(buffer_descriptor* d) {
    uint8_t* buffer = buffers[ d->id ];
    uint16_t len    = d->length;

    vildehaye_packet_t p;

    uint8_t cmd;
    bool ret;
    uint32_t t;
    uint32_t addr, l;
    //int i;
    uint8_t type;
    uint8_t rc;
    uint8_t ll;
    uint32_t returnedItemAddress;

    vildehayeInitPacket(&p, buffer, 0, 256);
    cmd = (uint8_t) vildehayeGetUint32(buffer, 1);

    switch (cmd) {
    case LOGGER_CMD_RESET:
      nvmWakeup();
      SysCtrlSystemReset();
      break;
    case LOGGER_CMD_ID:
      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      vildehayeAddUInt8(&p, flashId[0]);
      vildehayeAddUInt8(&p, flashId[1]);
      vildehayeAddUInt8(&p, flashId[2]);
      vildehayeAddUInt8(&p, LOGGER_MEMORY_TYPE_NOR); // revise to self reporting by nvm module
      vildehayeAddUInt32(&p, nvmSize);
      vildehayeAddUInt32(&p, NVM_SECTOR_SIZE);      // size of erase sectors
      vildehayeAddUInt32(&p, NVM_PAGE_SIZE);      // size of pages
      vildehayeAddUInt32(&p, loggerFreeFlashPointer); // revise size
      vildehayeAddUInt32(&p, CATALOG_VERSION);
        //buffer[1] = cmd;
        //buffer[0] = LOGGER_ACK;
        //buffer[2] = flashId[0];
        //buffer[3] = flashId[1];
        //buffer[4] = flashId[2];
      d->length = p.length;
      break;

    case LOGGER_CMD_CHIP_ERASE:
      t = Timestamp_get32();
      nvmWakeup();
      ret = spiFlashEraseChip();
      //spiFlashPowerDown();
      while (spiFlashWriteInProgress() != 0);
      //ret = true;
      //Task_sleep(10000000/Clock_tickPeriod);
      t = Timestamp_get32() - t;

      loggerFreeFlashPointer = 0;
      loggerCurrentItemAddress = 0;
      //nextItemAddress = 0; // commented out Sivan Oct 2020

      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      vildehayeAddUInt32(&p, t);
      vildehayeAddUInt32(&p, timestampFreq);

      d->length = p.length;
      break;
#if 0
    case LOGGER_CMD_BUFFER_CLEAR:
      nvmBufferClear();
      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      d->length = p.length;
      break;

    case LOGGER_CMD_BUFFER_WRITE:
      addr = vildehayeGetUint32(buffer+1, 4); // here it's just an offset
      spiFlashBufferWrite(addr, len-5, buffer+5);

      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      d->length = p.length;
      break;

    case LOGGER_CMD_BUFFER_PRGRM:
      addr = vildehayeGetUint32(buffer+1, 4);
      t = Timestamp_get32();
      nvmWakeup();
      ret = nvmBufferProgram(addr);
      if (ret) while (spiFlashWriteInProgress() != 0);
      t = Timestamp_get32() - t;

      if (ret) {
        vildehayeAddUInt8(&p, LOGGER_ACK);
        vildehayeAddUInt8(&p, cmd);
        vildehayeAddUInt32(&p, t);
        vildehayeAddUInt32(&p, timestampFreq);
      } else {
        vildehayeAddUInt8(&p, LOGGER_NACK_SPI_FAILED);
        vildehayeAddUInt8(&p, cmd);
      }
      d->length = p.length;
      break;
#endif

    case LOGGER_CMD_FIND_FIRST:
      t = Timestamp_get32();
      nvmWakeup();
      addr = loggerFindFirstErasedSector();
      t = Timestamp_get32() - t;

      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      vildehayeAddUInt32(&p, addr);
      vildehayeAddUInt32(&p, t);
      vildehayeAddUInt32(&p, timestampFreq);
      d->length = p.length;
      break;

    case LOGGER_CMD_READ:
      addr = vildehayeGetUint32(buffer+1  , 4);
      l    = vildehayeGetUint32(buffer+1+4, 4);
      t = Timestamp_get32();
      //ret = spiFlashEraseChip();
      //if (ret) while (getWIP() != 0);
      nvmWakeup();
      // spiFlashRead(addr, l, (p.packet)+10); // skip the first 10 bytes
      nvmBufferedReadRange(addr, l, (p.packet)+10); // skip the first 10 bytes
      ret = true;
      //Task_sleep(10000000/Clock_tickPeriod);
      t = Timestamp_get32() - t;

      if (ret) {
        vildehayeAddUInt8(&p, LOGGER_ACK);
        vildehayeAddUInt8(&p, cmd);
        vildehayeAddUInt32(&p, addr);
        vildehayeAddUInt32(&p, l);
        p.length += l;
        //for (i=0; i<l; i++) {
        //    if (i>=128) break; // in case l is bugus
        //    vildehayeAddUInt8(&p, (uint8_t) i);
        //}
      } else {
        vildehayeAddUInt8(&p, LOGGER_NACK_SPI_FAILED);
        vildehayeAddUInt8(&p, cmd);
      }
      d->length = p.length;
      break;

    case LOGGER_CMD_PROGRAM:
        break;

    case LOGGER_CMD_LOG:
      addr = loggerFreeFlashPointer;
      type = (uint8_t) vildehayeGetUint32(buffer+1,1);
      rc = loggerLog(type, buffer+2, len-2);

      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      vildehayeAddUInt8(&p, rc);
      vildehayeAddUInt32(&p,addr);
      vildehayeAddUInt32(&p,loggerFreeFlashPointer);
      d->length = p.length;
      break;

    case LOGGER_CMD_COMMIT:
      addr = loggerFreeFlashPointer;
      rc = loggerCommit();

      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      vildehayeAddUInt8(&p, rc);
      vildehayeAddUInt32(&p,addr);
      vildehayeAddUInt32(&p,loggerFreeFlashPointer);
      d->length = p.length;
      break;

    case LOGGER_CMD_ITERATOR_INIT:
      loggerIteratorInit(&hostIterator);
      vildehayeAddUInt8(&p, LOGGER_ACK);
      vildehayeAddUInt8(&p, cmd);
      d->length = p.length;
      break;

    case LOGGER_CMD_ITERATOR_NEXT:
      rc = loggerIteratorNext(&ll, &type,
          buffer + 1+1+4+1,                       // skip: ack, cmd, returned address, type
          &hostIterator,&returnedItemAddress);
      if (rc==0) {
        vildehayeAddUInt8 (&p, LOGGER_ACK);
        vildehayeAddUInt8 (&p, cmd);
        vildehayeAddUInt32(&p, returnedItemAddress);
        vildehayeAddUInt8 (&p, type);
        d->length = p.length + ll;
      } else { // other possibilities?
        buffer[1] = buffer[0];
        buffer[0] = LOGGER_NACK_NO_MORE_ITEMS;
        d->length = 2;
      }
      break;

    default:
      buffer[1] = buffer[0];
      buffer[0] = LOGGER_NACK_UNKNOWN_CMD;
      d->length = 2;
      break;
    }
}
#endif

/****************************************************/
/* INITIALIZATION                                   */
/****************************************************/

buffer_descriptor d;
#define uart_printf(args...) Mailbox_pend(freeMailbox,&d,BIOS_WAIT_FOREVER);sprintf((char*)buffers[d.id],args);d.length=strlen((char*)buffers[d.id]);Mailbox_post(uartTxMailbox,&d,BIOS_WAIT_FOREVER);

static timestampInit(){
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);
    timestampFreq = freq.lo;
}

void loggerClose() {
  if (loggerState != LOGGER_STATE_UNKNOWN) {
    System_printf("Closing NVM\n");
    nvmBufferInvalidate(); // if it is stuck in write mode, it won't do single byte reads when another card is inserted
    nvmClose();
  }
}

extern uint8_t nvmBufferState();

void loggerInit() {

  System_printf("loggerInit 1 bufferState=%d\n",nvmBufferState());
  consoleFlush();

  timestampInit();

  //nvmInit(); // spiFlash_init();

  if (!nvmInit()) {
    // no flash or a flash that we did not expect
    System_printf("no flash\n");
    consoleFlush();
    loggerState = LOGGER_NO_FLASH;
    return;
  }

  loggerState = LOGGER_UNFORMATTED; // in case the host does not respond.

  System_printf("loggerInit reading header type\n");
  System_printf("  bufferState=%d\n",nvmBufferState());

  uint8_t buffer[36];
  System_printf("loggerInit reading header data\n");
  nvmBufferedReadRange(0, 36, &(buffer[0])); // offset of flash size in log header

  // verify header length & type
  if (buffer[0]==0xFF || buffer[1] != LOGGER_DATATYPE_LOG_HEADER) {
    System_printf("logger no header (found %02x %02x)\n",buffer[0],buffer[1]);
    nvmSleep();
    return;
  }

  // ok, we assume that we have a header
  loggerLogCreationTime  = vildehayeGetUint32(&(buffer[8]), 4);

  nvm_t headerSectorSize = vildehayeGetUint64(&(buffer[20]), 4);
  nvm_t headerPageSize   = vildehayeGetUint64(&(buffer[24]), 4);
  nvm_t headerNvmSize    = vildehayeGetUint64(&(buffer[28]), 8);

  System_printf("logger hdr sect %d page %d size %d nvmSize %d\n",headerSectorSize,headerPageSize,headerNvmSize, nvmSize);
  consoleFlush();

  if (nvmSize == 0) { // if the flash should not determine its size, get it from the header
    nvmSize = headerNvmSize;
    System_printf("logger log header size %d 32-bit flash size %d\n",buffer[0],(int) nvmSize);
  } else {
    if (nvmSize != headerNvmSize) {
      System_printf("logger inconsistent size (header %d device %d)\n",headerNvmSize, nvmSize);
      nvmSleep();
      return;
    }
  }

  if (headerSectorSize != NVM_SECTOR_SIZE) {
    System_printf("logger inconsistent sector size (header %d device %d)\n",headerSectorSize, NVM_SECTOR_SIZE);
    nvmSleep();
    return;
  }

  if (headerPageSize != NVM_PAGE_SIZE) {
    System_printf("logger inconsistent sector size (header %d device %d)\n",headerSectorSize, NVM_SECTOR_SIZE);
    nvmSleep();
    return;
  }

#if OLD
  uint8_t b1,b2;

  b1 = nvmBufferedReadByte(0); // header length
  b2 = nvmBufferedReadByte(1); // header type
  if (b1==0xFF || b2 != LOGGER_DATATYPE_LOG_HEADER) {
    System_printf("logger no header (found %02x %02x)\n",b1,b2);
    nvmSleep();
    return;
  }

  if (nvmSize == 0) { // if the flash should not determine its size, get it from the header
    // Sivan Aug 2019: should really be 36
    uint8_t buffer[40];
    System_printf("loggerInit reading header data\n");
    nvmBufferedReadRange(0, 40, &(buffer[0])); // offset of flash size in log header

    //uint32_t xxx;
    //for (xxx=0; xxx<40; xxx++) System_printf("%02x ",buffer[xxx]);
    //System_printf("\n");
    //xxx = vildehayeGetUint32(buffer, 4);
    //System_printf("logger log header size %d 32-bit page size %d\n",b1,xxx);

    //nvmBufferedReadRange(20, 8, &(buffer[0]));
    //xxx = vildehayeGetUint32(buffer, 4);
    //System_printf("logger log header size %d 32-bit sector size %d\n",b1,(int) xxx);

    loggerLogCreationTime = vildehayeGetUint32(&(buffer[8]), 4);
    //nvmBufferedReadRange(28, 8, &(buffer[0])); // offset of flash size in log header
    nvmSize = vildehayeGetUint64(&(buffer[28]), 8);
    System_printf("logger log header size %d 32-bit flash size %d\n",b1,(int) nvmSize);
  }
#endif

  initFreeFlashPointer();

  loggerState = LOGGER_LOGGING; // in case the host does not respond.

    // we cannot log anything yet (boot marker), because the client may still be doing some reading.

  nvmSleep();

#ifdef UART_HANDSHAKE
      // found flash, now see if a host wants to talk to us via UART
      uartTasks_init(HOST_UART);

      buffer_descriptor d;

#ifdef CC1310_LAUNCHPAD
      leds_on(LEDS_RX);
#endif

      System_printf("loggerInit wait for host\n");
      int i;
      for (i=0; i<10; i++) {
          Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
          buffers[ d.id ][0] = LOGGER_KEEPALIVE;
          d.length = 1;
          Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

          if (Mailbox_pend(uartRxMailbox, &d, 1000000/Clock_tickPeriod)) {
              loggerState = LOGGER_UART_CTRL;
              break;
          }
      }
      System_printf("loggerInit wait for host ended\n");

//#ifdef CC1310_LAUNCHPAD
      leds_off(LEDS_RX);
//#endif

      if (loggerState == LOGGER_UART_CTRL) {
          System_printf("loggerInit UART control\n");

          leds_setBlinkCounter(255);
          leds_blink(LEDS_RX,4);

          while (1) {
              loggerParseAndExecute(&d);

              Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

              Mailbox_pend(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
          }
      }

      System_printf("loggerInit no UART control\n");

#endif

      //if (loggerFreeFlashPointer!=0) {
      //    // if the flash is formatted and has a header, then ...
      //    loggerLog(LOGGER_DATATYPE_LOG_BOOT_MARKER, 0, 0);
      //}

}

#ifdef USE_FLASHCONTROLLER
static void flashcontrollerFunction(UArg arg0, UArg arg1) {

  timestampInit();

#if 0
  SPI_init();
  spiFlash_init();
  spiFlashRDID(flashId, sizeof(flashId));

  System_printf("loggerInit RDID %0x2 %02x %02x\n",flashId[0], flashId[1], flashId[2]);

  if (flashId[0]==0xC2 && flashId[1]==0x20 && flashId[2]==0x1A) nvmSize = 1 << 0x1A;
  if (flashId[0]==0xC2 && flashId[1]==0x28 && flashId[2]==0x14) nvmSize = 1 << 0x14;

  if (nvmSize==0) {
    spiFlashUseInternalFlash();

    spiFlashRDID(flashId, sizeof(flashId));

    System_printf("loggerInit RDID %0x2 %02x %02x (2nd attempt)\n",flashId[0], flashId[1], flashId[2]);

    if (flashId[0]==0xC2 && flashId[1]==0x20 && flashId[2]==0x1A) nvmSize = 1 << 0x1A;
    if (flashId[0]==0xC2 && flashId[1]==0x28 && flashId[2]==0x14) nvmSize = 1 << 0x14;
  }
#endif


  // note that nvmInit is called inside conditional
  if (nvmInit() && nvmSize!=0) {
    loggerState = LOGGER_LOGGING; // in case the host does not respond.
    System_printf("flashcontroller  flashSize %d flashSize %08x\n",nvmSize,nvmSize);

    initFreeFlashPointer();

    leds_slow(LEDS_TX);

    //nvmSleep();
  } else {
      System_printf("flashcontroller no flash\n");
      // no flash or a flash that we did not expect, start tag task and return.
      loggerState = LOGGER_NO_FLASH;
      //leds_on(LEDS_TX);
      //while (1);

      leds_fast(LEDS_TX);
 }

  // found flash, now see if a host wants to talk to us via UART
  //uartTasks_init(Board_UART,BUFFER_COUNT);
  //uartTasks_init(HOST_UART,115200, 1, 0 /* framing=SLIP */);
  uartTasks_init(HOST_UART, 115200, 1, FRAMING_SLIP);

  buffer_descriptor d;

  System_printf("loggerInit wait for host\n");
  while (true) {
    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
    buffers[ d.id ][0] = LOGGER_KEEPALIVE;
    d.length = 1;
    Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

    if (Mailbox_pend(uartRxMailbox, &d, 1000000/Clock_tickPeriod)) {
      loggerState = LOGGER_UART_CTRL;
      break;
    }
  }
  System_printf("loggerInit wait for host ended\n");

//#ifdef CC1310_LAUNCHPAD
  leds_off(LEDS_TX);
//#endif

  System_printf("loggerInit UART control\n");

  //leds_setBlinkCounter(255);
  //leds_blink(LEDS_RX,4);

  leds_off(LEDS_TX);

  while (1) {
    loggerParseAndExecute(&d);

    Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

    Mailbox_pend(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
  }
}

#define FLASHCONTROLLER_TASK_STACK_SIZE 1024

static Task_Params flashcontrollerTaskParams;
Task_Struct flashcontrollerTask;    /* not static so you can see in ROV */
static uint8_t flashcontrollerTaskStack[FLASHCONTROLLER_TASK_STACK_SIZE];

void flashcontrollerTask_init() {

    Task_Params_init(&flashcontrollerTaskParams);
    flashcontrollerTaskParams.stackSize = FLASHCONTROLLER_TASK_STACK_SIZE;
    flashcontrollerTaskParams.priority  = 5;
    flashcontrollerTaskParams.stack     = &flashcontrollerTaskStack;
    flashcontrollerTaskParams.arg0       = (UInt)1000000;

    Task_construct(&flashcontrollerTask, flashcontrollerFunction, &flashcontrollerTaskParams, NULL);
}

#endif // flashcontroller

#endif // VH_LOGGER
