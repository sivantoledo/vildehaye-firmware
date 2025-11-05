/*
 * nvm.c
 *
 * Sivan Toledo, August 2018
 *
 * Generic algorithms for buffered reads and writes.
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include <xdc/runtime/System.h>

#include "config.h"

#include "nvm.h"

#if defined(USE_NVM)

/**********************************************************************************/
/* COMMON GLOBAL                                                                  */
/**********************************************************************************/

nvm_t nvmSize = 0;

/**********************************************************************************/
/* A PAGE-SIZE BUFFER                                                             */
/**********************************************************************************/

/*
 * The buffer is used for reading pages, writing pages, and for some of the
 * initialization commands.
 */

uint8_t nvmPageBuffer[NVM_PAGE_SIZE];
typedef enum {
  INVALID = 0,
  WRITING = 1,
  READING = 2
} buffer_state_t;

static buffer_state_t bufferState = INVALID;

uint8_t nvmBufferState() { return (uint8_t) bufferState; }
/**********************************************************************************/
/* BUFFERED READS, WRITES                                                         */
/**********************************************************************************/

static nvm_t bufferAddress;

uint8_t nvmBufferedReadByte(nvm_t addr) {
  bool byteInBuffer = true; // speculation, will check.
  if (bufferState == WRITING) {
#ifdef NVM_UNBUFFERED_READS
    return nvmReadByte(addr);
#else
    System_printf("buffered read failed, BufferState==WRITING\n");
    return 0; // this is an error, should report...
#endif
  }
  if (bufferState == INVALID)                              byteInBuffer = false;
  if (byteInBuffer && bufferAddress > addr)                byteInBuffer = false;
  if (byteInBuffer && addr - bufferAddress >= NVM_PAGE_SIZE) byteInBuffer = false;

  if (!byteInBuffer) {
    nvm_t p = pageAddress(addr);
    //System_printf("sdcard read address %d\n",(int) p);
    nvmPageRead(p);
    bufferAddress = p;
    bufferState = READING;
  }

  //System_printf("sdcard buffer address %d\n",(int) bufferAddress);

  return nvmPageBuffer[addr - bufferAddress];
}

/* we can optimize but for now we focus on functionality.
 *
 */
bool nvmBufferedReadRange(nvm_t addr, uint32_t length, uint8_t* data) {
  uint32_t i;
  if (bufferState == WRITING) {
#ifdef NVM_UNBUFFERED_READS
    return nvmReadRange(addr,length,data);
#else
    return false; // an precondition, client must satisfy
#endif
  }
  for (i=0; i<length; i++)
    data[i] = nvmBufferedReadByte(addr + i);
  return true;
}

/**********************************************************************************/
/* BUFFERED WRITES                                                                */
/**********************************************************************************/

void nvmBufferInvalidate() {
  bufferState = INVALID;
}

void nvmBufferClear() { // not really necessary, client should always fill pages
  uint32_t i;
  for (i=0; i<sizeof(nvmPageBuffer); i++) nvmPageBuffer[i] = 0xFF;
  bufferState = WRITING;

  System_printf("***!!! bufferState=WRITING !!!***\n");
}

void nvmBufferWrite(uint32_t offset, uint32_t length, uint8_t* data) {
  uint32_t i;
  if (bufferState != WRITING) nvmBufferClear();
  for (i=0; i<length; i++) nvmPageBuffer[ offset+i ] = data[i];
}

bool nvmBufferProgram(nvm_t addr) {
  if (bufferState != WRITING) {
    System_printf("trying to write data but nvm not in WRITING state\n");
    return false;
  }
  bool result = nvmPageWrite(addr);
  bufferState = INVALID;
  return result;
}

#endif // LOGGER

/**********************************************************************************/
/* END OF FILE                                                                    */
/**********************************************************************************/


