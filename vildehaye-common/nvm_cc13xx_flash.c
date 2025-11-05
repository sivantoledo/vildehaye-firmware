/*
 * Sivan Toledo 2019
 *
 * nvm implementation using the internal flash
 */

#include "config.h"

#ifdef USE_NVM_CC13XX_FLASH

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <xdc/runtime/System.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/devices/cc13x0/driverlib/rom.h>
#include <ti/devices/cc13x0/driverlib/vims.h>
#include <ti/devices/cc13x0/driverlib/flash.h>

#include "nvm.h"

#define PAGE_SIZE NVM_PAGE_SIZE

// these should be defined in the linker script
extern void __FLASH_LOG_SIZE;
extern void __FLASH_LOG_ADDR;

static nvm_t startAddress; //  = (nvm_t) (&__FLASH_LOG_ADDR);

/****************************************************/
/* LOW LEVEL UTILITIES                              */
/****************************************************/

void nvmSleep() {}

void nvmWakeup() {}

/****************************************************/
/* ERASE FUNCTIONS                                  */
/****************************************************/

/****************************************************/
/* WHOLE-PAGE WRITE                                 */
/****************************************************/

bool nvmPageWrite(nvm_t addr) {
  System_printf("nvmPageWrite %08x + %08x\n",addr,startAddress);

  VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true /* blocking */);
  VIMSLineBufDisable(VIMS_BASE);

  int i;
  uint32_t rc;

  for (i=0; i<PAGE_SIZE; i += 64) {
    CPUcpsid();

    rc = FlashProgram(nvmPageBuffer+i, (startAddress+addr+i), 64);

    CPUcpsie();

    if (rc != FAPI_STATUS_SUCCESS) break;
  }

  VIMSLineBufEnable(VIMS_BASE);
  VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true /* blocking */);

  if (rc != FAPI_STATUS_SUCCESS) {
    System_printf("FlashProgram error %d\n",rc);
    return false;
  }

  /*
  VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
  while (VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);

  int i;

  for (i=0; i<PAGE_SIZE; i += 8) {
    CPUcpsid();

    HapiProgramFlash(nvmPageBuffer+i, (startAddress+addr+i), 8);

    CPUcpsie();
  }


  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
*/
  return true;
}


/****************************************************/
/* BUFFERED READ                                    */
/****************************************************/

bool nvmPageRead(nvm_t addr){
  uint8_t* p = (uint8_t*) (startAddress+addr);

  int i;

  for (i=0; i<PAGE_SIZE; i++) {
    nvmPageBuffer[i] = p[i];
  }
  return true;
}



/****************************************************/
/* READ                                             */
/****************************************************/

bool nvmReadRange(nvm_t addr, uint32_t length, uint8_t* data){
  uint8_t* p = (uint8_t*) (startAddress+addr);

  int i;

  for (i=0; i<length; i++) {
    data[i] = p[i];
  }
  return true;
}

/****************************************************/
/* READ SINGLE BYTE                                 */
/****************************************************/

/*
 * In NOR flash we can read any byte or range; no need to read aligned pages.
 */

uint8_t nvmReadByte(nvm_t addr) {
  return *((uint8_t*) (startAddress+addr));
}


/****************************************************/
/* INITIALIZATION                                   */
/****************************************************/

//static uint8_t flashId[3];
//nvm_t nvmSize = 0;

bool nvmInit() {

  /*
   * We need to find the beginning of the log
   */

  System_printf("cc13xx nvm init\n");

  uint8_t* candidate = (uint8_t*) 0x1e000 - 4096; // 1e000 is the start of the configuration packet
  int found = 0;

  while (candidate != 0) {
    System_printf("candidate=%08x\n",candidate);
    if (candidate[0] == 34 && candidate[1]==8 /* log header */ && candidate[2]==0xAA && candidate[3]==0x55) {
      found = 1;
      break;
    }
    candidate -= 4096;
  }

  if (!found) {
    System_printf("log not found\n");
    nvmSize = 0;
    return false;
  }

  startAddress = (nvm_t) candidate;

  nvmSize = (nvm_t) (0x1e000 - startAddress);

  System_printf("flash log size = %d (at %08x)\n", nvmSize , startAddress);

  return true;
}

#endif // cc13xx flash

/****************************************************/
/* END OF FILE                                      */
/****************************************************/



