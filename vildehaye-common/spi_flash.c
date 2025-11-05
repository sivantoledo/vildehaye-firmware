/*
 * Nir Zaidman and Sivan Toledo 2018
 */

#include "config.h"

#ifdef USE_SPI_FLASH

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <xdc/runtime/System.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

//#include "Board.h"
#include "nvm.h"

#define PAGE_SIZE NVM_PAGE_SIZE
//#define HEADER_SIZE 5

#define FLASH_PAGE_PROGRAM     0x02
#define FLASH_READ             0x03
#define FLASH_CMD_READ_4B      0x13
#define FLASH_PAGE_PROGRAM_4B  0x12

#define FLASH_SECTOR_ERASE     0x20
#define FLASH_SECTOR_ERASE_4B  0x21
// chip erase is also 0x60
#define FLASH_CHIP_ERASE       0xC7

#define FLASH_WRITE_ENABLE     0x06
#define FLASH_READ_STAT_REG    0x05
#define FLASH_READ_CONF_REG    0x15
#define FLASH_WRITE_STAT_REG   0x01

#define FLASH_DEEP_PWR_DOWN    0xB9
#define FLASH_RELEASE_PWRDN    0xAB

#define FLASH_ENTER_4B_MODE    0xB7
#define FLASH_EXIT_4B_MODE     0xE9

#define FLASH_READ_ID          0x9F

static PIN_Handle hFlashPin = NULL;
static PIN_State pinState;

static PIN_Config BoardFlashPinTable[] = {
    SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Ext. flash chip select */
    PIN_TERMINATE
};

static uint32_t csPin;

SPI_Handle      spiHandle;
SPI_Transaction spiTransaction;
uint8_t spiTxBuffer[1+4]; // space for command and a 4-byte address
//uint8_t spiTxBuffer[PAGE_SIZE+HEADER_SIZE];
//uint8_t spiRxBuffer[PAGE_SIZE+HEADER_SIZE];

/****************************************************/
/* LOW LEVEL UTILITIES                              */
/****************************************************/

// Write in progress?
int spiFlashWriteInProgress() {
  // read status regiter command
  uint8_t rxBuffer[2];
  spiTxBuffer[0] = FLASH_READ_STAT_REG;
  spiTransaction.count = 2;
  spiTransaction.txBuf = spiTxBuffer;
  spiTransaction.rxBuf = rxBuffer;
  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
  SPI_transfer(spiHandle, &spiTransaction);
  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

  return (rxBuffer[1] & 1);
}

void nvmSleep() {
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    spiTxBuffer[0] = FLASH_DEEP_PWR_DOWN;

    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    Task_sleep(1 + 10 / Clock_tickPeriod);
}

void nvmWakeup() {

    spiTxBuffer[0] = FLASH_RELEASE_PWRDN;

    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    /*
     * must keep CS high until the flash goes into standby mode,
     * maximum 30us (tRES1 for the MX66L51235F)
     */
    Task_sleep(1 + 30 / Clock_tickPeriod);
}

static uint8_t spiFlashAddressLength = 3;
static void spiFlashEnter4ByteMode() {
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // set 4 byte mode command
    spiTxBuffer[0] = FLASH_ENTER_4B_MODE;
    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    spiFlashAddressLength = 4;

    System_printf("spiFlash 4-byte addressing\n");
}

uint8_t flashId[3];

static void spiFlashRDID() {
  // read status register command
  spiTxBuffer[0] = FLASH_READ_ID;

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);

  spiTransaction.count = 1;
  spiTransaction.txBuf = spiTxBuffer;
  spiTransaction.rxBuf = NULL;
  SPI_transfer(spiHandle, &spiTransaction);

  spiTransaction.count = 3;
  spiTransaction.txBuf = NULL;
  spiTransaction.rxBuf = flashId;
  SPI_transfer(spiHandle, &spiTransaction);
  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
}

bool setWEN(){
    // read status regiter command
    uint8_t rxBuffer[2];

    spiTxBuffer[0] = FLASH_WRITE_ENABLE; // was 0x06
    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    // read status regiter command
    spiTxBuffer[0] = FLASH_READ_STAT_REG;
    spiTransaction.count = 2;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = rxBuffer;
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    ret &= SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    if (ret == false || (rxBuffer[1] & 2)== 0)
        return false;

    return true;
}

/****************************************************/
/* ERASE FUNCTIONS                                  */
/****************************************************/

bool spiFlashEraseChip(){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // set write enable
    if (setWEN()){
        // write command
        spiTxBuffer[0] = FLASH_CHIP_ERASE;

        // write address
        spiTransaction.count = 1;
        spiTransaction.txBuf = spiTxBuffer;
        spiTransaction.rxBuf = NULL;

        PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
        bool ret = SPI_transfer(spiHandle, &spiTransaction);
        PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
        return ret;
    }
    return false;
}

// apparently not used.
bool spiFlashReset(){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // reset enable
    spiTxBuffer[0] = 0x66;
    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    if (ret == false)
        return false;

    Task_sleep(1000 / Clock_tickPeriod);

    // reset memory
    spiTxBuffer[0] = 0x99;
    spiTransaction.count = 1;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    return ret;
}

/****************************************************/
/* WHOLE-PAGE WRITE                                 */
/****************************************************/
#if 0
bool spiFlashWritePage(uint32_t page, uint8_t* data){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // set write enable
    if (setWEN()){
        // write command
        spiTxBuffer[0] = 0x12;

        // write address
        page=page*PAGE_SIZE;
        spiTxBuffer[4] = page & 0xFF;
        spiTxBuffer[3] = (page & 0xFF00) >> 8;
        spiTxBuffer[2] = (page & 0xFF0000) >> 16;
        spiTxBuffer[1] = (page & 0xFF000000) >> 24;
        spiTransaction.count = PAGE_SIZE+HEADER_SIZE;
        memcpy(spiTxBuffer + HEADER_SIZE, data, PAGE_SIZE);
        PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
        bool ret = SPI_transfer(spiHandle, &spiTransaction);
        PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
        return ret;
    }
    return false;
}
#endif

bool nvmPageWrite(nvm_t addr) {
  //System_printf("spiFlash nvmPageWrite %08x: ",addr);
  //int i;
  //for (i=0; i<10; i++) System_printf("%02x ",nvmPageBuffer[i]);
  //System_printf("...\n");

  // wait until previous action is done
  while (spiFlashWriteInProgress() != 0);

  // set write enable
  if (setWEN()){
    // write command
    spiTxBuffer[0] = 0x12;

    // write address
    //spiTxBuffer[4] = address & 0xFF;
    //spiTxBuffer[3] = (address & 0xFF00) >> 8;
    //spiTxBuffer[2] = (address & 0xFF0000) >> 16;
    //spiTxBuffer[1] = (address & 0xFF000000) >> 24;

    if (spiFlashAddressLength==4) {
        spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
        spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[3] = (addr & 0xFF00) >> 8;
        spiTxBuffer[4] = addr & 0xFF;
    } else {
        spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[2] = (addr & 0xFF00) >> 8;
        spiTxBuffer[3] = addr & 0xFF;
    }

    //memcpy(spiTxBuffer + HEADER_SIZE, data, PAGE_SIZE);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);

    //spiTransaction.count = HEADER_SIZE;
    spiTransaction.count = 1 + spiFlashAddressLength;
    spiTransaction.txBuf = spiTxBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(spiHandle, &spiTransaction);

    spiTransaction.count = PAGE_SIZE;
    spiTransaction.txBuf = nvmPageBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(spiHandle, &spiTransaction);

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

    //spiTransaction.txBuf = spiTxBuffer;
    //spiTransaction.rxBuf = spiRxBuffer;

    return true;
  }
  return false;
}

#if 0
/****************************************************/
/* BUFFERED WRITE                                   */
/****************************************************/

void spiFlashBufferClear() {
  memset(spiTxBuffer + 1 + spiFlashAddressLength, 0xff, PAGE_SIZE);
}

void spiFlashBufferWrite(uint32_t offset, uint32_t length, uint8_t* data){
  memcpy(spiTxBuffer + 1 + spiFlashAddressLength+ offset, data, length);
}

bool spiFlashBufferProgram(uint32_t addr){
    // wait until previous action is done

  //leds_on(LEDS_TX);
  while (spiFlashWriteInProgress() != 0);
  //leds_off(LEDS_TX);

  // set write enable
    if (!setWEN()) return false;

    // write command
    spiTxBuffer[0] = FLASH_PAGE_PROGRAM;

    // read address
    if (spiFlashAddressLength==4) {
        spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
        spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[3] = (addr & 0xFF00) >> 8;
        spiTxBuffer[4] = addr & 0xFF;
    } else {
        spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[2] = (addr & 0xFF00) >> 8;
        spiTxBuffer[3] = addr & 0xFF;
    }
    spiTransaction.count = 1 + spiFlashAddressLength + 256;

    //leds_on(LEDS_TX);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
    //leds_off(LEDS_TX);
    //bool ret = 0;
    return ret;
}
#endif

/****************************************************/
/* BUFFERED READ                                    */
/****************************************************/

#if 0
bool spiFlashBufferRead(uint32_t addr){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // command
    spiTxBuffer[0] = FLASH_READ;

    // read address
    if (spiFlashAddressLength==4) {
        spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
        spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[3] = (addr & 0xFF00) >> 8;
        spiTxBuffer[4] = addr & 0xFF;
    } else {
        spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[2] = (addr & 0xFF00) >> 8;
        spiTxBuffer[3] = addr & 0xFF;
    }
    spiTransaction.count = 1 + spiFlashAddressLength + 256;

    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
    return ret;
}
#endif

bool nvmPageRead(nvm_t addr){
  // wait until previous action is done
  while (spiFlashWriteInProgress() != 0);

  // command
  spiTxBuffer[0] = FLASH_READ;

  // read address
  if (spiFlashAddressLength==4) {
    spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
    spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[3] = (addr & 0xFF00) >> 8;
    spiTxBuffer[4] = addr & 0xFF;
  } else {
    spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[2] = (addr & 0xFF00) >> 8;
    spiTxBuffer[3] = addr & 0xFF;
  }

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);

  spiTransaction.count = 1 + spiFlashAddressLength;
  spiTransaction.txBuf = spiTxBuffer;
  spiTransaction.rxBuf = NULL;
  SPI_transfer(spiHandle, &spiTransaction);

  spiTransaction.count = PAGE_SIZE;
  spiTransaction.txBuf = NULL;
  spiTransaction.rxBuf = nvmPageBuffer;
  SPI_transfer(spiHandle, &spiTransaction);

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

  //System_printf("spiFlash nvmPageRead %08x: ",addr);
  //int i;
  //for (i=0; i<10; i++) System_printf("%02x ",nvmPageBuffer[i]);
  //System_printf("...\n");

  return true;
}


#if 0
uint8_t spiFlashBufferGetByte(uint32_t offset) {
   offset = offset + 1 + spiFlashAddressLength;
   return spiRxBuffer[ offset ];
}

void spiFlashBufferGetRange(uint32_t offset, uint32_t length, uint8_t* data) {
   offset = offset + 1 + spiFlashAddressLength;
   memcpy(data, spiRxBuffer + offset, length);
}
#endif

/****************************************************/
/* READ                                             */
/****************************************************/

bool nvmReadRange(nvm_t addr, uint32_t length, uint8_t* data){
  // wait until previous action is done
  while (spiFlashWriteInProgress() != 0);

  // command
  spiTxBuffer[0] = FLASH_READ;

  // read address
  if (spiFlashAddressLength==4) {
    spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
    spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[3] = (addr & 0xFF00) >> 8;
    spiTxBuffer[4] = addr & 0xFF;
  } else {
    spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[2] = (addr & 0xFF00) >> 8;
    spiTxBuffer[3] = addr & 0xFF;
  }

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);

  spiTransaction.count = 1 + spiFlashAddressLength;
  spiTransaction.txBuf = spiTxBuffer;
  spiTransaction.rxBuf = NULL;
  SPI_transfer(spiHandle, &spiTransaction);

  spiTransaction.count = length;
  spiTransaction.txBuf = NULL;
  spiTransaction.rxBuf = data;
  SPI_transfer(spiHandle, &spiTransaction);

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

  //System_printf("spiFlash nvmRead %08x: ",addr);
  //int i;
  //for (i=0; i<10; i++) System_printf("%02x ",nvmPageBuffer[i]);
  //System_printf("...\n");

  return true;
}

#if 0
bool spiFlashReadPage(uint32_t page, uint8_t* data){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);


    // read command
    spiTxBuffer[0] = 0x13;

    // read address
    page=page*PAGE_SIZE;
    spiTxBuffer[4] = page & 0xFF;
    spiTxBuffer[3] = (page & 0xFF00) >> 8;
    spiTxBuffer[2] = (page & 0xFF0000) >> 16;
    spiTxBuffer[1] = (page & 0xFF000000) >> 24;
    spiTransaction.count = PAGE_SIZE+HEADER_SIZE;
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
    memcpy(data, spiRxBuffer + HEADER_SIZE, PAGE_SIZE);
    return ret;

}
#endif

#if 0
// length must be less than 256 bytes
bool spiFlashRead(uint32_t addr, uint32_t length, uint8_t* data){
    // wait until previous action is done
    while (spiFlashWriteInProgress() != 0);

    // read command
    spiTxBuffer[0] = FLASH_READ;

    // read address
    if (spiFlashAddressLength==4) {
        spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
        spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[3] = (addr & 0xFF00) >> 8;
        spiTxBuffer[4] = addr & 0xFF;
    } else {
        spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
        spiTxBuffer[2] = (addr & 0xFF00) >> 8;
        spiTxBuffer[3] = addr & 0xFF;
      }
    spiTransaction.count = 1 + spiFlashAddressLength + length;
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);
    bool ret = SPI_transfer(spiHandle, &spiTransaction);
    PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);
    memcpy(data, spiRxBuffer + (1 + spiFlashAddressLength), length);
    return ret;
}
#endif

/****************************************************/
/* READ SINGLE BYTE                                 */
/****************************************************/

/*
 * In NOR flash we can read any byte or range; no need to read aligned pages.
 */

uint8_t nvmReadByte(nvm_t addr) {
  uint8_t b;

  // wait until previous action is done
  while (spiFlashWriteInProgress() != 0);

  // command
  spiTxBuffer[0] = FLASH_READ;

  // read address
  if (spiFlashAddressLength==4) {
    spiTxBuffer[1] = (addr & 0xFF000000) >> 24;
    spiTxBuffer[2] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[3] = (addr & 0xFF00) >> 8;
    spiTxBuffer[4] = addr & 0xFF;
  } else {
    spiTxBuffer[1] = (addr & 0xFF0000) >> 16;
    spiTxBuffer[2] = (addr & 0xFF00) >> 8;
    spiTxBuffer[3] = addr & 0xFF;
  }

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_ON);

  spiTransaction.count = 1 + spiFlashAddressLength;
  spiTransaction.txBuf = spiTxBuffer;
  spiTransaction.rxBuf = NULL;
  SPI_transfer(spiHandle, &spiTransaction);

  spiTransaction.count = 1;
  spiTransaction.txBuf = NULL;
  spiTransaction.rxBuf = &b;
  SPI_transfer(spiHandle, &spiTransaction);

  PIN_setOutputValue(hFlashPin,csPin,SPI_FLASH_CS_OFF);

  //System_printf("spiFlash nvmReadByte %08x: %02x\n",addr, b);

  return b;
}


/****************************************************/
/* INITIALIZATION                                   */
/****************************************************/

void spiFlashUseInternalFlash() {
  csPin = SPI_FLASH_CS;
}

//static uint8_t flashId[3];
//nvm_t nvmSize = 0;

bool nvmInit() {

  SPI_init();

  hFlashPin = PIN_open(&pinState, BoardFlashPinTable);

  csPin = SPI_FLASH_CS;

  System_printf("nvmInit csPin %d\n",csPin);


  SPI_Params      spiparams;
  SPI_Params_init(&spiparams);
  //spiparams.bitRate  = 40000;
  spiparams.bitRate  = 1000000; // Sivan July 2018
  spiparams.mode = SPI_MASTER;
  spiparams.transferMode = SPI_MODE_BLOCKING;
  spiparams.frameFormat = SPI_POL1_PHA1; // default
  spiparams.dataSize = 8; // bits, default
  spiHandle = SPI_open(SPI_FLASH_INDEX , &spiparams);

  //spiTransaction.txBuf = spiTxBuffer;
  //spiTransaction.rxBuf = spiRxBuffer;

  //spiFlashBufferClear(); // make sure first buffer is clear
  System_printf("waking up flash\n");

  nvmWakeup(); // in case we were left in that state from a previous reset

  System_printf("reading id\n");

  spiFlashRDID(); // SPI problem

  System_printf("loggerInit RDID %0x2 %02x %02x\n",flashId[0], flashId[1], flashId[2]);

  if (flashId[0]==0xC2 && flashId[1]==0x20 && flashId[2]==0x1A) nvmSize = 1 << 0x1A;
  if (flashId[0]==0xC2 && flashId[1]==0x28 && flashId[2]==0x14) nvmSize = 1 << 0x14;
  if (flashId[0]==0xC2 && flashId[1]==0x28 && flashId[2]==0x17) nvmSize = 1 << 0x17;

  if (nvmSize == 0) {
    System_printf("spiFlash could not recognize flash chip (or none connected!), no logging\n");
    nvmSleep(); // just in case it is there, at least it should be in deep power down
    return false;
  } else {
    System_printf("spiFlash size = %d\n",nvmSize);
  }

  if (nvmSize > (1 << 24)) spiFlashEnter4ByteMode();

  return true;
}

#endif // SPI_FLASH

/****************************************************/
/* END OF FILE                                      */
/****************************************************/



