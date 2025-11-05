/*
 * SD Card driver, based on code from a project
 * by Daniel Kuperman and Sagi Aharoni at Tel-Aviv University
 *
 * Heavily modified by Sivan Toledo
 */

#include "config.h"

#ifdef USE_SPI_SDCARD

#include "string.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>

//#include "Board.h"
#include "leds.h"

#include "nvm.h"

#define BLOCK_SIZE NVM_PAGE_SIZE

//#define CS_PIN_SPI				Board_SPI_EXT_FLASH_CS

#define ACMD41_GIVEUP_SENDING		10
#define SMALL_DELAY				500
#define WRITE_OK					0x05
#define GIVEUP_WRITING		2000
#define GIVEUP_READING		2000
#define GIVEUP_INIT  			10

/* SPI interface */
static SPI_Handle spiHandle = NULL;
static SPI_Params spiParams;

// receive buffers, size for 0-8 0xFF bytes followed by a 1-5 byte response
//static uint8_t bufferFF[8+5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Used for transmitting only.
//static uint8_t receiveBuffer[8+5];

uint8_t nvmPageBuffer[NVM_PAGE_SIZE];

/**********************************************************************************/
/* SPI low level routines                                                         */
/**********************************************************************************/

/* CS Pin */
static PIN_Handle csPinHandle; //Chip select for SPI.
static PIN_State csPinState;

/*
 * Set defaultTxBufValue to 0xFFFF in SPI0 so that MOSI is set to 0xFF
 * when we receive with no transmit buffer (NULL)
 */

bool spiOpen(uint32_t bitRateSpeed) {
	System_printf("Starting SPI!\n");
	SPI_Params_init(&spiParams);
	spiParams.transferMode = SPI_MODE_BLOCKING;
	spiParams.mode = SPI_MASTER;
	spiParams.bitRate  = bitRateSpeed; //in Hz
	spiParams.dataSize = 8; /* dataSize can range from 4 to 8 bits */
	spiParams.mode = SPI_POL0_PHA0;
	spiHandle = SPI_open(SPI_SDCARD_INDEX, &spiParams);
	if (!spiHandle) System_printf("SPI did not open\n");
	else System_printf("SPI opened!\n");

	// Open SPI CS pin.
	PIN_Config csPinTable[] = { SDCARD_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL, PIN_TERMINATE };
	csPinHandle = PIN_open(&csPinState, csPinTable);
	if(!csPinHandle) System_abort("Error initializing board CS pin!\n");

	return spiHandle==NULL;
}

void spiClose(void) {
	SPI_close(spiHandle);
	PIN_close(csPinHandle);
}

static void csSelect(void) { //CS Low - device can communicate.
  PIN_setOutputValue(csPinHandle, SDCARD_CS, 0);
}
static void csDeselect(void) { //CS High - device can't communicate.
  PIN_setOutputValue(csPinHandle, SDCARD_CS, 1);
}

static bool spiReadWriteBuffer(const uint8_t *txBuf, uint8_t *rxBuf, size_t wlen, size_t rlen) {
	/* Initialize master SPI transaction structure */
	SPI_Transaction spiTransaction;
	spiTransaction.count = wlen + rlen;
	spiTransaction.txBuf = (void*) txBuf;
	spiTransaction.rxBuf = rxBuf;
	spiTransaction.arg   = NULL;

	/* Initiate SPI transfer */
	bool transferOK = SPI_transfer(spiHandle, &spiTransaction);
	if (!transferOK) System_printf("SPI transfer error\n");
	//System_printf("SPI transfer OK. Sent: %s Received: %s\n", spiTransaction.txBuf, spiTransaction.rxBuf);
	return transferOK;
}

static bool spiWrite(const uint8_t *txBuf, size_t wlen) {
	return spiReadWriteBuffer(txBuf, NULL, wlen, 0);
}

static bool spiRead(uint8_t *rxBuf, size_t rlen) {
	//if (rlen>sizeof(bufferFF)) System_abort("SPI cannot read more than 8 bytes at once.\n");
	//When reading a byte, 0xFF must also be transmitted.
  //int i;
  //for (i=0; i<rlen; i++) writeBuffer[i] = 0xFF;
  //return spiReadWriteBuffer(bufferFF, rxBuf, 0, rlen);
  return spiReadWriteBuffer(NULL, rxBuf, 0, rlen);
}

/**********************************************************************************/
/* Print utilities                                                                */
/**********************************************************************************/

void _printBuffer(uint8_t *buf, uint32_t len, bool ascii) {
	char *fmt = ascii ? "%c" : "0x%02x ";
	int i;
	for (i=0; i<len; i++) {
		if (i!=0 && i%8==0) System_printf("\n");
		System_printf(fmt, buf[i]);
		if (i%24==0) System_flush();
	}
	System_printf("\n");
}

void printBuffer(uint8_t *buf, uint32_t len) {
	_printBuffer(buf, len, false);
}

void print(uint8_t *fmt) {
	System_printf("%s",fmt);
	System_flush();
}

/**********************************************************************************/
/* COMMANDS                                                                       */
/**********************************************************************************/

const uint8_t cmd0[] ={ 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 };
// response type R1, no errors, in idle state
#define CMD0_OK           0x01
bool sendCMD0(void) {
	//CMD0 = Reset command. Puts the SD in SPI mode.
  //System_printf("sizeof cmd0 = %d\n",sizeof(cmd0));
  int i;
	csSelect();
	spiWrite(cmd0, sizeof(cmd0));
	spiRead(nvmPageBuffer, 8+1);
	csDeselect();

  //System_printf("CMD0: ");
  //for (i=0; i<(8+1); i++) System_printf("%02x ",buffer[i]);
  //System_printf("\n");

	for (i=0; i<(8+1); i++) if (nvmPageBuffer[i]==CMD0_OK) return true;
	return false;
}

//CMD8 = SEND_IF_COND, to turn off CRC
const uint8_t cmd8[] = {0x48, 0x00, 0x00, 0x01, 0xAA, 0x87}; // indicates voltage 2.7-3.6V, 0xAA check pattern
//#define CMD8_OK           0x01AA
bool sendCMD8(void) {
  int i;
	csSelect();
	spiWrite(cmd8, sizeof(cmd8));
	spiRead(nvmPageBuffer, 8+5);
	csDeselect();
	for (i=0; i<(8+1); i++) if (nvmPageBuffer[i]==CMD0_OK) break; // first byte of response, indicating idle state, no error
	if (nvmPageBuffer[i]!=CMD0_OK) return false;

  //System_printf("CMD8: ");
  //for (i=0; i<(8+5); i++) System_printf("%02x ",buffer[i]);
  //System_printf("\n");

  return (nvmPageBuffer[i+3]==cmd8[3] && nvmPageBuffer[i+4]==cmd8[4]); // card should echo voltage, check pattern; could check other bytes too
}

const uint8_t  cmd55[] = { 0x77, 0x00, 0x00, 0x00, 0x00, 0x01 }; // prefix for ACMD's
const uint8_t acmd41[] = { 0x69, 0x40, 0x10, 0x00, 0x00, 0x5F }; // initialize (for SD only, for MMC use CMD1)s
#define ACMD41_OK         0x00

bool sendACMD41(void) {
  //CMD 55: APP_CMD. Should be sent before ACMD41.

  int i, j;
  int notIdle;
  int idle;

  for (j=0; j<200; j++) {
    notIdle = idle = 0; // we don't know our state...
    csSelect();
    spiWrite(cmd55, sizeof(cmd55));
    spiRead(nvmPageBuffer, 8+1);

    spiWrite(acmd41, sizeof(acmd41));
    spiRead(nvmPageBuffer, 8+1);
    csDeselect();

    for (i=0; i<(8+1); i++) {
      if (nvmPageBuffer[i]==0x00) notIdle = 1;
      if (nvmPageBuffer[i]==0x01) idle    = 1;
    }

    if (idle) {
      Task_sleep(10000 / Clock_tickPeriod); // wait 10ms
      continue; // go on with loop
    }

    if (notIdle) {
      System_printf("ACMD41 not idle after %d cycles\n",j);
      return true;
    }

    System_printf("Missing no-error R1 response to ACMD41, aborting\n");
    System_printf("CMD41: ");
    for (i=0; i<(8+1); i++) System_printf("%02x ",nvmPageBuffer[i]);
    System_printf("\n");
    return false;
  }

  System_printf("Card is still in idle mode after %d ACMD41, aborting\n",j);
  return false; // nothing after too many attempts
}

const uint8_t  cmd58[] = { 0x7A, 0x00, 0x00, 0x00, 0x00, 0x01 };
uint8_t highCapacity = 0;

bool sendCMD58(void) {
  //CMD58 = READ_OCR
  int i;
	csSelect();
  spiWrite(cmd58, sizeof(cmd58));
  spiRead(nvmPageBuffer, 8+5);
  csDeselect();

  for (i=0; i<(8+1); i++) if (nvmPageBuffer[i]==0x00) break; // first byte of response, indicating idle state, no error
  if (nvmPageBuffer[i]!=0x00) return false;

  System_printf("CMD58: ");
  System_printf("%02x ",nvmPageBuffer[i+1]);
  System_printf("%02x ",nvmPageBuffer[i+2]);
  System_printf("%02x ",nvmPageBuffer[i+3]);
  System_printf("%02x ",nvmPageBuffer[i+4]);
  System_printf("\n");

  if (nvmPageBuffer[i+1] & 0x40) highCapacity = 1; // this implies 512-byte addressing
  else                           highCapacity = 0;

  System_printf("highCapacity=%d\n",highCapacity);


  return true;
}

const uint8_t cmd16[] = { 0x50, 0x00, 0x00, 0x02, 0x00, 0x01 }; // 32-bit block length; here set to 512 (mandatory for some cards)

void sendCMD16(void) {
    //CMD16 = SET_BLOCKLEN
	csSelect();
  spiWrite(cmd16, sizeof(cmd16));
  spiRead(nvmPageBuffer, 8+1);
	csDeselect();
}

/**********************************************************************************/
/* BLOCK READ AND WRINTE (INCLUDING CID, CSD, ETC)                                */
/**********************************************************************************/

static bool readUntil(uint8_t giveup, uint8_t value) {
  uint8_t byteBuffer;
  int j = 0;
  do {
    spiRead(&byteBuffer, 1);
    j++;
    if (j>=GIVEUP_READING) return false;
  } while (byteBuffer!=value);
  return true;
}

const uint8_t  cmd10[] = { 0x4A, 0x00, 0x00, 0x00, 0x00, 0x01 };
void readCID(uint8_t* respBuffer) {
    //CMD10 = SEND_CID
	csSelect();
  spiWrite(cmd10, sizeof(cmd10));

  // wait for the data token
	do {
		spiRead(nvmPageBuffer, 1);
	} while(nvmPageBuffer[0]!=0xFE);

	spiRead(nvmPageBuffer, 8);
	memcpy(respBuffer, nvmPageBuffer, 8);
	spiRead(nvmPageBuffer, 8);
	memcpy(respBuffer+8, nvmPageBuffer, 8);
	spiRead(nvmPageBuffer, 2); // must read CRC
	csDeselect();
}

#if 0
bool writeBlock(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t writeDataBuffer[BLOCK_SIZE]) {
    //CMD24 = write 1 block.
	csSelect();
	spiWrite((uint8_t[]) {0xFF, 0x58, addr1, addr2, addr3, addr4, 0x00, 0xFF}, 8); //Send CMD24, 32bit-address, null CRC.
	spiRead(nvmPageBuffer, 1);

	spiWrite((uint8_t[]) {0xFE}, 1); //Creating a data packet.
	int i;
	for (i=0;i<BLOCK_SIZE/8;i++) {
		spiWrite(writeDataBuffer+8*i, 8);
	}

	spiWrite((uint8_t[]) {0x00, 0x00}, 2); //2bit-CRC

	spiRead(nvmPageBuffer, 1);

	if ((nvmPageBuffer[0] & 0x0F)!=WRITE_OK) return false;

	//Busy check.
	int j = 0;
	uint8_t result = 0xFF;
	while(result != 0x00) {
		spiRead(nvmPageBuffer, 1);
		result = nvmPageBuffer[0];
		if (j>=GIVEUP_WRITING) return false;
		j++;
	}

	//Read until SD card is not busy (returning 0xFF).
	if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;

	csDeselect();
	return true;
}

bool readBlock(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t readDataBuffer[BLOCK_SIZE]) {
	//CMD17 = read 1 block.
	csSelect();
	spiWrite((uint8_t[]) {0xFF, 0x51, addr1, addr2, addr3, addr4, 0x00, 0xFF}, 8); //Send CMD17, 32bit-address, null CRC.
	spiRead(nvmPageBuffer, 1);

	int j = 0;
	do {
		spiRead(nvmPageBuffer, 1);
		j++;
		if (j>=GIVEUP_READING) return false;
	} while(nvmPageBuffer[0]!=0xFE);

	int i;
	for (i=0;i<BLOCK_SIZE/8;i++) {
		spiRead(nvmPageBuffer, 8);
		memcpy(readDataBuffer+8*i, nvmPageBuffer, 8);
	}

	spiRead(nvmPageBuffer, 2); //Read CRC (mandatory).
	spiWrite((uint8_t[]) {0xFF}, 1); //Found this necessary

	//Read until SD card is not busy (returning 0xFF).
	if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;

	csDeselect();
	return true;
}
#endif

bool nvmPageRead(uint64_t address) {
  //CMD17 = read 1 block.
  uint32_t blockAddress;
  if (highCapacity) blockAddress = address >> 9;
  else              blockAddress = address;

  System_printf("reading page at address %d highCapacity? %d\n",blockAddress, highCapacity);

  uint8_t cmd17[] = { 0x51,
                      (blockAddress>>24) & 0xFF,
                      (blockAddress>>16) & 0xFF,
                      (blockAddress>> 8) & 0xFF,
                      (blockAddress    ) & 0xFF,
                      0x00 };

  //System_printf("\nreadblocknew: ");
  //printBuffer(cmd17,sizeof(cmd17));
  //System_flush();

  csSelect();
  spiWrite(cmd17, sizeof(cmd17)); //Send CMD17, 32bit-address, null CRC.
  //spiRead(buffer, 1);

  // wait for data token
  int j = 0;
  do {
    spiRead(nvmPageBuffer, 1);
    j++;
    if (j>=GIVEUP_READING) return false;
  } while(nvmPageBuffer[0]!=0xFE);

  spiRead(nvmPageBuffer, 512);

  uint8_t crcBuffer[2];
  spiRead(crcBuffer, 2); //Read CRC (mandatory).
  uint8_t ffBuffer[] = { 0xFF };
  spiWrite(ffBuffer, 1); //Found this necessary; this sends 0xFF

  //Read until SD card is not busy (returning 0xFF).
  if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;
  csDeselect();
  return true;
}

bool nvmPageWrite(uint64_t address) {
  //CMD24 = write 1 block.
  uint32_t blockAddress;
  if (highCapacity) blockAddress = address >> 9;
  else              blockAddress = address;

  System_printf("writing page at address %d highCapacity? %d\n",blockAddress, highCapacity);

  uint8_t cmd24[] = { 0x58,
                      (blockAddress>>24) & 0xFF,
                      (blockAddress>>16) & 0xFF,
                      (blockAddress>> 8) & 0xFF,
                      (blockAddress    ) & 0xFF,
                      0x00 };


  csSelect();
  spiWrite(cmd24, sizeof(cmd24)); //Send CMD24, 32bit-address, null CRC.

  // wait for R1 okay
  if (!readUntil((uint8_t)GIVEUP_READING, 0x00)) return false;

  spiWrite(NULL, 1); // one spacer byte

  // now start the data packet
  uint8_t tokenBuffer[] = { 0xFE };
  spiWrite(tokenBuffer, 1); //Creating a data packet.
  spiWrite(nvmPageBuffer, 512);
  uint8_t crcBuffer[] = {0x00, 0x00};
  spiWrite(crcBuffer, sizeof(crcBuffer));

  uint8_t response;
  spiRead(&response, 1);

  if ((response & 0x0F)!=WRITE_OK) return false;

  // wait for okay response
  if (!readUntil((uint8_t)GIVEUP_READING, 0x00)) return false;
  //Read until SD card is not busy (returning 0xFF).
  if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;

  csDeselect();
  return true;
}

////////////////////////////////////////

/*
 * Sivan: Did not test multiple read, write
 */

//writes secNo blocks to address {addr1, addr2, addr3, addr4} (32 bit block address)
bool multipleWrite(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint32_t amount, uint8_t *writeDataBuffer) {
    //CMD25 = write multiple blocks.
    csSelect();
    spiWrite((uint8_t[]) {0xFF, 0x59, addr1, addr2, addr3, addr4, 0x00, 0xFF}, 8); //Send CMD25, 32bit-address, null CRC.
    spiRead(nvmPageBuffer, 2); //Read response byte + dummy byte

    int i;
    for (i=0; i<amount; i++) { //Write a packet in each iteration
        //CMD25 data token - packet's "header"
        spiWrite((uint8_t[]) {0xFC}, 1);

        //Write data block
        int j;
        for(j=0; j<BLOCK_SIZE/8; j++) {
            spiWrite(writeDataBuffer+8*j+BLOCK_SIZE*i, 8);
        }

        //Write 2byte-CRC - not checked but expected
        spiWrite((uint8_t[]) {0x00, 0x00}, 2);

        spiRead(nvmPageBuffer, 1);
        //Read response message
        if ((nvmPageBuffer[0] & 0x0F)!=WRITE_OK) return false;

        //Read until SD card is not busy (returning 0xFF).
        if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;
    }

    //Send stop transaction token
    spiWrite((uint8_t[]) {0xFD}, 1);

    //Read until SD card is not busy (returning 0xFF).
    if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;

    csDeselect();
    return true;
}

bool multipleRead(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint32_t amount, uint8_t *readDataBuffer) {
    //CMD18 = read multiple blocks.
    csSelect();
    spiWrite((uint8_t[]) {0xFF, 0x52, addr1, addr2, addr3, addr4, 0x00, 0xFF}, 8); //Send CMD18, 32bit-address, null CRC.
    spiRead(nvmPageBuffer, 1);

    if (!readUntil((uint8_t)GIVEUP_READING, 0xFE)) return false; //Return if 0xFE not returned.

    int i,j;
    for (i=0;i<amount;i++) {
        for (j=0;j<BLOCK_SIZE/8;j++) {
            spiRead(nvmPageBuffer, 8);
            memcpy(readDataBuffer+8*j+BLOCK_SIZE*i, nvmPageBuffer, 8);
        }
        if (!readUntil((uint8_t)GIVEUP_READING, 0xFE)) return false; //Return if 0xFE not returned.
    }

    spiWrite((uint8_t[]) {0xFF, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, 8); //Send CMD12 = stop reading.
    spiRead(nvmPageBuffer, 1);

    //Read until SD card is not busy (returning 0xFF).
    if (!readUntil((uint8_t)GIVEUP_READING, 0xFF)) return false;

    csDeselect();
    return true;
}

void printCID() {
	uint8_t cidBuffer[16];
	readCID(cidBuffer); //CID reads: 035344534c3038478002f449fc00e865.
	System_printf("\nCID is:\n");
	printBuffer(cidBuffer,16);
	System_printf("SD model # is: ");
	_printBuffer(cidBuffer+3,5,true);
	System_flush();

#if 0
	readBlockNew(0);
	System_printf("\nPAGE 0: ");
	printBuffer(nvmPageBuffer,16);
	System_flush();

  readBlockNew(1000000);
  System_printf("\nPAGE 1MB: ");
  printBuffer(nvmPageBuffer,16);
  System_flush();

  readBlockNew(10000000);
  System_printf("\nPAGE 1GB: ");
  printBuffer(nvmPageBuffer,16);
  System_flush();

  nvmPageBuffer[0] = 0xAA;
  nvmPageBuffer[1] = 0x55;
  writeBlockNew(10000000);
  readBlockNew(10000000);
  System_printf("\nPAGE 1GB: ");
  printBuffer(nvmPageBuffer,16);
  System_flush();
#endif
}

/**********************************************************************************/
/* UNBUFFERED BUFFERED READS (none here...)                                       */
/**********************************************************************************/

/*
 * Sivan July 2019: this is weird. This function was commented out but the code seemed to have worked.
 *
 * This should not be here because the buffered read can either fail or call this one. So no calls to the
 * buffered version.???
 */

uint8_t nvmReadByte(nvm_t addr) {
  return nvmBufferedReadByte(addr); // we cannot read single bytes from an SD card, only entire pages
}

/**********************************************************************************/
/* POWER CONTROL IS IMPLICIT, NO SLEEP/WAKEUP COMMANDS                            */
/**********************************************************************************/

void nvmSleep()  {}
void nvmWakeup() {}

/**********************************************************************************/
/* INITIALIZATION                                                                 */
/**********************************************************************************/

// defined in nvm.c
//nvm_t nvmSize = 0; // if it's zero, the logger should get it from the log header

bool nvmInit() {
  nvmSize = 0; // unknown

  SPI_init(); // this is in logger, need to decide where to put this later
  spiOpen(300000); //Open SPI. Speed is 200KHz (slow).

  System_printf("sdcard nvmInit\n");

  // Tell the card to go to SPI mode.
  // send > 74 clocks with CS high and DI=MOSI high.
  //Chip select should be high.
  csDeselect();
  int i;
  uint8_t ff = 0xFF;
  for (i=0; i<16; i++) spiWrite(&ff, 1);
  //csDeselect();

  //Begin initialization.
  bool result;
  for (i=0;i<GIVEUP_INIT;i++) {
    result = sendCMD0();
    if (result) break;
  }
  if (!result) {
    System_printf("Error in cmd0: Unknown card.\n");
    spiClose();
    return false;
  }

  //Check supported voltage.
  result = sendCMD8();
  if (!result) {
    System_printf("Error in sendCMD8: Voltage not supported.\n");
    spiClose();
    return false;
  }

  //Leave idle state
  result = sendACMD41();
  if (!result) {
    System_printf("Error in sendACMD41: SD card refuses leaving IDLE_STATE.\n");
    spiClose();
    return false;
  }

  //Read On Chip Register (OCR).
  sendCMD58();

  //Set block length.
  sendCMD16();

  //Reopen SPI in higher speed.
  spiClose();
  spiOpen(1000000);

  //nvmSize = (1024 * 1024*1024); // dummy for now...
  System_printf("sdcard nvmInit end\n");

  return true;
}

void nvmClose() {
  spiClose();
}

#endif // SPI_SDCARD

/**********************************************************************************/
/* END OF FILE                                                                    */
/**********************************************************************************/
