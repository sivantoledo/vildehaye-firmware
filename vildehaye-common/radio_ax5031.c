/*
 * AX5031 PSK transmitter
 *
 * Original code by Andrey Leshchenko.
 * Modified for Vildehaye tags by Sivan Toledo
 */

#include "config.h"

#ifdef USE_AX5031

/* For usleep */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <ti/sysbios/BIOS.h>
/* Driver Header files */
//#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>

#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
//#include <ti/sysbios/knl/Semaphore.h>
//#include <ti/sysbios/knl/Clock.h>

//#define SPI_CS_PIN PIN_ID(11)

#include "radio.h"

/* Board Header file */


static SPI_Handle handle;

static PIN_Handle pinHandle = NULL;
static PIN_State pinState;
static PIN_Config AX5031PinTable[] = {
    AX5031_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Ext. flash chip select */
    PIN_TERMINATE
};

//static uint32_t csPin;

//static SPI_Handle      spiHandle;

#define AX_REG_REVISION           0x00
#define AX_REG_SCRATCH            0x01
#define AX_REG_PWRMODE            0x02
#define AX_REG_XTALOSC            0x03
#define AX_REG_FIFOCTRL           0x04
#define AX_REG_FIFODATA           0x05
#define AX_REG_IRQMASK            0x06
#define AX_REG_IRQREQUEST         0x07
#define AX_REG_PINCFG1            0x0C
#define AX_REG_PINCFG2            0x0D
#define AX_REG_PINCFG3            0x0E
#define AX_REG_IRQINVERSION       0x0F
#define AX_REG_MODULATION         0x10
#define AX_REG_ENCODING           0x11
#define AX_REG_FRAMING            0x12
#define AX_REG_CRCINIT3           0x14
#define AX_REG_CRCINIT2           0x15
#define AX_REG_CRCINIT1           0x16
#define AX_REG_CRCINIT0           0x17
#define AX_REG_VREG               0x1B

#define AX_REG_FREQB3             0x1C
#define AX_REG_FREQB2             0x1D
#define AX_REG_FREQB1             0x1E
#define AX_REG_FREQB0             0x1F
#define AX_REG_FREQ3              0x20
#define AX_REG_FREQ2              0x21
#define AX_REG_FREQ1              0x22
#define AX_REG_FREQ0              0x23
#define AX_REG_FSKDEV2            0x25
#define AX_REG_FSKDEV1            0x26
#define AX_REG_FSKDEV0            0x27
#define AX_REG_PLLLOOP            0x2C
#define AX_REG_PLLRANGING         0x2D

#define AX_REG_TXPWR              0x30
#define AX_REG_TXRATEHIGH         0x31
#define AX_REG_TXRATEMID          0x32
#define AX_REG_TXRATELOW          0x33
#define AX_REG_MODMISC            0x34

#define AX_REG_FIFOCOUNT          0x35
#define AX_REG_FIFOTHRESH         0x36
#define AX_REG_FIFOCONTROL2       0x37

#define AX_REG_XTALCAP            0x4F
#define AX_REG_FOURFSK            0x50

#define AX_PWRMODE_RST        0b10000000
#define AX_PWRMODE_POWERDOWN  0b00000000
#define AX_PWRMODE_VREGON     0b00000100
#define AX_PWRMODE_STANDBY    0b01100101
#define AX_PWRMODE_SYNTHTX    0b01101100
#define AX_PWRMODE_FULLTX     0b01101101

// flags for the address byte (not for the 16-bit word)
#define AX_WRITE_FLAG            0b10000000
#define AX_READ_FLAG             0b00000000

//int fifo_underflows = 0;
//int fifo_overflows = 0;

//uint16_t axTXBuffer[64];
//uint16_t axRXBuffer[64];

//uint8_t transmitted_message[9000 / 8];
//uint8_t debugbuff[9000 / 8];
//size_t transmitted_message_length;

//#include "key_srsha1_972002000555.h"

//void handle_spi_error()
//{
//  while (1);
//}

//
// Code for single transactions. Too slow for 1 Mbps transmission.
//

struct setups_st {
  uint32_t freq;
  uint8_t  modulation_type;
  uint32_t symbolrate;
  uint32_t deviation;
  uint32_t rxbw;
  int8_t   dbm;
} setups[MAX_RADIO_SETUPS];

void radioSetup_init() {}

void radioSetup_frequency(uint32_t index, uint32_t f) {
  setups[index].freq            = f;
}
void radioSetup_modulation(uint32_t index,
                           uint8_t modulation_type,
                           uint32_t symbolrate,
                           uint32_t deviation,
                           uint32_t rxbw) {
  setups[index].modulation_type = modulation_type;
  setups[index].symbolrate      = symbolrate;
  setups[index].deviation       = deviation;
  setups[index].rxbw            = rxbw;
}
void radioSetup_txPower(uint32_t index, int8_t dbm) {
  setups[index].dbm             = dbm;

}
void radioSetup_packetFormat(uint32_t index, int8_t packetFormat) {
}

#if 0
static uint16_t doTransaction_ax5031(uint16_t tx) {
  SPI_Transaction transaction;
  uint16_t rx;

  transaction.count = 1;
  transaction.rxBuf = &rx;
  transaction.txBuf = &tx;

  if (!SPI_transfer(handle, &transaction))
  {
    handle_spi_error();
  }

  return rx;
}
#endif

static uint8_t ax5031ReadRegister(uint8_t address) {
  SPI_Transaction transaction;
  uint8_t rx[2];
  uint8_t tx[2];

  tx[0] = address;
  tx[1] = 0;

  transaction.count = 2;
  transaction.rxBuf = &rx;
  transaction.txBuf = &tx;

  PIN_setOutputValue(pinHandle,AX5031_CS,0);
  bool success = SPI_transfer(handle, &transaction);
  PIN_setOutputValue(pinHandle,AX5031_CS,1);
  if (!success) {
    System_printf("failed to read register %02x\n",address);
    return 0;
  }

  //System_printf("rd %02x %02x %02x\n",tx[0],rx[0],rx[1]);
  return rx[1];
}

static void ax5031WriteRegister(uint8_t address, uint8_t value) {
  SPI_Transaction transaction;
  uint8_t rx[2];
  uint8_t tx[2];

  tx[0] = address | AX_WRITE_FLAG;
  tx[1] = value;

  transaction.count = 2;
  transaction.rxBuf = &rx;
  transaction.txBuf = &tx;

  PIN_setOutputValue(pinHandle,AX5031_CS,0);
  bool success = SPI_transfer(handle, &transaction);
  PIN_setOutputValue(pinHandle,AX5031_CS,1);
  if (!success) {
    System_printf("failed to write register %02x <- %02x\n",address, value);
  }

  //System_printf("wr %02x %02x %02x\n",tx[0],tx[1],rx[0]);
}

//
// Code for batch transmissions.
//
#if 0
uint16_t axCreateReadFrame(uint8_t address)
{
  return address << 8;
}


uint16_t axCreateWriteFrame(uint8_t address, uint8_t value)
{
  return (1 << 15) + (address << 8) + value;
}
#endif

#define AX_PLLLOOP_FLT_NOMINAL   0x01
#define AX_PLLLOOP_FLT_5X        0x10
#define AX_PLLLOOP_FLT_2X        0x11
#define AX_PLLLOOP_CPI_100KHZ    0b00001000
#define AX_PLLLOOP_CPI_50KHZ     0b00000100
#define AX_PLLLOOP_CPI_200KHZ    0b00001000
#define AX_PLLLOOP_CPI_500KHZ    0b00011100
#define AX_PLLLOOP_BANDSEL_433   0b00100000
#define AX_PLLLOOP_BANDSEL_HIGH  0b00000000
#define AX_PLLLOOP_FREQSEL_FREQ  0b10000000
#define AX_PLLLOOP_FREQSEL_FREQB 0b00000000
#define AX_PLLLOOP_RESERVED_MASK 0b01000000

#define AX_MODULATION_ASK        0
#define AX_MODULATION_ASK_SHAPED 2
#define AX_MODULATION_PSK        4
#define AX_MODULATION_PSK_SHAPED 5
#define AX_MODULATION_OQSK       6
#define AX_MODULATION_MSK        7
#define AX_MODULATION_FSK        (1<<6)

#define AX_FRAMING_RAW           0b000000
#define AX_FRAMING_RAW_SOFT      0b000001
#define AX_FRAMING_HDLC          0b000010
#define AX_FRAMING_RAW_PREAMBLE  0b000011
#define AX_FRAMING_802_15_4      0b000110
// crc is meaningful only in HDLC
#define AX_FRAMING_CRC_CCITT     0b000000
#define AX_FRAMING_CRC_16        0b001000
#define AX_FRAMING_CRC_32        0b010000
// also only in HDLC
#define AX_FRAMING_HSUPP         0b100000

#define AX_PLLRANGING_VCOR_MASK  0b00001111
#define AX_PLLRANGING_VCOR_DFLT  0b00001000
#define AX_PLLRANGING_RNGSTART   0b00010000
#define AX_PLLRANGING_RNGERR     0b00100000
#define AX_PLLRANGING_PLLLOCK    0b01000000
#define AX_PLLRANGING_STICKYLOCK 0b10000000

#define AX_TXPWR_RESERVED_MASK   0b11110000

#define AX_FIFOCOUNT_MASK        0b00111111
#define AX_FIFOCTRL_FIFO_EMPTY   0b00000100
#define AX_STATUS_UNDERFLOW      0b00001000
#define AX_STATUS_OVERFLOW       0b00000100

const int8_t txpwrTable[][2] = {
    0b0000, -51,
    0b0001, 0,
    0b0010, 5,
    0b0011, 9,    // 9.2
    0b0100, 11,
    0b0101, 12, // 12.3
    0b0110, 14,
    0b0111, 15,
    0b1000, 16,
    0b1001, 17,
    0b1010, 17,
    0b1011, 18,
    0b1100, 18,
    0b1101, 18,
    0b1110, 19,
    0b1111, 19,

};

#define AX_FIFO_SIZE 32
uint8_t txbuffer[ 2*AX_FIFO_SIZE ]; // 2-byte frames
uint8_t rxbuffer[ 2*AX_FIFO_SIZE ];

//unsigned char code[] = {
//0xe7, 0x96, 0xab, 0x5c, 0x2d, 0x1d, 0x2b, 0x7b, 0xd0, 0xfd, 0x67, 0x5f, 0x66, 0x7f, 0xe9, 0xb4, 0xd7, 0x6e, 0xfe, 0x5b, 0x21, 0xcc, 0x04, 0xc7, 0x16, 0x82, 0x4c, 0xcd, 0xa2, 0x5c, 0x1f, 0xca, 0xb0, 0xc1, 0xe1, 0xdb, 0x11, 0x94, 0x61, 0xf6, 0x2f, 0x27, 0x22, 0x54, 0x9b, 0xa2, 0x18, 0xe7, 0xfd, 0xa0, 0x29, 0x71, 0xd8, 0xe8, 0x31, 0x20, 0x2a, 0x92, 0xb7, 0x16, 0xe2, 0xef, 0x58, 0x83, 0x97, 0x59, 0x46, 0xed, 0x24, 0x61, 0x7b, 0xd0, 0x7c, 0x20, 0xba, 0xc5, 0x89, 0xa1, 0xfc, 0x51, 0xde, 0x11, 0x3c, 0x05, 0x59, 0x63, 0x57, 0xee, 0xf8, 0x00, 0xc1, 0x16, 0xce, 0x34, 0x84, 0x40, 0xd2, 0x30, 0xa0, 0xb7, 0x9e, 0x78, 0x68, 0xaf, 0xea, 0x6c, 0x19, 0xd4, 0x5d, 0x8c, 0x1b, 0xb1, 0x65, 0xcb, 0xf5, 0x72, 0xa4, 0x23, 0x14, 0x45, 0x7b, 0x62, 0x31, 0xff, 0x34, 0x0c, 0x41, 0x5f, 0xab, 0x7a, 0x05, 0xcd, 0x99, 0xbb, 0xfe, 0x94, 0xc8, 0x07, 0x12, 0x66, 0x43, 0xac, 0xdc, 0xed, 0xa0, 0x88, 0xd7, 0xc7, 0x71, 0x20, 0xdb, 0xe3, 0x6a, 0x09, 0x61, 0x98, 0x97, 0xdc, 0xd7, 0x43, 0x7c, 0x3e, 0x71, 0x24, 0x1c, 0xba, 0xc3, 0xc0, 0xf9, 0x8d, 0x61, 0x24, 0x52, 0xc3, 0x97, 0x8c, 0xd0, 0x16, 0x67, 0xf9, 0xd1, 0xfb, 0xf2, 0x50, 0x8d, 0x3d, 0xe0, 0xec, 0x94, 0xc3, 0x61, 0x56, 0xd9, 0x08, 0x67, 0xb7, 0x2a, 0x0a, 0x26, 0xed, 0x97, 0xe2, 0x0b, 0x3f, 0x76, 0x1e, 0xe3, 0x22, 0x20, 0xb7, 0x65, 0x80, 0x14, 0x10, 0xe5, 0x14, 0x42, 0xb1, 0x14, 0x24, 0xbf, 0x68, 0x51, 0xd8, 0x16, 0x0f, 0xfe, 0x88, 0xef, 0x84, 0x18, 0xa8, 0x12, 0x79, 0xe5, 0x1a, 0x5a, 0x20, 0xbf, 0x8e, 0x09, 0xf1, 0x73, 0x7d, 0x13, 0x20, 0x4f, 0x05, 0x59, 0xe9, 0x92, 0xf9, 0xed, 0x75, 0x92, 0xd6, 0x65, 0x3b, 0x67, 0x9e, 0x98, 0x4a, 0xd3, 0xbc, 0x18, 0x00, 0x87, 0x67, 0xaa, 0x67, 0xc6, 0xf9, 0xbc, 0xa0, 0xef, 0x7e, 0x67, 0xbc, 0x81, 0x0e, 0x2c, 0x1e, 0x33, 0x9e, 0xd6, 0x5c, 0x5a, 0x5c, 0xaf, 0x66, 0x8b, 0x00, 0x4b, 0xd0, 0x92, 0x51, 0x32, 0x4e, 0xf9, 0x9e, 0x54, 0x14, 0x77, 0xa4, 0x53, 0x9a, 0xe0, 0xdc, 0xd3, 0xe1, 0x35, 0x35, 0xa7, 0x73, 0x0b, 0xec, 0x80, 0x1f, 0xdc, 0x8b, 0xfd, 0x8d, 0x0a, 0x08, 0x76, 0x19, 0x06, 0x53, 0x98, 0x84, 0x96, 0x35, 0x4f, 0xd8, 0xf7, 0x1a, 0x24, 0x6e, 0xb3, 0xb3, 0x8c, 0xf4, 0xd5, 0xa1, 0xde, 0xfd, 0x82, 0xcd, 0xf9, 0x8a, 0x5f, 0x2b, 0x5e, 0x70, 0x2f, 0xee, 0xba, 0x1b, 0xdb, 0x4d, 0xb5, 0x8e, 0x53, 0x26, 0x2a, 0xd2, 0x76, 0x7e, 0x10, 0x35, 0x73, 0xcc, 0x14, 0x28, 0xc2, 0x9f, 0x30, 0x70, 0x0e, 0x2c, 0xe8, 0x54, 0x74, 0xf9, 0x86, 0xea, 0x9b, 0xbc, 0xa2, 0x3c, 0xe7, 0xe3, 0xd8, 0x76, 0x65, 0x4e, 0x3d, 0x3a, 0x51, 0x8f, 0x31, 0xba, 0x72, 0x41, 0x22, 0xea, 0xce, 0x29, 0x27, 0x2f, 0x5a, 0x26, 0x95, 0xba, 0xd4, 0x30, 0x6d, 0xb4, 0x4b, 0x75, 0x85, 0x8a, 0xb5, 0x36, 0x31, 0x74, 0x72, 0x18, 0x41, 0x3d, 0x4a, 0x16, 0x99, 0x14, 0x06, 0x7e, 0xfb, 0x9d, 0x72, 0x53, 0x21, 0x78, 0xd1, 0x2f, 0xb6, 0x9f, 0xdd, 0x0a, 0x5e, 0x3e, 0x1e, 0x01, 0xdc, 0x6a, 0x2c, 0x24, 0x13, 0xd7, 0x21, 0x23, 0xe2, 0x67, 0x2e, 0x43, 0xbd, 0x92, 0x03, 0xf0, 0xd6, 0x98, 0x7b, 0x2a, 0xa7, 0xf9, 0xa4, 0x7e, 0x7c, 0xff, 0x8d, 0x3f, 0x94, 0x09, 0x48, 0x6d, 0x20, 0xb7, 0x10, 0x11, 0x7b, 0x04, 0x61, 0x7a, 0x7a, 0x87, 0x01, 0x21, 0xfd, 0x6c, 0xe0, 0xa0, 0x80, 0xfb, 0x25, 0xea, 0x1b, 0xc5, 0xd6, 0x3a, 0x3c, 0x46, 0xa3, 0x29, 0x4b, 0x23, 0x75, 0xe5, 0x1f, 0x0f, 0x07, 0xcb, 0xb6, 0x5d, 0x99, 0xe2, 0xb1, 0xa8, 0x8e, 0xd4, 0x67, 0x7d, 0x29, 0x99, 0x2f, 0x47, 0x66, 0x82, 0x7a, 0x03, 0x88, 0x0b, 0x49, 0x7c, 0x22, 0x7b, 0x28, 0xb8, 0xbe, 0xbe, 0xb3, 0x9a, 0x18, 0xd5, 0xdd, 0xaa, 0x61, 0xc2, 0x62, 0xb1, 0xd0, 0x6f, 0xf8, 0x4a, 0x82, 0x07, 0x88, 0x51, 0xd2, 0xf1, 0x74, 0xf8, 0xdb, 0x90, 0xdd, 0x5a, 0x78, 0x89, 0x7c, 0xdf, 0x5b, 0xbf, 0x4c, 0x41, 0xab, 0xaf, 0x53, 0x6e, 0x8b, 0x79, 0xb2, 0xa2, 0xd6, 0x1d, 0x41, 0x6e, 0x90, 0xac, 0xd9, 0xf3, 0xd7, 0x91, 0x87, 0xe5, 0xc3, 0xc8, 0x38, 0xa4, 0x39, 0x64, 0x9a, 0x64, 0x61, 0xdf, 0x85, 0x78, 0x59, 0x17, 0xd7, 0x71, 0x22, 0x40, 0xb1, 0x11, 0x63, 0xc2, 0x39, 0xb3, 0x6c, 0x7f, 0xd8, 0xad, 0xc8, 0x2a, 0xbf, 0xa5, 0xc1, 0xf3, 0x89, 0xe6, 0x9d, 0xde, 0x75, 0xc5, 0x04, 0x5b, 0xbe, 0x4e, 0x9d, 0xe9, 0x11, 0xf6, 0xd9, 0xb5, 0x1b, 0xc2, 0x33, 0x6a, 0x1c, 0x6a, 0xd7, 0x7b, 0x25, 0x73, 0x72, 0x3f, 0x3f, 0xdb, 0x4a, 0x7d, 0x8d, 0x59, 0xa3, 0xce, 0xb8, 0x61, 0x7a, 0x59, 0x80, 0x65, 0x60, 0x95, 0x16, 0xfd, 0xed, 0x28, 0xa8, 0x6f, 0x15, 0x33, 0x76, 0x66, 0x3f, 0x85, 0x1e, 0x3f, 0x8b, 0x0e, 0x28, 0x5c, 0xaf, 0x7c, 0x5a, 0x6a, 0x19, 0x80, 0x0f, 0xc7, 0xa0, 0x91, 0xde, 0x5e, 0x9a, 0xba, 0x38, 0x86, 0x33, 0xcb, 0xd0, 0x75, 0xe6, 0x14, 0x2c, 0x17, 0xa7, 0x1b, 0x7b, 0x43, 0xdd, 0xe2, 0xe3, 0x26, 0x70, 0xb1, 0x85, 0x85, 0x20, 0xb0, 0x6f, 0x88, 0x70, 0xa6, 0x39, 0x25, 0x79, 0x81, 0x33, 0x4d, 0xbe, 0x1d, 0x28, 0x68, 0x03, 0x9a, 0x01, 0xc0, 0x24, 0x53, 0xc0, 0x75, 0xff, 0xb0, 0x5a, 0xc9, 0xd0, 0xcf, 0x8b, 0x65, 0x82, 0x65, 0x3a, 0x7e, 0x4c, 0x0a, 0xc2, 0xa9, 0x5b, 0x88, 0xba, 0x74, 0xc0, 0xf2, 0x9b, 0x40, 0x7b, 0xc3, 0xc6, 0xdc, 0x8a, 0xf7, 0xc6, 0x99, 0x6d, 0x60, 0x91, 0xb7, 0x1b, 0x4c, 0x98, 0x50, 0xe7, 0x0d, 0xf9, 0xac, 0xd9, 0x99, 0xb2, 0x3c, 0x00, 0x6f, 0x09, 0xd9, 0x2a, 0x38, 0x24, 0x09, 0x2c, 0x64, 0xfd, 0xf9, 0xd6, 0x0a, 0x3e, 0x65, 0x0e, 0x6f, 0xea, 0x1e, 0xbe, 0x49, 0xa4, 0x4c, 0x27, 0xe4, 0xa5, 0x38, 0xe8, 0x9c, 0xce, 0xaf, 0xdc, 0xf6, 0x97, 0xb7, 0xf1, 0x64, 0xa0, 0x60, 0x88, 0xba, 0x2f, 0x94, 0x7d, 0x75, 0x96, 0xc4, 0x35, 0xd1, 0xbc, 0x61, 0x4b, 0xaf, 0xe7, 0xb7, 0xa5, 0x2c, 0x14, 0xf7, 0x1c, 0x8e, 0x41, 0xed, 0x7b, 0xcf, 0x98, 0x74, 0x16, 0x8f, 0xc3, 0x4c, 0x38, 0x3b, 0x52, 0x39, 0x59, 0xad, 0xe1, 0x10, 0xc4, 0x99, 0x38, 0xdc, 0xc8, 0x00, 0x02, 0xe4, 0x88, 0xc7, 0x0e, 0x3f, 0x70, 0x1e, 0x2e, 0xc9, 0x52, 0x65, 0x32, 0x7f, 0x2c, 0x40, 0xd6, 0x1e, 0x01, 0xed, 0xb9, 0xda, 0x82, 0xef, 0xf2, 0x90, 0x5a, 0xfe, 0xa9, 0xac, 0xde, 0x50, 0x4e, 0x6b, 0x4b, 0xc2, 0x74, 0xbc, 0x4e, 0x0c, 0x86, 0xf3, 0xe4, 0x9d, 0x4d, 0xa0, 0x28, 0x24, 0x62, 0xac, 0xec, 0x2a, 0x79, 0xc3, 0x52, 0x56, 0x56, 0xa4, 0xf4, 0x36, 0xa1, 0x13, 0xc8, 0xcc, 0x52, 0x72, 0xbf, 0x30, 0x82, 0x28, 0x5a, 0x11, 0x77, 0xeb, 0xa5, 0x40, 0x91, 0xdd, 0xae, 0xec, 0x17, 0xd6, 0x41, 0xe4, 0x09, 0x51, 0x46, 0xf1, 0xf0, 0x08, 0x77, 0x5d, 0x97, 0x46, 0x43, 0xfa, 0x39, 0x02, 0xc8, 0x9a, 0xab, 0x89, 0x29, 0xb3, 0xce, 0x16, 0x00, 0xb9, 0xfe, 0x7c, 0x04, 0x96, 0xdf, 0xbe, 0x8f, 0x49, 0x6e, 0xa8, 0xf4, 0x99, 0x1a, 0xe8, 0x2f, 0x22, 0x59, 0xa0, 0xde, 0xa3, 0x0d, 0xfa, 0x32, 0x4a,
//};

//extern int  radioTransmit(uint8_t* data, size_t length);
int  radioReceiveMessage(uint32_t howLongUs) { return RADIO_SUCCESS; }
int  radioReceiveStart() { return RADIO_SUCCESS; }
void radioReceiveStop() {}

int radioTransmit(uint8_t* data, int len, int scheduleType, uint32_t timestamp, int32_t offset) {
  System_printf("AX5031: tx start\n");
  //data = code;
  //const uint8_t fifocount_address = 0x35;
  //const uint8_t fifocount_mask = 0x3F;

  //const uint8_t fifoctrl_address = 0x4;
  //const uint8_t fifoctrl_fifo_under_mask = 1 << 4;
  //const uint8_t fifoctrl_fifo_over_mask = 1 << 5;
  //const uint16_t fifoctrl_real_instr = axCreateReadFrame(fifoctrl_address);
  //const uint16_t fifocount_real_instr = axCreateReadFrame(fifocount_address);

  //const uint8_t fifodata_address = 0x5;
  //const uint8_t fifodata_write_instr = (1 << 7) + fifodata_address;

  //const size_t fifo_size = 32;

  SPI_Transaction transaction;
  transaction.txBuf = txbuffer;
  transaction.rxBuf = rxbuffer;
  //uint8_t *txBufferBytes = (uint8_t*)axTXBuffer;

  int i;
  uint8_t fifocount;

  ax5031ReadRegister(AX_REG_FIFOCTRL); // Clear the error bits
  fifocount = ax5031ReadRegister(AX_REG_FIFOCOUNT) & AX_FIFOCOUNT_MASK;
  //System_printf("AX5031: fifocount %d\n",fifocount);

  // initial filling of the FIFO

  int p = 0; // pointer to txbuffer
  int q = 0; // pointer to data
  transaction.count = 0;
  for (i=0; i<(AX_FIFO_SIZE - fifocount) && q<len; i++) {
    txbuffer[p++] = 0x85; // (AX_REG_FIFODATA | AX_WRITE_FLAG);
    (transaction.count)++;
    txbuffer[p++] = data[q++];
    (transaction.count)++;
  }
  PIN_setOutputValue(pinHandle,AX5031_CS,0);
  SPI_transfer(handle, &transaction);
  PIN_setOutputValue(pinHandle,AX5031_CS,1);

  //fifocount = ax5031ReadRegister(AX_REG_FIFOCOUNT) & AX_FIFOCOUNT_MASK;
  //System_printf("AX5031: fifocount %d txcount %d q %d len %d\n",fifocount,transaction.count,q,len);

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_FULLTX);

  //fifocount = ax5031ReadRegister(AX_REG_FIFOCOUNT) & AX_FIFOCOUNT_MASK;
  //System_printf("AX5031: fifocount %d\n",fifocount);

  uint8_t fifomin = 255;
  while (q < len) {
    do {
      fifocount=(ax5031ReadRegister(AX_REG_FIFOCOUNT) & AX_FIFOCOUNT_MASK);
      if (fifocount<fifomin) fifomin = fifocount;
      //System_printf(". %d q %d\n",fifocount,q); // wait for half empty
    } while (fifocount > AX_FIFO_SIZE / 2);
    p = 0; // pointer to txbuffer
    transaction.count = 0;
    for (i=0; i<(AX_FIFO_SIZE - fifocount) && q<len; i++) {
      txbuffer[p++] = 0x85; // (AX_REG_FIFODATA | AX_WRITE_FLAG);(AX_REG_FIFODATA | AX_WRITE_FLAG);
      (transaction.count)++;
      txbuffer[p++] = data[q++];
      (transaction.count)++;
    }
    PIN_setOutputValue(pinHandle,AX5031_CS,0);
    SPI_transfer(handle, &transaction);
    PIN_setOutputValue(pinHandle,AX5031_CS,1);
    // check the last received status
    //uint8_t status = rxbuffer[ transaction.count - 2 ];
    //if ((status & AX_STATUS_UNDERFLOW) != 0 || (status & AX_STATUS_OVERFLOW) != 0) {
    //  System_printf("AX5031: under or overflow status %02x\n",status);
    //  ax5031Shutdown();
    //  return;
    //}
  }

  uint8_t fifoctrl;
  do {
    fifoctrl = ax5031ReadRegister(AX_REG_FIFOCTRL);
  } while ((fifoctrl & AX_FIFOCTRL_FIFO_EMPTY) == 0);
  System_printf("AX5031: tx done fifomin=%d len=%d\n",fifomin,len);

  return RADIO_SUCCESS;
}

int radioPrepare(uint8_t setupIndex, int bitInversion, int tx) {
  uint8_t current;

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_STANDBY);
  //System_printf("AX5031: pwrmode %02x (should be at standby)\n",ax5031ReadRegister(AX_REG_PWRMODE));

  ax5031WriteRegister(AX_REG_XTALOSC, 0b0010); // transconductance; default is 2
  ax5031WriteRegister(AX_REG_XTALCAP, 6); // 12pF; value in pF is register value + 2



  // 1. Configure FLT and PLLCPI to recommended settings
  // Set BANDSEL to select 433 MHz
  // Set FREQSEL to 1

  //uint8_t pllloop_address = 0x2C;
  current = ax5031ReadRegister(AX_REG_PLLLOOP);
  //pllloop = (pllloop & (1 << 6)) + 0b00101001;
  // recommended PLL loop settings (for 433, all modulations and bitrates).
  ax5031WriteRegister(AX_REG_PLLLOOP,
                        (current & AX_PLLLOOP_RESERVED_MASK) |
                        AX_PLLLOOP_FLT_NOMINAL |
                        //AX_PLLLOOP_CPI_100KHZ  |
                        AX_PLLLOOP_CPI_50KHZ  |
                        ((setups[setupIndex].freq < 500000000) ? AX_PLLLOOP_BANDSEL_433 : AX_PLLLOOP_BANDSEL_HIGH) |
                        AX_PLLLOOP_FREQSEL_FREQ );
  //System_printf("AX5031: pllloop %02x\n",ax5031ReadRegister(AX_REG_PLLLOOP));

  // 2. Set carrier frequency

  {
    double freqCarrier = setups[setupIndex].freq;
    double freqOscillator = AX5031_XTAL_FREQ;

    uint32_t transmitterFreq = (uint32_t)((freqCarrier / freqOscillator) * (1 << 24) + 0.5);
    transmitterFreq |= 1; // From programming manual. Something to do with tonal behavior.

    ax5031WriteRegister(AX_REG_FREQ3, (transmitterFreq >> 24) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQ2, (transmitterFreq >> 16) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQ1, (transmitterFreq >>  8) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQ0,  transmitterFreq        & 0xFF);

    ax5031WriteRegister(AX_REG_FREQB3, (transmitterFreq >> 24) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQB2, (transmitterFreq >> 16) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQB1, (transmitterFreq >>  8) & 0xFF);
    ax5031WriteRegister(AX_REG_FREQB0,  transmitterFreq        & 0xFF);
    System_printf("AX5031: tx freq %d\n",transmitterFreq);
  }

  // 3. Set TXPWR according to the desired output
  // TODO: Add code for setting the power

  {
    uint8_t txpwr = ax5031ReadRegister(AX_REG_TXPWR);
    //System_printf("AX5031: txpwr %02x\n",txpwr);
    //ax5031WriteRegister(AX_REG_TXPWR, (txpwr & AX_TXPWR_RESERVED_MASK) | 0b0100 /* about 10dBm */);
    ax5031WriteRegister(AX_REG_TXPWR, (txpwr & AX_TXPWR_RESERVED_MASK) | 0b1111 /* about 14.5dBm */);
    System_printf("AX5031: txpwr %02x\n",ax5031ReadRegister(AX_REG_TXPWR));
  }
  // 4. Fsk Deviation, skipped here

  // 5. Set the bit-rate

  {
    double bitrate = setups[setupIndex].symbolrate;
    double freqOscillator = AX5031_XTAL_FREQ;

    uint32_t transmitterBitrate = (uint32_t)((bitrate / freqOscillator) * (1 << 24) + 0.5);

    ax5031WriteRegister(AX_REG_TXRATEHIGH, (transmitterBitrate >> 16) & 0xFF);
    ax5031WriteRegister(AX_REG_TXRATEMID,  (transmitterBitrate >> 8 ) & 0xFF);
    ax5031WriteRegister(AX_REG_TXRATELOW,   transmitterBitrate        & 0xFF);
    System_printf("AX5031: tx rate %d\n",transmitterBitrate);
  }

  // 6. Set modulation type

  {
    //const uint8_t psk_modulation_non_shaped = 0b100;
    //const uint8_t psk_modulation_shaped = 0b101;
    //uint8_t modulation_address = 0x10;
    //uint8_t modulation = ax5031ReadRegister(AX_REG_MODULATION); // now sure why Andrey read this
    //ax5031WriteRegister(modulation_address, (modulation & (1 << 7)) + psk_modulation_non_shaped);
    uint8_t modulation;
    if (setups[setupIndex].modulation_type == RADIO_MODULATION_PSK) modulation = AX_MODULATION_PSK; // Sivan Feb 2020 ??? PSK_SHAPED ?
    if (setups[setupIndex].modulation_type == RADIO_MODULATION_FSK) modulation = AX_MODULATION_FSK;
    ax5031WriteRegister(AX_REG_MODULATION,modulation);
    System_printf("AX5031: modulation %02x\n",ax5031ReadRegister(AX_REG_MODULATION));
  }

  if (setups[setupIndex].modulation_type == RADIO_MODULATION_FSK) {
    double deviation = setups[setupIndex].deviation;
    double freqOscillator = AX5031_XTAL_FREQ;

    uint32_t transmitterDeviation = (uint32_t)((deviation / freqOscillator) * (1 << 24) + 0.5);

    ax5031WriteRegister(AX_REG_FSKDEV2, (transmitterDeviation >> 16) & 0xFF);
    ax5031WriteRegister(AX_REG_FSKDEV0,  (transmitterDeviation >> 8 ) & 0xFF);
    ax5031WriteRegister(AX_REG_FSKDEV1,   transmitterDeviation        & 0xFF);
    System_printf("AX5031: tx dev %d\n",transmitterDeviation);
  }
  // 7. Set encoding type to raw

  //{
  //const uint8_t encoding_address = 0x11;
  //uint8_t encoding = ax5031ReadRegister(AX_REG_ENCODING); // Sivan I don't think we need to read this register
  //ax5031WriteRegister(AX_REG_ENCODING, encoding & (~0xFF));
  ax5031WriteRegister(AX_REG_ENCODING, 0 /* change nothing */);
  System_printf("AX5031: encoding %02x\n",ax5031ReadRegister(AX_REG_ENCODING));
  //}

  // 8. Frame mode

  // Leave default frame mode
  ax5031WriteRegister(AX_REG_FRAMING, AX_FRAMING_RAW);
  //System_printf("AX5031: framing %02x\n",ax5031ReadRegister(AX_REG_FRAMING));

  // turn on the synthesizer
  // shouldn't we wait for the oscillator?

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_SYNTHTX);
  //System_printf("AX5031: p wrmode %02x (should be at synthtx)\n",ax5031ReadRegister(AX_REG_PWRMODE));

  // Perform VCO Auto-Ranging

  //uint8_t pllranging_address = 0x2D;
  uint8_t pllranging = ax5031ReadRegister(AX_REG_PLLRANGING);
  //System_printf("AX5031: initial PLL ranging is %02x\n",pllranging);

  //ax5031WriteRegister(AX_REG_PLLRANGING, pllranging | (1 << 4));
  ax5031WriteRegister(AX_REG_PLLRANGING, AX_PLLRANGING_RNGSTART | AX_PLLRANGING_VCOR_DFLT); // we start at VcOR=8 (recommended, also reset value)

  while ((ax5031ReadRegister(AX_REG_PLLRANGING) & AX_PLLRANGING_RNGSTART) != 0) {
    //System_printf("AX5031: PLL ranging still going on\n");
  }

  pllranging = ax5031ReadRegister(AX_REG_PLLRANGING);

  if ((pllranging & AX_PLLRANGING_RNGERR) != 0) {
    radioShutdown();
    System_printf("AX5031: PLL Ranging failed, value = %2x\n",pllranging);
    return RADIO_SYNTHESIZER_FAILURE;
  }

  System_printf("AX5031: Ready to transmit\n");

  return RADIO_SUCCESS;
}

void radioShutdown() {
  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);
}

#if 0

// If turn_pwrmode_fulltx is true - the transmitter will be turned on by the funciton
// after pre-filling the FIFO with data. This will avoid fifo underflows on start.
// The current implementation may read up to 31 bytes after the end of the data.
// This happens because we avoid any if statements inside the loop to be quick enough.
void axTransmit(uint8_t *data, size_t length, int turn_pwrmode_fulltx)
{
  const uint8_t fifocount_address = 0x35;
  const uint8_t fifocount_mask = 0x3F;

  const uint8_t fifoctrl_address = 0x4;
  const uint8_t fifoctrl_fifo_under_mask = 1 << 4;
  const uint8_t fifoctrl_fifo_over_mask = 1 << 5;
  const uint16_t fifoctrl_real_instr = axCreateReadFrame(fifoctrl_address);
  const uint16_t fifocount_real_instr = axCreateReadFrame(fifocount_address);

  const uint8_t fifodata_address = 0x5;
  const uint8_t fifodata_write_instr = (1 << 7) + fifodata_address;

  const size_t fifo_size = 32;

  SPI_Transaction transaction;
  transaction.txBuf = axTXBuffer;
  transaction.rxBuf = axRXBuffer;
  uint8_t *txBufferBytes = (uint8_t*)axTXBuffer;

  uint8_t fifocount;
  int sent = 0;
  int shifted_overflows = 0;
  int shifted_underflows = 0;
  int failures = 0;

  ax5031ReadRegister(AX_REG_FIFOCTRL); // Clear the underflow bit
  fifocount = ax5031ReadRegister(AX_REG_FIFOCOUNT) & fifocount_mask;

  // We will place the command to begin transmitting at the end
  // of the expected transaction.
  if (turn_pwrmode_fulltx)
  {
    int message_bytes_sent = fifo_size - fifocount;
    axTXBuffer[message_bytes_sent + 2] = axCreateWriteFrame(AX_REG_PWRMODE, AX_PWRMODE_FULLTX);
    turn_pwrmode_fulltx = 1;
  }

  // for testing
  int repeatRounds = 1;

  while (repeatRounds--)
  {
    sent = 0;
    while (sent < length)
    {
      int message_bytes_sent = fifo_size - fifocount;
      int transaction_size = message_bytes_sent + 2 + turn_pwrmode_fulltx;
      int i;

      // We increase the transaction size by 1, and now we can zero the variable.
      turn_pwrmode_fulltx = 0;

      for (i = 0; i < message_bytes_sent; i++)
      {
        txBufferBytes[(i << 1)] = data[sent + i];
        txBufferBytes[(i << 1) + 1] = fifodata_write_instr; // Assume little endianness
      }

      sent += message_bytes_sent;

      axTXBuffer[message_bytes_sent] = fifoctrl_real_instr;
      axTXBuffer[message_bytes_sent + 1] = fifocount_real_instr;

      transaction.count = transaction_size;

      failures += !SPI_transfer(handle, &transaction);
      failures += transaction_size - transaction.count;

      fifocount = axRXBuffer[message_bytes_sent + 1] & fifocount_mask;
      shifted_underflows += axRXBuffer[message_bytes_sent] & fifoctrl_fifo_under_mask;
      shifted_overflows += axRXBuffer[message_bytes_sent] & fifoctrl_fifo_over_mask;
    }
  }

  if (failures)
  {
    handle_spi_error();
  }

  fifo_underflows += shifted_underflows / fifoctrl_fifo_under_mask;
  fifo_overflows += shifted_overflows / fifoctrl_fifo_over_mask;
}

void startupAndTransmit(SPI_Handle handle)
{
  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_STANDBY);

  // 1. Configure FLT and PLLCPI to recommended settings
  // Set BANDSEL to select 433 MHz
  // Set FREQSEL to 1

  {
    uint8_t pllloop_address = 0x2C;
    uint8_t pllloop = ax5031ReadRegister(pllloop_address);
    pllloop = (pllloop & (1 << 6)) + 0b00101001;
    ax5031WriteRegister(pllloop_address, pllloop);
  }

  // 2. Set carrier frequency

  {
    double freqCarrier = 433.92;
    double freqOscillator = 16;

    uint32_t transmitterFreq = (uint32_t)((freqCarrier / freqOscillator) * (1 << 24) + 0.5);
    transmitterFreq |= 1; // From programming manual. Something to do with tonal behavior.

    ax5031WriteRegister(0x23, transmitterFreq & 0xFF);
    ax5031WriteRegister(0x22, (transmitterFreq >> 8) & 0xFF);
    ax5031WriteRegister(0x21, (transmitterFreq >> 16) & 0xFF);
    ax5031WriteRegister(0x20, (transmitterFreq >> 24) & 0xFF);
  }

  // 3. Set TXPWR according to the desired output

  // TODO: Add code for setting the power

  // 4. Fsk Deviation, skipped here

  // 5. Set the bit-rate

  {
    double bitrate = 1e6;
    double freqOscillator = 16e6;

    uint32_t transmitterBitrate = (uint32_t)((bitrate / freqOscillator) * (1 << 24) + 0.5);

    ax5031WriteRegister(0x33, transmitterBitrate & 0xFF);
    ax5031WriteRegister(0x32, (transmitterBitrate >> 8) & 0xFF);
    ax5031WriteRegister(0x31, (transmitterBitrate >> 16) & 0xFF);
  }

  // 6. Set modulation type

  {
    const uint8_t psk_modulation_non_shaped = 0b100;
    //const uint8_t psk_modulation_shaped = 0b101;
    uint8_t modulation_address = 0x10;
    uint8_t modulation = ax5031ReadRegister(modulation_address);
    ax5031WriteRegister(modulation_address, (modulation & (1 << 7)) + psk_modulation_non_shaped);
  }

  // 7. Set encoding type to raw

  {
    const uint8_t encoding_address = 0x11;
    uint8_t encoding = ax5031ReadRegister(encoding_address);
    ax5031WriteRegister(encoding_address, encoding & (~0xFF));
  }

  // 8. Frame mode

  // Leave default frame mode

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_SYNTHTX);

  // Perform VCO Auto-Ranging

  uint8_t pllranging_address = 0x2D;
  uint8_t pllranging = ax5031ReadRegister(pllranging_address);
  ax5031WriteRegister(pllranging_address, pllranging | (1 << 4));

  while (ax5031ReadRegister(pllranging_address) & (1 << 4))
  {
    Task_sleep(10);
  }

  pllranging = ax5031ReadRegister(pllranging_address);

  if (pllranging & (1 << 5))
  {
    handle_spi_error();
  }

  // axWrite(AX_PWRMODE, AX_PWRMODE_FULLTX);
  const int turn_pwrmode_fulltx = 1;
  axTransmit(transmitted_message, transmitted_message_length, turn_pwrmode_fulltx);

  // Wait for the transmission to finish
  {
    uint8_t fifoctrl;

    do
    {
      fifoctrl = ax5031ReadRegister(0x4);
    } while (!(fifoctrl & (1 << 2)));
  }

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);

  // Put a breakpoint here to check for over/underflows
  fifo_underflows = 0;
  fifo_overflows = 0;
}
#endif

#if 0
void createMessage()
{
  const uint8_t preamble = 0xAA;
  const int preamble_length = 4;
  const uint8_t postamble = 0xAA;
  const int postamble_length = 2;

  const int code_length = sizeof(code);

  memset(transmitted_message, preamble, preamble_length);
  memcpy(transmitted_message + preamble_length, code, code_length);
  memset(transmitted_message + preamble_length + code_length, postamble, postamble_length);

  transmitted_message_length = preamble_length + code_length + postamble_length;
}
#endif

//Semaphore_Handle clockTickSemaphore;

//void clockTickCallback(xdc_UArg arg)
//{
//  Semaphore_post(clockTickSemaphore);
//}

int radioInit() {
  pinHandle = PIN_open(&pinState, AX5031PinTable);
  if (pinHandle==NULL) {
    System_printf("AX5031: Could not open CS pin\n");
    return RADIO_NO_RADIO;
  }

  SPI_init();

  SPI_Params params;
  //Clock_Handle clockHandle;
  //Clock_Params clockParams;

  SPI_Params_init(&params);
  params.transferMode = SPI_MODE_BLOCKING;
  params.transferTimeout = 1 * 1000;
  params.mode = SPI_MASTER;
  //params.bitRate = 4800000; // 4500 * 1000; // Beware: lower bitrates may cause FIFO underflows
  params.bitRate = 8000000; // 4500 * 1000; // Beware: lower bitrates may cause FIFO underflows
  params.dataSize = 8;
  params.frameFormat = SPI_POL0_PHA0;

  handle = SPI_open(SPI_AX5031_INDEX, &params);

  if (!handle) {
    System_printf("AX5031: Could not open SPI peripheral\n");
    return RADIO_NO_RADIO;
  }

  //createMessage();

  uint8_t value;
  value = ax5031ReadRegister(AX_REG_REVISION);
  System_printf("AX5031: revision %02x\n",value);
  if (value != 0x21) {
    System_printf("AX5031: wrong chip revision\n");
    return RADIO_NO_RADIO;
  }

  ax5031WriteRegister(AX_REG_SCRATCH, 0xAA);
  value = ax5031ReadRegister(AX_REG_SCRATCH);
  System_printf("AX5031: scratch==0xAA? %02x\n",value);
  if (value != 0xAA) {
    System_printf("AX5031: can't write to scratch\n");
    return RADIO_NO_RADIO;
  }

  ax5031WriteRegister(AX_REG_SCRATCH, 0x53);
  value = ax5031ReadRegister(AX_REG_SCRATCH);
  System_printf("AX5031: scratch==0x53? %02x\n",value);
  if (value != 0x53) {
    System_printf("AX5031: can't write to scratch\n");
    return RADIO_NO_RADIO;
  }

  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_RST);
  value = ax5031ReadRegister(AX_REG_PWRMODE);
  System_printf("AX5031: pwrmode %02x (should be at reset)\n",value);
  ax5031WriteRegister(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN);
  value = ax5031ReadRegister(AX_REG_PWRMODE);
  System_printf("AX5031: pwrmode %02x (should be at power down)\n",value);
  //clockTickSemaphore = Semaphore_create(0, NULL, NULL);

  //Clock_Params_init(&clockParams);
  //clockParams.period = 1000 * 1000 / Clock_tickPeriod;

  //clockHandle = Clock_create(clockTickCallback, 0, &clockParams, NULL);
  //Clock_start(clockHandle);

  //while (1)
  //{
  //  Semaphore_pend(clockTickSemaphore, BIOS_WAIT_FOREVER);
  //
  //  startupAndTransmit(handle);
  //}
  return RADIO_SUCCESS;
}


#endif // AX5031
