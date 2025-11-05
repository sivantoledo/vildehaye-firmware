/*
 * console.c
 *
 *  Sivan Toledo 2019
 *
 *  This file supports console operations (System_printf etc).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <xdc/std.h>
//#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/drivers/UART.h>

#include "config.h"
#include "i2c.h"
#include "console.h"

#ifdef USE_CONSOLE
#ifdef DeviceFamily_CC13X2
#define CONSOLE_BUFFER_LENGTH      4096
#else
#define CONSOLE_BUFFER_LENGTH      256
#endif
static uint8_t  uartPrintf_outArray[CONSOLE_BUFFER_LENGTH];
static uint16_t uartPrintf_head = 0;
static uint16_t uartPrintf_tail = 0;
static UART_Handle hUart = NULL;

void consoleInit() {
#ifdef CONSOLE_I2C
#else
  UART_Params uartParams;
  UART_Params_init(&uartParams);
  uartParams.writeDataMode = UART_DATA_BINARY;
  //uartParams.writeReturnMode = UART_RETURN_FULL; // don't add a new line, we add it in slipEncode
  uartParams.writeTimeout = UART_WAIT_FOREVER;

  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.readTimeout = UART_WAIT_FOREVER;

  uartParams.baudRate = 115200;

  uartParams.writeMode = UART_MODE_BLOCKING;
  uartParams.readMode = UART_MODE_BLOCKING;

  hUart = UART_open(HOST_UART, &uartParams);
#endif
}

void consoleClose() {
    if (hUart!=NULL) UART_close(hUart);
}


void consolePutch(char ch) {
    // uartPrintf_tail should never catch up with uartPrintf_head. Discard in-between bytes.
  if ( (uartPrintf_head + 1) % CONSOLE_BUFFER_LENGTH == uartPrintf_tail )
    return;

  uartPrintf_outArray[uartPrintf_head] = ch;
  uartPrintf_head++;

  if (uartPrintf_head >= CONSOLE_BUFFER_LENGTH)
    uartPrintf_head = 0;
}

void consoleFlush() {
  // Abort in case UART hasn't been initialized.
  if (NULL == hUart)
    return;

  // Lock head position to avoid race conditions
  uint16_t curHead = uartPrintf_head;

  // Find out how much data must be output, and how to output it.
  bool needWrap = curHead < uartPrintf_tail;
  uint16_t outLen = needWrap?(CONSOLE_BUFFER_LENGTH-uartPrintf_tail+curHead):(curHead-uartPrintf_tail);

#ifdef CONSOLE_I2C
  if (outLen) {
    if (needWrap) {

      i2cTransaction.writeBuf   = (&uartPrintf_outArray[uartPrintf_tail]);
      i2cTransaction.writeCount = CONSOLE_BUFFER_LENGTH - uartPrintf_tail;
      i2cTransaction.readBuf  = NULL;
      i2cTransaction.readCount = 0;
      i2cTransaction.slaveAddress = 0x22;

      I2C_transfer(i2c, &i2cTransaction);

      //UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], CONSOLE_BUFFER_LENGTH - uartPrintf_tail);

      i2cTransaction.writeBuf   = uartPrintf_outArray;
      i2cTransaction.writeCount = curHead;
      i2cTransaction.readBuf  = NULL;
      i2cTransaction.readCount = 0;
      i2cTransaction.slaveAddress = 0x22;

      I2C_transfer(i2c, &i2cTransaction);
      //UART_write(hUart, uartPrintf_outArray, curHead);
    } else {
      i2cTransaction.writeBuf   = &uartPrintf_outArray[uartPrintf_tail];
      i2cTransaction.writeCount = outLen;
      i2cTransaction.readBuf  = NULL;
      i2cTransaction.readCount = 0;
      i2cTransaction.slaveAddress = 0x22;

      I2C_transfer(i2c, &i2cTransaction);
      //UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], outLen);
    }
  }
#else
  if (outLen) {
    if (needWrap) {
      UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], CONSOLE_BUFFER_LENGTH - uartPrintf_tail);
      UART_write(hUart, uartPrintf_outArray, curHead);
    } else {
      UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], outLen);
    }
  }
#endif

  uartPrintf_tail = curHead;
}
#else
void consoleInit()         {}
void consolePutch(char ch) {}
void consoleFlush()        {}
#endif




