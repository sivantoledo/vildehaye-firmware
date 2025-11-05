/*
 * I2C interface to Vesper tags
 */

#include <config.h>

#ifdef USE_VESPER

#include "vesper.h"

#include <ti/drivers/I2C.h>
#include "i2c.h"

uint32_t loggerLogCreationTime = 3456;
nvm_t loggerCurrentItemAddress = 0;
nvm_t loggerFreeFlashPointer = 0; // pointer to the next item
nvm_t loggerQueueSize() {
  return 2000000;
}
// wrong, need to get it from Vesper
nvm_t nvmSize = 0;

#define VESPER_SLAVE_ADDRESS 0x14

//static I2C_Handle i2cHandle;
#define RX_LEN_LIMIT 1+4+1+1+224
static uint8_t buffer[RX_LEN_LIMIT];
#define TX_LEN_LIMIT 1+4
static uint8_t cmd[TX_LEN_LIMIT];

static uint8_t wakeup = 0xFF;

void loggerInit() {
  //i2cInit(); // now down in main.c
}

static nvm_t ackedAddress = 0xFFFF;

void loggerAck(nvm_t address) {
  ackedAddress = address;
}

uint32_t loggerNext(uint8_t* length, uint8_t* type,
                    uint8_t* data,
                    nvm_t*   returnedItemAddress) {
  //  *returnedItemAddress = 1024;
  //  *data = "    ";
  //  *length = 4;
  //  *type = 13;
  //  return LOGGER_SUCCESS;


  //buffer_descriptor_t temp;
  wakeup = 0xFF; // no wakeup
  *length = 0xFF; // an indication that there is no log item to send

  if (i2c == 0) {
    // no i2c connection
    return LOGGER_ERR_COMM;
  }

  I2C_Transaction i2cTransaction;
  i2cTransaction.slaveAddress = 0x14; // VESPER_SLAVE_ADDRESS;
  i2cTransaction.readBuf     = buffer;
  i2cTransaction.readCount   = RX_LEN_LIMIT;
  i2cTransaction.writeBuf    = cmd;

  if (ackedAddress == 0xFFFF) {
    cmd[0] = 1;
    i2cTransaction.writeCount = 1;
  } else {
    cmd[0] = 10;
    uint8_t* ackp = (uint8_t*) &ackedAddress;
    int i;
    for (i=0; i<4; i++) cmd[1+i] = ackp[i];
    i2cTransaction.writeCount = 5;
  }

  bool success = I2C_transfer(i2c, &i2cTransaction);

  if (success) {
    nvmSize = 1000000; // to indicate communication with vesper
    if (ackedAddress != 0xFFFF) ackedAddress = 0xFFFF; // vesper received our ack, clear address
    if (buffer[0] == 1) {
      uint8_t* ackp = (uint8_t*) returnedItemAddress;
      int i;
      for (i=0; i<4; i++) ackp[i] = buffer[1+i];
      *type = buffer[5];
      *length  = buffer[6];
      for (i=0; i<(*length); i++) data[i] = buffer[7+i];
      return LOGGER_SUCCESS;
    }
    if (buffer[0] == 2) { // ok but no data item
      return LOGGER_SUCCESS;
    }
    if (buffer[0] == 3) { // ok and a wakeup command
      *length = 0xFF; // an indication that there is no log item to send
      wakeup = buffer[1]; // no wakeup
      return LOGGER_SUCCESS;
    }
    // if we got here, we did not receive a valid respose from vesper
    return LOGGER_ERR_MISBEHAVIOR;
  } else {
    return LOGGER_ERR_COMM;
  }
}

/*
 * This is called periodically if the main tag loop
 * did not manage to send a log item (or if there is no
 * log item to send at all), so that communication with
 * vesper keeps going.
 */
uint32_t vesperKeepalive() {
  //  return LOGGER_SUCCESS;

  wakeup = 0xFF; // no wakeup

  if (i2c == 0) {
    // no i2c connection
    return LOGGER_ERR_COMM;
  }

  I2C_Transaction i2cTransaction;
  i2cTransaction.slaveAddress = 0x14; // VESPER_SLAVE_ADDRESS;
  i2cTransaction.readBuf     = buffer;
  i2cTransaction.readCount   = RX_LEN_LIMIT;
  i2cTransaction.writeBuf    = cmd;

  cmd[0] = 20;
  i2cTransaction.writeCount = 1;

  bool success = I2C_transfer(i2c, &i2cTransaction);

  if (success) {
    nvmSize = 1000000; // to indicate communication with vesper
    if (ackedAddress != 0xFFFF) ackedAddress = 0xFFFF; // vesper received our ack, clear address
    if (buffer[0] == 1) {
      return LOGGER_ERR_MISBEHAVIOR;
    }
    if (buffer[0] == 2) { // ok but no data item
      return LOGGER_SUCCESS;
    }
    if (buffer[0] == 3) { // ok and a wakeup command
      wakeup = buffer[1]; // no wakeup
      return LOGGER_SUCCESS;
    }
    // if we got here, we did not receive a valid respose from vesper
    return LOGGER_ERR_MISBEHAVIOR;
  } else {
    return LOGGER_ERR_COMM;
  }

}

uint8_t vesperWakeup() {
  uint8_t w = wakeup;
  wakeup = 0xFF;
  return w;
}

#endif



