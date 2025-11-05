/*
 * i2c.c
 *
 *  Created on: 7 ????? 2019
 *      Author: stoledo
 */

#include <stdint.h>
#include <stdbool.h>

#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/I2C.h>

#include "config.h"

#include "i2c.h"

I2C_Handle      i2c;
I2C_Transaction i2cTransaction;

void i2cInit() {
  I2C_init();
  I2C_Params      params;
  I2C_Params_init(&params);
  params.transferMode  = I2C_MODE_BLOCKING;
  params.bitRate       = I2C_100kHz;
  i2c = I2C_open(0 /* peripheral index */, &params);
  if (!i2c) {
    System_printf("I2C did not open");
    return;
  }
}

