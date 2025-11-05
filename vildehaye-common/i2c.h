#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/I2C.h>

extern I2C_Handle      i2c;
extern I2C_Transaction i2cTransaction;

void i2cInit();

#endif // I2C_H
