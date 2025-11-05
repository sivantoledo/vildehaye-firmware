/*
 * sensors_batmon.c
 *
 *  Created on: December 2016
 *      Author: stoledo
 */

#include "config.h"

#include <stdint.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)

static uint8_t batmonEnabled = 0;

void batmonInit() {
	batmonEnabled = 1;
	AONBatMonEnable();
}

/*
 * return temperature in whole degrees Celsius
 */
int32_t batmonTemp() {
	if (!batmonEnabled) batmonInit();
	return AONBatMonTemperatureGetDegC();
}

/*
 * returns voltage in 3.8 format in Volts
 */

uint32_t batmonVoltage() {
	if (!batmonEnabled) batmonInit();
	return AONBatMonBatteryVoltageGet();
}




