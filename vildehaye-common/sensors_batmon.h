/*
 * sensors_batmon.h
 *
 *  Created on: 28 בדצמ 2016
 *      Author: stoledo
 */

#ifndef SENSORS_BATMON_H_
#define SENSORS_BATMON_H_

void batmonInit();
int32_t batmonTemp();
uint32_t batmonVoltage();

#endif /* SENSORS_BATMON_H_ */
