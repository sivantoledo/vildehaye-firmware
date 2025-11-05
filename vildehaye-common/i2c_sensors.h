/*
 * i2c_sensors.h
 *
 *  Created on: 3 ×‘×�×¤×¨ 2017
 *      Author: stoledo
 */

#ifndef I2C_SENSORS_H_
#define I2C_SENSORS_H_

void i2cSensors_init();
void bmi160SetupAccelGRange(const uint8_t* sensorData);
void bmi160SetupAccelTicksFactor(const uint8_t* sensorData, uint16_t dataLength);
void bmi160SetupAccelSampleRate(const uint8_t* sensorData, uint16_t dataLength);
void bmi160SetupAccelSampleDuration(const uint8_t* sensorData);
void I2CSensorsSemaphore_post();

#endif /* I2C_SENSORS_H_ */
