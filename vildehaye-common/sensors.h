/*
 * sensors.h
 *
 *  Created on: 26 ????? 2018
 *      Author: stoledo
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#ifdef USE_VH_SENSORS

#define SENSORS_SUCCESS                 0
#define SENSORS_ERR_TOO_MANY_SENSORS    1
#define SENSORS_ERR_UNKNOWN_SENSOR      2
#define SENSORS_ERR_SENSOR_NO_RESPONSE  3


extern void     sensorsInit();
extern uint16_t sensorsConfigure(uint16_t* configData, uint8_t sizeOfConfigArray);
extern uint16_t sensorsSense(uint16_t periodCounter);

#endif // VH_LOGGER

#endif /* SENSORS_H_ */
