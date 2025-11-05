/*
 * ublox.h
 *
 *  Created on: 1 ????? 2018
 *      Author: stoledo
 */

#ifndef UBLOX_H_
#define UBLOX_H_

//extern Mailbox_Handle ubloxOutputMailbox;

#define UBLOX_STATE_UNKNOWN      0
#define UBLOX_STATE_PORT_FAILURE 1
#define UBLOX_STATE_PENDING      2
#define UBLOX_STATE_NORMAL       3

extern uint8_t ubloxState;

typedef struct __attribute__((packed)) {
  uint32_t  iTOW;
  uint32_t  tAcc;
   int32_t  nano;
  uint16_t  year;
  uint8_t   month;
  uint8_t   day;
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   valid; // 3 bits
} NAV_TIMEUTC;

typedef struct __attribute__((packed)) {
  uint32_t  iTOW;    // GPS Millisecond Time of Week
  int32_t   lon;     // Longitude (1e-7)
  int32_t   lat;     // Latitude (1e-7)
  int32_t   height;  // Height above Ellipsoid (mm)
  int32_t   hMSL;    // Height above mean sea level (mm)
  uint32_t  hAcc;    // Horizontal Accuracy Estimate (mm)
  uint32_t  vAcc;    // Vertical Accuracy Estimate (mm)
} NAV_POSLLH;

typedef struct __attribute__((packed)) {
  uint32_t  iTOW;

  uint16_t  year;
  uint8_t   month;
  uint8_t   day;
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   valid; // 3 bits
  uint32_t  tAcc;  // time accuracy, ns
   int32_t  nano;  // fraction of second

  uint8_t   fixType;
  uint8_t   flags;
  uint8_t   reserved1;
  uint8_t   numSV;

  int32_t   lon;     // Longitude (1e-7)
  int32_t   lat;     // Latitude (1e-7)
  int32_t   height;  // Height above Ellipsoid (mm)
  int32_t   hMSL;    // Height above mean sea level (mm)
  uint32_t  hAcc;    // Horizontal Accuracy Estimate (mm)
  uint32_t  vAcc;    // Vertical Accuracy Estimate (mm)

  // There is more, but we do not need the rest
} NAV_PVT;

void ubloxInit();
bool ubloxPoll();

uint8_t ubloxEnableMsg(uint8_t* buffer, uint8_t class, uint8_t msg, uint8_t rate);
uint8_t ubloxConfigureUart1(uint8_t* buffer, uint32_t baudRate, bool nmea, bool ubx);
void ubloxParseMessage(uint8_t* buffer, uint16_t length);

uint32_t utc2unix( uint16_t year, uint8_t mon, uint8_t day, uint8_t hrs, uint8_t min, uint8_t sec);

#endif /* UBLOX_H_ */
