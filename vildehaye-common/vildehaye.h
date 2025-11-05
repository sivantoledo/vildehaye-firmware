/*
 * Sivan Toledo, 2016
 */

#ifndef VILDEHAYE_H_
#define VILDEHAYE_H_

#include <stdint.h>

#define VH_SOURCE_ID             0
#define VH_DESTINATION_ID        1
#define VH_TAG_STATE             2
#define VH_SEQUENCE_NUMBER       3
#define VH_END_OF_SESSION        4
#define VH_LOCAL_CLOCK           5
#define VH_BLINK_NOW             6
#define VH_SET_CLOCK             7

#define VH_GOTO_CONFIGURATION    8
#define VH_WAKEUP                9
#define VH_SECS_SINCE_WAKEUP     10
#define VH_LOG_ITEM              11
#define VH_LOG_ACK               12
#define VH_LOGGER_STATE          13
#define VH_LOG_ITEM_NEW          14
#define VH_DEF_RADIO_SETUPS      100
#define VH_DEF_CONFIGURATION_SCHEDULES 101
#define VH_DEF_ATLAS_CODE        102
//#define VH_RANDOM_BEACON       103
#define VH_DEF_ID                103
#define VH_NOP                   104
#define VH_WAKEUP_INTENTS        105
#define VH_DEF_SENSOR_CONFIGURATION 106
#define VH_DEF_MODULES_CONFIGURATION 107
#define VH_DEF_TRIGGER_WAKEUP    120
#define VH_DEF_TRIGGER_NO_WAKEUP 121
#define VH_DEF_BLINK_COUNTER     130
#define VH_DEF_WATCHDOG_PERIOD_S 131
#define VH_DEF_VOLTAGE_THRESH    132
#define VH_DEF_POWER_POLICY      137
#define VH_BASESTATION_STATE     63000
#define VH_TEST_UINTARRAY        63003
#define VH_TEST_UNSIGNED         64004
#define VH_GOTO_SETUP            65001
#define BMI160_G_RANGE           133
//#define BMI160_ACCEL_FACTOR      134
//#define BMI160_ACCEL_DURATION    135
//#define BMI160_ACCEL_RATE        136

#define VH_TAGSTATE_ATLAS_CODE_INDEX_MASK     0x0F
#define VH_TAGSTATE_ATLAS_INVERTED      0x80
#define VH_TAGSTATE_ATLAS_RANDOM_SIGN   0x40

#define VH_TAGSTATE_NON_SESSION_CMD_OK  0x80
#define VH_TAGSTATE_SESSION_OK          0x40
#define VH_TAGSTATE_HAS_DATA            0x20
#define VH_TAGSTATE_NEEDS_CONFIGURATION 0x10

#define VH_TAGSTATE_CONFIGURATION_INDEX_MASK 0x03

//#define VH_TAGSTATE_WAKEUP              0x08
//#define VH_TAGSTATE_WAKEUP_ALT          0x04
#define VH_TAGSTATE_LISTEN_ONLY         0x01
#define VH_OPTIONS_BEACON_WAKEUP_MASK  0x0700
#define VH_OPTIONS_BEACON_WAKEUP1      0x0100
#define VH_OPTIONS_BEACON_WAKEUP2      0x0200
#define VH_OPTIONS_BEACON_WAKEUP3      0x0300
#define VH_OPTIONS_BEACON_WAKEUP4      0x0400
#define VH_OPTIONS_BEACON_WAKEUP5      0x0500
#define VH_OPTIONS_BEACON_WAKEUP6      0x0600
#define VH_OPTIONS_BEACON_WAKEUP7      0x0700
#define VH_OPTIONS_BEACON_CLOCK        0x0800
#define VH_OPTIONS_RESERVED1           0x1000
#define VH_OPTIONS_BEACON_SECS_SINCE_WAKEUP   0x2000
#define VH_OPTIONS_BEACON_RANDOM_SLOT  0x4000
#define VH_OPTIONS_LISTEN_ONLY         0x8000

typedef struct videhaye_packet_st {
	uint8_t* packet;
	uint16_t length;
	uint16_t pointer;
	uint16_t size;
} vildehaye_packet_t;

void vildehayeInitPacket(vildehaye_packet_t* p, uint8_t* buffer, uint16_t length, uint16_t size);
void vildehayeAddUInt8(vildehaye_packet_t* p, uint8_t v);
void vildehayeAddUInt16(vildehaye_packet_t* p, uint16_t v);
void vildehayeAddUInt32(vildehaye_packet_t* p, uint32_t v);
void vildehayeAddHeader(vildehaye_packet_t* p, uint16_t type, uint16_t len);
void vildehayeAddUIntWithHeader(vildehaye_packet_t* p, uint16_t type, uint64_t v);
void vildehayeAddTagState(vildehaye_packet_t* p, uint8_t flags, uint8_t maxPacketLen, uint16_t periodMilliseconds);

uint32_t vildehayeGetUint32(const uint8_t* field, uint16_t len);
uint64_t vildehayeGetUint64(const uint8_t* field, uint16_t len);

#define VH_CONTINUE 0
#define VH_ABORT    1

void vildehayeHandlePacket(uint8_t* buffer);
uint16_t vildehayeHandlePacketNaked(const uint8_t* buffer,uint16_t payloadLength);
uint32_t vildehayeCallbackTagId        (uint16_t typeCode, uint64_t id);
uint32_t vildehayeCallbackClock        (uint16_t typeCode, uint32_t clock);
uint32_t vildehayeCallbackRadioSetups  (uint16_t typeCode, const uint8_t* data, uint16_t len);
uint32_t vildehayeCallbackLog          (uint16_t typeCode, const uint8_t* data, uint16_t len);
uint32_t vildehayeCallbackTagState     (uint16_t typeCode, const uint8_t* data, uint16_t len);
uint32_t vildehayeCallbackWakeupIntents(uint16_t typeCode, const uint8_t* data, uint16_t len);

#endif /* VILDEHAYE_H_ */
