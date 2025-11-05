/*
 * Sivan Toledo, 2016
 */

#include <stdint.h>
#include "vildehaye.h"
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/cc26xx/Seconds.h>

#include "config.h"
#include "tag.h"
#include "basestation.h"
#include "leds.h"
#include "watchdog.h"
#include "radio_setup.h"
//#include "i2c_sensors.h"
//#include "sensors.h"

/*
typedef struct videhaye_packet_st {
	uint8_t* packet;
	uint16_t length;
	uint16_t pointer;
	uint16_t size;
} vildehaye_packet_t;
*/

void vildehayeInitPacket(vildehaye_packet_t* p, uint8_t* buffer, uint16_t length, uint16_t size) {
	p->packet  = buffer;
	p->length  = length;
	p->size    = size;
	p->pointer = 0;
}

uint64_t vildehayeGetUint64(const uint8_t* field, uint16_t len) {
	uint64_t v = 0;
	uint64_t x;
	uint16_t i;
	uint8_t shift = 0;
	for (i=0; i<len; i++) {
		//System_printf("%02x\n",field[i]);
		x = field[i];
		x <<= shift;
		v = v + x; // | did not work on 64-bit
		//ystem_printf("%02x %08x %08x\n",field[i],x,v);
		shift += 8;
	}
	//System_printf("get64 %d\n",v);
	return v;
}

// call this only if you know that len<=4
uint32_t vildehayeGetUint32(const uint8_t* field, uint16_t len) {
	uint32_t v = 0;
	uint32_t x;
	uint16_t i;
	uint8_t shift = 0;
	for (i=0; i<len; i++) {
		x = field[i];
		x <<= shift;
		v = v | x;
		shift += 8;
	}
	return v;
}

void vildehayeAddUInt8(vildehaye_packet_t* p, uint8_t v) {
	*( (uint8_t*) ((p->packet) + (p->length)) ) = v;

	(p->length) += sizeof(uint8_t);
	//System_printf("packing 1-byte uint %d %02x\n",v, v);
	//System_exit(1);
}

void vildehayeAddUInt16(vildehaye_packet_t* p, uint16_t v) {
	*( (uint16_t*) ((p->packet) + (p->length)) ) = v;

	(p->length) += sizeof(uint16_t);

	//System_printf("packing 2-byte uint %d %04x\n",v, v);
	//System_exit(1);
}

void vildehayeAddUInt32(vildehaye_packet_t* p, uint32_t v) {
	*( (uint32_t*) ((p->packet) + (p->length)) ) = v;

	(p->length) += sizeof(uint32_t);

	//System_printf("packing 4-byte uint %d %08x\n",v, v);
	//System_exit(1);
}

void vildehayeAddUInt48(vildehaye_packet_t* p, uint64_t v) {
	*( (uint32_t*) ((p->packet) + (p->length)) ) = (uint32_t) v;
	(p->length) += sizeof(uint32_t);
	*( (uint16_t*) ((p->packet) + (p->length)) ) = (uint16_t) (v>>32);
	(p->length) += sizeof(uint16_t);
	System_printf("packing 6-byte uint %8x:%08x\n",(uint32_t) (v>>32), (uint32_t) v);
	//System_exit(1);
}

void vildehayeAddUInt64(vildehaye_packet_t* p, uint64_t v) {
	*( (uint32_t*) ((p->packet) + (p->length)) ) = (uint32_t) v;
	(p->length) += sizeof(uint32_t);
	*( (uint32_t*) ((p->packet) + (p->length)) ) = (uint32_t) (v>>32);
	(p->length) += sizeof(uint32_t);
	System_printf("packing 8-byte uint %8x:%08x\n",(uint32_t) (v>>32), (uint32_t) v);
	//System_exit(1);
	//*( (uint64_t*) ((p->packet) + (p->length)) ) = v;
	//(p->length) += sizeof(uint64_t);
}

void vildehayeAddUIntWithHeader(vildehaye_packet_t* p, uint16_t type, uint64_t v) {
	System_printf("packing int %8x:%08x\n",(uint32_t) (v>>32), (uint32_t) v);
	if ((v <= 0xFFLL)) {
		vildehayeAddHeader(p, type, 1);
		vildehayeAddUInt8(p, (uint8_t) v);
		//System_printf("1 byte\n");
		return;
	}
	if ((v <= 0xFFFFLL)) {
		vildehayeAddHeader(p, type, 2);
		vildehayeAddUInt16(p, (uint16_t) v);
        //System_printf("2 byte\n");
		return;
	}
	if ((v <= 0xFFFFFFFFLL)) {
		vildehayeAddHeader(p, type, 4);
		vildehayeAddUInt32(p, (uint32_t) v);
        //System_printf("4 byte\n");
		return;
	}
	if ((v <= 0xFFFFFFFFFFLL)) {
		vildehayeAddHeader(p, type, 6);
		vildehayeAddUInt48(p, v);
        //System_printf("6 bytes\n");
		return;
	}
	vildehayeAddHeader(p, type, 8);
	vildehayeAddUInt64(p, v);
    //System_printf("8 byte\n");
}

void vildehayeAddHeader(vildehaye_packet_t* p, uint16_t type, uint16_t len) {
  uint8_t h = 0;
  uint8_t lenCode;
  switch (len) {
  case 0: lenCode = 0; break;
  case 1: lenCode = 1; break;
  case 2: lenCode = 2; break;
  case 4: lenCode = 3; break;
  case 6: lenCode = 4; break;
  case 8: lenCode = 5; break;
  default:
    if (len <= 255) lenCode = 6;
    else            lenCode = 7;
  }

  uint16_t typeCode = type;
  if ( (type <= 255) && (type > 29) ) typeCode = 30; // additional 1 byte
  if (  type > 255 )                  typeCode = 31; // additional two bytes

  h = typeCode | (lenCode << 5);
  //System.out.printf("TX(%d) h=0x%02x code %d len %d\n", length, h, typeCodeFull, len);
  vildehayeAddUInt8(p, h);

  if (lenCode==6) vildehayeAddUInt8 (p, len);
  if (lenCode==7) vildehayeAddUInt16(p, len);
  if (typeCode == 30) vildehayeAddUInt8 (p, type);
  if (typeCode == 31) vildehayeAddUInt16(p, type);
}

void vildehayeAddTagState(vildehaye_packet_t* p, uint8_t flags, uint8_t maxPacketLen, uint16_t periodMilliseconds) {
	if ((flags  &VH_TAGSTATE_NON_SESSION_CMD_OK)==0 && (flags  &VH_TAGSTATE_SESSION_OK)==0) {
		vildehayeAddHeader(p, VH_TAG_STATE, 1);
		vildehayeAddUInt8(p, flags);
		return;
	}

	vildehayeAddHeader(p, VH_TAG_STATE, 4);
	vildehayeAddUInt8 (p, flags);
	vildehayeAddUInt8 (p, maxPacketLen);
	vildehayeAddUInt16(p, periodMilliseconds);
}

#if OBSOLETE
void vildehayeHandlePacket(uint8_t* buffer) {
	uint8_t elementLength = buffer[0];
	uint8_t payloadLength = buffer[1];
	uint8_t rssi          = buffer[1+1+payloadLength];
	uint8_t tsp = 1+1+payloadLength+1;
	uint32_t ts =  buffer[tsp] | (buffer[tsp+1]<<8) | (buffer[tsp+2]<<16) | (buffer[tsp+3]<<24);
	uint8_t  setup        = buffer[1+1+payloadLength+1+4];

	System_printf("vdh len=%d elen=%d\n",payloadLength,elementLength);

	uint16_t p = 2; // payload pointer
	while (p < (1+1+payloadLength)) {
    uint8_t h = buffer[p++];
    uint8_t lenCode = (h >> 5) & 0x7; // upper 3 bits of the byte
    uint8_t typeCode = h & 0x1F; // lower 5 bits

    uint16_t len = -1;
    uint16_t typeCodeFull = -1;

    switch (lenCode) {
    case 0: len = 0; break;
    case 1: len = 1; break;
    case 2: len = 2; break;
    case 3: len = 4; break;
    case 4: len = 6; break;
    case 5: len = 8; break;
    case 6:
    	len = buffer[p++];
    	break;
    case 7:
    	len  = buffer[p++];
    	len |= ((uint16_t) (buffer[p++]) << 8);
    	break;
    }

    if (typeCode < 30)  typeCodeFull = typeCode;
    if (typeCode == 30) typeCodeFull = buffer[p++];
    if (typeCode == 31) {
    	typeCodeFull  = buffer[p++];
    	typeCodeFull |= ((uint16_t) (buffer[p++]) << 8);
    }

    switch (typeCodeFull) {
    case VH_LOCAL_CLOCK:
    	System_printf("VDH local clock length %d value %d\n",len, vildehayeGetUint32(buffer+p, len));
    	Seconds_set( vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_DEF_RADIO_SETUPS:
    	System_printf("VDH radio setup length %d\n",len);
    	basestationRadioSetup(buffer+p, len);
    	break;
    default:
    	System_printf("VDH code %d (no handler) length %d\n",typeCodeFull,len);
    	break;
    }

    p += len; // skip the field
	}
}
#endif


uint16_t vildehayeHandlePacketNaked(const uint8_t* buffer, uint16_t payloadLength) {
	uint16_t p = 0;
  uint64_t dest = 0;

	while (p < payloadLength) {
    uint8_t h = buffer[p++];
    uint8_t lenCode = (h >> 5) & 0x7; // upper 3 bits of the byte
    uint8_t typeCode = h & 0x1F; // lower 5 bits

    uint16_t len          = 0xFFFF;
    uint16_t typeCodeFull = 0xFFFF;

    switch (lenCode) {
    case 0: len = 0; break;
    case 1: len = 1; break;
    case 2: len = 2; break;
    case 3: len = 4; break;
    case 4: len = 6; break;
    case 5: len = 8; break;
    case 6:
    	len = buffer[p++];
    	break;
    case 7:
    	len  = buffer[p++];
    	len |= ((uint16_t) (buffer[p++]) << 8);
    	break;
    }

    if (typeCode < 30)  typeCodeFull = typeCode;
    if (typeCode == 30) typeCodeFull = buffer[p++];
    if (typeCode == 31) {
    	typeCodeFull  = buffer[p++];
    	typeCodeFull |= ((uint16_t) (buffer[p++]) << 8);
    }

    System_printf("VDH code %d %x\n",typeCodeFull,typeCodeFull);

    switch (typeCodeFull) {
    case VH_TAG_STATE:
    	System_printf("TAGSTATE\n");
    	vildehayeCallbackTagState(typeCodeFull, buffer+p, len);
      //if (buffer[p] & VH_TAGSTATE_WAKEUP) tag_triggerWakeup(0); // TODO move to a trigger command!
      //otherwise we do not care about other tags' state
    	break;
    // Sivan July 2018; moved destination id out of tag so that it also applies to base stations.
   // case VH_DESTINATION_ID:
   //     dest = vildehayeGetUint64(buffer+p, len);
   //     System_printf("DESTID %ld length %d tagid %d\n",dest,len,tagId);
   //     //if (tag_setId(dest) != 0) return; // we have an id and dest!=our id
   //     if (dest != tagId) goto done; // return; // not us
   //    break;
   case VH_DESTINATION_ID:
   case VH_DEF_ID:
   case VH_SOURCE_ID:
        dest = vildehayeGetUint64(buffer+p, len);
        //System_printf("SOURCE ID %ld length %d tagid %d\n",dest,len,tagId);
        //if (tag_setId(dest) != 0) return; // we have an id and dest!=our id
        if (vildehayeCallbackTagId(typeCodeFull, dest) == VH_ABORT) goto done;
        break;
    // same for local clock, until July 2018 it only applied to tags.
   case VH_SET_CLOCK:
   case VH_LOCAL_CLOCK:
        //System_printf("VDH local clock length %d value %d\n",len, vildehayeGetUint32(buffer+p, len));
        if (vildehayeCallbackClock(typeCodeFull, vildehayeGetUint32(buffer+p, len)) == VH_ABORT) goto done;
        //Seconds_set( vildehayeGetUint32(buffer+p, len) );
        break;
   case VH_LOG_ITEM:
   case VH_LOG_ITEM_NEW:
   case VH_LOG_ACK:
        vildehayeCallbackLog(typeCodeFull, buffer+p, len);
        //Seconds_set( vildehayeGetUint32(buffer+p, len) );
        break;
   case VH_WAKEUP_INTENTS:
        vildehayeCallbackWakeupIntents(typeCodeFull, buffer+p, len);
        //Seconds_set( vildehayeGetUint32(buffer+p, len) );
        break;
#ifdef USE_TAG
    //case VH_DEF_ID:
    //	dest = vildehayeGetUint64(buffer+p, len);
    //	System_printf("DEF_ID %ld length %d tagid %d\n",dest,len,tagId);
    //	if (tag_setId(dest) != 0) goto done; // return; // we have an id and dest!=our id
    //	break;
    case VH_GOTO_CONFIGURATION:
    	System_printf("GOTO CONF\n");
    	tag_gotoConfiguration( (uint8_t) vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_WAKEUP:
    	System_printf("WAKEUP\n");
    	tag_triggerWakeup( (uint8_t) vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_SEQUENCE_NUMBER:
    	System_printf("SEQNO\n");
    	if (dest == tagId)
    		if (tag_sessionSeqno(vildehayeGetUint32(buffer+p, len)) == 0)
    			goto done; // return; // incorrect seqno, ignore
    	break;
    case VH_DEF_TRIGGER_WAKEUP:
    	System_printf("TRIGGER WAKEUP\n");
    	tag_triggerWakeupSetup( buffer+p+1 ); // skip array descriptor
    	break;
    case VH_DEF_TRIGGER_NO_WAKEUP:
    	System_printf("TRIGGER NO WAKEUP\n");
    	tag_triggerNoWakeupSetup( (const uint16_t*) (buffer+p+1) ); // skip array descriptor
    	break;
    case VH_DEF_VOLTAGE_THRESH:
    	System_printf("VDH VOLTAGE THRESH DEF !!! OBSOLETE !!!\n");
     	//tag_setVoltageThreshold( vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_DEF_CONFIGURATION_SCHEDULES:
    	System_printf("CONF SCHED\n");
    	System_printf("VDH configuration length %d\n",len);
    	// we skip the data length and type byte in the array, which we assume to be uint16_t
    	schedule_init((const uint16_t*) (buffer+p+1), len-2);
    	break;
    case VH_DEF_ATLAS_CODE:
    	System_printf("ATLAS CODE length %d\n",len);
    	atlas_setCode(*(buffer+p+1), buffer+p+2, len-2); // there's an array descriptor followed by the code index
    	break;
    case VH_DEF_POWER_POLICY :
      System_printf("CONF POWER POLICY\n");
      vildehayeCallbackPowerPolicy(typeCodeFull, (uint8_t*) (buffer+p+1), (len-1)>>1);
      break;
    case VH_DEF_SENSOR_CONFIGURATION :
      //System_printf("SENSOR_CONFIG array elen %d total len %d\n",*(buffer+p),len);
      vildehayeCallbackSensorConfig(typeCodeFull, (uint16_t*) (buffer+p+1), (len-1)>>1);
      break;
#endif // tag firmware
#ifdef USE_BASESTATION
    case VH_DEF_MODULES_CONFIGURATION :
      //System_printf("SENSOR_CONFIG array elen %d total len %d\n",*(buffer+p),len);
      vildehayeCallbackModulesConfig(typeCodeFull, (uint16_t*) (buffer+p+1), (len-1)>>1);
      break;
    case VH_BASESTATION_STATE :
      //System_printf("SENSOR_CONFIG array elen %d total len %d\n",*(buffer+p),len);
      vildehayeCallbackBasestationState(typeCodeFull, buffer+p+1, len-1); // single-byte array, skip the size
      break;
#endif
#ifdef VH_BASESTATION_SETUP
    case VH_GOTO_SETUP:
    	System_printf("GOTO SETUP\n");
    	basestation_gotoSetup( (uint8_t) vildehayeGetUint32(buffer+p, len) );
    	break;
#endif
    case VH_BLINK_NOW:
    	System_printf("BLINK NOW\n");
    	leds_incrementBlinkCounter();
    	leds_blink(LEDS_ANY, (uint8_t) vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_DEF_BLINK_COUNTER:
    	System_printf("BLINK CTRL\n");
     	leds_setBlinkCounter( (uint8_t) vildehayeGetUint32(buffer+p, len) );
    	break;
    case VH_DEF_WATCHDOG_PERIOD_S:
    	System_printf("VDH WATCHDOG DEF (removed!)\n");
     	//watchdog_init( vildehayeGetUint32(buffer+p, len)*1000, 8*vildehayeGetUint32(buffer+p, len)*1000 );
    	break;
    	/*
    case VH_RANDOM_BEACON:
    	System_printf("VDH random beacon length %d\n",len);
    	// we skip the data length and type byte in the array, which we assume to be uint16_t
    	random_schedule_init(buffer+p+1, len-2);
    	break;
    	*/
    case VH_DEF_RADIO_SETUPS:
    	System_printf("RADIO SETUP length %d\n",len);
//#ifdef BASESTATION_FIRMWARE
    	//basestationRadioSetup(buffer+p, len);
//#endif
//#ifdef TAG_FIRMWARE
    	vildehayeCallbackRadioSetups(typeCodeFull, buffer+p, len);
    	//radioSetup_configureFromBuffer(buffer+p, len);
//#endif
    	break;
#if 0
    case BMI160_G_RANGE:
            System_printf("BMI160 G RANGE length %d\n",len);
            bmi160SetupAccelGRange(buffer+p);
            break;
    case BMI160_ACCEL_FACTOR:
            System_printf("BMI160 ACCEL FACTOR length %d\n",len);
            bmi160SetupAccelTicksFactor(buffer+p,len);
            break;
    case BMI160_ACCEL_DURATION:
            System_printf("BMI160 ACCEL DURATION length %d\n",len);
            bmi160SetupAccelSampleDuration(buffer+p);
            break;
    case BMI160_ACCEL_RATE:
            System_printf("BMI160 ACCEL RATE length %d\n",len);
            bmi160SetupAccelSampleRate(buffer+p,len);
            break;
#endif
    default:
    	System_printf("VDH code %d (no handler) length %d\n",typeCodeFull,len);
    	break;
    }

    p += len; // skip the field
	}

	done:
	return p;
}
