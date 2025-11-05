#include "config.h"

#ifdef USE_TAG

#ifndef RECEIVE_TIME_LIMIT_US
#define RECEIVE_TIME_LIMIT_US 8000
#endif

#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/devices/DeviceFamily.h>
#include "radio.h"

#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)


#include "config.h"
#include "tag.h"
#include "hooks.h"
#include "radio_setup.h"
#include "leds.h"
//#include "board.h"
#include "receive.h"

#include "watchdog.h"
#include "sensors_batmon.h"
#include "vildehaye.h"
#include "hallsensor-switch.h"
#include "scif-low-voltage-suspender.h"

#include "i2c.h"
#include "console.h"

#ifdef USE_VH_SENSORS
#include "sensors.h"
#endif

#ifdef USE_VH_LOGGER
#include "logger_catalog.h"
#include "logger.h"
#endif

#ifdef USE_VESPER
#include "vesper.h"
//#include "logger.h"
#endif

/*****************************************************************************/
/* WRITE CONFIGURATION TO FLASH                                              */
/*****************************************************************************/

static uint8_t* flashConfigurationData = (uint8_t*) 0x0001e000;
#define flashPageSize 0x1000
// this should be adjusted when we find out how much space there is past the configuration packet
static uint32_t flashConfigurationEnd  = 0x0001e000 + (flashPageSize-1);

#include DeviceFamily_constructPath(driverlib/rom.h)
#include DeviceFamily_constructPath(driverlib/vims.h)
//#include <ti/devices/cc13x0/driverlib/rom.h>
//#include <ti/devices/cc13x0/driverlib/vims.h>
//extern uint32_t ROM_FlashProgram(uint8_t* buffer, uint32_t address, uint32_t count);
//#include <ti/devices/cc13x0/driverlib/flash.h>

void flashSetConfigurationLimit(uint16_t length) {
	flashConfigurationEnd = 0x0001e000 + length;
}

uint8_t flashGetLastConfiguration() {
	uint8_t* p = flashConfigurationData + (flashPageSize - 1);
	uint8_t  last = 255; // nothing found yet;

	while ((uint32_t) p > flashConfigurationEnd) {
		uint8_t current = *p;
		System_printf("checking address 0x%08x = %d\n",(uint32_t) p,current);
		if (current == 255) break; // no more programmed bytes here
		last = current;
		p--;
	}
	return last;
}

void flashWriteLastConf(uint8_t c) {
	uint8_t* p = flashConfigurationData + (flashPageSize - 1);
	while ((uint32_t) p > flashConfigurationEnd) {
	  uint8_t current = *p;
		if (current == 255 && current != c) {
		  //ROM_FlashProgram(&configuration, (uint32_t) p, 1);
		  VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
		  while (VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);
		  CPUcpsid();
		  HapiProgramFlash(&c, (uint32_t) p, 1);
		  // uint32_t rc = HapiProgramFlash(&c, (uint32_t) p, 1);
		  CPUcpsie();
		  VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
		  //System_printf("*** flash write result %d ***\n",rc);
		  return;
		}
		p--;
	}
}

/*****************************************************************************/
/* PROTOCOL HANDLERS                                                         */
/*****************************************************************************/
uint64_t tagId = 0xFFFFFFFF;
uint32_t vildehayeCallbackTagId(uint16_t typeCode, uint64_t id) {
  switch (typeCode) {
  case VH_SOURCE_ID:
    // tags don't care (for now at least) about source ids
    return VH_CONTINUE;
    //break;
  case VH_DESTINATION_ID:
    // continue only if the packet is destined to me
    if (tagId == id) return VH_CONTINUE;
    else             return VH_ABORT;
    //break;
  case VH_DEF_ID:
    // set tagId if not set, otherwise check that it's the same as the current id
    if (tagId==0xFFFFFFFF) {
       //System_printf("setting tag id to %d (lower digits)\n",(uint32_t) (id%1000000));
       //System_printf("setting tag id to %8x:%08x\n",(uint32_t) (id>>32), (uint32_t) id);
       //System_exit(1);
       tagId = id;
       return VH_CONTINUE;
     } else {
       if (tagId == id) return VH_CONTINUE;
       else             return VH_ABORT;
     }

    //break;
  default:
    return VH_CONTINUE; // should never happen.
  }
}

uint32_t vildehayeCallbackClock(uint16_t typeCode, uint32_t clock) {
  switch (typeCode) {
  case VH_LOCAL_CLOCK:
    break;
  case VH_SET_CLOCK:
    Seconds_set( clock );
    break;
  default:
    break;
  }
  return VH_CONTINUE;
}

uint32_t vildehayeCallbackRadioSetups(uint16_t typeCode, const uint8_t* radioData, uint16_t radioDataLength) {
  radioSetup_configureFromBuffer(radioData, radioDataLength);
  return VH_CONTINUE;
}

void suspenderSetPolicy(const uint8_t* data, uint16_t length);
uint32_t vildehayeCallbackPowerPolicy(uint16_t typeCode, const uint8_t* data, uint16_t length) {
  suspenderSetPolicy(data, length);
  return VH_CONTINUE;
}

#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
static uint8_t  logItem[224];
static uint8_t  logItemLength = 0xFF; // 0xFF means there is no log item to report
static uint8_t  logItemType;
static uint32_t logItemAddress;    // this serves a a kind of uid for the item
#endif

uint32_t vildehayeCallbackLog(uint16_t typeCode, const uint8_t* logData, uint16_t logDataLength) {
#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
  extern uint32_t lastWakeupSecs; // defined later
  switch (typeCode) {
  case VH_LOG_ITEM:
  case VH_LOG_ITEM_NEW:
    // should never happen in a tag
    break;
  case VH_LOG_ACK:
    // got an ack from a base station
    lastWakeupSecs = Seconds_get(); // this is considered a wakeup, to keep the tag in the download configuration

    if (vildehayeGetUint32(logData,4) == logItemAddress) {
      loggerAck(logItemAddress);
      logItemLength = 0xFF; // this will cause the main loop to read another item, if there is one
    }
    break;
  default:
    break;
  }
#endif
  return VH_CONTINUE;
}

uint32_t vildehayeCallbackSensorConfig(uint16_t typeCode, const uint16_t* config, uint8_t configSize) {
#ifdef USE_VH_SENSORS
    System_printf("\r\nvildehayeCallbackSensorConfig\r\n");
    consoleFlush();
    Task_sleep( 10*1000 / Clock_tickPeriod);

  sensorsConfigure(config, configSize); // an array of uint16_t, skip the element length
#endif
  return VH_CONTINUE;
}

/*
int32_t tag_setId(uint64_t id) {
	if (tagId==0xFFFFFFFF) {
		//System_printf("setting tag id to %d (lower digits)\n",(uint32_t) (id%1000000));
		//System_printf("setting tag id to %8x:%08x\n",(uint32_t) (id>>32), (uint32_t) id);
		//System_exit(1);
		tagId = id;
		return 0; // all okay
	}
	if (tagId == id) return 0; // packet destination is us
	return -1; // packet for some other tag
}
*/

uint32_t vildehayeCallbackWakeupIntents(uint16_t typeCode, const uint8_t* data, uint16_t len) {return VH_CONTINUE;}
uint32_t vildehayeCallbackTagState     (uint16_t typeCode, const uint8_t* data, uint16_t len) {return VH_CONTINUE;}

/*****************************************************************************/
/* PROTOCOL HANDLERS                                                         */
/*****************************************************************************/

#define DATA_PROTOCOL_ATLAS                     16
#define DATA_PROTOCOL_VILDEHAYE                  8
#define DATA_PROTOCOL_VILDEHAYE_BEACON           3
#define DATA_PROTOCOL_VILDEHAYE_BEACON_RESPONSE  2
#define DATA_PROTOCOL_VILDEHAYE_SESSION          1

#define SLOT_ACTION_TX     1
#define SLOT_ACTION_TX_ADV 2
#define SLOT_ACTION_RX     3
#define SLOT_ACTION_NOP    4

typedef struct slot_action_st {
	uint8_t  action;
	uint8_t  opts;
	uint8_t* packet;
	uint16_t packet_len;
	uint8_t  dbm; // Sivan new Nov 2021
} slot_action_t;

static slot_action_t nopSlotAction = {
        .action     = SLOT_ACTION_NOP,
        .opts       = 0,
        .packet     = 0,
        .packet_len = 0,
        .dbm        = 10,
};


//typedef slot_action_t* (*slot_preparer_t)(uint8_t configuration, uint8_t setup, uint16_t slot);

uint16_t tagPeriodMs = 1000;

/*****************************************************************************/
/* TAG SCHEDULE                                                              */
/*****************************************************************************/

#ifndef MAX_CONFIGURATIONS
#define MAX_CONFIGURATIONS 4
#endif

//static uint8_t radioSetupsCount;

slot_action_t* vildehayeGetSlotAction(uint8_t configuration, uint8_t setup, uint16_t slot);
slot_action_t* atlasGetSlotAction(uint8_t configuration, uint8_t setup, uint16_t slot);

	// skip radio setups
//slot_preparer_t setupPreparers[ MAX_RADIO_SETUPS ];

/*
 * Each configuration is descibed by an array that specifies frame length (in tag-period units)
  * and the slot allocation (start slot, step, and count limit) for each radio setup.
 * Note that the limit is on the number of slots used by the setup, it is not the largest
 * slot index allowed (the end index).
 */
#define CONF_FRAME_LEN     0
#define SETUP_START(SETUP) (1+((SETUP)*5)+0)
#define SETUP_STEP(SETUP)  (1+((SETUP)*5)+1)
#define SETUP_LIMIT(SETUP) (1+((SETUP)*5)+2)
#define SETUP_DBM(SETUP)   (1+((SETUP)*5)+3)
#define SETUP_OPTS(SETUP)  (1+((SETUP)*5)+4)
uint16_t configurations[MAX_CONFIGURATIONS][1+(MAX_RADIO_SETUPS*5)];

uint8_t configurationsCount;
 int8_t  currentConfiguration = 0; // default
//uint16_t currentFrameLength;
uint16_t slotInFrameIndex;
 int16_t nextSlotForSetup [ MAX_RADIO_SETUPS ];
uint16_t slotStepForSetup [ MAX_RADIO_SETUPS ];
uint16_t slotCountForSetup[ MAX_RADIO_SETUPS ];
uint16_t slotLimitForSetup[ MAX_RADIO_SETUPS ];
 int8_t  slotDbmForSetup  [ MAX_RADIO_SETUPS ];
uint8_t  slotOptsForSetup [ MAX_RADIO_SETUPS ];

 int8_t slotRadioSetup; // which radio setup is used by this slot

 slot_action_t* slotActionPtr; // pointer to what to do in this slot

 /*
  *
  */

void tag_gotoConfiguration(uint8_t c) {
	// TODO: right now only works in initial configuration
	//System_printf("goto configuration %d\n",c);
	if (currentConfiguration==c) return;

	currentConfiguration = c;
	slotInFrameIndex=65535; // this causes a new frame to start
}

uint8_t wakeupTriggers[4][4];
uint8_t wakeupTriggersCount = 0;
void tag_triggerWakeupSetup(const uint8_t* wtarray) {
	System_printf("wakeup trigger (%d,%d) -> %d\n",wtarray[0], wtarray[1], wtarray[2]);
	wakeupTriggers[ wakeupTriggersCount ][0] = wtarray[0]; // fromConf
	wakeupTriggers[ wakeupTriggersCount ][1] = wtarray[1]; // fromSetup
	wakeupTriggers[ wakeupTriggersCount ][2] = wtarray[2]; // toConf
	wakeupTriggers[ wakeupTriggersCount ][3] = wtarray[3]; // trigger index
	wakeupTriggersCount++;
}

uint32_t lastWakeupSecs = 0;
uint32_t noWakeupTriggerSecs;
 int8_t  noWakeupTriggerConf = -1;
uint8_t  noWakeupTriggerArmed = 0;
void tag_triggerNoWakeupSetup(const uint16_t* nwtarray) {
	noWakeupTriggerConf  = nwtarray[0]; // toConf
	noWakeupTriggerSecs  = nwtarray[1]; // secs
	noWakeupTriggerSecs += (nwtarray[2] * 3600 ); // hours
	noWakeupTriggerSecs += (nwtarray[2] * 3600 * 24 ); // days
	//lastWakeupSecs = 0;
	noWakeupTriggerArmed = 1;
	System_printf("no wakeup trigger %d after %ds\n",noWakeupTriggerConf, noWakeupTriggerSecs);
}

static int8_t wakeupTriggerConf = -1; // configuration to go to
void tag_triggerWakeup(uint8_t trigger) {
	lastWakeupSecs = Seconds_get();
	if (noWakeupTriggerConf!=-1) {
		noWakeupTriggerArmed = 1; // start counting
	}
	if (trigger < configurationsCount) {
	  wakeupTriggerConf = trigger;
	  flashWriteLastConf(wakeupTriggerConf);
	}	else
	  wakeupTriggerConf = configurationsCount-1; // instruction to move temporarily to download configuration

// Sivan Aug 2018: triggers should simply move the tag to the stated configuration
#if 0
  uint8_t i;
	for (i=0; i<wakeupTriggersCount; i++) {
		if (wakeupTriggers[i][0] == currentConfiguration
				&& wakeupTriggers[i][1] == slotRadioSetup
				&& wakeupTriggers[i][3] == trigger) {
			wakeupTriggerConf = wakeupTriggers[i][2];
			System_printf("wakup -> %d\n",wakeupTriggerConf);
			return;
		}
	}
	System_printf("wakup trigger %d not found\n",trigger);
#endif
}

/*
 * This function is called at the end of a slot to prepare the schedule for the next slot.
 */

void scheduleAdvance() {
	uint8_t i;

	 //System_printf("0 conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);
	uint8_t newFrame = 0;
	if (slotInFrameIndex>=configurations[currentConfiguration][CONF_FRAME_LEN]-1) newFrame = 1;

	// Sivan Aug 2018 trying to switch at the end of a frame only
	/*
	if (wakeupTriggerConf != -1) { // we got a wakeup packet
		flashWriteLastConf(wakeupTriggerConf);

		newFrame = 1;
		currentConfiguration = wakeupTriggerConf;
		wakeupTriggerConf = -1;
		System_printf("wup->%d!\n",currentConfiguration);
	}
	if (noWakeupTriggerArmed && (Seconds_get() - lastWakeupSecs) > noWakeupTriggerSecs) {
		noWakeupTriggerArmed = 0; // disarm
		newFrame = 1;
		currentConfiguration = noWakeupTriggerConf;
		wakeupTriggerConf = -1;
		System_printf("wdn->%d!\n",currentConfiguration);
	}
	*/

	//if (slotInFrameIndex>=configurations[currentConfiguration][CONF_FRAME_LEN]-1) { // this frame is done
	if (newFrame) { // this frame is done

	  if (wakeupTriggerConf == -1
	      && noWakeupTriggerArmed
	      && (Seconds_get() - lastWakeupSecs) > noWakeupTriggerSecs) {
	    noWakeupTriggerArmed = 0; // disarm
	    currentConfiguration = noWakeupTriggerConf;
	    //wakeupTriggerConf = -1;
	  }
    if (wakeupTriggerConf != -1) { // we got a wakeup packet
      //flashWriteLastConf(wakeupTriggerConf); // Sivan bug need to do this only for manual wakeup
      currentConfiguration = wakeupTriggerConf;
      wakeupTriggerConf = -1;
    }

		//System_printf("nf: %d!\n",currentConfiguration);
		// apply triggers here

		// now currentConfiguration is setup correctly, initialize the frame.
		slotInFrameIndex = 0; // we start a new frame
		for (i=0; i<radioSetupsCount; i++) {
			nextSlotForSetup [i] = configurations[currentConfiguration][SETUP_START(i)];
			slotStepForSetup [i] = configurations[currentConfiguration][SETUP_STEP (i)];
			slotLimitForSetup[i] = configurations[currentConfiguration][SETUP_LIMIT(i)];
			slotCountForSetup[i] = 0;
			//System_printf("* %d\n",i);
		}
	} else {
		if (slotRadioSetup >= 0) { // -1 means no radio setup uses the slot
			slotCountForSetup[slotRadioSetup]++;
			if (slotCountForSetup[slotRadioSetup] >= slotLimitForSetup[slotRadioSetup]) { // we reached the limit, drop the setup from frame
				nextSlotForSetup[slotRadioSetup] = -1;
			} else {
				nextSlotForSetup[slotRadioSetup] += slotStepForSetup[slotRadioSetup];
			}
			//System_printf("next[%d] = %d\n",slotRadioSetup,nextSlotForSetup[slotRadioSetup]);
		}

		//System_printf("inc\n");
	  slotInFrameIndex++;
	}

	 //System_printf("1 conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);

	 /*
	 * Now the frame is set up, the slot index is correct for the next slot,
	 * and the radio setup for the last slot has been updated.
	 * Find out which setup will use the next slot.
	 */
	// the next slot and tell it to prepare.

	for (i = 0; i<radioSetupsCount; i++) {
		if (slotInFrameIndex == nextSlotForSetup[i]) {
			// this is the setup we will use!
			slotRadioSetup = i;

			switch (radioSetupDataProtocol[ slotRadioSetup ]) {
			case DATA_PROTOCOL_ATLAS:
				slotActionPtr = atlasGetSlotAction(currentConfiguration, slotRadioSetup, slotInFrameIndex);
				//System_printf("3A conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);
	            System_printf("AT c%d s%d r%d a%d\n",currentConfiguration,slotInFrameIndex,slotRadioSetup,slotActionPtr->action);
				return;
			case DATA_PROTOCOL_VILDEHAYE:
			case DATA_PROTOCOL_VILDEHAYE_BEACON:
			case DATA_PROTOCOL_VILDEHAYE_BEACON_RESPONSE:
			case DATA_PROTOCOL_VILDEHAYE_SESSION:
				slotActionPtr = vildehayeGetSlotAction(currentConfiguration, slotRadioSetup, slotInFrameIndex);
				//System_printf("3V conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);
	            System_printf("VH c%d s%d r%d a%d\n",currentConfiguration,slotInFrameIndex,slotRadioSetup,slotActionPtr->action);
				return;
			default:
				//System_printf("3X conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);
	            System_printf("!! c%d s%d r%d a%d\n",currentConfiguration,slotInFrameIndex,slotRadioSetup,slotActionPtr->action);
				return;
			}
		}
	}

	// Sivan July 2018 changed the indication of no action from slotRadioSetup==-1 to a nop action
	slotRadioSetup = -1; // no radio setup uses the next slot.
	slotActionPtr = &nopSlotAction;
	//System_printf("oops c%d s%d r%d\n",currentConfiguration,slotInFrameIndex,slotRadioSetup);
	//System_printf("3Y conf %d slot %d setup %d\n",currentConfiguration, slotInFrameIndex,slotRadioSetup);
}

void schedule_init(const uint16_t* configurationData, uint16_t configurationDataLength) {
	 tagPeriodMs = configurationData[ 0 ];
	 configurationsCount  = 0;
	 uint8_t confIndex = 0; // may come from gotoConfiguration

	 System_printf("tag period=%d ms initial configuration %d\n",tagPeriodMs,confIndex);

	 uint16_t i = 1;
	 uint16_t l = configurationDataLength / sizeof(uint16_t);
	 while (i<l) {
		 //configurations[ currentConfiguration ][0] = configurationData[ i++ ];
		 uint8_t j;
		 for (j=0; j<(1+(radioSetupsCount*5)); j++) {
			 //System_printf(":: %d -> %d\n",j,configurationData[ i ]);
			 configurations[ confIndex ][ j ] = configurationData[ i++ ];
		 }
		 confIndex++;
	 }
	 configurationsCount = confIndex;

	 System_printf("#conf=%d #setups=%d\n",configurationsCount,radioSetupsCount);

	 for (confIndex = 0; confIndex<configurationsCount; confIndex++) {
		 System_printf("%d frame len=%d\n",confIndex,configurations[confIndex][CONF_FRAME_LEN]);
		 for (i=0; i<radioSetupsCount; i++)
			 System_printf("  start=%d step=%d limit=%d dbm=%d opts=%x\n",
					 configurations[confIndex][SETUP_START(i)],
					 configurations[confIndex][SETUP_STEP(i)],
					 configurations[confIndex][SETUP_LIMIT(i)],
					 configurations[confIndex][SETUP_DBM(i)],
					 configurations[confIndex][SETUP_OPTS(i)]
					 );
	 }

	 /*** now setup initial state ***/

	 //currentConfiguration = 0;
	 slotInFrameIndex = 65535; // over the frame length, will cause frame initialization
	 scheduleAdvance();
	 System_printf("schedule init ended\n");
}

/*****************************************************************************/
/* PROTOCOL HANDLERS: ATLAS                                                  */
/*****************************************************************************/

// define in config.h, but defined here for compatability

#ifndef ATLAS_CODE_BUFFER_BYTES
// dual-frequency tags and beacons use two ATLAS codes, logging tags should use only one
#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
#define ATLAS_CODE_BUFFER_BYTES 1024
#else
#define ATLAS_CODE_BUFFER_BYTES 2048
#endif
#endif

static uint8_t atlas_code[ATLAS_CODE_BUFFER_BYTES];

/*
// dual-frequency tags and beacons use two ATLAS codes, logging tags should use only one
#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
static uint8_t atlas_code[1024];
#else
static uint8_t atlas_code[2048];
#endif
*/


static uint8_t* atlas_codes[4];
static uint16_t atlas_code_lengths[4];
static uint16_t atlas_code_ptr = 0;

void atlas_setCode(uint8_t index, const uint8_t* p, uint16_t len) {
	int32_t l = len;
	if (l > sizeof(atlas_code) - atlas_code_ptr) l = sizeof(atlas_code) - atlas_code_ptr;
	if (l < 0) return;

	System_printf("setting atlas code %d to length %d (requested %d)\n",index,l,len);
#ifdef USE_AX5031
	int i;
	for (i=0; i<l; i++) {
	  // reverse MSB to LSB first
	  uint8_t b;
	  b = p[i];
	  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	  atlas_code[atlas_code_ptr+i] = b;
	}
#else
	memcpy(atlas_code + atlas_code_ptr, p, l);
#endif
	atlas_codes       [index] = atlas_code + atlas_code_ptr; //p;
	// TODO: check if the address is in flash and modify only then
	//atlas_codes       [index] = (uint8_t*) (((uint32_t) p) | 0xA0000000); // modify the flash address for the radio core
	atlas_code_lengths[index] = l;
	atlas_code_ptr += l;
}

static slot_action_t atlasSlotAction = {
		.action     = SLOT_ACTION_TX_ADV,
		.opts       = 0,
		.packet     = 0,
		.packet_len = 0,
        .dbm        = 10,
};

slot_action_t* atlasGetSlotAction(uint8_t configuration, uint8_t setup, uint16_t slot) {
	// int8_t dbm  = ( configurations[configuration][SETUP_DBMO(setup)]     & 0xFF);
    uint8_t opts        = ((configurations[configuration][SETUP_OPTS(setup)]) & 0xFF);
	uint8_t index       = opts & VH_TAGSTATE_ATLAS_CODE_INDEX_MASK;
	uint8_t invert      = (opts & VH_TAGSTATE_ATLAS_INVERTED)    != 0;
	uint8_t random_sign = (opts & VH_TAGSTATE_ATLAS_RANDOM_SIGN) != 0;

	//System_printf("atlas code %d len %d invert %d\n",index,atlas_code_lengths[index],invert);
	atlasSlotAction.packet = atlas_codes[index];
	atlasSlotAction.packet_len = atlas_code_lengths[index];
    atlasSlotAction.opts = opts & (~VH_TAGSTATE_ATLAS_CODE_INDEX_MASK);

	atlasSlotAction.dbm = configurations[configuration][SETUP_DBM(setup)];

	System_printf("atlas code %02x %02x %02x len %d opts %02x\n",
			(atlasSlotAction.packet)[0],
			(atlasSlotAction.packet)[1],
			(atlasSlotAction.packet)[2],
			atlas_code_lengths[index],atlasSlotAction.opts);

	return &atlasSlotAction;
}

/*****************************************************************************/
/* PROTOCOL HANDLERS: VILDE HAYE                                             */
/*****************************************************************************/
#include "vildehaye.h"
#include "random.h"

//uint8_t vildehayeIdPacket[] = { 0x62, 0x80, 0xc8, 0xe8, 0x03, 0x60, 0x09, 0xd6, 0xd7, 0x3b, 0x25, 0x4d, };
uint8_t vildehayePayload[256]; // TODO mark it as in use or not, or switch to buffers

static slot_action_t vildehayeSlotAction = {
		.action     = SLOT_ACTION_TX,
		.opts       = 0,
		.packet     = vildehayePayload, // vildehayeIdPacket,
		.packet_len = 12,
		.dbm        = 10,
};

static vildehaye_packet_t vildehayePacket;
//static uint8_t            vildehayePacketValid = 0;
static uint8_t            vildehayeInSession   = 0;
static uint32_t           vildehayeSeqno       = 0; // last sequence number we transmitted
static uint8_t            vildehayeRetransmissions;

typedef enum {
	SYN = 0,
	ACK = 1,
	NACK = 2,
	UNKNOWN = 3,
} vildehayeProtocolState;

uint8_t tag_sessionSeqno(uint32_t seqno) {
	vildehayeProtocolState state = UNKNOWN;

	if (!vildehayeInSession) state = SYN;
	if ( vildehayeInSession && seqno == vildehayeSeqno+1) state = ACK;
	if ( vildehayeInSession && seqno == vildehayeSeqno-1) state = NACK;

	switch (state) {
	case UNKNOWN:
		// wrong response, ignore
		break;
	case SYN:
		vildehayeInSession = 1;
		vildehayeSeqno = seqno + 1;
		vildehayeRetransmissions = 0;
		break;
	case ACK:
		// continue with the protocol
		break;
	case NACK:
		break;
	}

	if (state==SYN) {
	} else {

	}
	return 0; // incorrect seq number
}

//uint32_t xxxI2CTest;

#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
//static uint8_t loggerStateCountdown = 0;
static uint32_t loggerTimeOfLastStateReport = 0;
#endif

static void vildehayeCreateBeaconPacket(uint16_t opts, uint16_t period) {
	uint8_t flags = opts & 0xFF;
	flags |= (currentConfiguration & 0x03); // we can announce configurations 0 to 3
#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
	if (logItemLength != 0xFF                                        // we have data to send
	    && loggerQueueSize() >= 4096) // actually a lot of data (for testing)
	  flags |= VH_TAGSTATE_HAS_DATA;
#endif
	System_printf("bcn flags %02x %d\n",flags,currentConfiguration);
	vildehayeInitPacket(&vildehayePacket, vildehayePayload, 0, 256);
	vildehayeAddTagState(&vildehayePacket, flags, 256-8, period); // tag state
	vildehayeAddUIntWithHeader(&vildehayePacket, VH_SOURCE_ID, tagId);

    if ((opts & VH_OPTIONS_BEACON_WAKEUP_MASK) != 0) {
		//System_printf("beaconing wakeup %d\n", (opts & VH_OPTIONS_BEACON_WAKEUP_MASK) >> 8);
		vildehayeAddUIntWithHeader(&vildehayePacket, VH_WAKEUP, (opts & VH_OPTIONS_BEACON_WAKEUP_MASK) >> 8);
	}

	uint32_t now = Seconds_get();
	if ((opts & VH_OPTIONS_BEACON_CLOCK) != 0) {
        //System_printf("beaconing clock\n");
		vildehayeAddHeader (&vildehayePacket, VH_LOCAL_CLOCK, 4); // local clock
		vildehayeAddUInt32 (&vildehayePacket, now);
		//vildehayeAddUInt32 (&vildehayePacket, xxxI2CTest);
	}

  if ((opts & VH_OPTIONS_BEACON_SECS_SINCE_WAKEUP) != 0) {
      //System_printf("beaconing sec since\n");
    vildehayeAddHeader (&vildehayePacket, VH_SECS_SINCE_WAKEUP, 4); // local clock
    vildehayeAddUInt32 (&vildehayePacket, (Seconds_get() - lastWakeupSecs));
  }

#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
  if (((opts & VH_OPTIONS_BEACON_CLOCK) != 0) // beaconing is allowed
      && ((opts & VH_TAGSTATE_SESSION_OK)==0) // dowloads are now (will be handled below)
      && ((now - loggerTimeOfLastStateReport) >= 60)) {
    loggerTimeOfLastStateReport = now;
    vildehayeAddHeader (&vildehayePacket, VH_LOGGER_STATE, 1+3*4);
    vildehayeAddUInt8  (&vildehayePacket, 4); // 4-byte elements, unsigned (signed has 0x80 bit set)
    vildehayeAddUInt32 (&vildehayePacket, nvmSize);
    vildehayeAddUInt32 (&vildehayePacket, loggerFreeFlashPointer);
    vildehayeAddUInt32 (&vildehayePacket, loggerCurrentItemAddress);
  }
#endif

	//System_printf("exiting\n");
	//System_exit(1);
#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
	if (opts & VH_TAGSTATE_SESSION_OK) {
	  //loggerStateCountdown++;
	  if ((now - loggerTimeOfLastStateReport) > 60) { // (loggerStateCountdown>=10) {
	    vildehayeAddHeader (&vildehayePacket, VH_LOGGER_STATE, 1+3*4);
	    vildehayeAddUInt8  (&vildehayePacket, 4); // 4-byte elements, unsigned (signed has 0x80 bit set)
	    vildehayeAddUInt32 (&vildehayePacket, nvmSize);
	    vildehayeAddUInt32 (&vildehayePacket, loggerFreeFlashPointer);
	    vildehayeAddUInt32 (&vildehayePacket, loggerCurrentItemAddress);
	    //vildehayeAddUInt32 (&vildehayePacket, logItemLength);
	    //vildehayeAddUInt32 (&vildehayePacket, logItemAddress);
	    //vildehayeAddUInt32 (&vildehayePacket, logItemType);
	    loggerTimeOfLastStateReport = now;
	    //loggerStateCountdown = 0;
	  } else {
	    if (logItemLength != 0xFF) {
        //vildehayeAddHeader (&vildehayePacket, VH_LOG_ITEM, 4+1+logItemLength);
        vildehayeAddHeader (&vildehayePacket, VH_LOG_ITEM_NEW, 4+4+1+logItemLength);
        vildehayeAddUInt32 (&vildehayePacket, loggerLogCreationTime);
        vildehayeAddUInt32 (&vildehayePacket, logItemAddress);
	      vildehayeAddUInt8  (&vildehayePacket, logItemType);
	      int i;
	      for (i=0; i<logItemLength; i++) // memcpy and incrementing the length would be more efficient, but more invasive
	        vildehayeAddUInt8  (&vildehayePacket, logItem[i]);
	    }
	  }
	}
#endif // USE_VH_LOGGER
    //System_printf("beacon l=%d\n",vildehayePacket.length);
}

slot_action_t* vildehayeGetSlotAction(uint8_t configuration, uint8_t setup, uint16_t slot) {

	//uint16_t count  = configurations[configuration][SETUP_LIMIT(setup)];
	uint16_t step   = configurations[configuration][SETUP_STEP(setup)];
	uint16_t start  = configurations[configuration][SETUP_START(setup)];

	 uint8_t dbm  = configurations[configuration][SETUP_DBM(setup)];
	uint16_t opts = configurations[configuration][SETUP_OPTS(setup)];

	//System_printf("==> %04x\n",opts);

	uint16_t period = tagPeriodMs * step;

	uint16_t parity = ((slot-start)/step) & 0x0001;

	vildehayeSlotAction.dbm = dbm;

	if (opts & VH_TAGSTATE_LISTEN_ONLY) {
		vildehayeSlotAction.action = SLOT_ACTION_RX;
		return &vildehayeSlotAction;
	}

	if ( (opts & VH_OPTIONS_BEACON_RANDOM_SLOT)) {
		uint16_t frameLen = configurations[configuration][CONF_FRAME_LEN];
		// transmit probability is 1/frameLen
		int32_t r = random() % frameLen; // between 0 and frameLen-1
		//System_printf("random %d expectation %d\n",r,frameLen);
		// now do it (or skip)
		if (r<=0) {
			System_printf("!\n");
			vildehayeCreateBeaconPacket(opts, period);

			vildehayeSlotAction.packet_len = vildehayePacket.length;
			vildehayeSlotAction.action = SLOT_ACTION_TX;
		} else {
			vildehayeSlotAction.action = SLOT_ACTION_NOP;
		}
		return &vildehayeSlotAction;
	}

	//System_printf("*** *** conf %d setup %d period %d slot %d parity %d\n",configuration,setup,period,slot,parity);

	uint8_t reasonToReceive = 0;
	//reasonToReceive |= (opts & VH_TAGSTATE_SESSION_OK);
	//reasonToReceive |= (opts & VH_TAGSTATE_NON_SESSION_CMD_OK);

	if (parity==0 || !reasonToReceive) {
		vildehayeCreateBeaconPacket(opts, period);

		//vildehayeInitPacket(&vildehayePacket, vildehayePayload, 0, 256);
		//vildehayeAddTagState(&vildehayePacket, opts & 0xFF, 256-8, period); // tag state
		//vildehayeAddUIntWithHeader(&vildehayePacket, VH_SOURCE_ID, tagId);
		//vildehayeAddHeader (&vildehayePacket, VH_LOCAL_CLOCK, 4); // local clock
		//vildehayeAddUInt32 (&vildehayePacket, Seconds_get());

		vildehayeSlotAction.packet_len = vildehayePacket.length;
		vildehayeSlotAction.action = SLOT_ACTION_TX;
	} else {
		vildehayeSlotAction.action = SLOT_ACTION_RX;
	}
	return &vildehayeSlotAction;
}

//void vildehayeHandlePacket(uint8_t setup, uint8_t packet, uint16_t len) {
//}

/*****************************************************************************/
/* GLOBAL DEFINITIONS                                                        */
/*****************************************************************************/

// #define PACKET_INTERVAL     (uint32_t)(4000000*5.0f) /* Set packet interval to 5000ms */

/***** Variable declarations *****/
//static RF_Object rfObject;
//static RF_Handle rfHandle;

#if 0
//uint32_t time;
//static uint8_t packet[PAYLOAD_LENGTH];
#define ID_PACKET_LENGTH   16
#define COMM_PACKET_LENGTH 16
//static uint8_t id_packet[ID_PACKET_LENGTH+1] =     { ID_PACKET_LENGTH, 2, 3   };
#define ID_PACKET_LENGTH   12
static uint8_t id_packet[] = { ID_PACKET_LENGTH, 0x62, 0x80, 0xc8, 0xe8, 0x03, 0x60, 0x09, 0xd6, 0xd7, 0x3b, 0x25, 0x4c, };
static uint8_t comm_packet[COMM_PACKET_LENGTH+1] = { COMM_PACKET_LENGTH, 2, 3 };
//const uint8_t packet[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
//static uint16_t seqNumber;


typedef struct slot_st {
	int8_t   setup;
	uint8_t* packet;
	uint16_t packet_length;
} slot_t;

slot_t slots[] = {
	{ RADIO_SETUP_TOA,  atlas_code , 1024                 },
	{ RADIO_SETUP_ID,   id_packet  , ID_PACKET_LENGTH  +1 },
	{ RADIO_SETUP_COMM, comm_packet, COMM_PACKET_LENGTH+1 },
	{ -1 },
};

uint8_t slot_index = 0;

uint8_t comm_state = 0; // tx

#endif

/*****************************************************************************/
/* PERIODIC WAKEUPS                                                          */
/*****************************************************************************/

//uint8_t tagPeriod = 1;

#ifdef SEMWAIT
Semaphore_Handle wakeupSemaphore;

static void wakeupFunction(UArg arg) {
	Semaphore_post(wakeupSemaphore);
}
#endif

#define UNKNOWN 99
#define TX_OK   0
#define TX_FAIL 1
#define RX_OK   2
#define RX_FAIL 3
#define RX_TIMEOUT 4

/*****************************************************************************/
/* Sensing and Logging Scheduler                                             */
/*****************************************************************************/

#ifdef USE_VH_SENSORS
static uint16_t sensorsPeriodCounter = 0;
static uint16_t sensorsPeriodNext    = 0;
#endif

/*****************************************************************************/
/* inversions logging                                                        */
/*****************************************************************************/

/*
 * Inversions are logged with a header that specifies the starting period counter,
 * and the period counter at a particular wall-clock second.
 */
// inversions buffer contains time stamp and 16 32-bit words (64 bytes)
#ifdef USE_VH_SENSORS

#ifdef INVERSIONS_BY_PERIOD
static uint32_t inversionsBuffer[1+16];
static  int8_t* inversions = (int8_t*) &(inversionsBuffer[1]);
static uint8_t  inversionsPointer     = 0;
static uint16_t inversionsNextPeriod  = 0;
static uint8_t  inversionsInitialized = 0;

static void tagLogInversionsRestart(int8_t invert, uint16_t periodCounter) {
    inversionsBuffer[0] = Seconds_get();
    memset(inversions, 0, 64);
    inversions[0] = invert;
    inversionsPointer = 1;
    inversionsNextPeriod = periodCounter + 1;
    inversionsInitialized = 1;
}

static void tagLogInversions(int8_t invert, uint16_t periodCounter) {
  if (!inversionsInitialized) {
      tagLogInversionsRestart(invert, periodCounter);
      return;
  }

  while (inversionsNextPeriod != periodCounter) {
      inversionsPointer++;
      inversionsNextPeriod++;

      if (inversionsPointer==64) {
          // log old data
          loggerLog(CATALOG_DATATYPE_TX_INVERSIONS, (uint8_t*) inversionsBuffer, sizeof(uint32_t)*(1+16));
          // start new buffer; no need to log all the intermediate zeros
          tagLogInversionsRestart(invert, periodCounter);
          return;
      }
  }

  inversions[ inversionsPointer ] = invert;
  inversionsPointer++;
  inversionsNextPeriod++;

  if (inversionsPointer==64) {
      loggerLog(CATALOG_DATATYPE_TX_INVERSIONS, (uint8_t*) inversionsBuffer, sizeof(uint32_t)*(1+16));
      inversionsInitialized = 0; // the next call will initialize
      return;
  }
}
#else
static uint32_t inversionsBuffer[1+16];
static  int8_t* inversions = (int8_t*) &(inversionsBuffer[1]);
static uint8_t  inversionsPointer     = 0;
static uint8_t  inversionsInitialized = 0;

static void tagLogInversionsRestart(int8_t invert, uint16_t periodCounter) {
    inversionsBuffer[0] = Seconds_get();
    memset(inversions, 0, 64);
    inversionsPointer = 0;
    inversionsInitialized = 1;
}

static void tagLogInversions(int8_t invert, uint16_t periodCounter) {
  if (!inversionsInitialized) tagLogInversionsRestart(invert, periodCounter);

  inversions[ inversionsPointer ] = invert;
  inversionsPointer++;

  if (inversionsPointer==64) {
    // log old data
    loggerLog(CATALOG_DATATYPE_TX_INVERSIONS, (uint8_t*) inversionsBuffer, sizeof(uint32_t)*(1+16));
    // start new buffer
    tagLogInversionsRestart(invert, periodCounter);
  }
}
#endif

#endif // USE_VH_LOGGER

/*****************************************************************************/
/* VOLTAGE THRESHOLD                                                         */
/*****************************************************************************/

//static uint32_t voltageThreshold = 0;
//void tag_setVoltageThreshold( uint32_t vth ) { voltageThreshold = vth; }

/*************************************************************************************/
/* NEW TAG TASK FUNCTION (with LVS waits)                                            */
/*************************************************************************************/

/*
 * Auxiliary wait function
 */

#include "scif-low-voltage-suspender.h"

static uint32_t maxLVSDelayTicks = 0;
static uint32_t tickAtLastAction;
static uint32_t targetTick;
static uint32_t earlyTick;
static uint32_t maxEarlyActionTicks;

static int32_t t1,t2,t3,t4,t5,t6,t7,t8,t9;
static float   f1;

static uint8_t powerPolicy = 0;
static float   voltageThreshold;
static uint8_t dropCount = 0;

void suspenderSetPolicy(const uint8_t* data, uint16_t length) {
  powerPolicy = data[0];
  /*
   * Power policies:
   * 0    timed waits only; default if policy is not set explicitly
   * 1    timed waits followed by waiting for a fixed threshold
   * 2    timed waits followed by waiting for an adaptive threshold
   */
  if (powerPolicy == 1 || powerPolicy == 2)
    voltageThreshold = 0.02f * (data[1]); // units are 0.02 volts, to allow voltages up to about 5V.

}

static void suspenderInit() {
  System_printf("### power policy %d threshold %f\n",powerPolicy,voltageThreshold);

  lvsInit();
  tickAtLastAction = Clock_getTicks();
}

static void suspenderReset() {
  tickAtLastAction = Clock_getTicks();
}

static uint16_t suspenderSuspend(uint32_t periods, uint32_t periodInClockTicks, uint32_t earlyByInClockTicks) {
  t1 = tickAtLastAction;
  targetTick = tickAtLastAction + (periods*periodInClockTicks);
  tickAtLastAction = targetTick;
  earlyTick  = targetTick - earlyByInClockTicks;

  t2 = targetTick;
  t3 = earlyTick;

  uint32_t tick = Clock_getTicks();
  int32_t wait = earlyTick - tick - maxLVSDelayTicks;

  if (wait<0) {
    System_printf("warning negative wait %d\n",wait);
    return 1; // as if we were suspended; this tells the scheduler to restart
  }

  t4 = tick;
  t5 = wait;
  f1 = wait*Clock_tickPeriod*1e-6f;
  //System_printf("suspender req delay %d %f\n",wait,wait*Clock_tickPeriod*1e-6f);
  // what to do if delay < 0?
  uint16_t suspension = lvsWait(wait*Clock_tickPeriod*1e-6f, ( powerPolicy==0 ? NAN : voltageThreshold ));
  //uint16_t suspension = lvsWait(wait*Clock_tickPeriod*1e-6f, 2.6f);
  uint32_t tock = Clock_getTicks();

  if (powerPolicy == 2) {
    if (dropCount < 100) dropCount++;
    else if (dropCount == 100) {
      dropCount++; // so that we don't do this again
      // we have seen 100 drops, so we can be reasonably confident that this is the largest drop we'll see for a while
      System_printf("### Vthreshold was %f ",powerPolicy,voltageThreshold);
      voltageThreshold = 1.8f + 0.1f + lvsMaxDrop();
      System_printf("now %f\n",voltageThreshold);
   } else {
      // reduce the threshold if necessary; as we go along, the drops might get larger because the cap's voltage drops
      float newThreshold = 1.8f + 0.1f + lvsMaxDrop();
      if (newThreshold > voltageThreshold) voltageThreshold = newThreshold;
    }
  }

  t6 = tock;

  if (suspension != 0) return suspension; // for now we only report the suspension and assume the caller will call reset

  int32_t actualWait = (tock-tick);
  int32_t delay = actualWait - wait;
  if (delay <= 0 || delay>periodInClockTicks) delay = 0;
  if (delay > maxLVSDelayTicks) {
    maxLVSDelayTicks = delay;
  }

  // sleep some more if we are early
  int32_t earlyBy = earlyTick - tock;
  //System_printf(": %d\n",earlyBy);
  if (earlyBy > 0) Task_sleep( earlyBy );

  earlyTick = Clock_getTicks();
  //System_printf("suspender s ret\n");
  t7 = earlyTick;

  return suspension;
 }

static int32_t suspenderAlign() {
  //System_printf("suspender a start\n");
  uint32_t now = Clock_getTicks();

  t8 = now;

  int32_t earlyActionTicks = now - earlyTick;
  int32_t earlyBy          = targetTick - now;
  if (earlyActionTicks >= 0 && earlyActionTicks > maxEarlyActionTicks) maxEarlyActionTicks = earlyActionTicks;

  if (earlyBy > 0) {
    while (Clock_getTicks() < targetTick);
  }

  t9 = Clock_getTicks();
  //System_printf("%f %d %d %d %d %d %d %d %d %d\n",f1,t1,t2,t3,t4,t5,t6,t7,t8,t9);

  return earlyBy;
}
/*
   uint32_t now = Clock_getTicks();

   System_printf("suspender tgt %d now %d wait %d delay %d,%f max %d,%f extra %d\n",
       targetTick, now,
       wait,
       delay,      (float) (delay*Clock_tickPeriod),
       maxLVSDelay,(float) (maxLVSDelay*Clock_tickPeriod),
       extra);
   leds_blink(LEDS_RX,1);
 }
 */
 //sensorcontroller_ack(); // tell the sensor controller that the system CPU is going down

#include "i2c_sensors.h"
//extern uint16_t periods;
//extern uint16_t compb;
//extern uint16_t adc;
//uint32_t clockPrev = 0;

static int debugWatchdogCounter = 0;

static void tagTaskFunction(UArg arg0, UArg arg1) {

  //while (1) {
  //  System_printf("1234\n");
  //  Task_sleep(100000);
  //}

#ifdef USE_HOOKS
  hooksInit();
#endif

  Task_sleep(4*1000000/Clock_tickPeriod); // 4s wait to let the battery recover

  /*
   * We start the log first, because when the configuration packet
   * configures sensors, the code logs the sensor configuration.
   *
   * Note that this function may never return, if the host
   * establishs a UART connection with the tag.
   */

#ifdef USE_VH_LOGGER
  loggerInit();

  if (loggerState == LOGGER_LOGGING || loggerState == LOGGER_FLASH_IS_FULL) {
    // all okay, continue starting up
  } else {
    leds_on(LEDS_TX);
    while (1) {
        System_printf("logger tag but no working log, halted\n");
        consoleFlush();
        Task_sleep(4000000/Clock_tickPeriod);
    }
  }
#endif

#ifdef USE_VESPER
  //leds_blink(LEDS_TX,1);
  //Task_sleep(500000/Clock_tickPeriod);
  //leds_blink(LEDS_TX,1);
  //Task_sleep(500000/Clock_tickPeriod);
  //leds_blink(LEDS_TX,1);
  //Task_sleep(500000/Clock_tickPeriod);
  //leds_blink(LEDS_TX,1);
  //Task_sleep(500000/Clock_tickPeriod);
  //leds_blink(LEDS_TX,1);

  loggerInit();
#endif

#ifdef USE_VH_SENSORS
  System_printf("calling sensorsInit\n");
  consoleFlush();
  Task_sleep(500000/Clock_tickPeriod);
  sensorsInit();
#endif


  // shoud move after the configuration!
#ifdef USE_AX5031_NOW_INTEGRATED
  ax5031Init();
  while (1) {
    uint32_t t1,t2,t3;
    t1 = Clock_getTicks();
    if (ax5031Prepare() != 0) continue;
    t2 = Clock_getTicks();
    ax5031Transmit(atlas_code[0], 1024);
    t3 = Clock_getTicks();
    ax5031Shutdown();
    System_printf("t %d %d\n",t2-t1,t3-t2);
    Task_sleep(1000000/Clock_tickPeriod);
  }
#endif

  // the battery-recovery delay used to be here.

  radioInit();

  /*
   * Read and process the configuration packet.
   */
#ifdef DeviceFamily_CC13X0
  uint8_t* cdata = (uint8_t*) 0x0001e000; // configuration page
#endif
#ifdef DeviceFamily_CC13X2
  uint8_t* cdata = (uint8_t*) 0x00054000; // configuration page
#endif
  uint16_t length = *((uint16_t*) cdata);
  if (length == 0xFFFF) {
    //System_printf("configuration data missing, halting\n");
    leds_on(LEDS_TX);
    System_abort("configuration data missing, halting\n");
  } else {
    System_printf("configuration length %d\n",length);
    consoleFlush();
    Task_sleep(500000/Clock_tickPeriod);

    uint16_t len = vildehayeHandlePacketNaked(cdata+2, length);
    //I2CSensorsSemaphore_post();
    flashSetConfigurationLimit(len);
    uint8_t c = flashGetLastConfiguration();
    if (c != 255) tag_gotoConfiguration(c);
    //flashWriteLastConf(1); // for testing only
  }

  randomSetState((uint32_t) tagId);

  System_printf("tag %d starting (lower digits)\n",(uint32_t) (tagId%1000000ll));

#ifdef USE_VH_LOGGER
  loggerLog(LOGGER_DATATYPE_LOG_BOOT_MARKER, NULL, 0);
  loggerLog(CATALOG_DATATYPE_TAG_ID, (uint8_t*) &tagId, sizeof(tagId));
  loggerCommit();
#endif

  System_printf("period = %d\n", tagPeriodMs);

  Types_FreqHz tsf;
  Timestamp_getFreq(&tsf);

  //int8_t previousSetup = -1;
  uint8_t radioOpen = 0;

  /*
   * New scheduling code
   */

  int32_t earlyBy = 0;

  suspenderInit();

  // we run the suspender 4 times to get delay stats
  //System_printf("scheduler now %d\n",Clock_getTicks());
  const uint32_t syncWaits = 250000;
  //const uint32_t syncWaits = 1000000;
  int i;
  for (i=0; i<4; i++) {
    // add 100us for the ealy-action estimate
    suspenderSuspend(1, syncWaits/Clock_tickPeriod, maxEarlyActionTicks+100/Clock_tickPeriod);
    Task_sleep( 1000 / Clock_tickPeriod ); // simulate a 1ms early action
    int32_t earlyBy = suspenderAlign();
    uint32_t now = Clock_getTicks();
    //System_printf("scheduler now %d earlyBy %d maxs %d %d\n",now,earlyBy,maxLVSDelayTicks, maxEarlyActionTicks);
  }
  System_printf("Suspender done\n");

  //tagPeriodMs = 5000; // xxx just for testing suspension

  while (1) {

      debugWatchdogCounter++;
      if (debugWatchdogCounter > 10) break;

#ifdef HALL_SENSOR
    if (Semaphore_getCount(scStopSemaphore)==1) { // we need to stop
      Semaphore_pend(scStopSemaphore, BIOS_NO_WAIT);

      //System_printf("we are told to go to deep sleep\n");
      if (radioOpen) RF_close(rfHandle);
      radioOpen = 0;
      rat_known = 0; // the timer will be useless when we wake up.
      sensorcontroller_ack(); // tell the sensor controller that the system CPU is going down
      Semaphore_pend(scStartSemaphore, BIOS_WAIT_FOREVER); // wait for wake up.

      sensorcontroller_ack(); // tell the sensor controller that the system CPU is active

      //System_printf("woke up!\n");
    }
#endif

#ifdef NO_LONGER_USED
    watchdog_pacify();
#endif

    uint32_t waitPeriods = 1; // how many periods do we need to wait until the next action

    while ((slotRadioSetup == -1) || (slotActionPtr -> action)==SLOT_ACTION_NOP) {
#ifdef USE_VH_SENSORS
      // Sivan July 2018 allow for sensing and logging
      if (sensorsPeriodNext == sensorsPeriodCounter) {
          System_printf("sensors nop sense\n");
          break;
      }
      sensorsPeriodCounter++;
#endif

      waitPeriods++;

      scheduleAdvance();
      continue;
    }

    // before suspending operations, we turn off the watchdoc pacifier
#ifdef WATCHDOG_PACIFIER_PIN
    watchdog_pacify_stop();
#endif


    /*
     * New scheduling, now wait.
     */

    {
      uint32_t suspensionTimes[2];
      suspensionTimes[0] = Seconds_get();
      System_printf("scheduler suspending for %d (%f): last earlyBy %d maxs %d %d\n",
          waitPeriods, waitPeriods*tagPeriodMs*1e-3,
          earlyBy,maxLVSDelayTicks, maxEarlyActionTicks);
      // add 100us for the ealy-action estimate

      // going to sleep, flush console first
      consoleFlush();

      uint16_t suspension = suspenderSuspend(waitPeriods, (tagPeriodMs*1000)/Clock_tickPeriod, maxEarlyActionTicks+100/Clock_tickPeriod);
      if (suspension != 0) {
        // we got suspended; we skip this slot and continue; we do not try to align the schedule in time.
        // needs correction for sensors!
        System_printf("suspension! restarting schedule\n");
        suspenderReset();

#ifdef USE_VH_LOGGER
        suspensionTimes[1] = Seconds_get();
        loggerLog(CATALOG_DATATYPE_LVS_SUSPENSION, (uint8_t*) suspensionTimes, 2*sizeof(uint32_t));
        loggerCommit();
#endif

        goto slotWrapUp;
      }
    }

    //System_printf("!!\n");
    //consoleFlush();
    //System_printf("setup %d\n",slotRadioSetup);
    //consoleFlush();
    //System_printf("opts %d\n",slotActionPtr -> opts);
    //consoleFlush();

#ifdef WATCHDOG_PACIFIER_PIN
    watchdog_pacify();
#endif


    int8_t invert = 0; // 0 means not an invrsion/no-inversion slot
    if (slotRadioSetup!=-1) {
      radioOpen = 1;

      int inv = RADIO_IDENTITY_BITS;

      if (((slotActionPtr -> opts) & VH_TAGSTATE_ATLAS_INVERTED) != 0) {
        //System_printf("!1 %0x %d %d\r\n",slotActionPtr->opts,inv,invert);
        inv = RADIO_INVERT_BITS;
      }
      if (((slotActionPtr -> opts) & VH_TAGSTATE_ATLAS_RANDOM_SIGN) != 0) {
        //System_printf("!2 %0x %d %d\r\n",slotActionPtr->opts,inv,invert);
        invert = (random() & 0x04) != 0; // pick a bit in a random word
        if (invert==0) invert=-1; // a marker for the logging code that's invoked after transmission.
        if (invert==1) inv = RADIO_INVERT_BITS;
        else           inv = RADIO_IDENTITY_BITS;
      }

      // Sivan new Nov 2021
      //radioSetup_txPower(slotRadioSetup, slotActionPtr->dbm);

      //System_printf("!3 %0x %d %d\r\n",slotActionPtr->opts,inv,invert);
      //inv = RADIO_IDENTITY_BITS; // XXX for testing
      //System_printf("tag pre %d %d %d %d\n",slotRadioSetup,inv,((slotActionPtr->action) == SLOT_ACTION_TX) || ((slotActionPtr->action) == SLOT_ACTION_TX_ADV) ? 1 :0,slotActionPtr->dbm);
      //consoleFlush();
      radioPrepare(slotRadioSetup, inv,
                   (((slotActionPtr->action) == SLOT_ACTION_TX) || ((slotActionPtr->action) == SLOT_ACTION_TX_ADV)) ? 1 :0 ,
                   slotActionPtr->dbm);
      //System_printf("!2\n");

    }

    //uint32_t rt2 = Timestamp_get32();

    //rfc_propRxOutput_t rxout;

    uint8_t action = slotActionPtr -> action;

    earlyBy = suspenderAlign();
    //System_printf(".1\n");

    // we allow 1ms jitter */
    if (earlyBy < 0 && earlyBy < -(1000/Clock_tickPeriod)) {
      action = SLOT_ACTION_NOP;
      System_printf("Suspender late by %d %dus, comp %d skipping the radio action for the slot\n",
          earlyBy,earlyBy*Clock_tickPeriod,
          ((earlyBy*Clock_tickPeriod) > 1000)
          );
    }

    //System_printf("!\n");
    int txResultCode, rxResultCode;

    switch (action) {
    case SLOT_ACTION_NOP:
      radioShutdown();
      System_printf("cmd_nop\n"); // sivan Sep 2019
      break;
    case SLOT_ACTION_RX:
      rxResultCode = radioReceiveMessage(RECEIVE_TIME_LIMIT_US);
      if (rxResultCode == RADIO_SUCCESS) {
        System_printf("rx ok\n");
#ifdef LEDS_RX
        leds_blink(LEDS_RX, 1);
#endif
        receiveGetBuffer();
        {
            uint8_t* rxpacket = buffers[ incomingBuffer.id ];
            uint8_t payloadLength = rxpacket[1];
            uint8_t tsp = 1+1+payloadLength+1;
            uint32_t rx_ts;
            rx_ts =  rxpacket[tsp] | (rxpacket[tsp+1]<<8) | (rxpacket[tsp+2]<<16) | (rxpacket[tsp+3]<<24);
            vildehayeHandlePacketNaked(rxpacket+2,payloadLength);
            Mailbox_post(freeMailbox, &incomingBuffer, BIOS_WAIT_FOREVER);
            //System_printf("rxout status=%04x ts=%lu %d %lu\n",radio_cmd_prop_rx[slotRadioSetup].status,rx_ts,rxout.nRxOk, rxout.timeStamp);
        }
      } else {
        System_printf("rx err or no data\n");
#ifdef LEDS_RX
        leds_blink(LEDS_RX, 2);
#endif
      }

      break;

      // else we are in a transmit state, just go to transmit case
    case SLOT_ACTION_TX:
      t1 = Clock_getTicks();
      txResultCode = radioTransmit(slotActionPtr->packet, slotActionPtr->packet_len, RADIO_SCHEDULE_NOW, 0, 0);
      if (txResultCode == RADIO_SUCCESS) {
        System_printf("tx ok\n");
        System_printf("t %d\n",(Clock_getTicks()-t1)*Clock_tickPeriod);
        leds_blink(LEDS_TX, 1);
      } else {
        System_printf("tx err\n");
        //leds_blink(LEDS_TX, 2);
      }

      /*
       * Sivan: Immediate responses, July/Aug 2018
       */

      if (txResultCode == RADIO_SUCCESS) {
        radioPrepare(slotRadioSetup, RADIO_IDENTITY_BITS, 0 /* receive */, RADIO_DBM_KEEP);
        rxResultCode = radioReceiveMessage(RECEIVE_TIME_LIMIT_US); // receive for 10ms
        if (rxResultCode == RADIO_SUCCESS) {
          System_printf("rx ok\n");
          System_printf("t %d\n",(Clock_getTicks()-t1)*Clock_tickPeriod);
#ifdef LEDS_RX
          leds_blink(LEDS_RX, 1);
#endif
          receiveGetBuffer();
          {
              uint8_t* rxpacket = buffers[ incomingBuffer.id ];
              uint8_t payloadLength = rxpacket[1];
              uint8_t tsp = 1+1+payloadLength+1;
              uint32_t rx_ts;
              rx_ts =  rxpacket[tsp] | (rxpacket[tsp+1]<<8) | (rxpacket[tsp+2]<<16) | (rxpacket[tsp+3]<<24);
              vildehayeHandlePacketNaked(rxpacket+2,payloadLength);
              Mailbox_post(freeMailbox, &incomingBuffer, BIOS_WAIT_FOREVER);
              //System_printf("rxout status=%04x ts=%lu %d %lu\n",radio_cmd_prop_rx[slotRadioSetup].status,rx_ts,rxout.nRxOk, rxout.timeStamp);
          }
        } else {
          System_printf("rx err or no data\n");
          System_printf("t %d\n",(Clock_getTicks()-t1)*Clock_tickPeriod);
#ifdef LEDS_RX
          //leds_blink(LEDS_RX, 2);
#endif
        }
      }

      /*
       * End of immediate responses
       */


      break;

      case SLOT_ACTION_TX_ADV:
        //System_printf(".2\n");
        t1 = Clock_getTicks();
        if (radioTransmit(slotActionPtr->packet, slotActionPtr->packet_len, RADIO_SCHEDULE_NOW, 0, 0) == RADIO_SUCCESS) {
          System_printf("tx adv ok\n");
          System_printf("t %d\n",(Clock_getTicks()-t1)*Clock_tickPeriod);
          leds_blink(LEDS_TX, 1);
        } else {
          System_printf("tx adv err\n");
          leds_blink(LEDS_TX, 2);
        }

        break;
    } // switch on what type of slot we are in

    slotWrapUp:

    /*
     * xyz part of new scheduling, close the radio after every slot.
     */

    //if (radioOpen) {
    //  radioShutdown();
    //  radioOpen = 0;
    //}

    /*
     * Call the sensing and logging module, if requested.
     * Both variables wrap around, but this is okay.
     */
#ifdef USE_VH_SENSORS
    if (sensorsPeriodCounter==sensorsPeriodNext) {
      sensorsPeriodNext += sensorsSense(sensorsPeriodCounter);
    }
    sensorsPeriodCounter++;

    System_printf("invert %d\n",invert);
    if (invert!=0) tagLogInversions(invert,sensorsPeriodCounter);
    // invert will get cleared in the next loop anyway.

#endif

#if defined(USE_VH_LOGGER) || defined(USE_VESPER)
    if (logItemLength==0xFF) {
      uint32_t loggerResult = loggerNext(&logItemLength, &logItemType, &(logItem[0]), &logItemAddress);
      if (loggerResult != LOGGER_SUCCESS) {
        logItemLength = 0xFF; // loggerNext might have assigned a value, in particular possibly 0
        //logItemAddress = loggerResult; // for reporting...
        System_printf("LOGGER ERROR: result %d length %d (loggerNext)\n",loggerResult,logItemLength);
      } else {
        System_printf("LOGGER SUCCESS: result %d length %d (loggerNext)\n",loggerResult,logItemLength);
      }
    } else {
#ifdef USE_VESPER
      vesperKeepalive();
#endif
    }
#endif

#ifdef USE_VESPER
    uint8_t wakeup = vesperWakeup();
    if (wakeup != 0xFF) tag_triggerWakeup(wakeup);
    System_printf("VESPER: wakeup %d\n",wakeup);
#endif

#ifdef USE_HOOKS
    hookEndOfSlot(currentConfiguration,slotInFrameIndex);
#endif

    scheduleAdvance();

#if 0
    double t  = (rt0 - prev_time) / (double) tsf.lo;
    double t1 = (rt1 - rt0) / (double) tsf.lo;
    double t2 = (rt2 - rt1) / (double) tsf.lo;
    double t3 = (rt3 - rt2) / (double) tsf.lo;
    double t4 = (rt4 - rt3) / (double) tsf.lo;
    System_printf("ts diff=%.6f RAT diff=%.6f %.6f %.6f %.6f %0.6f slack %d\n",
                  t,(rat_time-rat_prev)/4e6,t1,t2,t3,t4,rat_slack);
    prev_time = rt0;
#endif
  }
}

/*****************************************************************************/
/* TRANSMIT TASK                                                             */
/*****************************************************************************/

#define TAG_TASK_STACK_SIZE 1024

static Task_Params tagTaskParams;
Task_Struct tagTask;    /* not static so you can see in ROV */
static uint8_t tagTaskStack[TAG_TASK_STACK_SIZE];

void tagTask_init() {

	Task_Params_init(&tagTaskParams);
	tagTaskParams.stackSize = TAG_TASK_STACK_SIZE;
	tagTaskParams.priority  = TAG_TASK_PRIORITY;
	tagTaskParams.stack     = &tagTaskStack;
	tagTaskParams.arg0      = (UInt)1000000;

	Task_construct(&tagTask, tagTaskFunction, &tagTaskParams, NULL);
}

#endif // tag firmware
