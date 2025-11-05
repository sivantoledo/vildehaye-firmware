#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdlib.h>
#include <time.h>

#include "config.h"

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/sysbios/family/arm/cc26xx/Seconds.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>

#include "radio_setup.h"
#include "buffers.h"
#include "uart.h"

#include "leds.h"
//#include "board.h"

#include "rf_queue_pointer.h"

/* buffer configuration */
#define DATA_ENTRY_HEADER_SIZE 8   /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             128 /* Max length byte the radio will accept */
//#define NUM_DATA_ENTRIES       2   /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     (1+1+1+4+1)  /* The Data Entries data field will contain:
                                             * 1 element length byte
                                             * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                             * Max 30 payload bytes
                                             * 1 RSSI byte
                                             * 4 timestamp bytes
                                             * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


#define QUEUE_SIZE 2

extern dataQueue_t       dataQueue;
extern buffer_descriptor readyBuffer;
extern buffer_descriptor incomingBuffer;

extern Semaphore_Handle packetReceivedSemaphore;

extern void receiveGetBuffer();
extern void receiveCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
extern void receive_init();

#endif // #ifndef RECEIVE_H
