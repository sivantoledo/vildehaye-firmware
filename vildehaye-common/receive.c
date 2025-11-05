#include "config.h"

#include <stdlib.h>
#include <time.h>

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

#include "config.h"
#include "radio_setup.h"
#include "buffers.h"
#include "uart.h"

#include "leds.h"
//#include "board.h"

#include "rf_queue_pointer.h"

#include "receive.h"

/* buffer configuration */
//#define DATA_ENTRY_HEADER_SIZE 8   /* Constant header size of a Generic Data Entry */
//#define MAX_LENGTH             128 /* Max length byte the radio will accept */
//#define NUM_DATA_ENTRIES       2   /* NOTE: Only two data entries supported at the moment */
//#define NUM_APPENDED_BYTES     (1+1+1+4+1)  /* The Data Entries data field will contain:
//                                             * 1 element length byte
//                                             * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
//                                             * Max 30 payload bytes
//                                             * 1 RSSI byte
 //                                            * 4 timestamp bytes
//                                             * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if 0
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                         MAX_LENGTH,
                                                                         NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            MAX_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif
#endif

//#define QUEUE_SIZE 2
#if defined(__TI_COMPILER_VERSION__)
  #pragma DATA_ALIGN (rxDataEntryBuffer, 4)
  static uint8_t rxDataEntryBuffer[QUEUE_SIZE*RF_QUEUE_POINTER_ELEMENT_SIZE];
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma data_alignment = 4
  static uint8_t rxDataEntryBuffer[QUEUE_SIZE*RF_QUEUE_POINTER_ELEMENT_SIZE];
#elif defined(__GNUC__)
  static uint8_t rxDataEntryBuffer [QUEUE_SIZE*RF_QUEUE_POINTER_ELEMENT_SIZE] __attribute__ ((aligned (4)));
#else
  #error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
dataQueue_t             dataQueue;
//static rfc_dataEntryPointer_t* currentDataEntry;

buffer_descriptor readyBuffer;
buffer_descriptor incomingBuffer;

Semaphore_Handle packetReceivedSemaphore;
Semaphore_Struct packetReceivedSemaphoreStruct;

void receiveGetBuffer() {
	rfc_dataEntryPointer_t*	currentDataEntry = RFQueue_getDataEntry();

  uint32_t* bufferIdPtr = (uint32_t*) (((uint8_t*) currentDataEntry)+sizeof(rfc_dataEntryPointer_t));
  incomingBuffer.id     = (uint16_t) *bufferIdPtr;
  incomingBuffer.length = *(currentDataEntry->pData) + 1; // 1-byte length at start of buffer, but not including itself

  // replace the data buffer in the queue
  currentDataEntry->pData = buffers[readyBuffer.id];
  *bufferIdPtr = readyBuffer.id;

  RFQueue_nextEntry();

  // mark the ready buffer as not really ready
  //readyBuffer.id = 0xFF;
  Mailbox_pend(freeMailbox, &readyBuffer,  BIOS_WAIT_FOREVER);
}

void receiveCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
	if (e & RF_EventRxEntryDone) {
#if 0
		/* Toggle pin to indicate RX */
		//PIN_setOutputValue(ledPinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));

		//rx_timestamp = Timestamp_get32();
		//Seconds_getTime(&rx_secs);

		/* Get current unhandled data entry */
		rfc_dataEntryPointer_t*	currentDataEntry = RFQueue_getDataEntry();

    uint32_t* bufferIdPtr = (uint32_t*) (((uint8_t*) currentDataEntry)+sizeof(rfc_dataEntryPointer_t));
    incomingBuffer.id     = (uint16_t) *bufferIdPtr;
    incomingBuffer.length = *(currentDataEntry->pData); // 1-byte length at start of buffer

    // replace the data buffer in the queue
    currentDataEntry->pData = buffers[readyBuffer.id];
    *bufferIdPtr = readyBuffer.id;

    // mark the ready buffer as not really ready
    readyBuffer.id = 0xFF;

		/* Copy the payload + the status byte to the packet variable */
		//memcpy(rxpacket, packetDataPointer, (packetLength + 1));
		//memcpy(rxpacket, packetDataPointer, packetLength);
		RFQueue_nextEntry();
#endif
		Semaphore_post(packetReceivedSemaphore);
	}
}

void receive_init() {
	System_printf("RX task creating semaphore\n");

	Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  Semaphore_construct(&packetReceivedSemaphoreStruct, 0, &semParams);
  packetReceivedSemaphore = Semaphore_handle(&packetReceivedSemaphoreStruct);

  Mailbox_pend(freeMailbox, &readyBuffer,  BIOS_WAIT_FOREVER);

  System_printf("Creating queue ready buffer = %d\n",readyBuffer.id);

  if( RFQueue_defineQueue(&dataQueue,
  		                    rxDataEntryBuffer,
													sizeof(rxDataEntryBuffer),
													QUEUE_SIZE,
													MAX_LENGTH + NUM_APPENDED_BYTES)) {
        /* Failed to allocate space for all data entries */
        while(1);
  }

  incomingBuffer.id = 0xFF;
}

