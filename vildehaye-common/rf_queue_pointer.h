#ifndef RF_QUEUE_H
#define RF_QUEUE_H

#include <stdint.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)

#define RF_QUEUE_POINTER_ELEMENT_SIZE  (sizeof(rfc_dataEntryPointer_t)+sizeof(uint32_t))

extern rfc_dataEntryPointer_t* RFQueue_getDataEntry();
extern uint8_t RFQueue_nextEntry();
extern uint8_t RFQueue_defineQueue(dataQueue_t *dataQueue,
		                        uint8_t *buf,
														uint16_t buf_len,
														uint8_t numEntries,
														uint16_t length);
#endif
