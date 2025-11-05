/******************************************************************************
*  Filename:       rf_queue.h
*  Revised:        $ $
*  Revision:       $ $
*
*  Description:    Help functions for handling queues
*
*  Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdint.h>
#include <stdlib.h>

#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)

#include "config.h"

#include "buffers.h"
#include "rf_queue_pointer.h"


/* Receive entry pointer to keep track of read items */
rfc_dataEntryPointer_t* readEntry;

rfc_dataEntryPointer_t* RFQueue_getDataEntry() {
  return (readEntry);
}

uint8_t RFQueue_nextEntry() {
  readEntry->status = DATA_ENTRY_PENDING;
  readEntry = (rfc_dataEntryPointer_t*)readEntry->pNextEntry;
  return (readEntry->status);
}

//*****************************************************************************
//
//! Define a queue
//!
//! \param dataQueue is a pointer to the queue to use
//! \param buf is the prealocated byte buffer to use
//! \param buf_len is the number of preallocated bytes
//! \param numEntries are the number of dataEntries to split the buffer into
//! \param length is the length of data in every dataEntry
//!
//! \return uint8_t
//
//*****************************************************************************
uint8_t RFQueue_defineQueue(dataQueue_t *dataQueue,
		                        uint8_t *buf,
														uint16_t buf_len,
														uint8_t numEntries,
														uint16_t length) {

  if (buf_len < (numEntries * RF_QUEUE_POINTER_ELEMENT_SIZE)) {
    /* queue does not fit into buffer */
    return (1);
  }

  /* Set the Data Entries common configuration */
  uint8_t *first_entry = buf;
  int i;
  for (i = 0; i < numEntries; i++) {
  	// set up pointer to queue entry
  	buf = first_entry + i * RF_QUEUE_POINTER_ELEMENT_SIZE;

  	// get a buffer
  	buffer_descriptor d;
  	Mailbox_pend(freeMailbox, &d,  BIOS_WAIT_FOREVER);

  	// mark buffer id at end of queue entry
    uint32_t* bufferIdPtr = (uint32_t*) (buf+sizeof(rfc_dataEntryPointer_t));
    *bufferIdPtr = d.id;

    ((rfc_dataEntryPointer_t*)buf)->status        = DATA_ENTRY_PENDING;        // Pending - starting state
    ((rfc_dataEntryPointer_t*)buf)->config.type   = DATA_ENTRY_TYPE_PTR;       // Pointer Data Entry
    ((rfc_dataEntryPointer_t*)buf)->config.lenSz  = 1;                         // No length indicator byte in data
    ((rfc_dataEntryPointer_t*)buf)->length        = BUFFER_SIZE;                  // Total length of data field
    ((rfc_dataEntryPointer_t*)buf)->pData         = buffers[d.id];             // Total length of data field

    //((rfc_dataEntryPointer_t*)buf)->pNextEntry = &(((rfc_dataEntryPointer_t*)buf)->data)+length+pad;
    ((rfc_dataEntryPointer_t*)buf)->pNextEntry = first_entry + (i+1) * RF_QUEUE_POINTER_ELEMENT_SIZE;
  }
  /* Make circular Last.Next -> First */
  ((rfc_dataEntryPointer_t*)buf)->pNextEntry = first_entry;

  /* Create Data Entry Queue and configure for circular buffer Data Entries */
  dataQueue->pCurrEntry = first_entry;
  dataQueue->pLastEntry = NULL;

  /* Set read pointer to first entry */
  readEntry = (rfc_dataEntryPointer_t*)first_entry;

  return (0);
}


