/*
 * buffers.h
 *
 */

#ifndef BUFFERS_H_
#define BUFFERS_H_

#include "config.h"
#include <ti/sysbios/knl/Mailbox.h>

extern uint8_t buffers[BUFFER_COUNT][BUFFER_SIZE];

typedef struct buffer_descriptor_t {
	uint16_t id;
	uint16_t length;
} buffer_descriptor;

extern Mailbox_Handle freeMailbox;

extern void buffers_init();

#endif /* BUFFERS_H_ */
