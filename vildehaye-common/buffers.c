#include "config.h"
#include "buffers.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>

uint8_t buffers[BUFFER_COUNT][BUFFER_SIZE];

Mailbox_Handle freeMailbox;

void buffers_init() {
	freeMailbox   = Mailbox_create(sizeof(buffer_descriptor),BUFFER_COUNT,NULL,NULL);

	int i;
	for (i=0; i<BUFFER_COUNT; i++) {
		buffer_descriptor d;
		d.id     = i;
		d.length = 0;
		Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
	}
}

