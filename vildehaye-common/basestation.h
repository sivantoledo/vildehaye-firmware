/*
 * basestation.h
 *
 */

#ifndef BASESTATION_H_
#define BASESTATION_H_

#ifdef USE_BASESTATION

#include <ti/sysbios/knl/Mailbox.h>

Mailbox_Handle packetHandlingMailbox;

extern uint64_t tagId;

void basestationTask_init();
void basestation_gotoSetup(uint8_t c);

#endif // base station firmware

#endif /* BASESTATION_H_ */

