#ifndef LVS_H
#define LVS_H

#include <stdint.h>

#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

extern uint16_t periods;

extern Semaphore_Handle scStartSemaphore;
//extern Semaphore_Handle scStopSemaphore;

uint16_t lvsWait(float wait, float voltageThreshold);
void lvsInit();
float lvsMaxDrop();

#endif
