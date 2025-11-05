#ifndef SENSORCONTRLLER_H
#define SENSORCONTRLLER_H

#include <stdint.h>

#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

extern Semaphore_Handle scStartSemaphore;
extern Semaphore_Handle scStopSemaphore;

void sensorcontroller_ack();
void sensorcontroller_init();

#endif
