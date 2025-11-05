/*
 * uartTask.h
 *
 */

#ifndef UARTTASK_H_
#define UARTTASK_H_

#include <stdint.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Mailbox.h>

// Board_UART is defined as 0, we use a high number to request the sensor-controller uart.
//#define SensorController_UART 16

#define FRAMING_SLIP  0
#define FRAMING_UBLOX 1

extern Mailbox_Handle uartTxMailbox;
extern Mailbox_Handle uartRxMailbox;

extern void uartTasks_init(uint32_t uart_index, uint32_t baudRate, uint32_t txMailboxSize, uint8_t framingMode);

//int_fast32_t scuart_write(UART_Handle h, uint8_t* buffer, size_t size);
void uartBlockingWrite(uint32_t uart_index, uint8_t* buffer, size_t size);

#endif /* UARTTASK_H_ */
