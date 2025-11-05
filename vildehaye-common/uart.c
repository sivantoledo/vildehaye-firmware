#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stddef.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#ifdef USE_UART2
#include <ti/drivers/UART2.h>
#endif

#include "config.h"
#include "uart.h"
#include "buffers.h"
#include "leds.h"
/*
#define QUEUE_ELEMENT_COUNT 8

// This structure can be added to a Queue because the first field is a Queue_Elem.
typedef struct uartQueueElement_st {
 Queue_Elem elem;
 uint16_t length;
 uint8_t  data[256];
} uartQueueElement_t;

uartQueueElement_t uartQueueElement[QUEUE_ELEMENT_COUNT];

Queue_Handle uartTxQueue;
Queue_Handle uartRxQueue;
Queue_Handle freeQueue;

*/

#ifdef USE_UART2
typedef UART2_Handle uart_handle_t;
#else
typedef UART_Handle uart_handle_t;
#endif

static uart_handle_t uartHandle = NULL;

Mailbox_Handle uartTxMailbox;
Mailbox_Handle uartRxMailbox;

static Semaphore_Handle uartRxSemaphore;

static UART_ReadFxn  blocking_read;
static UART_WriteFxn blocking_write;

/***********************************************************************/
/* FRAMING                                                             */
/***********************************************************************/

static uint8_t framing = FRAMING_SLIP; // 0 means SLIP, 1 is for GPS

// END is 0xC0 in SLIP
#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static const uint8_t slip_end[] = { SLIP_END };
//static uint8_t slip_esc[]     = { SLIP_ESC };
static const uint8_t slip_esc_end[] = { SLIP_ESC, SLIP_ESC_END };
static const uint8_t slip_esc_esc[] = { SLIP_ESC, SLIP_ESC_ESC };


/***********************************************************************/
/* FUNCTIONS FOR SENSOR-CONTROLLER SOFTWARE UART                       */
/***********************************************************************/

#if defined(USE_BASESTATION) && defined(DeviceFamily_CC13X0)
// use sensor controller for the second uart

#include SensorControllerDirectory(scif.h)

#define BV(n)               (1 << (n))

// Display error message if the SCIF driver has been generated with incorrect operating system setting
#if !(defined(SCIF_OSAL_TIRTOS_H) || defined(SCIF_OSAL_TIDPL_H))
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'TI-RTOS' or 'TI Driver Porting Layer' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Semaphore used to wait for Sensor Controller task ALERT event
//static Semaphore_Struct semScTaskAlert;

void scCtrlReadyCallback(void) {}
void scTaskAlertCallback(void) {
  scifClearAlertIntSource();
  scifUartClearEvents();
  scifAckAlertEvents();

  // Wake up the OS task
  //Semaphore_post(Semaphore_handle(&semScTaskAlert));
  Semaphore_post(uartRxSemaphore);
}

/*
 * In this module, size is always 1, so we ignore it.
 */
#ifdef SCUART_POLLING_WAIT
int_fast32_t scuart_read(uart_handle_t h, uint8_t* buffer, size_t size) {
  while (scifUartGetRxFifoCount()==0) {
    Task_sleep(10000/Clock_tickPeriod);
  }
  *buffer = scifUartRxGetChar();
  return 1;
}
#else
int_fast32_t scuart_read(uart_handle_t h, uint8_t* buffer, size_t size) {
  //int rxFifoCount = scifUartGetRxFifoCount();
  while (scifUartGetRxFifoCount()==0) {
    //System_printf("*1\n");
    Semaphore_pend(uartRxSemaphore, BIOS_WAIT_FOREVER);

    //scifClearAlertIntSource();
    //scifUartClearEvents();
    //scifAckAlertEvents();
    //System_printf("*2\n");
  }
  *buffer = scifUartRxGetChar();
  return 1;
}
#endif

int_fast32_t scuart_write(uart_handle_t h, uint8_t* buffer, size_t size) {
  uint32_t i;
  for (i=0; i<size; i++) {
    while (scifUartGetTxFifoCount()>=SCIF_UART_TX_FIFO_MAX_COUNT) {
      Task_sleep(10000/Clock_tickPeriod);
    }
    scifUartTxPutChar((char) buffer[i]);
  }
  return i;
}

void scuart_init(uint32_t baudRate) {
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);

  // Start the UART emulator
  scifExecuteTasksOnceNbl(BV(SCIF_UART_EMULATOR_TASK_ID));

  // Enable baud rate generation
  scifUartSetBaudRate(baudRate);

  // Enable RX (10 idle bit periods required before enabling start bit detection)
  scifUartSetRxFifoThr(SCIF_UART_RX_FIFO_MAX_COUNT / 2);
  scifUartSetRxTimeout(10 * 2);
  scifUartSetRxEnableReqIdleCount(10 * 2);
  scifUartRxEnable(1);

  // Enable events (half full RX FIFO or 10 bit period timeout
  scifUartSetEventMask(BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR | BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT);

}

#endif /* BASESTATION and cc13x0 */

/***********************************************************************/
/* FUNCTIONS FOR BUILT-IN UART                                         */
/***********************************************************************/

// auxiliary code to emulate blocking calls in call-back mode

/*
 * There seems to be a bug in BLOCKING mode that
 * causes errors when UART_read is called from one
 * task and UART_write from another.
 */
#define UART_USE_CALLBACK_MODE
#ifdef UART_USE_CALLBACK_MODE
static Semaphore_Handle uartTxSemaphore;
static Semaphore_Handle uartAccessSemaphore;

static int_fast32_t uartLocalRead(uart_handle_t h, void* vb, size_t size) {
  uint8_t* buffer = (uint8_t*) vb;
  Semaphore_pend(uartAccessSemaphore, BIOS_WAIT_FOREVER);
#ifdef USE_UART2
  size_t n;
  UART2_read(h, vb, size, &n);
#else
  UART_read(h, buffer, size);
#endif
  Semaphore_post(uartAccessSemaphore);
  Semaphore_pend(uartRxSemaphore, BIOS_WAIT_FOREVER);
  return 1;
}
static int_fast32_t uartLocalWrite(uart_handle_t h, const uint8_t* buffer, size_t size) {
	Semaphore_pend(uartAccessSemaphore, BIOS_WAIT_FOREVER);
#ifdef USE_UART2
	  size_t n;
	  UART2_write(h, buffer, size, &n);
#else
	UART_write(h, buffer, size);
#endif
	Semaphore_post(uartAccessSemaphore);
	Semaphore_pend(uartTxSemaphore, BIOS_WAIT_FOREVER);
  return 1;
}
static void uartTxCallback(uart_handle_t h, void* buffer, size_t size) {
	Semaphore_post(uartTxSemaphore);
}
static void uartRxCallback(uart_handle_t h, void* buffer, size_t size) {
	Semaphore_post(uartRxSemaphore);
}
//#define uart_read(h,b,s) uartLocalRead(h,b,s)
//#define uart_write(h,b,s) uartLocalWrite(h,b,s)
//#else
//#define uart_read(h,b,s) UART_read(h,b,s)
//#define uart_write(h,b,s) UART_write(h,b,s)
#endif

void uartBlockingWrite(uint32_t uart_index, uint8_t* buffer, size_t size) {
    (*blocking_write)(uartHandle,buffer,size);
#ifdef OBSOLETE
    if (uart_index==0 ) (*blocking_write)(uartHandle,buffer,size);
    if (uart_index==1 ) (*blocking_write)(uartHandle,buffer,size);
    if (uart_index==16) (*blocking_write)(uartHandle /* dummy */,buffer,size);
#endif
}


//static uint8_t uartBuffer[256];

/*

static uint8_t slipTxBuffer[2*BUFFER_SIZE+1];
static uint32_t slipEncode(uint8_t* p, uint32_t length) {
	uint32_t i, j;

	j=0;
	for (i=0; i<length; i++) {
		uint8_t b = *p;
		p++;
		switch (b) {
		case SLIP_END:
			slipTxBuffer[ j++ ] = SLIP_ESC;
			slipTxBuffer[ j++ ] = SLIP_ESC_END;
			break;
		case SLIP_ESC:
			slipTxBuffer[ j++ ] = SLIP_ESC;
			slipTxBuffer[ j++ ] = SLIP_ESC_ESC;
			break;
		default:
			slipTxBuffer[ j++ ] = b;
			break;
		}
	}
	slipTxBuffer[ j++ ] = SLIP_END;

	return j;
}

static uint8_t slipRxBuffer[2*BUFFER_SIZE+1];
static uint32_t slipDecode(uint8_t* outp, uint32_t length) {
	uint32_t i, j, k;

	j=0;
	k=0;
	for (i=0; i<length; i++) {
		uint8_t b = slipRxBuffer[ k++ ];
		switch (b) {
		case SLIP_END:
			return j; // we are done; normally the end need not be returned at all
		case SLIP_ESC:
			b = slipRxBuffer[ k++ ];
			if (b == SLIP_ESC_END) outp[ j++ ] = SLIP_END;
			if (b == SLIP_ESC_ESC) outp[ j++ ] = SLIP_ESC;
			break;
		default:
			outp[ j++ ] = b;
			break;
		}
	}

	return j;
}

*/

/***********************************************************************/
/* BUILT-IN UART TRANSMIT TASK, FRAMING IS ALWAYS SLIP                 */
/***********************************************************************/

static uint8_t txbyte;

static void uartTxTaskFunction(UArg arg0, UArg arg1) {
	uint16_t i;
	buffer_descriptor d;

	//System_printf("in uart tx task\n");

	while (1) {
		Mailbox_pend(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

		if (framing == 0) {
		//System_printf("uart write %d bytes\n",d.length);
		(*blocking_write)(uartHandle, slip_end, 1); // just an escape character
		//System_printf("00\n");
		for (i=0; i<d.length; i++) {
			txbyte = buffers[ d.id ][ i ];
			switch (txbyte) {
			case SLIP_END:
			  (*blocking_write)(uartHandle, slip_esc_end, 2);
				break;
			case SLIP_ESC:
			  (*blocking_write)(uartHandle, slip_esc_esc, 2);
				break;
			default:
			  (*blocking_write)(uartHandle, &txbyte, 1);
				break;
			}
			//System_printf("> %d\n",i);
		}
		(*blocking_write)(uartHandle, slip_end, 1); // just an escape character
		//System_printf("uart write %d bytes done\n",d.length);
		}

		if (framing == 1) {
	    for (i=0; i<d.length; i++) {
	      txbyte = buffers[ d.id ][ i ];
	      (*blocking_write)(uartHandle, &txbyte, 1);
	    }
		}
		//Task_sleep(1000000);
		//System_printf("uart task got buffer %d ");
		//int i;
		//for (i=0; i<d.length; i++) System_printf("%c",buffers[d.id][i]);

		//uint32_t n = slipEncode(buffers[d.id], d.length);
		//UART_write(uartHandle, slipTxBuffer, n);

		Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

		//uartQueueElement_t* e = Queue_get(uartTxQueue);


		//Queue_put(freeQueue, &(e->elem));
	}
}

#define UART_TX_TASK_STACK_SIZE 512

static Task_Params uartTxTaskParams;
static Task_Struct uartTxTask; /* not static so you can see in ROV */
static uint8_t uartTxTaskStack[UART_TX_TASK_STACK_SIZE];

//#include <ti/drivers/Power.h>
//#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>

/*
static int16_t offset;
static int16_t readToNewline() {
	offset=-1;
	do {
		offset++;
		UART_read(uartHandle, slipRxBuffer+offset, 1);
	} while (slipRxBuffer[offset] != '\n' && offset<sizeof(slipRxBuffer)-1);
	return offset+1;
}
*/

static void uartRxTaskFunction(UArg arg0, UArg arg1) {
	uint8_t b;
	uint8_t rxbyte;
  buffer_descriptor d;

  //Task_sleep(BIOS_WAIT_FOREVER);

	if (framing == FRAMING_UBLOX) {
	  //System_printf("uart framing gps\n");

	  while (1) {
	    do {
	      (*blocking_read)(uartHandle, &rxbyte, 1);
	      //System_printf("u1 %02x\n",rxbyte);
	    } while (rxbyte != '$' && rxbyte != 0xB5);

	    if (rxbyte=='$') {
	      uint16_t i = 0;
	      //uint8_t done = 0;
        Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
	      while (1) {
	        //uart_read(uartHandle, &rxbyte, 1);
	        //System_printf("u2 %02x %d\n",rxbyte,i);
	        buffers[ d.id ][ i++ ] = rxbyte;
	        if (i == BUFFER_SIZE) i--; // don't overrun the buffer

	        if (rxbyte == 0x0A) break; // lines end with 0x0D 0x0A (\r\n)

	        (*blocking_read)(uartHandle, &rxbyte, 1);
	      }
	      //System_printf("u3 %02x %d\n",rxbyte,i);

	      if (i==0 || i==BUFFER_SIZE-1) { // empty packet or buffer overrun
	        Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
	      } else {
	        //System_printf("in uart rx read %d bytes\n",i);
	        d.length = i;
	        Mailbox_post(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
	      }
	    }

      if (rxbyte==0xB5) {
        uint16_t i = 0;
        //uint8_t done = 0;
        Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
        d.length = 0;

        uint8_t  state = 0;
        //uint8_t  class, id;
        uint8_t  chk1=0, chk2=0;
        uint16_t pktLen=0, pktIdx=0;
        uint8_t  done = 0;

        while (1) {

          uint8_t cc = rxbyte;

          buffers[ d.id ][ i++ ] = cc;
          if (i == BUFFER_SIZE) i--; // don't overrun the buffer//System_printf("%02x,%d,%d\n,",cc,i,n);

          //System_printf("u7 %02x %d state=%d\n",rxbyte,i,state);

          switch (state) {
          case 0:  // Wait for sync 1 (0xB5)
            chk1 = chk2 = 0;
            if (cc == 0xB5) state++;
            else            done = 1;
            break;
          case 1:  // wait for sync 2 (0x62)
            if (cc == 0x62) state++;
            else            done = 1;
            //System_printf("S1"); System_flush();
            break;
          case 2:  // read for class code
            //class = cc;
            chk1 += cc;
            chk2 += chk1;
            state++;
            //System_printf("S2(%d)",class); System_flush();
            break;
          case 3:  // read id
            //id = cc;
            chk1 += cc;
            chk2 += chk1;
            state++;
            //System_printf("S3(%d)",id); System_flush();
            break;
          case 4:  // wait for length byte 1
            pktLen = cc;
            chk1 += cc;
            chk2 += chk1;
            state++;
            //System_printf("S4"); System_flush();
            break;
          case 5:  // wait for length byte 2
            pktLen |= (((uint16_t) cc) << 8);
            pktIdx = 0;
            chk1 += cc;
            chk2 += chk1;
            state++;
            //System_printf("S5(%d)",pktLen); System_flush();
            break;
          case 6:  // wait for <pktLen> payload bytes
            //UBX_PKTS.buf[pktIdx++] = cc;
            // we collect the payload in the buffer in any case.
            pktIdx++;
            chk1 += cc;
            chk2 += chk1;
            if (pktIdx == pktLen) {
              state++;
            }
            break;
          case 7:  // wait for checksum 1
            if (chk1 == cc) {
              state++;
            } else {
              System_printf("Checksum1 failure\n");
              done = 1;
            }
            break;
          case 8:  // wait for checksum 2
            if (chk2 == cc) {
              // checksum is good...
              //ubloxState = UBLOX_STATE_NORMAL;
              //if (class==0x01 && id==0x02) handle_NAV_POSLLH(buffer);
              //else if (class==0x01 && id==0x21) handle_NAV_TIMEUTC(buffer);
              //else System_printf("ublox class=%02x id=%02x pktLen=%d i+1=%d\n",class,id,pktLen,i+1);
              if (i<BUFFER_SIZE) d.length = i; // this marks the packet as good

            } else {
              // if we simply keep d.length=0, the packet will be discarded.
              System_printf("Checksum2 failure\n");
            }
            done = 1;
            //System_printf("S8"); System_flush();
            break;
          }

         //uart_read(uartHandle, &rxbyte, 1);
          //System_printf("u2 %02x %d\n",rxbyte,i);

          if (done) break;

          (*blocking_read)(uartHandle, &rxbyte, 1);
        }
        //System_printf("u3 %02x %d\n",rxbyte,i);

        if (d.length == 0) {
          //System_printf("ublox failure state %d i=%d\n",state,i);
          Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
        } else {
          //System_printf("ublox success state %d i=%d\n",state,i);
          Mailbox_post(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
        }
      }

	  }
	} // if FRAMING_UBLOX
	//System_printf("in uart rx task skip to END\n");

	do {
	  (*blocking_read)(uartHandle, &rxbyte, 1);
	} while (rxbyte != SLIP_END);

	while (1) {
		buffer_descriptor d;
		Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

		uint16_t i = 0;
		uint8_t done = 0;
		while (!done) {
		  (*blocking_read)(uartHandle, &rxbyte, 1);
			switch (rxbyte) {
			case SLIP_END:
				done = 1;
				break;
			case SLIP_ESC:
			  (*blocking_read)(uartHandle, &b, 1);
				if (b==SLIP_ESC_ESC) buffers[ d.id ][ i++ ] = SLIP_ESC;
				if (b==SLIP_ESC_END) buffers[ d.id ][ i++ ] = SLIP_END;
				break;
			default:
				buffers[ d.id ][ i++ ] = rxbyte;
				break;
			}
			if (i == BUFFER_SIZE) i--; // don't overrun the buffer
		}

		if (i==0 || i==BUFFER_SIZE-1) { // empty packet or buffer overrun
			Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
			continue;
		}

		//System_printf("in uart rx read %d bytes\n",i);
		d.length = i;
		Mailbox_post(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
	}
}
/*
static void XXXuartRxTaskFunction(UArg arg0, UArg arg1) {
	System_printf("uart rx task 1\n");

	//Power_setConstraint(Power_IDLE_PD_DISALLOW);
	//Power_setConstraint(Power_SB_DISALLOW);

	//UART_read(uartHandle, slipRxBuffer, sizeof(slipRxBuffer)); // ignore first packet (may be incomplete)
	readToNewline();
	System_printf("uart rx task 2\n");

	while (1) {
		buffer_descriptor d;
		System_printf("uart rx task 3\n");

		//uint32_t n = UART_read(uartHandle, slipRxBuffer, sizeof(slipRxBuffer));
		uint16_t n = readToNewline();
		System_printf("uart rx task 4\n");
		if (n==sizeof(slipRxBuffer)) {
			System_printf("uart rx skip a full buffer\n",n);
			continue; // got to end of buffer, not END byte
		}

		System_printf("uart rx read %d bytes 1st 0x%02x\n",n,slipRxBuffer[0]);


		Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

		System_printf("uart rx read %d bytes 1st 0x%02x\n",n,slipRxBuffer[0]);
		d.length = (uint8_t) slipDecode(buffers[d.id], n);

		//leds_blink(LEDS_RED, 3);

		System_printf("uart rx task posting buffer %d len %d 1st 0x%02x\n",d.id, d.length, buffers[d.id][0]);
		Mailbox_post(uartRxMailbox, &d, BIOS_WAIT_FOREVER);
	}
}
*/

#ifdef OBSOLETE

/* Stack size in bytes */
//#define STACKSIZE    312

Task_Struct scuartTask;
//Char scuartTaskStack[STACKSIZE];

/*------------------------------------------------------------*/
void scuartTaskFxn(UArg a0, UArg a1) {
  uint8_t b;
  uint8_t rxbyte;
  buffer_descriptor d;

  // Initialize the Sensor Controller
  scuart_init();

  while (1) {
    do {
      rxbyte = scuart_read();
      //System_printf("u1 %02x\n",rxbyte);
    } while (rxbyte != '$' && rxbyte != 0xB5);

    if (rxbyte=='$') {
      uint16_t i = 0;
      //uint8_t done = 0;
      Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
      while (1) {
        //uart_read(uartHandle, &rxbyte, 1);
        //System_printf("u2 %02x %d\n",rxbyte,i);
        buffers[ d.id ][ i++ ] = rxbyte;
        if (i == BUFFER_SIZE) i--; // don't overrun the buffer

        if (rxbyte == 0x0A) break; // lines end with 0x0D 0x0A (\r\n)

        rxbyte = scuart_read();
        //System_printf("u2 %02x\n",rxbyte);
      }
      //System_printf("u3 %02x %d\n",rxbyte,i);

      if (i==0 || i==BUFFER_SIZE-1) { // empty packet or buffer overrun
        Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
      } else {
        //System_printf("in uart rx read %d bytes\n",i);
        d.length = i;
        Mailbox_post(ubloxOutputMailbox, &d, BIOS_WAIT_FOREVER);
      }
    }

    if (rxbyte==0xB5) {
      uint16_t i = 0;
      //uint8_t done = 0;
      Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
      d.length = 0;

      uint8_t  state = 0;
      uint8_t  class, id, chk1=0, chk2=0;
      uint16_t pktLen=0, pktIdx=0;
      uint8_t  done = 0;

      while (1) {

        uint8_t cc = rxbyte;

        buffers[ d.id ][ i++ ] = cc;
        if (i == BUFFER_SIZE) i--; // don't overrun the buffer//System_printf("%02x,%d,%d\n,",cc,i,n);

        //System_printf("u7 %02x %d state=%d\n",rxbyte,i,state);

        switch (state) {
        case 0:  // Wait for sync 1 (0xB5)
          chk1 = chk2 = 0;
          if (cc == 0xB5) state++;
          else            done = 1;
          break;
        case 1:  // wait for sync 2 (0x62)
          if (cc == 0x62) state++;
          else            done = 1;
          //System_printf("S1"); System_flush();
          break;
        case 2:  // read for class code
          class = cc;
          chk1 += cc;
          chk2 += chk1;
          state++;
          //System_printf("S2(%d)",class); System_flush();
          break;
        case 3:  // read id
          id = cc;
          chk1 += cc;
          chk2 += chk1;
          state++;
          //System_printf("S3(%d)",id); System_flush();
          break;
        case 4:  // wait for length byte 1
          pktLen = cc;
          chk1 += cc;
          chk2 += chk1;
          state++;
          //System_printf("S4"); System_flush();
          break;
        case 5:  // wait for length byte 2
          pktLen |= (((uint16_t) cc) << 8);
          pktIdx = 0;
          chk1 += cc;
          chk2 += chk1;
          state++;
          //System_printf("S5(%d)",pktLen); System_flush();
          break;
        case 6:  // wait for <pktLen> payload bytes
          //UBX_PKTS.buf[pktIdx++] = cc;
          // we collect the payload in the buffer in any case.
          pktIdx++;
          chk1 += cc;
          chk2 += chk1;
          if (pktIdx == pktLen) {
            state++;
          }
          break;
        case 7:  // wait for checksum 1
          if (chk1 == cc) {
            state++;
          } else {
            System_printf("Checksum1 failure\n");
            done = 1;
          }
          break;
        case 8:  // wait for checksum 2
          if (chk2 == cc) {
            // checksum is good...
            //ubloxState = UBLOX_STATE_NORMAL;
            //if (class==0x01 && id==0x02) handle_NAV_POSLLH(buffer);
            //else if (class==0x01 && id==0x21) handle_NAV_TIMEUTC(buffer);
            //else System_printf("ublox class=%02x id=%02x pktLen=%d i+1=%d\n",class,id,pktLen,i+1);
            if (i<BUFFER_SIZE) d.length = i; // this marks the packet as good

          } else {
            // if we simply keep d.length=0, the packet will be discarded.
            System_printf("Checksum2 failure\n");
          }
          done = 1;
          //System_printf("S8"); System_flush();
          break;
        }

       //uart_read(uartHandle, &rxbyte, 1);
        //System_printf("u2 %02x %d\n",rxbyte,i);

        if (done) break;

        rxbyte = scuart_read();
        //System_printf("u3 %02x state %d cili %d %d %d %d\n",rxbyte,state,class,id,pktLen,pktIdx);
      }
      //System_printf("u3 %02x %d\n",rxbyte,i);

      if (d.length == 0) {
        //System_printf("ublox failure state %d i=%d\n",state,i);
        Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
      } else {
        //System_printf("ublox success state %d i=%d\n",state,i);
        Mailbox_post(ubloxOutputMailbox, &d, BIOS_WAIT_FOREVER);
      }
    }

  }
}

#endif

/*------------------------------------------------------------*/

#ifdef OBSOLETE

extern uint8_t uartRxTaskStack[];
void ubloxInit() {
  Task_Params taskParams;

  // Create the semaphore used to wait for Sensor Controller ALERT events
  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  semParams.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&semScTaskAlert, 0, &semParams);

  ubloxOutputMailbox = Mailbox_create(sizeof(buffer_descriptor),BUFFER_COUNT, NULL,NULL);

  Task_Params_init(&taskParams);
  taskParams.stack = uartRxTaskStack; //scuartTaskStack;
  taskParams.stackSize = 512; // sizeof(scuartTaskStack);
  taskParams.priority = SCUART_TASK_PRIORITY;
  Task_construct(&scuartTask, scuartTaskFxn, &taskParams, NULL);

}

#endif

#define UART_RX_TASK_STACK_SIZE 512

static Task_Params uartRxTaskParams;
static Task_Struct uartRxTask; /* not static so you can see in ROV */
uint8_t uartRxTaskStack[UART_RX_TASK_STACK_SIZE];

void uartTasks_init(uint32_t uart_index, uint32_t baudRate, uint32_t txMailboxSize, uint8_t framingMode) {
  framing = framingMode;

  //Mailbox_Params mboxParams;
	//Mailbox_Params_init(&mboxParams);
//System_printf("1 uart init\n");
	uartTxMailbox = Mailbox_create(sizeof(buffer_descriptor),txMailboxSize, NULL,NULL);
//System_printf("2 uart init\n");
	uartRxMailbox = Mailbox_create(sizeof(buffer_descriptor),BUFFER_COUNT, NULL,NULL);
//System_printf("3 uart init\n");

  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  semParams.mode = Semaphore_Mode_BINARY;
  uartRxSemaphore = Semaphore_create(0, &semParams, NULL);

	if (uart_index < 16 ) { // built-in uarts
#ifdef USE_UART2
#error "unimplemented"
#else
	  UART_Params uartParams;
	  UART_Params_init(&uartParams);
	  uartParams.writeDataMode = UART_DATA_BINARY;
	  //uartParams.writeReturnMode = UART_RETURN_FULL; // don't add a new line, we add it in slipEncode
	  uartParams.writeTimeout = UART_WAIT_FOREVER;

	  uartParams.readDataMode = UART_DATA_BINARY;
	  uartParams.readReturnMode = UART_RETURN_FULL;
	  uartParams.readEcho = UART_ECHO_OFF;
	  uartParams.readTimeout = UART_WAIT_FOREVER;

	  //uartParams.baudRate = 9600;
	  uartParams.baudRate = baudRate;
	  //uartParams.baudRate = 230400;

    #ifdef UART_USE_CALLBACK_MODE
	  uartParams.readCallback = uartRxCallback;
	  uartParams.writeCallback = uartTxCallback;
	  uartParams.writeMode = UART_MODE_CALLBACK;
	  uartParams.readMode = UART_MODE_CALLBACK;

	  Semaphore_Params_init(&semParams);
	  semParams.mode = Semaphore_Mode_BINARY;
	  uartTxSemaphore = Semaphore_create(0, &semParams, NULL);

	  Semaphore_Params_init(&semParams);
	  semParams.mode = Semaphore_Mode_BINARY;
	  uartAccessSemaphore = Semaphore_create(1, &semParams, NULL);

    blocking_read  = uartLocalRead;
    blocking_write = uartLocalWrite;

#else
	  uartParams.writeMode = UART_MODE_BLOCKING;
	  uartParams.readMode = UART_MODE_BLOCKING;

    blocking_read  = UART_read;
    blocking_write = UART_write;
#endif

	  uartHandle = UART_open(uart_index, &uartParams);

	  if (uartHandle==NULL) {
	      // try to close, perhaps open in console (not clear how this worked up to now)
#ifdef USE_CONSOLE
	      consoleClose();
#endif
	      uartHandle = UART_open(uart_index, &uartParams);

	      if (uartHandle==NULL) System_printf("falied to open UART\n");
	  }
#endif // USE_UART2
	}

	if (uart_index >= 16) { // SCUART
#if defined(USE_BASESTATION) && defined(DeviceFamily_CC13X0)
	  scuart_init(baudRate);

	  blocking_read  = scuart_read;
	  blocking_write = scuart_write;
#else
	  System_printf("Fatal error: UART %d (>=16) without Sensor Controller UART implementation\n",uart_index);
#endif
	}

	Task_Params_init(&uartTxTaskParams);
  uartTxTaskParams.stackSize = UART_TX_TASK_STACK_SIZE;
  uartTxTaskParams.priority = UART_TX_TASK_PRIORITY;
  uartTxTaskParams.stack = &uartTxTaskStack;
  uartTxTaskParams.arg0 = (UInt)1000000;

	System_printf("uart tx task init\n");
  Task_construct(&uartTxTask, uartTxTaskFunction, &uartTxTaskParams, NULL);

	Task_Params_init(&uartRxTaskParams);
  uartRxTaskParams.stackSize = UART_RX_TASK_STACK_SIZE;
  uartRxTaskParams.priority = UART_RX_TASK_PRIORITY;
  uartRxTaskParams.stack = &uartRxTaskStack;
  uartRxTaskParams.arg0 = (UInt)1000000;

	System_printf("uart rx task init\n");
  Task_construct(&uartRxTask, uartRxTaskFunction, &uartRxTaskParams, NULL);
}
