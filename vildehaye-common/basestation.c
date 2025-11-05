#include "config.h"

#ifdef USE_BASESTATION

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/sysbios/family/arm/cc26xx/Seconds.h>

#include <ti/drivers/PIN.h>

#include <ti/devices/DeviceFamily.h>
#if MOVED
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)

//#include <ti/devices/cc13x0/driverlib/rf_data_entry.h>
#include <ti/drivers/rf/RF.h>
#include "radio_setup.h"
#endif

#include "radio.h"

// for the MAC addresses:

#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_fcfg1.h)

//#include "CC1310_LAUNCHXL.h"

#include "buffers.h"
#include "uart.h"

#include "leds.h"
//#include "board.h"


#include "rf_queue_pointer.h"

#include "receive.h"
#include "vildehaye.h"

#include "watchdog.h"

#include "tag.h"

#include "nvm.h"
#include "logger.h"
#include "logger_catalog.h"

#include "ublox.h"

#include "i2c.h"

#include "ssd1306_i2c.h"
#include "bme280.h"

#include "console.h"

#if 0
/***************************************************************/
/* DISPLAY                                                     */
/***************************************************************/

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
//#include <ti/display/DisplayExt.h>
//#include <ti/display/DisplayUart.h>
#include <ti/display/DisplaySharp.h>
#include <ti/display/AnsiColor.h>

Display_Handle    display;

void displayInit() {
  //int i = 0;

  //while (Display_config[i].fxnTablePtr != 0) {
  //  System_printf("display config %d exists!\n",i);
  //  i++;
  //}

  System_printf("display calling init\n");

  //Display_init();

  Display_Params    params;
  Display_Params_init(&params);
  params.lineClearMode = DISPLAY_CLEAR_BOTH;

  //handle = Display_open(1 /*someConfigIndexValue */, &params);
  //Display_open(Display_Type_HOST, &params);
  //Display_open(Display_Type_ANY, &params);
  //Display_open(Display_Type_UART, NULL);
  //Display_open(Display_Type_UART | Display_Type_LCD, &params);
  display = Display_open(Display_Type_LCD, &params /* can also pass NULL */);

  System_printf("display handle %x\n",display);

  Display_clear(display);
  Display_clearLines(display, 0, 4);
  Display_printf(display, 0, 0, "   --- *** ---");
  Display_printf(display, 1, 0, "How are you %s?", "Sivan");
  // Note that for floating point support, the .cfg file must have a line like
  // System.extendedFormats = "%$L%$S%$F%f"; // Add '%f' support
  Display_printf(display, 2, 0, "E");
  Display_printf(display, 3, 0, "Pi is %f", 3.1415);
  Display_printf(display, 4, 1, "E");
  Display_printf(display, 5, 2, "E");
  Display_printf(display, 6, 3, "6");
  Display_printf(display, 7, 3, "7");
  Display_printf(display, 8, 3, "8");
  Display_printf(display, 9, 3, "9");
  Display_printf(display, 10, 3, "9999");
  Display_printf(display, 11, 3, "99999996");
  //Display_close(handle);
}

#endif

#define DATA_PROTOCOL_ATLAS                     16
#define DATA_PROTOCOL_VILDEHAYE_BEACON           3
#define DATA_PROTOCOL_VILDEHAYE_BEACON_RESPONSE  2
#define DATA_PROTOCOL_VILDEHAYE_SESSION          1

/***** Variable declarations *****/
//static RF_Object rfObject;
//static RF_Handle rfHandle;

uint8_t countRx, countTxOk, countTxTooLate, countTxErr, state;
uint32_t timestampRx, timestampTx, timestampPostTx;

static Mailbox_Handle packetHandlingMailbox;

//Mailbox_Handle basestationLoggerMailbox;

static Mailbox_Handle loggingMailbox;

#ifdef DeviceFamily_CC13X2
#define LOGGING_TASK_STACK_SIZE 1024
#else
#define LOGGING_TASK_STACK_SIZE 768
#endif
static Task_Params basestationLoggerUartTaskParams;
static Task_Struct basestationLoggerUartTask;
static uint8_t basestationLoggerUartTaskStack[LOGGING_TASK_STACK_SIZE];

static void basestationUartTaskFunction(UArg arg0, UArg arg1);



#define BASESTATION_TASK_STACK_SIZE 1024
#define BASESTATION_RECEIVE_TASK_STACK_SIZE 1024

static Task_Params basestationTaskParams;
Task_Struct basestationReceiveTask;    /* not static so you can see in ROV */
static uint8_t basestationReceiveTaskStack[BASESTATION_RECEIVE_TASK_STACK_SIZE];

static Task_Params basestationTxTaskParams;
static Task_Struct basestationTxTask;
static uint8_t basestationTxTaskStack[BASESTATION_TASK_STACK_SIZE];

//static Task_Params basestationUartTaskParams;
//static Task_Struct basestationUartTask;
//static uint8_t basestationUartTaskStack[BASESTATION_TASK_STACK_SIZE];

static const uint8_t BASESTATION_TYPE_UNKNOWN  = 0;
static const uint8_t BASESTATION_TYPE_TETHERED = 1;
static const uint8_t BASESTATION_TYPE_LOGGING  = 2;
static uint8_t basestationType = BASESTATION_TYPE_UNKNOWN;

static uint8_t radioSetupInUse = 0xFF; // radio not actually in use

static uint32_t clockRemote;
static uint8_t  clockMineValid = 0;

static uint8_t acks = 0;
static uint8_t gps  = 0;


/*
	Error handling function – replaces the default while(1) in RF driver.
	This is necessary for certain versions of TI-RTOS, not all; see
	http://processors.wiki.ti.com/index.php/CC1310_rev_B_PCN_information
*/
/*
static void errorCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
  if ((int32_t)ch == RF_ERROR_CMDFS_SYNTH_PROG) {
    // Handle error
    // If CMD_FS is executed standalone, re-start CMD_FS in this function if there is time. If not, handle as in previous example
    // When CMD_FS is chained with RX/TX commands, the RX/TX is executed in parallel with this callback by the radio and will subsequently fail. Error can be handled as in previous example.
  }
}
*/

/***********************************************************************/
/* Heard Set                                                           */
/***********************************************************************/

#define HEARDSET_SIZE 4
#define CLOCK_VALIDITY_UNKNOWN 0
#define CLOCK_VALIDITY_INVALID 1
#define CLOCK_VALIDITY_VALID   2

struct heardset_st {
  uint64_t tagId;
  uint8_t  clockValidity;
  uint8_t  hasData;
  uint32_t time;
} heardset[HEARDSET_SIZE];

static void heardsetUpdate(uint64_t tagId, uint8_t clockValidity, uint8_t hasData) {
  int i;
  int j = -1;
  uint32_t now = Seconds_get();

  // is the tag already in the set?
  for (i=0; i<HEARDSET_SIZE; i++)
    if (heardset[i].tagId==tagId) j = i;

  // if not, is there an empty slot?
  if (j==-1) {
    for (i=0; i<HEARDSET_SIZE; i++)
      if (heardset[i].tagId==0) {
        j = i;
        break;
      }
  }

  // if not, replace the oldest slot
  uint32_t oldest = now;
  if (j==-1) {
    for (i=0; i<HEARDSET_SIZE; i++)
      if (heardset[i].time<=oldest) j = i;
  }

  if (j==-1) {
    System_printf("error, could not find slot for tag activity report\n");
    return;
  }

  // if this tag is new in the set, mark clock validity as unknown
  if (heardset[j].tagId != tagId) {
    heardset[j].tagId = tagId;
    heardset[j].clockValidity = '?';
  }
  heardset[j].time    = now;
  heardset[j].hasData = (hasData ? 'D' : 'E');
  if (clockValidity == CLOCK_VALIDITY_VALID)   heardset[j].clockValidity = 'C';
  if (clockValidity == CLOCK_VALIDITY_INVALID) heardset[j].clockValidity = 'O';
}


/***********************************************************************/
/* Update display                                                      */
/***********************************************************************/

static uint8_t displayMode = 0;

static PIN_Handle displayPinHandle = 0;
static PIN_State  displayPinState;
static const PIN_Config displayPinTable[] = {
#ifdef BOOSTERPACK_DISPLAY_POWER
    BOOSTERPACK_DISPLAY_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL, /* TPS2553 switch for display */
#endif
    //IOID_27 | PIN_INPUT_EN                       | PIN_PULLUP,              /* type-of-basestation indicator */
    //IOID_28 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,            /* ground for IO_27, to allow using a jumper */
#ifdef BOOSTERPACK_SENSE
    BOOSTERPACK_SENSE | PIN_INPUT_EN                       | PIN_PULLUP,              /* type-of-basestation indicator */
#endif
#if defined(BOOSTERPACK_SENSE_GND)
    BOOSTERPACK_SENSE_GND | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL,            /* ground for IO_27, to allow using a jumper */
#endif
    PIN_TERMINATE
};

void displayNextMode() {
  // now this happens at init
  //if (displayPinHandle == 0) displayPinHandle = PIN_open(&displayPinState, displayPinTable);

  switch (displayMode) {
  case 0:
    ssd1306_clearScreen(); // Set all pixels to black
    break;
  case 1:
    System_printf("turning off display\n");
    PIN_setOutputValue(displayPinHandle, BOOSTERPACK_DISPLAY_POWER, 0 ); // turn off display
    break;
  case 2:
    System_printf("turning on display\n");
    PIN_setOutputValue(displayPinHandle, BOOSTERPACK_DISPLAY_POWER, 1 ); // turn on display
    Task_sleep(10000 / Clock_tickPeriod);
    ssd1306_init();
    break;
  }

  displayMode++;
  if (displayMode==3) displayMode = 0;
}


void displayUpdate() {
  char line[17];

  if (displayMode == 0) {
    sprintf(line,"Radio  %c",radioSetupInUse==0xFF ? '-' : 'Y');
    ssd1306_printText(line, 0);

    sprintf(line,"T=%d",Seconds_get());
    ssd1306_printText(line, 1);

    sprintf(line,"Clock  %c Acks %c",clockMineValid==0 ? '-' : 'Y', acks==0 ? '-' : 'Y');
    ssd1306_printText(line, 2);

    char sd = '-';
    if (loggerState==LOGGER_LOGGING)       sd = 'Y';
    if (loggerState==LOGGER_STATE_UNKNOWN) sd = '-';
    if (loggerState==LOGGER_NO_FLASH)      sd = '-';
    if (loggerState==LOGGER_FLASH_IS_FULL) sd = 'F';
    if (loggerState==LOGGER_UNFORMATTED)   sd = 'U';

    sprintf(line,"SDCard %c GPS  %c",sd, gps==0 ? '-' : 'Y');
    ssd1306_printText(line, 3);
  }

  if (displayMode == 1) {
    int i;

    //heardset[0].tagId = 49001000123L;
    //heardset[1].tagId =  1002000444L;

    for (i=0; i<HEARDSET_SIZE; i++) {
      if (heardset[i].tagId == 0) {
        ssd1306_printText("-", i);
        continue;
      }

      int j;
      int d;
      uint64_t r = heardset[i].tagId;
      for (j=0; j<12; j++) {
        d = '0' + (uint8_t) (r % 10);
        if (r==0) d = ' ';
        r = r / 10;
        line[11-j] = d;
      }
      line[12] = ' ';
      line[13] = heardset[i].clockValidity;
      line[14] = ' ';
      line[15] = heardset[i].hasData;
      line[16] = 0;
      ssd1306_printText(line, i);
    }
  }
}

/***********************************************************************/
/* LoggerTask                                                          */
/***********************************************************************/
//extern int buttons_getState(int button);

#if 0
static int buttons_getState(int button) {
  if (button == 0)
    return PIN_getInputValue(Board_BTN1);
  else
    return PIN_getInputValue(Board_BTN2);
}

static int button(int b) { return PIN_getInputValue(b); }

#endif

uint8_t lastCardDetect = 0; // we never tried to initialize SD card


static void basestationLoggerMonitor() {

  // the buttons are all active low, so 1 means not pressed, 0 means pressed

  System_printf("BTNs: %d %d %d %d\n",
                PIN_getInputValue(LAUNCHPAD_BTN1),
                PIN_getInputValue(LAUNCHPAD_BTN2),
                PIN_getInputValue(BOOSTERPACK_BUTTON),    // logger boosterpack button
                PIN_getInputValue(BOOSTERPACK_CARD_DETECT)
                );

  if (PIN_getInputValue(LAUNCHPAD_BTN1) == LAUNCHPAD_BTN1_PRESSED) displayNextMode();
  displayUpdate();

  uint8_t cardDetect   = (PIN_getInputValue(BOOSTERPACK_CARD_DETECT)==BOOSTERPACK_CARD_DETECT_PRESSED);
  uint8_t ejectRequest = (    (PIN_getInputValue(BOOSTERPACK_BUTTON)==BOOSTERPACK_BUTTON_PRESSED )
                           || (PIN_getInputValue(LAUNCHPAD_BTN2)    ==LAUNCHPAD_BTN2_PRESSED     ) );

  System_printf("cardDetect=%d ejectRequest==%d\n",cardDetect,ejectRequest);

  if (!cardDetect && !lastCardDetect) { // no card, now or earlier
    System_printf("no SD card\n");
  }

  if (cardDetect && lastCardDetect && !ejectRequest) {
    // no change, card is inserted, no eject request
  }

  if (cardDetect && lastCardDetect && ejectRequest) {
    // no change, card is inserted, eject request!
    System_printf("eject request\n");
    if (loggerState != LOGGER_STATE_UNKNOWN) {
      System_printf("Closing NVM\n");
      loggerClose();
    }
    loggerState = LOGGER_STATE_UNKNOWN;
    leds_on(LEDS_SDCARD);
  }

  if (!cardDetect && lastCardDetect) { // card was just ejected
    loggerState = LOGGER_STATE_UNKNOWN; // in case we ejected without requesting
    leds_fast(LEDS_SDCARD);
  }


  if (cardDetect && !lastCardDetect) { // card was just inserted!

    System_printf("Trying to initialize logger and sd card\n");

    Task_sleep(1000000 / Clock_tickPeriod); // let the SD card initialize

    System_printf("Sleep ended ...\n");

    loggerState = LOGGER_STATE_UNKNOWN;
    loggerInit();

    System_printf("Logger state = %d (unknown,noflash,logging,uart,unformatted,full)\n",loggerState);

    if (loggerState==LOGGER_LOGGING) {
      leds_slow(LEDS_SDCARD); // indicate we're logging

      // read the configuration of the base station off the log
      buffer_descriptor d;
      Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
      nvm_t address;
      loggerIteratorInit(&address);
      while (address < NVM_PAGE_SIZE) {
        uint8_t type, len;
        if (loggerIteratorNext(&len,&type,buffers[ d.id ],&address,NULL) == LOGGER_ERR_NO_MORE_ITEMS) {
          break;
        } else {
          System_printf("Logger item addr %d type %d len %d\n",(int) address, type,len);
          if (type == CATALOG_DATATYPE_VILDEHAYE_CONFIG) {
            vildehayeHandlePacketNaked(buffers[ d.id ],len);
          }
        }
      }
      Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

      // we are done reading from the log, so we can can start writing (logging)
      loggerLog(LOGGER_DATATYPE_LOG_BOOT_MARKER, 0, 0);
      loggerCommit();

      System_printf("Logged a boot marker\n");

      /*
      uint32_t macAddressLow, macAddressHigh;
      macAddressLow  = *((uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0));
      macAddressHigh = *((uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_1));
      System_printf("BLE MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                     (macAddressHigh >>  8) & 0xFF,
                     (macAddressHigh      ) & 0xFF,
                     (macAddressLow  >> 24) & 0xFF,
                     (macAddressLow  >> 16) & 0xFF,
                     (macAddressLow  >>  8) & 0xFF,
                     (macAddressLow       ) & 0xFF
                    );

      macAddressLow  = *((uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_15_4_0));
      macAddressHigh = *((uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_15_4_1));
      System_printf("15.4 MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                    (macAddressHigh >> 24) & 0xFF,
                    (macAddressHigh >> 16) & 0xFF,
                    (macAddressHigh >>  8) & 0xFF,
                    (macAddressHigh      ) & 0xFF,
                    (macAddressLow  >> 24) & 0xFF,
                    (macAddressLow  >> 16) & 0xFF,
                    (macAddressLow  >>  8) & 0xFF,
                    (macAddressLow       ) & 0xFF
                   );
      */
      int iii;
      for (iii=0; iii<8; iii++)
        System_printf(" %02x",*((uint8_t*) (FCFG1_BASE + FCFG1_O_MAC_15_4_0 + iii)));
      System_printf(" (802.15.4 MAC)\n");

      for (iii=0; iii<8; iii++)
        System_printf(" %02x",*((uint8_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0 + iii)));
      System_printf(" (BLE MAC)\n");

      loggerLog(CATALOG_DATATYPE_BLE_MAC_ADDRESS,      (uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0),  2*sizeof(uint32_t));
      loggerLog(CATALOG_DATATYPE_802_15_4_MAC_ADDRESS, (uint32_t*) (FCFG1_BASE + FCFG1_O_MAC_15_4_0), 2*sizeof(uint32_t));

    } else {
      leds_fast(LEDS_SDCARD); // something is still wrong
    }
  }

  lastCardDetect = cardDetect;

#if OBSOLETE
  if (buttons_getState(0) == 0) { // BTN1 is pressed, eject request
    if (loggerState != LOGGER_STATE_UNKNOWN) {
      System_printf("Closing NVM\n");
      nvmClose();
    }
    loggerState = LOGGER_STATE_UNKNOWN;
    leds_on(LEDS_SDCARD);
    return;
  }

  if (buttons_getState(1) == 0) {
    // no card!
    System_printf("sdcard no card!");
    lastSDCardButtonState = 0;
    return;
  } else {
    /*
     * If there is a card and we had a card last time, just return; either
     * the logger is working, or there is something wrong but no point
     * in trying again until the card is remove and re-inserted.
     */
    if (lastSDCardButtonState == 1) return;
    lastSDCardButtonState = 1;
  }

  //loggerInit(); // SD Card presense switch; if 0, there is no card!

  if (loggerState==LOGGER_LOGGING) {
    leds_slow(LEDS_SDCARD); // indicate we're logging

    // read the configuration of the base station off the log
    buffer_descriptor d;
    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
    nvm_t address;
    loggerIteratorInit(&address);
    while (address < NVM_PAGE_SIZE) {
      uint8_t type, len;
      if (loggerIteratorNext(&len,&type,buffers[ d.id ],&address,NULL) == LOGGER_ERR_NO_MORE_ITEMS) {
        break;
      } else {
        System_printf("Logger item addr %d type %d len %d\n",(int) address, type,len);
        if (type == CATALOG_DATATYPE_VILDEHAYE_CONFIG) {
          vildehayeHandlePacketNaked(buffers[ d.id ],len);
        }
      }
    }
    Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

    // we are done reading from the log, so we can can start writing (logging)
    loggerLog(LOGGER_DATATYPE_LOG_BOOT_MARKER, 0, 0);
  } else {
    leds_fast(LEDS_SDCARD); // something is still wrong
  }
#endif
}

void basestationUbloxInit() {
  buffer_descriptor d;
  uint8_t wakeup;

  System_printf("ublox init\n");

  //ubloxInit(); // currently (July 2019) this only starts the scuart task

  System_printf("waking up ublox\n");

  wakeup = 0xFF; // dummy to wake up ublox receiver if in inactive mode
  uartBlockingWrite(GPS_UART,&wakeup,1);

  System_printf("done waking up ublox\n");
  System_flush();

  Task_sleep(100000 / Clock_tickPeriod);

  System_printf("ublox configure uart!");
  // clear old message if any
  while (Mailbox_pend(uartRxMailbox, &d, BIOS_NO_WAIT)) Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxConfigureUart1(buffers[d.id], 9600, false, true); // ubx only
  // Sivan July 2019 switching from ARM uart to sensor-controller uart
  uartBlockingWrite(GPS_UART,buffers[d.id],d.length);
  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
  //Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  // wait for ACK/NACK

  if (Mailbox_pend(uartRxMailbox, &d, 1*1000000 / Clock_tickPeriod)) {
    uint8_t* buffer = buffers[d.id];
    if (buffer[0] == 0xB5) {
      uint8_t cls = buffer[2];
      uint8_t id = buffer[3];
      if (cls==0x05 && id==0x00) { System_printf("ubx NACK for %02x %02x\n",buffer[6],buffer[7]); }
      else if (cls==0x05 && id==0x01) { System_printf("ubx ACK for %02x %02x\n",buffer[6],buffer[7]);  }
      else System_printf("ublox class=%02x id=%02x len=%d\n",cls,id,d.length);
    } else
      System_printf("ublox received NMEA reply\n");
  } else
    System_printf("ublox no reply\n");

  // Task_sleep(450000 / Clock_tickPeriod);

  //Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  //d.length = ubloxEnableMsg(buffers[d.id], 0x01, 0x02, 10); // nav-posllh; should go to nav-pvt, in ublox >= 7
  //Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  System_printf("ublox configure nav-pvt!");

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x01, 0x07, 10); // nav-pv
  uartBlockingWrite(GPS_UART, buffers[d.id],d.length);
  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
  //Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  //Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  //d.length = ubloxEnableMsg(buffers[d.id], 0x01, 0x21, 10); // nav-timeutc
  //Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  // wait for ACK/NACK

  if (Mailbox_pend(uartRxMailbox, &d, 1*1000000 / Clock_tickPeriod)) {
    uint8_t* buffer = buffers[d.id];
    if (buffer[0] == 0xB5) {
      uint8_t cls = buffer[2];
      uint8_t id = buffer[3];
      if (cls==0x05 && id==0x00) { System_printf("ubx NACK for %02x %02x\n",buffer[6],buffer[7]); }
      else if (cls==0x05 && id==0x01) { System_printf("ubx ACK for %02x %02x\n",buffer[6],buffer[7]);  }
      else System_printf("ublox class=%02x id=%02x len=%d\n",cls,id,d.length);
    } else
      System_printf("ublox received NMEA reply\n");
  } else
    System_printf("ublox no reply\n");

#if 0
  // disable NMEA messages that are on by default
  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x00, 0); // GGA
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x01, 0); // GLL
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x02, 0); // GSA
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x03, 0); // GSV
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x04, 0); // RMC
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  d.length = ubloxEnableMsg(buffers[d.id], 0x0F, 0x05, 0); // VTG
  Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
#endif
}

void basestationUbloxMonitor() {
  System_printf("polling gps\n");

  int i=0;
  int j=0;
  buffer_descriptor d;
  //while (Mailbox_pend(uartRxMailbox, &d, BIOS_NO_WAIT)) {
  while (Mailbox_pend(uartRxMailbox, &d, BIOS_NO_WAIT)) {
    //buffers[ d.id ][ d.length ] = 0; // add zero termination
    //if (buffers[d.id][0] == '$') System_printf("NMEA: %s\n",buffers[ d.id ]);
    uint8_t* buffer = buffers[d.id];
    if (buffer[0] == 0xB5) {
      i++;
      //ubloxParseMessage(buffers[d.id], d.length);

      uint32_t now = Seconds_get();

      uint8_t cls = buffer[2];
      uint8_t id = buffer[3];

      if (cls==0x01 && id==0x02) {
        NAV_POSLLH* posllh = (NAV_POSLLH*) (buffer+6);
        int32_t lat = posllh->lat;
        System_printf("ubx posllh %d %d\n",posllh->lat,posllh->lon);
      }
      if (cls==0x01 && id==0x07) {
        NAV_PVT* pvt = (NAV_PVT*) (buffer+6);
        int valid_fully = (((pvt->valid) >> 2) & 1);
        int valid_time = (((pvt->valid) >> 1) & 1);
        int valid_date = (((pvt->valid)     ) & 1);

        System_printf("ubx pvt timevalid=%d fix=%02x numSV=%d %d %d\n",valid_fully, pvt->fixType, pvt->numSV, pvt->lat, pvt->lon);

        if (valid_fully==1 && valid_time==1 && valid_date==1) {
          uint32_t t = utc2unix(pvt->year,pvt->month,pvt->day,pvt->hour,pvt->min,pvt->sec);
          if (now-t > 2 || t-now > 2) {
            Seconds_set(t);
            clockMineValid = 1;
            leds_slow(LEDS_CLOCK);
            //ssd1306_printText("SDCard N GPS  T", 3);
          }
        }
        if ( ((pvt->flags) & 1)==1 || pvt->fixType==2 || pvt->fixType==3 ) {
          System_printf("loc: %d %d %d\n",pvt->lon,pvt->lat,pvt->hMSL);
          gps = 1; // we have a GNSS fix
        }
      }
      else if (cls==0x01 && id==0x21) {
        NAV_TIMEUTC* timeutc = (NAV_TIMEUTC*) (buffer+6);
        int valid_utc = (((timeutc->valid) >> 2) & 1);
        int valid_wkn = (((timeutc->valid) >> 1) & 1);
        int valid_tow = (((timeutc->valid)     ) & 1);
        System_printf("ublox timeutc valid=%d\n",valid_utc);
        if (valid_utc==1 && valid_wkn==1 && valid_tow==1) {
          uint32_t t = utc2unix(timeutc->year,timeutc->month,timeutc->day,timeutc->hour,timeutc->min,timeutc->sec);
          uint32_t now = Seconds_get();
          if (now-t > 2 || t-now > 2) {
            Seconds_set(t);
            clockMineValid = 1;
            leds_slow(LEDS_CLOCK);
            //ssd1306_printText("SDCard N GPS  T", 3);
          }
        }

      }
      //else if (class==0x05 && id==0x00) { System_printf("ubx NACK for %02x %02x\n",buffer[6],buffer[7]); }
      //else if (class==0x05 && id==0x01) { System_printf("ubx ACK for %02x %02x\n",buffer[6],buffer[7]);  }
      else System_printf("ublox class=%02x id=%02x len=%d\n",cls,id,d.length);

      // now log the UBLOX message
      // use the first 6 bytes of the buffer (arriving as B5,62,class,id,pktLenLow,pktLenHigh)
      // as local time stamp plus class and id.
      //now = Seconds_get();
      buffer[4] = cls;
      buffer[5] = id;
      buffer[0] = (uint8_t) ((now      ) & 0xFF);
      buffer[1] = (uint8_t) ((now >>  8) & 0xFF);
      buffer[2] = (uint8_t) ((now >> 16) & 0xFF);
      buffer[3] = (uint8_t) ((now >> 24) & 0xFF);

      if (loggerState == LOGGER_LOGGING) {
        System_printf("ublox logging now=%d %u %u %u %u %u %u\n",now,
                      buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
        loggerLog(CATALOG_DATATYPE_SENSOR_UBLOX_DATA, buffers[ d.id ], d.length - 2 /* drop the checksum */);
      }


    }
    if (buffer[0] == '$') {
      j++;
    }
    //if (buffers[d.id][0] != '$' && buffers[d.id][0]!=0xB5) System_printf("GPS unknown message\n");
    Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
  }
  System_printf("got %d ubx + %d nmea messages\n",i,j);
  if (i==0) { // send a wakeup packet
    uint8_t wakeup = 0xFF;
    uartBlockingWrite(GPS_UART, &wakeup,1);
  }

  /*
  while (ubloxPoll());
  switch (ubloxState) {
  case UBLOX_STATE_NORMAL:
    leds_slow(LEDS_GPS);
    break;
  case UBLOX_STATE_PORT_FAILURE:
    leds_on(LEDS_GPS);
    break;
  default:
    leds_fast(LEDS_GPS);
    break;
  }
*/

}

/*
 * The logger is initialized by the radio task, so that it knows where to
 * send received packets to (to the logger or to the UART to the host.
 */

static uint8_t  BME280Buffer[4 + 6*3*4];
static uint8_t  BME280BufferPointer = 0xFF; // a sign that we have not started sensing yet.
static uint32_t bme280LastSenseTime;
static uint8_t  bme280ConfigurationLogged = 0;

static void basestationBME280Monitor() {
  uint32_t now = Seconds_get();

  System_printf("bme280 mon %d\n",now);

  if (BME280BufferPointer == 0xFF) { // we never used the sensor yet
    bme280SensorInit(0,7 /* sense all */);
    *((uint32_t*) BME280Buffer) = now; // sensing starts now
    BME280BufferPointer = 4; // next time will be the first in the buffer
    bme280Start();
    bme280LastSenseTime = now;

    return;
  }

  if (now - bme280LastSenseTime < 10) return;

  size_t filled;
  uint16_t rc = bme280GetDataAll( BME280Buffer+BME280BufferPointer, sizeof(BME280Buffer)-BME280BufferPointer, &filled);
  if (filled == 0) {
    // nothing filled, we assume that this is because there is no more space in the buffer, so log it
    if (loggerState==LOGGER_LOGGING) {
      if (bme280ConfigurationLogged==0) {
        const uint16_t configData[] = { CATALOG_DATATYPE_SENSOR_BME280_DATA // sensor type
                                      , 10   // interval in seconds
                                      , 0    // burst duration in ms
                                      , 0    // sample rate within burst
                                      , 7    // config; here 7 means all sensors, pressure, temperature, humidity
                                      };
        System_printf("bme280 logging sensor config length %d\n",sizeof(configData));
        bme280ConfigurationLogged = 1;
        loggerLog(CATALOG_DATATYPE_SENSOR_CONFIG, (uint8_t*) configData, sizeof(configData));
      }
      System_printf("bme280 data logging! length %d\n",BME280BufferPointer);
      loggerLog(CATALOG_DATATYPE_SENSOR_BME280_DATA, (uint8_t*) BME280Buffer, BME280BufferPointer);
    }
    *((uint32_t*) BME280Buffer) = bme280LastSenseTime;
    BME280BufferPointer = 4;
    // read data again, now there should be enough space
    rc = bme280GetDataAll( BME280Buffer+BME280BufferPointer, (4+120)-BME280BufferPointer, &filled);
  }
  System_printf("bme280 data %d %d\n",filled,BME280BufferPointer);
  BME280BufferPointer += filled;
  bme280LastSenseTime = now;
  bme280Start();
}


static void basestationLoggerTaskFunction(UArg arg0, UArg arg1) {
  buffer_descriptor d;

  uint32_t now;  // = Seconds_get(); //Timestamp_get32();
  uint32_t lastPeriodicAction   = 0; // Timestamp_get32() - timestampFreq; // one second ago.

  while (1) {
    now  = Seconds_get();
    //now  = Timestamp_get32();

    //if (now > last + 10*timestampFreq || now < 10*timestampFreq /* wrap around */) {
    if (now - lastPeriodicAction >= 1) {
      //System_printf("more than a second, sending state to PC %d %d %d\n",now,last, timestampFreq);
      lastPeriodicAction = now;

      consoleFlush();

      basestationUbloxMonitor();

      consoleFlush();

      basestationBME280Monitor();

      consoleFlush();
      //uint32_t pressure = 0; // dummy
      //size_t   bytes_filled;
      //bme280GetData(&pressure, sizeof(pressure), &bytes_filled);
      //bme280Start();
      //bme280TimeStamp = now;
      //System_printf("bme280 pressure is %d bytes_filled %d\n",pressure, bytes_filled);

      basestationLoggerMonitor();

      consoleFlush();
    }

    if (!Mailbox_pend(loggingMailbox, &d, 1*1000000 / Clock_tickPeriod)) {
      // nothing to log
      continue;
    }

    /*
     * Got a packet to log.
     * The structure of the incoming packet is
     * 1 byte: delivery length (excluding this byte)
     * 1 byte: payload length
     * payloadLength bytes: data from peer
     * 1 byte: rssi
     * 4 bytes: RAT time stamp (useless for logging)
     * 1 byte: radio setup
     */

    uint8_t deliveryLength = (uint8_t)  vildehayeGetUint32(buffers[ d.id ]  ,1);
    uint8_t payloadLength  = (uint8_t)  vildehayeGetUint32(buffers[ d.id ]+1,1);
    uint8_t rssi           = (uint8_t)  vildehayeGetUint32(buffers[ d.id ]+1+1+payloadLength,1);
    System_printf("loggging packet deliv %d %d payload %d rssi %d\n",d.length,deliveryLength,payloadLength,rssi);

    /*
     * Log it with the time stamp (we replace the RAT by absolute seconds) and rssi.
     */

    now = Seconds_get();
    uint8_t* rxpacket = buffers[ d.id ];
    uint8_t tsp = 1+1+payloadLength+1;
    uint32_t ts;
    rxpacket[tsp  ] = (now      ) & 0xFF;
    rxpacket[tsp+1] = (now >>  8) & 0xFF;
    rxpacket[tsp+2] = (now >> 16) & 0xFF;
    rxpacket[tsp+3] = (now >> 24) & 0xFF;

    if (loggerState == LOGGER_LOGGING)
      loggerLog(CATALOG_DATATYPE_VILDEHAYE_PACKET, buffers[ d.id ]+1+1, payloadLength+1+4);

    Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
  }
}

/***********************************************************************/
/* radio receive task function                                         */
/***********************************************************************/

static void basestationRadioReceiveTaskFunction(UArg arg0, UArg arg1) {
  while(1) {
    System_printf("waiting for a rx packet ready buffer is %d\n",readyBuffer.id);
    Semaphore_pend(packetReceivedSemaphore, BIOS_WAIT_FOREVER);

    leds_blink(LEDS_RX, 1); // received
    leds_blink(LEDS_RX2, 1); // received

  	// previous ready buffer is now used in queue, get another
  	//Mailbox_pend(freeMailbox, &readyBuffer,  BIOS_WAIT_FOREVER);

  	receiveGetBuffer();

  	uint8_t* rxpacket = buffers[ incomingBuffer.id ];
  	//uint8_t elementLength = rxpacket[0];
  	uint8_t payloadLength = rxpacket[1];
  	//uint8_t rssi          = rxpacket[1+1+payloadLength];
  	//uint8_t tsp = 1+1+payloadLength+1;
  	//uint32_t ts =  rxpacket[tsp] | (rxpacket[tsp+1]<<8) | (rxpacket[tsp+2]<<16) | (rxpacket[tsp+3]<<24);
  	//uint8_t  status       = rxpacket[1+1+payloadLength+1+4];

  	rxpacket[1+1+payloadLength+1+4] = radioSetupInUse; // replace the status by the radio setup index

  	rxpacket[1+1+payloadLength+1+4] = radioSetupInUse + 16; // setups >= 16 signal an incoming packet from radio
  	countRx++;
  	Mailbox_post(packetHandlingMailbox, &incomingBuffer, BIOS_WAIT_FOREVER);
  }
}

// todo: we probably do not need this
void basestation_gotoSetup(uint8_t c) {
  System_printf("goto setup %d\n",c);
  radioSetupInUse = 0;
}

/***********************************************************************/
/* wakeup intents                                                      */
/***********************************************************************/

#define WAKEUP_SIZE 8
#define WAKEUP_NOT_FOUND 0xFF
#define WAKEUP_DELETED   0xFE

static uint64_t tags[WAKEUP_SIZE];
static uint8_t  wkup[WAKEUP_SIZE];
static uint8_t  wkupCount = 0;

static uint8_t wakeupGet(uint64_t tag, uint8_t current) {
  int i;
  for (i=0; i<wkupCount; i++) {
    if (tags[i]==tag) {
      if (wkup[i] != current) return wkup[i];

      // otherwise, the tag is in the desired configuration, erase the wakeup intent

      while (i < wkupCount-1) {
        wkup[i] = wkup[i+1];
        tags[i] = tags[i+1];
        i++;
      }
      wkupCount--;
      return WAKEUP_DELETED;
    }
  }
  return WAKEUP_NOT_FOUND;
}

static void wakeupSet(uint64_t tag, uint8_t wu) {
  int i;

  // is it just a change?
  for (i=0; i<wkupCount; i++) {
    if (tags[i] == tag) {
      wkup[i] = wu;
      return;
    }
  }

  // if the list is full, loose the head.
  if (wkupCount == WAKEUP_SIZE) {
    for (i=0; i<wkupCount-1; i++) {
      wkup[i] = wkup[i+1];
      tags[i] = tags[i+1];
    }
    wkupCount--;
  }

  // now there is space, add to the tail
  wkup[ wkupCount ] = wu;
  tags[ wkupCount ] = tag;
  wkupCount++;
}

/***********************************************************************/
/* vildehaye protocol callbacks                                        */
/***********************************************************************/

static uint64_t peerTagId       = 0;
static uint64_t basestationId   = 0;
static uint8_t  peerConfiguration;
static uint8_t  peerHasData;

uint32_t vildehayeCallbackTagId(uint16_t typeCode, uint64_t id) {
  switch (typeCode) {
  case VH_SOURCE_ID:
    peerTagId = id;
    return VH_CONTINUE;

  case VH_DESTINATION_ID:
    if (id==0) return VH_CONTINUE;
    else       return VH_ABORT;

  case VH_DEF_ID:
    basestationId = id;
    return VH_CONTINUE;

  default:
    return VH_CONTINUE;

  }
}

uint32_t vildehayeCallbackClock(uint16_t typeCode, uint32_t clock) {
  switch (typeCode) {
  case VH_LOCAL_CLOCK:
    clockRemote = clock;
    break;
  case VH_SET_CLOCK:
    Seconds_set( clock );
    clockMineValid = 1;
    //Display_printf(display, 11, 0, "Clock set!");
    leds_slow(LEDS_CLOCK);
    break;
  default:
    break;
  }
  return VH_CONTINUE;
}

static uint8_t configured = 0; // do we already have a radio setup configuration?
#ifdef MOVED
static uint8_t radioOpen = 0;
static  RF_CmdHandle rxcmd;
#endif

uint32_t vildehayeCallbackRadioSetups(uint16_t typeCode, const uint8_t* radioData, uint16_t radioDataLength) {

  radioReceiveStop();  // cancel receive operation if active
  //radioShutdown();

#ifdef MOVED
  RF_Params rfParams;
  RF_Params_init(&rfParams);
  rfParams.nInactivityTimeout = 100; // in usecs

  if (radioOpen) { // meaning we are in rx
    RF_cancelCmd(rfHandle,rxcmd,0 /* 0=abrupt; 1=gracefull */);
    RF_close(rfHandle);
    radioOpen = 0;
  }
#endif

  radioSetup_configureFromBuffer(radioData, radioDataLength);

  radioSetupInUse = 0;

  radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 0 /* receive */, RADIO_DBM_DEFAULT);
  radioReceiveStart();

#ifdef MOVED
  System_printf("configuring radio setup\n");
  /* Request access to the radio */
  radioOpen = 1;
  rfHandle = RF_open(&rfObject, &(radio_mode[radioSetupInUse]), (RF_RadioSetup*)&(radio_cmd_prop_div_setup[radioSetupInUse]), &rfParams);

  /* Set the frequency */
  RF_runCmd(rfHandle, (RF_Op*)&(radio_cmd_fs[radioSetupInUse]), RF_PriorityNormal, NULL, 0);


  /* Modify CMD_PROP_RX command for application needs */
  (radio_cmd_prop_rx[radioSetupInUse]).status = IDLE;
  (radio_cmd_prop_rx[radioSetupInUse]).pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
  (radio_cmd_prop_rx[radioSetupInUse]).rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
  (radio_cmd_prop_rx[radioSetupInUse]).rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
  (radio_cmd_prop_rx[radioSetupInUse]).maxPktLen = BUFFER_SIZE-NUM_APPENDED_BYTES-1;
  (radio_cmd_prop_rx[radioSetupInUse]).pktConf.bRepeatOk  = 0; // End operation after receiving a packet correctly
  (radio_cmd_prop_rx[radioSetupInUse]).pktConf.bRepeatNok = 1; // Go back to sync search after receiving a packet with CRC error

  /* Enter RX mode and stay forever in RX */
  //RF_runCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_rx[radio_setup]), RF_PriorityNormal, &callback, IRQ_RX_ENTRY_DONE);
  //RF_postCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_rx[radio_setup]), RF_PriorityNormal, &receiveCallback, IRQ_RX_ENTRY_DONE);

  System_printf("receive task done setting up, waiting for packets\n");

  System_printf("posting rx\n");
  radio_cmd_prop_rx[radioSetupInUse].status = IDLE;
  rxcmd = RF_postCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_rx[radioSetupInUse]), RF_PriorityNormal, &receiveCallback, IRQ_RX_ENTRY_DONE);
#endif

  configured = 1;
  leds_off(LEDS_TX);
  leds_off(LEDS_RX);
  leds_off(LEDS_TX2);
  leds_off(LEDS_RX2);

  //Display_printf(display, 10, 0, "Radio configured!");

  return VH_CONTINUE;
}

uint8_t  logAck = 0;     // whether we received an item to log
uint32_t logItemAddress;

uint32_t vildehayeCallbackLog(uint16_t typeCode, const uint8_t* logData, uint16_t logDataLength) {
  switch (typeCode) {
  case VH_LOG_ITEM:
    logItemAddress = vildehayeGetUint32(logData,4);
    logAck = 1;
    break;
  case VH_LOG_ITEM_NEW:
    logItemAddress = vildehayeGetUint32(logData+4,4); // skip log creation time; not tested yet
    logAck = 1;
    break;
  case VH_LOG_ACK:
    // should never happen in a base station
    break;
  default:
    break;
  }

  return VH_CONTINUE;
}

uint32_t vildehayeCallbackBasestationState(uint16_t typeCode, const uint8_t* data, uint16_t len) {
  System_printf("bs state len %d value[0] %d\n",len,data[0]);
  if (data[0] == 0) loggerState = LOGGER_STATE_UNKNOWN;
  else              loggerState = LOGGER_LOGGING;
  return VH_CONTINUE;
}

uint32_t vildehayeCallbackWakeupIntents(uint16_t typeCode, const uint8_t* data, uint16_t len) {
  uint64_t tag;
  uint8_t  wu;

  int i;

  System_printf("wakeup binary = ");
  for (i=0; i<len; i++) System_printf("%02x ",data[i]);
  System_printf("\n");

  i=0;
  while (i < len) {
    tag = vildehayeGetUint64(data+i,8);
    i += 8;
    wu = vildehayeGetUint32(data+i,1);
    i++;
    //System_printf("wakeup intent %d -> %d i=%d\n",(uint32_t) (tag%1000000L),(uint32_t) tag,wu,i);
    wakeupSet(tag,wu);
  }

  return VH_CONTINUE;
}

uint32_t vildehayeCallbackTagState(uint16_t typeCode, const uint8_t* data, uint16_t len) {
  uint8_t  flags;

  flags = vildehayeGetUint32(data,1);
  peerConfiguration = flags & VH_TAGSTATE_CONFIGURATION_INDEX_MASK;

  peerHasData = ((flags & VH_TAGSTATE_HAS_DATA) != 0);

  return VH_CONTINUE;
}

uint32_t vildehayeCallbackSensorConfig(uint16_t typeCode, const uint16_t* config, uint8_t configSize) {
  return VH_CONTINUE;
}

/***********************************************************************/
/* packet handling task                                                */
/***********************************************************************/

/*
 * This task opens the radio and posts a receive command.
 * Responses from the receive command are not handled in this task but in the rx task.
 * Once the receive command is posted, the task repeatedly reads a packet from the tx mailbox.
 *
 * If the packet is marked with setup 0xFF (UART communication) or setup which is 16 or higher
 * (meaning it was received by the radio), the packet is send to the uart mailbox. It will be
 * sent to the host from there. In this case,
 * the radio receive command is posted again. If the setup is 16 or higher, it is adjusted down
 * before sending to the uart mailbox.
 *
 * If the packet has setup between 0 and 15, it is transmitted by the radio. To do that, we cancel
 * the receive command, post a transmit command with the time stamp given in the packet, then
 * release the buffer, and post the receive command again.
 */

#define HOST_COMM_TIMEOUT 10

static vildehaye_packet_t vildehayeResponsePacket;
static uint8_t            responseBuffer[64];
static uint8_t            beaconCounter = 0;

static void basestationPacketHandlingTaskFunction(UArg arg0, UArg arg1) {

  leds_on(LEDS_CLOCK);    // indicating that we do not have a GPS lock
  leds_on(LEDS_SDCARD); // indicating that we do not have an SD card

  //i2cInit(); // now done in main.c

  uint8_t result = bme280SensorInit(0, 7); // sense pressure, temperature, humidity
  System_printf("bme280 Init result %d (ok=%d, notfound=%d)\n",result,BME280_OK, BME280_E_DEV_NOT_FOUND);

  // logging base stations have a BME280

  //loggingBasestation = (result == BME280_OK);

  /*
   * potentially set radio setup, wakeup intents, and base station type
   */
#ifdef DeviceFamily_CC13X0
  uint8_t* cdata = (uint8_t*) 0x0001e000; // configuration page
#endif
#ifdef DeviceFamily_CC13X2
  uint8_t* cdata = (uint8_t*) 0x00054000; // configuration page
#endif
  uint16_t length = *((uint16_t*) cdata);
  if (length == 0xFFFF) {
    System_printf("configuration data missing, not an issue for a basestation\n");
  } else {
    System_printf("configuration length %d\n",length);
    vildehayeHandlePacketNaked(cdata+2, length);
  }

#ifdef BASESTATION_TYPE_FORCE_TETHERED
  basestationType = BASESTATION_TYPE_TETHERED;
#endif
#ifdef BASESTATION_TYPE_FORCE_LOGGING
  basestationType = BASESTATION_TYPE_LOGGING;
#endif

  if (basestationType == BASESTATION_TYPE_UNKNOWN) {
      uint32_t pinValue = PIN_getInputValue(BOOSTERPACK_SENSE);
      basestationType = (pinValue == 0) ? // pulled down
                        BASESTATION_TYPE_TETHERED
                        :
                        BASESTATION_TYPE_LOGGING;
  }

  if (basestationType == BASESTATION_TYPE_LOGGING) {
    System_printf("STANDALONE LOGGING BASESTATION!\n");

    ssd1306_init();
    ssd1306_printText("Sivan Toledo",     0);
    ssd1306_printText("Tel-Aviv Unvrsty", 1);
    ssd1306_printText("Vildehaye BS",     2);

    bme280Start();

    uartTasks_init(GPS_UART, 9600, 1, FRAMING_UBLOX);

    basestationUbloxInit();

    //ubloxInit(); // really initializes only i2c


    System_printf("sleep 450ms more\n");
    Task_sleep(450000 / Clock_tickPeriod);

    leds_fast(LEDS_CLOCK);  // indicating that we do not have a GPS lock
    leds_fast(LEDS_SDCARD); // indicating that we do not have an SD card

    /*
     * Create the logging task and the mailbox to send data to it.
     * It's a mailbox with 1 slot, since we insert with zero timeout,
     * so if the logger is slow, we do not want all the buffers to go
     * to this mailbox.
     */

    loggingMailbox = Mailbox_create(sizeof(buffer_descriptor),1, NULL,NULL);

    Task_Params_init(&basestationLoggerUartTaskParams);
    basestationLoggerUartTaskParams.stackSize = LOGGING_TASK_STACK_SIZE;
    basestationLoggerUartTaskParams.priority = LOGGER_TASK_PRIORITY;
    basestationLoggerUartTaskParams.stack = &basestationLoggerUartTaskStack;
    basestationLoggerUartTaskParams.arg0 = (UInt)1000000;

    Task_construct(&basestationLoggerUartTask, basestationLoggerTaskFunction, &basestationLoggerUartTaskParams, NULL);

    //System_printf("call logger monitor");
    //basestationLoggerMonitor(); // try to initialize SD card...

  } else {
    System_printf("TETHERED BASESTATION!\n");

    /*
     * we keep fast flashing the SDCARD led; normally the base station will not have this led,
     * so it won't blink and won' bother anybody. But in logging base stations, it is important
     * to keep the signal that the SD card is not working, in case it was not properly inserted
     * at reset time.
     */

    /*
     * Start the UART tasks, again with a 1-slot transmit mailbox
     */
    uartTasks_init(HOST_UART, 115200, 1, FRAMING_SLIP);

    // packets are sent to the host via UART.
    loggingMailbox = uartTxMailbox;

    Task_Params_init(&basestationLoggerUartTaskParams);
    basestationLoggerUartTaskParams.stackSize = LOGGING_TASK_STACK_SIZE;
    basestationLoggerUartTaskParams.priority = BASESTATION_UART_TASK_PRIORITY;
    basestationLoggerUartTaskParams.stack = &basestationLoggerUartTaskStack;
    basestationLoggerUartTaskParams.arg0 = (UInt)1000000;

    Task_construct(&basestationLoggerUartTask, basestationUartTaskFunction, &basestationLoggerUartTaskParams, NULL);

  }

  //System_printf("packet handler initializing GPS\n");
  //ubloxInit();
  //----------------------------------
  //----------------------------------


  //displayInit();

  //Display_printf(display,1,0,"host never heard");

  System_printf("packet handler looping on incoming packets\n");

  //uint32_t lastPacketFromHostTime = 0;
  uint32_t lastNackTime = 0; // last time the logging mailbox was full
  //uint32_t previousLag = 0;
  //uint32_t now = 0;

  while (1) {
    buffer_descriptor d;
    System_printf("waiting for tx request\n");
    state = 0; // waiting for mailbox, rx posted
    //Mailbox_pend(basestationTxMailbox, &d, BIOS_WAIT_FOREVER);
    bool received = Mailbox_pend(packetHandlingMailbox, &d, 1*1000000 / Clock_tickPeriod);

    beaconCounter++;
	if (radioSetupInUse==0 && !received && basestationId!=0 && beaconCounter > 6) {
	    beaconCounter = 0;
        radioReceiveStop();
        vildehayeInitPacket(&vildehayeResponsePacket, responseBuffer, 0, 64);
        vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_SOURCE_ID, basestationId);
        radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 1 /* transmit */, RADIO_DBM_DEFAULT);
	    if (radioTransmit(responseBuffer, vildehayeResponsePacket.length, RADIO_SCHEDULE_NOW, 0, 0) == RADIO_SUCCESS) {
	        countTxOk++;
	        //rat_next = rat_time + 4000*tagPeriodMs;
	        //outcome = TX_OK;
	        leds_blink(LEDS_TX,  1);
	        leds_blink(LEDS_TX2, 1);
	    } else {
	        countTxErr++;
	        //if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
	        //rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
	        //System_printf("CMD_TX> %04x\n",(radio_cmd_prop_tx[slotRadioSetup]).status); // ADV
	        //outcome = TX_FAIL;
	        leds_blink(LEDS_TX,  2);
	        leds_blink(LEDS_TX2, 2);
	    }
	    radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 0 /* receive */, RADIO_DBM_DEFAULT);
	    radioReceiveStart();

	    goto loopEnd;
	}

    if (!received) goto loopEnd; // continue;

  	uint8_t* rxpacket = buffers[ d.id ];
  	//uint8_t elementLength = rxpacket[0];
  	uint8_t payloadLength = rxpacket[1];
  	//uint8_t rssi          = rxpacket[1+1+payloadLength];
  	uint8_t tsp = 1+1+payloadLength+1;
  	//uint32_t ts =  rxpacket[tsp] | (rxpacket[tsp+1]<<8) | (rxpacket[tsp+2]<<16) | (rxpacket[tsp+3]<<24);
  	uint32_t ts;
  	ts  = rxpacket[tsp+3];
  	ts <<= 8;
  	ts |= rxpacket[tsp+2];
  	ts <<= 8;
  	ts |= rxpacket[tsp+1];
  	ts <<= 8;
  	ts |= rxpacket[tsp  ];
  	//uint8_t  status       = rxpacket[1+1+payloadLength+1+4];

  	uint8_t setup = rxpacket[1+1+payloadLength+1+4];

  	System_printf("got packet setup %d ts %d\n",setup,ts);

  	if (setup == 0xFF) {
			System_printf("packet to the basestation!!!\n");

			vildehayeHandlePacketNaked(rxpacket+2,rxpacket[1]);

      Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

      //lastPacketFromHostTime = Seconds_get();
      //leds_slow(LEDS_ACKS); // indicate we have communication
  	}

  	if (setup >= 16 && setup != 0xFF) {
      System_printf("processing a radio packet\n");

      radioShutdown(); // we will prapre it for rx or tx again later.

      rxpacket[1+1+payloadLength+1+4] -= 16; // adjust back
  		timestampRx = ts;

      /*
       * Now log the packet!
       *
       * The mailbox goes either to the UART to the PC, or to the log.
       * These tasks are lower priority, so processing here should
       * continue uninterrupted until we transmit a response or
       * re-post the receive command.
       */

      bool logged = Mailbox_post(loggingMailbox, &d, BIOS_NO_WAIT);
      if (!logged) lastNackTime = Seconds_get();

  		/*
  		 * Sivan Jul/Aug 2018: quick replies for clock correction, acks
  		 */

  		// clear fields that might be set by incoming radio packet.
  		peerTagId   = 0;
  		peerConfiguration = 0xFF;
  		clockRemote = 0;
  		logAck      = 0;

  		uint8_t destSent = 0;

      vildehayeHandlePacketNaked(rxpacket+2,rxpacket[1]);

      //logAck      = 0; // for now, don't ack

      // prepare response, so far empty
      vildehayeInitPacket       (&vildehayeResponsePacket, responseBuffer, 0, 64);

      if (peerTagId != 0) {
        uint8_t peerClockValidity = CLOCK_VALIDITY_UNKNOWN;
        if (clockRemote!=0 && clockMineValid!=0) {
          if (abs( Seconds_get() - clockRemote) > 2) peerClockValidity = CLOCK_VALIDITY_INVALID;
          else                                       peerClockValidity = CLOCK_VALIDITY_VALID;
        }
        heardsetUpdate(peerTagId,peerClockValidity,peerHasData);
      }


      if ((peerTagId != 0)
          && (clockRemote != 0)
          && (clockMineValid != 0)
          && (abs( Seconds_get() - clockRemote) > 2)) {
        // the clock of the tag that sent the packet is way off, ours is okay, send correction packet

        if (!destSent) vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_DESTINATION_ID, peerTagId);
        destSent = 1;
        //vildehayeAddHeader        (&vildehayeResponsePacket, VH_WAKEUP, 1);
        //vildehayeAddUInt8         (&vildehayeResponsePacket, 0);
        vildehayeAddHeader        (&vildehayeResponsePacket, VH_SET_CLOCK, 4);
        vildehayeAddUInt32        (&vildehayeResponsePacket, Seconds_get());
      }

      if ((peerTagId != 0) && (logAck != 0) && logged && loggerState==LOGGER_LOGGING) {
        // got a log item, send ack. We may need to check that the base station is actually feeding or storing data!

        if (!destSent) vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_DESTINATION_ID, peerTagId);
        destSent = 1;
        vildehayeAddHeader        (&vildehayeResponsePacket, VH_LOG_ACK, 4);
        vildehayeAddUInt32        (&vildehayeResponsePacket, logItemAddress);
      }

      //uint8_t wakeupCmd = 0xff;

      if ((peerTagId != 0) && peerHasData && (peerConfiguration != 3) && loggerState==LOGGER_LOGGING) {
        // send a wakeup trigger to tell peer to go to data download configuration

        if (!destSent) vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_DESTINATION_ID, peerTagId);
        destSent = 1;
        vildehayeAddHeader        (&vildehayeResponsePacket, VH_WAKEUP, 1);
        // way above the number of configurations, so it indicates "move temporarily to download conf."
        vildehayeAddUInt8         (&vildehayeResponsePacket, 15);
      }

#if OLD_WAKEUP_TRIGGGERS
      //System_printf("tag %d conf %d wakeup? %d\n",(uint32_t) (peerTagId%1000000L),peerConfiguration,wakeupGet(peerTagId,peerConfiguration));
      if ((peerTagId != 0) && (peerConfiguration != 0xFF)
          && (wakeupGet(peerTagId,peerConfiguration)<4)
          && loggerState==LOGGER_LOGGING) { // Sivan added April 2019, wakeup only if we can log data
        //System_printf("tag %d conf %d wakeup! %d\n",(uint32_t) (peerTagId%1000000L),peerConfiguration,wakeupGet(peerTagId,peerConfiguration));
        if (!destSent) vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_DESTINATION_ID, peerTagId);
        destSent = 1;
        vildehayeAddHeader        (&vildehayeResponsePacket, VH_WAKEUP, 1);
        vildehayeAddUInt8         (&vildehayeResponsePacket, wakeupGet(peerTagId,peerConfiguration));
        //wakeupCmd = wakeupGet(peerTagId,peerConfiguration);
      }
#endif

      if ((peerTagId != 0) && (peerConfiguration != 0xFF)
          && (wakeupGet(peerTagId,peerConfiguration)<4)
          && (wakeupGet(peerTagId,peerConfiguration)!=peerConfiguration)) {
        System_printf("tag %d conf %d wakeup! %d\n",(uint32_t) (peerTagId%1000000L),peerConfiguration,wakeupGet(peerTagId,peerConfiguration));
        if (!destSent) vildehayeAddUIntWithHeader(&vildehayeResponsePacket, VH_DESTINATION_ID, peerTagId);
        destSent = 1;
        vildehayeAddHeader        (&vildehayeResponsePacket, VH_WAKEUP, 1);
        vildehayeAddUInt8         (&vildehayeResponsePacket, wakeupGet(peerTagId,peerConfiguration));
        //wakeupCmd = wakeupGet(peerTagId,peerConfiguration);
      }

      if (vildehayeResponsePacket.length > 0) {
        radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 1 /* transmit */, RADIO_DBM_DEFAULT);
        if (radioTransmit(responseBuffer, vildehayeResponsePacket.length, RADIO_SCHEDULE_REL_TS, ts, 250) == RADIO_SUCCESS) {
          countTxOk++;
          //rat_next = rat_time + 4000*tagPeriodMs;
          //outcome = TX_OK;
          leds_blink(LEDS_TX,  1);
          leds_blink(LEDS_TX2, 1);
        } else {
          countTxErr++;
          //if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
          //rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
          //System_printf("CMD_TX> %04x\n",(radio_cmd_prop_tx[slotRadioSetup]).status); // ADV
          //outcome = TX_FAIL;
          leds_blink(LEDS_TX,  2);
          leds_blink(LEDS_TX2, 2);
        }
#if MOVED
        (radio_cmd_prop_tx[radioSetupInUse]).status = IDLE;
        (radio_cmd_prop_tx[radioSetupInUse]).pktLen = vildehayeResponsePacket.length;
        (radio_cmd_prop_tx[radioSetupInUse]).pPkt   = responseBuffer;
        (radio_cmd_prop_tx[radioSetupInUse]).startTrigger.triggerType = TRIG_ABSTIME;
        (radio_cmd_prop_tx[radioSetupInUse]).startTrigger.pastTrig = 1;
        (radio_cmd_prop_tx[radioSetupInUse]).startTime = ts + 1000; // receive time plus one msec
        //(radio_cmd_prop_tx[radioSetupInUse]).startTime = RF_getCurrentTime() + 4000000; // receive time plus one msec

        //while (RF_getCurrentTime() < ts + 4000000);
        //Task_sleep( 1000000 / Clock_tickPeriod );

        //timestampTx = ts;
        //timestampPostTx = RF_getCurrentTime();
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_tx[radioSetupInUse]), RF_PriorityNormal, NULL, 0);
        switch ((radio_cmd_prop_tx[radioSetupInUse]).status) {
        case PROP_DONE_OK:
          countTxOk++;
          //rat_next = rat_time + 4000*tagPeriodMs;
          //outcome = TX_OK;
          leds_blink(LEDS_TX,  1);
          leds_blink(LEDS_TX2, 1);
          break;
        case ERROR_PAST_START:
          countTxTooLate++;
          //if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
          //rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
          //System_printf("+100us\n");
          //outcome = TX_FAIL;
          leds_blink(LEDS_TX,  3);
          leds_blink(LEDS_TX2, 3);
          break;
        default:
          countTxErr++;
          //if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
          //rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
          //System_printf("CMD_TX> %04x\n",(radio_cmd_prop_tx[slotRadioSetup]).status); // ADV
          //outcome = TX_FAIL;
          leds_blink(LEDS_TX,  2);
          leds_blink(LEDS_TX2, 2);
          break;
        }
#endif
        //Display_printf(display, 8,0,"wu %d %d",wakeupCmd,ts);
      } /* if there is a packet to send... */

  		/*
  		 * Sivan Jul/Aug 2018, end.
  		 */

      /*
      if (loggerState == LOGGER_LOGGING) {
        //ystem_printf("should send this packet to the logger task, currently just releasing\n");
        Mailbox_post(basestationLoggerMailbox, &d, BIOS_NO_WAIT);
      } else {
        state=1; // sending to uart
        Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
        // post rx again
        state=0;
      }
      */

      /*
       * Post the receive command again
       */

#ifdef MOVED
  		radio_cmd_prop_rx[radioSetupInUse].status = IDLE;
  		rxcmd = RF_postCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_rx[radioSetupInUse]), RF_PriorityNormal, &receiveCallback, IRQ_RX_ENTRY_DONE);
#endif

        radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 0 /* receive */, RADIO_DBM_DEFAULT);
  		radioReceiveStart();
  	} // end of received-packet processing, if (setup >= 16 && setup != 0xFF)

  	if (setup < 16) { // the host tells us to transmit a packet on this setup
  		//RF_runDirectCmd(rfHandle, CMD_ABORT);
  		state=3; // canceling rx

#ifdef MOVED
  		RF_cancelCmd(rfHandle,rxcmd,0 /* 0=abrupt; 1=gracefull */);

  	  uint32_t rat_now = RF_getCurrentTime();
  	  uint32_t wait_us = ratDiffToUs(rat_now,ts);

  	  /* Make sure no RAT wrap around occurs while we wait for RAT; it does not work */
  	  if (rat_now > ts) { // wrap around
  	  	if (wait_us > 10000) // more than 10ms? if so, sleep until 10ms before deadline
  	  		Task_sleep((wait_us-10000) / Clock_tickPeriod);
  	  }

	  	(radio_cmd_prop_tx[radioSetupInUse]).status = IDLE;
	  	(radio_cmd_prop_tx[radioSetupInUse]).pktLen = payloadLength;
	  	(radio_cmd_prop_tx[radioSetupInUse]).pPkt   = rxpacket+2;
	  	(radio_cmd_prop_tx[radioSetupInUse]).startTrigger.triggerType = TRIG_ABSTIME;
	  	(radio_cmd_prop_tx[radioSetupInUse]).startTrigger.pastTrig = 0;
	  	(radio_cmd_prop_tx[radioSetupInUse]).startTime = ts;

	  	//System_printf("1\n");
	  	//timestampTx = ts;
	  	//timestampPostTx = RF_getCurrentTime();
	  	state=4; // posting tx, will wait
	  	RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_tx[radioSetupInUse]), RF_PriorityNormal, NULL, 0);
	  	switch ((radio_cmd_prop_tx[radioSetupInUse]).status) {
	  	case PROP_DONE_OK:
	  		countTxOk++;
	  		//rat_next = rat_time + 4000*tagPeriodMs;
	  		//outcome = TX_OK;
        leds_blink(LEDS_TX, 1);
        leds_blink(LEDS_TX2, 1);
	  		break;
	  	case ERROR_PAST_START:
	  		countTxTooLate++;
	  		//if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
	  		//rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
	  		//System_printf("+100us\n");
	  		//outcome = TX_FAIL;
        leds_blink(LEDS_TX, 3);
        leds_blink(LEDS_TX2, 3);
	  		break;
	  	default:
	  		countTxErr++;
	  		//if (rat_slack < rat_slack_max) rat_slack += rat_slack_step;
	  		//rat_next = RF_getCurrentTime() + 4000*tagPeriodMs + rat_slack;
	  		//System_printf("CMD_TX> %04x\n",(radio_cmd_prop_tx[slotRadioSetup]).status); // ADV
	  		//outcome = TX_FAIL;
        leds_blink(LEDS_TX, 2);
        leds_blink(LEDS_TX2, 2);
	  		break;
	  	}
#endif

	  radioReceiveStop();
      radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 1 /* transmit */, RADIO_DBM_DEFAULT);
      if (radioTransmit(rxpacket+2, payloadLength, RADIO_SCHEDULE_REL_TS, ts, 0) == RADIO_SUCCESS) {
        countTxOk++;
        leds_blink(LEDS_TX,  1);
        leds_blink(LEDS_TX2, 1);
      } else {
        countTxErr++;
        leds_blink(LEDS_TX,  2);
        leds_blink(LEDS_TX2, 2);
      }
      radioPrepare(radioSetupInUse, RADIO_IDENTITY_BITS, 0 /* receive */, RADIO_DBM_DEFAULT);

      state=5;
      Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

      // now repost the rx command
      state=0;

#if MOVED
      radio_cmd_prop_rx[radioSetupInUse].status = IDLE;
      rxcmd = RF_postCmd(rfHandle, (RF_Op*)&(radio_cmd_prop_rx[radioSetupInUse]), RF_PriorityNormal, &receiveCallback, IRQ_RX_ENTRY_DONE);
#endif
      radioReceiveStart();
  	} // end of transmission request

  	loopEnd:
  	{
  	  uint32_t now = Seconds_get();
      if (loggerState==LOGGER_LOGGING
          && (Seconds_get()-lastNackTime) > HOST_COMM_TIMEOUT) acks=1;
      else acks=0;
  	}
  } // end of infinite packet processing loop
}

/***********************************************************************/
/* uart task function (get uart packets from host and send pings       */
/***********************************************************************/

static vildehaye_packet_t vildehayePacket;

static void basestationUartTaskFunction(UArg arg0, UArg arg1) {

  buffer_descriptor d;

  //Types_FreqHz freq;
  //Timestamp_getFreq(&freq);
  //uint32_t timestampFreq = freq.lo;

  uint32_t now;  // = Seconds_get(); //Timestamp_get32();
  uint32_t lastPacketToHostTime   = 0; // Timestamp_get32() - timestampFreq; // one second ago.
  uint32_t lastPacketFromHostTime = 0;

	while (1) {
	  now  = Seconds_get();
	  //now  = Timestamp_get32();

    //if (now > last + 10*timestampFreq || now < 10*timestampFreq /* wrap around */) {
    if (now - lastPacketToHostTime >= 1) {
	    //System_printf("more than a second, sending state to PC %d %d %d\n",now,last, timestampFreq);
      lastPacketToHostTime = now;
	    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

	    vildehayeInitPacket(&vildehayePacket, (buffers[d.id]) + 2 /* skip elen, plen */, 0, 256-8);

	    if (!configured) {
	      vildehayeAddTagState(&vildehayePacket,
	          VH_TAGSTATE_NEEDS_CONFIGURATION | VH_TAGSTATE_NON_SESSION_CMD_OK,
	          256-8,
	          0); // tag state
	    } else {
	      vildehayeAddTagState(&vildehayePacket,
                                              VH_TAGSTATE_NON_SESSION_CMD_OK,
            256-8,
            0); // tag state
	    }

	    vildehayeAddHeader(&vildehayePacket, VH_BASESTATION_STATE, 1+1+4);
	    vildehayeAddUInt8 (&vildehayePacket, 1); // single bytes, unsigned
	    vildehayeAddUInt8 (&vildehayePacket, state);
	    vildehayeAddUInt8 (&vildehayePacket, countRx);
	    vildehayeAddUInt8 (&vildehayePacket, countTxOk);
	    vildehayeAddUInt8 (&vildehayePacket, countTxTooLate);
	    vildehayeAddUInt8 (&vildehayePacket, countTxErr);

	    //if (state==4) {
	    //  vildehayeAddHeader (&vildehayePacket, VH_TEST_UNSIGNED, 4);
	    //  vildehayeAddUInt32 (&vildehayePacket, timestampRx);
	    //  vildehayeAddHeader (&vildehayePacket, VH_TEST_UNSIGNED, 4);
	    //  vildehayeAddUInt32 (&vildehayePacket, timestampTx);
	    //  vildehayeAddHeader (&vildehayePacket, VH_TEST_UNSIGNED, 4);
	    //  vildehayeAddUInt32 (&vildehayePacket, timestampPostTx);
	    //}

	    //System_printf("local clock %d\n",Seconds_get());
	    vildehayeAddHeader (&vildehayePacket, VH_LOCAL_CLOCK, 4);
	    vildehayeAddUInt32 (&vildehayePacket, Seconds_get());

	    (buffers[d.id])[0] = 7 + vildehayePacket.length;
	    (buffers[d.id])[1] =     vildehayePacket.length;
	    (buffers[d.id])[2+vildehayePacket.length  ] = 0; // rssi
	    (buffers[d.id])[2+vildehayePacket.length+1] = 0; // timestamp
	    (buffers[d.id])[2+vildehayePacket.length+2] = 0; // timestamp
	    (buffers[d.id])[2+vildehayePacket.length+3] = 0; // timestamp
	    (buffers[d.id])[2+vildehayePacket.length+5] = 0xFF; // setup

	    d.length = 7 + vildehayePacket.length + 1;

      System_printf("Sending keepalive packet to host\n");

	    if (!Mailbox_post(uartTxMailbox, &d, BIOS_NO_WAIT)) {
	      // uart queue full, try again later.
        System_printf("uart tx queue full\n");
        //vildehayeHandlePacket(buffers[d.id], d.length);
        Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
	    }
      //last = Timestamp_get32(); // remember when we (tried to) send the packet to the host
	  } else {
	    //System_printf("waiting for uart packet (now %d last %d freq %d)\n",now, last, timestampFreq);
	    if (Mailbox_pend(uartRxMailbox, &d, 1*1000000 / Clock_tickPeriod)) { // 1s
	      lastPacketFromHostTime = Seconds_get();
	      // Sivan Aug 2019: now the host tells us whether it is logging or not
	      //loggerState = LOGGER_LOGGING; // we assume that the host will log the packets we upload
        System_printf("got packet from pc, posting to packet handler\n");
	      Mailbox_post(packetHandlingMailbox, &d, BIOS_WAIT_FOREVER);
	    } else {
	      now  = Seconds_get();
	      //if (now - lastPacketFromHostTime > HOST_COMM_TIMEOUT) loggerState = LOGGER_STATE_UNKNOWN;
	      System_printf("timeout reading uartRxMailbox, iterating\n");
	      //watchdog_pacify();
	    }
	  }
	}
}

uint32_t vildehayeCallbackModulesConfig(uint16_t typeCode, const uint16_t* config, uint8_t configSize) {
  int i;

  System_printf("Modules config (%d): ",configSize);
  for (i=0; i<configSize; i++) {
    System_printf("  %d: %d\n",i,config[i]);
    if (config[i] == 1) basestationType = BASESTATION_TYPE_TETHERED;
    if (config[i] == 2) basestationType = BASESTATION_TYPE_LOGGING;
  }

  return VH_CONTINUE;
}

/*
 * The initialization function of the basestation only starts
 * the UART task. The UART task will send requests for configuration,
 * and the response from the host will invoke basestationRadioSetup
 * which will perform the rest of the initialization.
 */

void basestationTask_init() {

  radioInit();

  displayPinHandle = PIN_open(&displayPinState, displayPinTable);
  if (displayPinHandle==NULL) {
    System_printf("could not allocate pins\n");
  } else {
    System_printf("success allocating pins\n");
  }

  System_flush();

  leds_on(LEDS_TX);
  leds_on(LEDS_TX2);
  leds_on(LEDS_RX);
  leds_on(LEDS_RX2);

#if NO_CONF_DATA_IN_BASESTATIONS
#ifdef DeviceFamily_CC13X0
  uint8_t* cdata = (uint8_t*) 0x0001e000; // configuration page
#endif
#ifdef DeviceFamily_CC13X2
  uint8_t* cdata = (uint8_t*) 0x00054000; // configuration page
#endif
  uint16_t length = *((uint16_t*) cdata);
  if (length == 0xFFFF) {
      System_printf("configuration data missing, not an issue for a basestation\n");
  } else {
      System_printf("configuration length %d\n",length);
      vildehayeHandlePacketNaked(cdata+2, length);
  }
#endif

	System_printf("basestation radio setup (starting radio tasks)\n");

	packetHandlingMailbox = Mailbox_create(sizeof(buffer_descriptor),BUFFER_COUNT, NULL,NULL);

	Task_Params_init(&basestationTaskParams);
	basestationTaskParams.stackSize = BASESTATION_RECEIVE_TASK_STACK_SIZE;
	basestationTaskParams.priority = BASESTATION_RECEIVE_TASK_PRIORITY;
	basestationTaskParams.stack = &basestationReceiveTaskStack;
	basestationTaskParams.arg0 = (UInt)1000000;

	Task_construct(&basestationReceiveTask, basestationRadioReceiveTaskFunction, &basestationTaskParams, NULL);

	Task_Params_init(&basestationTxTaskParams);
	basestationTxTaskParams.stackSize = BASESTATION_TASK_STACK_SIZE;
	basestationTxTaskParams.priority = BASESTATION_PACKET_HANDLER_TASK_PRIORITY;
	basestationTxTaskParams.stack = &basestationTxTaskStack;
	basestationTxTaskParams.arg0 = (UInt)1000000;

	Task_construct(&basestationTxTask, basestationPacketHandlingTaskFunction, &basestationTxTaskParams, NULL);
}

#endif // base station firmware
