/*
 * ublox.c
 *
 *  Created on: 27 ????? 2018
 *      Author: stoledo
 */

#include <stdint.h>
#include <stdbool.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/I2C.h>

#include "config.h"

#include "buffers.h"

#include "i2c.h"

#include "ublox.h"

Mailbox_Handle ubloxOutputMailbox;

uint8_t ubloxState = UBLOX_STATE_UNKNOWN;

/**************************** UNIX TIME STAMPS ***********************/
// https://github.com/msolters/make-unix-timestamp-c

#define SEC_PER_MIN         60
#define SEC_PER_HOUR        3600
#define SEC_PER_DAY         86400
#define MOS_PER_YEAR        12
#define EPOCH_YEAR          1970
#define IS_LEAP_YEAR(year)  ( (((year)%4 == 0) && ((year)%100 != 0)) || ((year)%400 == 0) )

static const int days_per_month[2][MOS_PER_YEAR] = {
  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
  { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
};

static const int days_per_year[2] = {
  365, 366
};

uint32_t utc2unix( uint16_t year, uint8_t mon, uint8_t day, uint8_t hrs, uint8_t min, uint8_t sec  ) {
  uint32_t ts = 0;

  //  Add up the seconds from all prev years, up until this year.
  uint8_t years = 0;
  uint8_t leap_years = 0;
  uint16_t y_k;
  for( y_k = EPOCH_YEAR; y_k<year; y_k++ ) {
    if( IS_LEAP_YEAR( y_k ) )
    {
      leap_years++;
    } else {
      years++;
    }
  }
  ts += ( (years*days_per_year[0]) + (leap_years*days_per_year[1]) ) * SEC_PER_DAY;

  //  Add up the seconds from all prev days this year, up until today.
  uint8_t year_index = (IS_LEAP_YEAR( year )) ? 1 : 0;
  uint8_t mo_k;
  for( mo_k = 0; mo_k<(mon-1); mo_k++ )
  { //  days from previous months this year
    ts += days_per_month[ year_index ][ mo_k ] * SEC_PER_DAY;
  }
  ts += (day-1) * SEC_PER_DAY; // days from this month

  //  Calculate seconds elapsed just today.
  ts += hrs * SEC_PER_HOUR;
  ts += min * SEC_PER_MIN;
  ts += sec;

  return ts;
}

/**************************** UNIX TIME STAMPS ***********************/

#define UBLOX_I2C_ADDRESS 0x42

typedef struct {
  uint32_t  iTOW;    // GPS Millisecond Time of Week
  uint16_t   gDOP;    // Geometric DOP
  uint16_t   pDOP;    // Position DOP
  uint16_t   tDOP;    // Time DOP
  uint16_t   vDOP;    // Vertical DOP
  uint16_t   hDOP;    // Horizontal DOP
  uint16_t   nDOP;    // Northing DOP
  uint16_t   eDOP;    // Easting DOP
} NAV_DOP;

typedef struct {
  uint32_t  iTOW;    // GPS Millisecond Time of Week
  uint8_t  gpsFix;  // Fix type: 0=none, 1=dead rek, 2=2D, 3=3D, etc.
  uint8_t  flags;   // Bitfield
  uint8_t  fixStat;
  uint8_t  flags2;  // Bitfield
  uint32_t  ttff;
  uint32_t  msss;
} NAV_STATUS;

typedef struct {
  uint8_t  rsvd;    // Reserved
  uint32_t  cpMesL;  // Carrier phase measurement [L1 cycles] (low 32 bits of IEEE 754 Double Precision)
  uint32_t  cpMesH;  // Carrier phase measurement [L1 cycles] (high 32 bits of IEEE 754 Double Precision)
  uint32_t  prMesL;  // Pseudorange measurement [m] (low 32 bits of IEEE 754 Double Precision)
  uint32_t  prMesH;  // Pseudorange measurement [m] (high 32 bits of IEEE 754 Double Precision)
  uint32_t  doMes;   // Doppler measurement [Hz] (IEEE 754 Single Precision)
  uint8_t  sv;      // Space Vehicle Number
  int8_t    mesQI;   // Nav Measurements Quality Indicator
  int8_t    cno;     // Signal strength C/No. (dbHz)
  uint8_t  lli;     // Loss of lock indicator (RINEX definition)
} RAW_ITEM;

typedef struct {
  uint32_t  iTOW;    // GPS Millisecond Time of Week
  uint16_t   week;    // Measurement GPS week number (Receiver Time).
  uint8_t  numSV;   // # of satellites following
  uint8_t  rsvd;    // Reserved
  RAW_ITEM       sat[];   // Array of RAW_ITEMs
} RXM_RAW;

typedef struct {
  uint8_t  chn;     // Channel
  uint8_t  svid;    // Satellite Id
  uint8_t  flags;   // B0=Used for Nav, B7=smoothed (see manual for others)
  uint8_t  quality; // Bits 3-0 - 0-Idle, 1=Search, 2=Aquired, 3=Unusable, 4=Code Lock, 5,6,7=Code and Carrier locked
  uint8_t  cno;     // Carrier to Noise Ratio (Signal Strength in dbHz)
  int8_t    elev;    // Elevation in integer degrees
  int            azim;    // Azimuth in integer degrees
  int32_t           prRes;   // Pseudo range residual in centimetres
} SVINFO_ITEM;

typedef struct {
  uint32_t  iTOW;    // GPS Millisecond Time of Week
  uint8_t  numCh;   // Number of channels
  uint8_t  gFlags;  // Bits 2-0 - 0 = Anataris, 1 = uBlox5, 2 = uBlox6
  uint16_t   rsvd;    // Reserved
  SVINFO_ITEM    sItem[]; // Array of SVINFO_ITEM
} NAV_SVINFO;

typedef struct {
  uint16_t   mask;
  uint8_t  dynModel;
  uint8_t  fixMode;
  int32_t           fixedAlt;
  uint32_t  fixedAltVar;
  int8_t    minElev;
  uint8_t  drLimit;
  uint16_t   pDop;
  uint16_t   tDop;
  uint16_t   pAcc;
  uint16_t   tAcc;
  uint8_t  staticHoldThresh;
  uint8_t  dgpsTimeOut;
  uint32_t  reserved2;
  uint32_t  reserved3;
  uint32_t  reserved4;
} CFG_NAV5;

typedef struct {
  char          swVersion[30];
  char          hwVersion[10];
  char          romVersion[30];
  char          extension[][30];
} MON_VER;

#if 0
union {
  //uint8_t  buf[BUF_SIZE];
  NAV_POSLLH     pos;
  NAV_DOP        dop;
  NAV_STATUS     stat;
  RXM_RAW        raw;
  CFG_NAV5       nav5;
  MON_VER        ver;
  NAV_SVINFO     svInfo;
} UBX_PKTS;
#endif

/***********************************************************************/
/* NAV TIMEUTC                                                         */
/***********************************************************************/
static void handle_NAV_TIMEUTC(uint8_t* buffer) {
  NAV_TIMEUTC* timeutc = (NAV_TIMEUTC*) buffer;

  int valid_utc = (((timeutc->valid) >> 2) & 1);
  int valid_wkn = (((timeutc->valid) >> 1) & 1);
  int valid_tow = (((timeutc->valid)     ) & 1);
  System_printf("valid? utc=%d wkn=%d tow=%d; %d/%d/%d unix=%d\n",
                valid_utc, valid_wkn, valid_tow, (int) (timeutc->day), (int) (timeutc->month), (int) (timeutc->year),
                utc2unix(timeutc->year,timeutc->month,timeutc->day,timeutc->hour,timeutc->min,timeutc->sec));
}

/***********************************************************************/
/* NAV POSLLH (Geodetic Solution)                                      */
/***********************************************************************/

static void handle_NAV_POSLLH(uint8_t* buffer) {
  NAV_POSLLH* posllh = (NAV_POSLLH*) buffer;
  System_printf("posllh = %d %d %d errs %d %d\n",
                posllh->lat,posllh->lon,posllh->hMSL,posllh->hAcc,posllh->vAcc);
}

/***********************************************************************/
/* command structure                                                   */
/***********************************************************************/

// message data should be in bytes 6 to 6+len-1 of the buffer
static uint8_t ubloxCommand(uint8_t* buffer, uint8_t class, uint8_t id, uint8_t len) {
  uint8_t ck_a, ck_b;
  int i;
  buffer[0] = 0xB5;
  buffer[1] = 0x62;
  buffer[2] = class;
  buffer[3] = id;
  buffer[4] = len & 0xFF;
  buffer[5] = len >> 8;

  ck_a = 0;
  ck_b = 0;
  for (i = 2; i < 6+len; i++) {
    ck_a = ck_a + buffer[i];
    ck_b = ck_b + ck_a;
  }
  buffer[6+len]   = ck_a;
  buffer[6+len+1] = ck_b;

  return 6+len+2;
}

/***********************************************************************/
/* configure ports                                                     */
/***********************************************************************/
typedef struct __attribute__((packed)) {
  uint8_t   portId;
  uint8_t   reserved1;
  uint16_t  txReady;
  uint32_t  mode;
  uint32_t  baudRate;    // only for UART
  uint16_t  inProtoMask;
  uint16_t  outProtoMask;
  uint16_t  flags;
  uint16_t  reserved2;
} CFG_PRT;

uint8_t ubloxConfigureUart1(uint8_t* buffer, uint32_t baudRate, bool nmea, bool ubx) {
  CFG_PRT* prt = (CFG_PRT*) (buffer + 6);

  uint16_t val = (ubx ? 1 : 0) | (nmea ? 2 : 0);

  // commented out April 2019, Sivan
  //process(buffer); // There should be an ACK or NACK

  prt->portId    = 1; // UART1
  prt->reserved1 = 0;
  prt->txReady   = 0;    // disabled
  prt->mode      = (3<<6) | (4<<9) | (0<<12); // 8N1
  prt->baudRate  = baudRate;
  prt->inProtoMask  = val;
  prt->outProtoMask = val;
  prt->flags        = 2;   // extended TX timeouts
  prt->reserved2 = 0;

  return ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT));
}

#if 0
static bool enableProtocols (uint8_t* buffer, bool nmea, bool ubx) {
  CFG_PRT* prt = (CFG_PRT*) (buffer + 6);

  uint16_t val = (ubx ? 1 : 0) | (nmea ? 2 : 0);

  System_printf("ublox enable protocols %d\n",val);

  // this is for I2C only
  prt->portId    = 0; // I2C
  prt->reserved1 = 0;
  prt->txReady   = 0;    // disabled
  prt->mode      = 0x84; // slave address, shifted by 1 to the left
  prt->baudRate  = 0;   // reserved in I2C
  prt->inProtoMask  = val; //3; // both protocols
  prt->outProtoMask = val; //val;
  prt->flags        = 2;   // extended TX timeouts
  prt->reserved2 = 0;

  if (!ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT))) return false;

  process(buffer); // There should be an ACK or NACK

  prt->portId    = 1; // UART1
  prt->reserved1 = 0;
  prt->txReady   = 0;    // disabled
  prt->mode      = (3<<6) | (4<<9) | (0<<12); // 8N1
  prt->baudRate  = 9600;
  prt->inProtoMask  = 0;
  prt->outProtoMask = 0;
  prt->flags        = 0;   // extended TX timeouts
  prt->reserved2 = 0;

  if (!ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT))) return false;
  process(buffer); // There should be an ACK or NACK

  prt->portId    = 2; // UART2

  if (!ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT))) return false;
  process(buffer); // There should be an ACK or NACK

  prt->portId    = 3; // USB
  prt->reserved1 = 0;
  prt->txReady   = 0;
  prt->mode      = 0;
  prt->baudRate  = 0;
  prt->inProtoMask  = 0;
  prt->outProtoMask = 0;
  prt->flags        = 0;   // extended TX timeouts
  prt->reserved2 = 0;

  if (!ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT))) return false;
  process(buffer); // There should be an ACK or NACK

  prt->portId    = 4; // SPI
  prt->reserved1 = 0;
  prt->txReady   = 0;
  prt->mode      = 0;
  prt->baudRate  = 0;
  prt->inProtoMask  = 0;
  prt->outProtoMask = 0;
  prt->flags        = 0;   // extended TX timeouts
  prt->reserved2 = 0;

  if (!ubloxCommand(buffer, 0x06, 0x00, sizeof(CFG_PRT))) return false;
  process(buffer); // There should be an ACK or NACK

  return true;
}

/***********************************************************************/
/* set update rate                                                     */
/***********************************************************************/

/*
 *  Set rate that measurements are taken
 *  msUpdate - milliseconds between updates (200 = 5Hz)
 */
typedef struct __attribute__((packed)) {
  uint16_t   measRate;
  uint16_t   navRate;    // Must equal 1
  uint16_t   timeRef;
} CFG_RATE;

static bool setUpdateRate (uint8_t* buffer, uint16_t msUpdate) {
  CFG_RATE* rate = (CFG_RATE*) (buffer + 6);

  rate->measRate = msUpdate;
  rate->navRate = 1;    // Must be 1 in protocol versions less than 18, up to 127 in higher
  rate->timeRef = 1;    // align measurements to; 0=utc, 1=gps, 2=GLONASS, 3=BeiDu, 4=Gallileo (>1 for 18 and higher)
  ubloxCommand(buffer, 0x06, 0x08, sizeof(CFG_RATE));
  process(buffer); // process the ack;
}
#endif

/***********************************************************************/
/* enable messages                                                     */
/***********************************************************************/

typedef struct __attribute__((packed)) {
  uint8_t  msgClass;
  uint8_t  msgId;
  uint8_t  rate[6]; // on this port...
} CFG_MSG;

uint8_t ubloxEnableMsg(uint8_t* buffer, uint8_t class, uint8_t msg, uint8_t rate) {
  CFG_MSG* enable = (CFG_MSG*) (buffer + 6);
  enable->msgClass = class;
  enable->msgId = msg;
  enable->rate[0] = 0;
  enable->rate[1] = rate;
  enable->rate[2] = 0;
  enable->rate[3] = 0;
  enable->rate[4] = 0;
  enable->rate[5] = 0;

  return ubloxCommand(buffer, 0x06, 0x01, sizeof(CFG_MSG));
}

#if 0
/*
 *  Set rate that measurements are taken
 *  msUpdate - milliseconds between updates (200 = 5Hz)
 */
void setUpdateRate (uint16_t msUpdate) {
  CFG_RATE rate;
  rate.measRate = msUpdate;
  rate.navRate = 1;    // Must be 1
  rate.timeRef = 1;    // Referenced to GPS time (?)
  ubloxCommand(0x06, 0x08, (uint8_t *) &rate, sizeof(rate));
}

  /*
   *  Set dynamic model, as follows:
   *    0 - Portable
   *    2 - Stationary
   *    3 - Pedestrian
   *    4 - Automotive
   *    5 - Sea
   *    6 - Airborne with <1g Acceleration
   *    7 - Airborne with <2g Acceleration
   *    8 - Airborne with <4g Acceleration
   */
void setDynamicModel (uint8_t model, int8_t elev) {
  CFG_NAV5 nav5;
  nav5.mask = 3;          // Set dynamic model and elev
  nav5.dynModel = model;
  nav5.minElev = elev;
#if 0
  nav5.fixMode = 0;
  nav5.fixedAlt = 0;
  nav5.fixedAltVar = 0;
  nav5.drLimit = 0;
  nav5.pDop = 0;
  nav5.tDop = 0;
  nav5.pAcc = 0;
  nav5.tAcc = 0;
  nav5.staticHoldThresh = 0;
  nav5.dgpsTimeOut = 0;
  nav5.reserved2 = 0;
  nav5.reserved3 = 0;
  nav5.reserved4 = 0;
  traceCmd(0x06, 0x24, (uint8_t *) &nav5, sizeof(nav5));
#endif
  ubloxCommand(0x06, 0x24, (uint8_t *) &nav5, sizeof(nav5));
}

void setup () {
 #if USE_SERIAL
  Serial1.begin(9600);
#else
  Wire.begin();
  Wire.beginTransmission(gpsAddress);
  send(0xFF);                      // Set regisaddress
  Wire.endTransmission();
#endif
  output.begin(115200);
  delay(4000);
#if USE_SERIAL
  output.println(F("U-Blox NEO-6P - Serial 1 Interface"));
#else
  output.println(F("U-Blox NEO-6P - DDC (I2C) Interface"));
#endif
  while (process())
    ;
  // Disable unused messages
  enableMsg(0x01, 0x02, 0);            // NAV-POSLLH - Geodetic Position Solution
  enableMsg(0x01, 0x06, 0);            // NAV-SOL - Navigation Solution Information (ECEF)
  enableMsg(0x01, 0x04, 0);            // NAV_DOP - Dilution of precision
  enableMsg(0x01, 0x01, 0);            // NAV-POSECEF - Position Solution in ECEF
  enableMsg(0x01, 0x32, 0);            // NAV-SBAS - SBAS Status Data
  enableMsg(0x01, 0x03, 0);            // NAV-STATUS - Receiver Navigation Status
  enableMsg(0x01, 0x30, 0);            // NAV-SVINFO - Space Vehicle Information
  enableMsg(0x02, 0x10, 0);            // RXM-RAW - Raw Measurement Data
  // Configure ports and other items
  ubloxCommand(0x0A, 0x04, 0, 0);           // Query MON-VER
  enableProtocols(true, false);        // CFG-PRT - Enable UBX, Disable NMEA
  setDynamicModel(2, 15);              // CFG-NAV5 - Navigation Engine Settings
  setUpdateRate(1000);                 // CFG-RATE (NEO-6P limited to 1000 ms)
#if 1
  // Verify that changes made by CFG-NAV5 have been applied
  ubloxCommand(0x06, 0x23, 0, 0);           // Query CFG-NAVX5 (responds with ACK?)
  ubloxCommand(0x06, 0x24, 0, 0);           // Query CFG-NAV5 (responds with ACK?)
#endif
  // Enable used messages
  enableMsg(0x01, 0x02, 1);            // NAV-POSLLH - Geodetic Position Solution
  enableMsg(0x01, 0x30, 1);            // NAV-SVINFO - Space Vehicle Information
}

#if DEBUG
int idx = 0;
#endif

#endif
/***********************************************************************/
/* UBX message reception                                               */
/***********************************************************************/

static uint8_t  state = 0;
static uint8_t  class, id, chk1, chk2;
static uint16_t pktLen, pktIdx;
static bool     acked;

static bool process(uint8_t* buffer) {
  //I2C_Transaction i2cTransaction;

  //bool processed = false;

  uint8_t regAddress = 0xFD;

  i2cTransaction.slaveAddress = UBLOX_I2C_ADDRESS;
  i2cTransaction.writeBuf = &regAddress;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = buffer;
  i2cTransaction.readCount = 2;
  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("I2C transaction failure\n");
    return false;
  }

  uint16_t count = (((uint16_t) buffer[0]) << 8) | buffer[1];

  //System_printf("ublox %d bytes in\n",count);

  if (count < 8) return false; // UBLOX PACKETS are at least 8 bytes long

  /* Now we read just the header, so that the actual message starts at the beginning of the buffer */

  int n = 0; // data in buffer
  int i = 0; // position in buffer
  while (count > 0) {

    //System_printf("CISN: %d %d %d %d",count,i,state,n);

    if (i >= n) { // read more data
      n = count;
      if (n > 256) n = 256;
      if (state < 6) n = 6-state; // don't read into the message itself
      if (state == 6 && n > pktLen+2) n = pktLen+2;

      i2cTransaction.slaveAddress = UBLOX_I2C_ADDRESS;
      i2cTransaction.writeBuf     = NULL;
      i2cTransaction.writeCount   = 0;
      i2cTransaction.readBuf      = buffer;
      i2cTransaction.readCount    = n;
      if (!I2C_transfer(i2c, &i2cTransaction)) {
        System_printf("I2C transaction failure\n");
        return false;
      }

      i = 0;

      //int j;
      //System_printf("GPS(%d): ",n);
      //for (j=0; j<6; j++) System_printf("%02x",buffer[j]);
      //System_printf("\n");
    }

    uint8_t cc = buffer[i];
    //System_printf("%02x,%d,%d\n,",cc,i,n);

    switch (state) {
    case 0:  // Wait for sync 1 (0xB5)
      chk1 = chk2 = 0;
      if (cc == 0xB5) state++;
      //System_printf("S0"); System_flush();
      break;
    case 1:  // wait for sync 2 (0x62)
      if (cc == 0x62) state++;
      else            state = 0;
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
        state = 0;
      }
      break;
    case 8:  // wait for checksum 2
      if (chk2 == cc) {
        // checksum is good...
        ubloxState = UBLOX_STATE_NORMAL;
        if (class==0x01 && id==0x02) handle_NAV_POSLLH(buffer);
        else if (class==0x01 && id==0x21) handle_NAV_TIMEUTC(buffer);
        else System_printf("ublox class=%02x id=%02x pktLen=%d i+1=%d\n",class,id,pktLen,i+1);

      } else {
        System_printf("Checksum2 failure\n");
      }
      state = 0;
      //System_printf("S8"); System_flush();
      break;
    }

    // continue the loop ...

    i++;
    count--;
  }
  return true;
}

#if 0
static bool processOld(uint8_t* buffer) {
  I2C_Transaction i2cTransaction;

  //bool processed = false;

  uint8_t regAddress = 0xFD;

  i2cTransaction.slaveAddress = UBLOX_I2C_ADDRESS;
  i2cTransaction.writeBuf = &regAddress;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = buffer;
  i2cTransaction.readCount = 2;
  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("I2C transaction failure\n");
    return false;
  }

  uint16_t count = (((uint16_t) buffer[0]) << 8) | buffer[1];

  //System_printf("ublox %d bytes in\n",count);

  if (count < 8) return false; // UBLOX PACKETS are at least 8 bytes long

  int n = 0; // data in buffer
  int i = 0; // position in buffer
  while (count > 0) {

    //System_printf("CISN: %d %d %d %d",count,i,state,n);

    if (i >= n) { // read more data
      n = count;
      if (n > 256) n = 256;

      i2cTransaction.slaveAddress = UBLOX_I2C_ADDRESS;
      i2cTransaction.writeBuf     = NULL;
      i2cTransaction.writeCount   = 0;
      i2cTransaction.readBuf      = buffer;
      i2cTransaction.readCount    = n;
      if (!I2C_transfer(i2c, &i2cTransaction)) {
        System_printf("I2C transaction failure\n");
        return false;
      }

      i = 0;

      //int j;
      //System_printf("GPS(%d): ",n);
      //for (j=0; j<6;  j++) System_printf("%02x",buffer[j]);
      //System_printf("\n");
    }

    uint8_t cc = buffer[i];
    //System_printf("%02x,%d,%d\n,",cc,i,n);

    switch (state) {
    case 0:  // Wait for sync 1 (0xB5)
      chk1 = chk2 = 0;
      if (cc == 0xB5) state++;
      //System_printf("S0"); System_flush();
      break;
    case 1:  // wait for sync 2 (0x62)
      if (cc == 0x62) state++;
      else            state = 0;
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
        state = 0;
      }
      break;
    case 8:  // wait for checksum 2
      state = 0;
      if (chk2 == cc) {
        System_printf("ublox class=%02x id=%02x\n",class,id);
        return true;
      } else {
        System_printf("Checksum2 failure\n");
        return false;
      }
      //System_printf("S8"); System_flush();
      //break;
    }

    // continue the loop ...

    i++;
    count--;
  }
  return true;
}
#endif

#if 0
void decodePacket () {
  // Note: process <pktLen> bytes
#if DEBUG
  output.println();
#endif
  if (cls == 0x01 && id == 0x02 && pktLen == 28) {
    // NAV_POSLLH packet
    output.print(UBX_PKTS.pos.iTOW, DEC);
    output.print(F(" - NAV_POSLLH - Lat: "));
    output.print((float) UBX_PKTS.pos.lat / 10000000.0, DEC);
    output.print(F(", Lon: "));
    output.print((float) UBX_PKTS.pos.lon / 10000000.0, DEC);
    output.print(F(", hAcc: "));
    output.print(UBX_PKTS.pos.hAcc, DEC);
    output.print(F(" mm"));
    output.println();
  } else if (class == 0x01 && id == 0x30) {
    // NAV_SVINFO packet
    output.print(UBX_PKTS.svInfo.iTOW, DEC);
    output.print(F(" - SVINFO - numCh: "));
    output.print(UBX_PKTS.svInfo.numCh, DEC);
    output.println();
    for (int ii = 0; ii < UBX_PKTS.svInfo.numCh; ii++) {
      output.print(F("  Id: "));
      decPrint3(UBX_PKTS.svInfo.sItem[ii].chn);
      output.print(F(", elev: "));
      decPrint3(UBX_PKTS.svInfo.sItem[ii].elev);
      output.print(F(", Azim: "));
      decPrint3(UBX_PKTS.svInfo.sItem[ii].azim);
      output.print(F(", cno: "));
      decPrint3(UBX_PKTS.svInfo.sItem[ii].cno);
      output.print(F(", prRes: "));
      decPrint5(UBX_PKTS.svInfo.sItem[ii].prRes);
      output.print(F(", used: "));
      output.print(((UBX_PKTS.svInfo.sItem[ii].flags & 0x01) != 0 ? "Y" : "N"));
      output.print(F(", smoothed: "));
      output.print(((UBX_PKTS.svInfo.sItem[ii].flags & 0x80) != 0 ? "Y" : "N"));
      output.println();
     }
  } else if (class == 0x02 && id == 0x10) {
    // RXM_RAW packet
    output.print(UBX_PKTS.raw.iTOW, DEC);
    output.print(F(" - RXM_RAW ("));
    output.print(UBX_PKTS.raw.numSV, DEC);
    output.println(F(" sats)"));
    for (int ii = 0; ii < UBX_PKTS.raw.numSV; ii++) {
      output.print(F("    "));
      output.print(F("Sat: "));
      int sat = UBX_PKTS.raw.sat[ii].sv;
      if (sat < 100)
        output.print(F(" "));
      if (sat < 10)
        output.print(F(" "));
      output.print(sat, DEC);
      output.print(F(", "));
      output.print(UBX_PKTS.raw.sat[ii].cpMesH, HEX);
      output.print(F(", "));
      output.print(UBX_PKTS.raw.sat[ii].cpMesL, HEX);
      output.println();
    }
  } else if (class == 0x06 && id == 0x23) {
    // CFG_NAVX5 Packet
    output.print(F("CFG_NAVX5 - usePPP: "));
    output.print((uint8_t) UBX_PKTS.buf[26], DEC);
    output.println();
  } else if (class == 0x06 && id == 0x24) {
    // CFG_NAV5 Packet
    output.print(F("CFG_NAV5 - dynModel: "));
    output.print(UBX_PKTS.nav5.dynModel, DEC);
    output.print(F(", minElev: "));
    output.print(UBX_PKTS.nav5.minElev, DEC);
    output.println();
  } else if (class == 0x0A && id == 0x04) {
    // MON_VER Packet (swVersion: 6.02 (36023), hwVersion: 00040007, romVersion: )
    output.print(F("MON_VER - swVersion: "));
    output.print((char *) &UBX_PKTS.ver.swVersion);
    output.print(F(", hwVersion: "));
    output.print((char *) &UBX_PKTS.ver.hwVersion);
    output.print(F(", romVersion: "));
    output.print((char *) &UBX_PKTS.ver.romVersion);
    if (pktLen > 70) {
      int exts = (pktLen - 70) / 30;
      for (int ii = 0; ii < exts; ii++) {
        output.print(F(", ext"));
        output.print(ii + 1, DEC);
        output.print(F(": "));
        output.print((char *) &UBX_PKTS.ver.extension[ii]);
      }
    }
    output.println();
  } else if (class == 0x05 && (id == 0x00 || id == 0x01) && pktLen == 2) {
    // ACK, or NAK packets
    acked = true;
    output.print(id == 1 ? "ACK" : "NAK");
    uint8_t aCls = UBX_PKTS.buf[0];
    uint8_t aId = UBX_PKTS.buf[1];
    if (aCls == 0x06 && aId == 0x01) {
      output.println(F(" CFG_MSG"));
    } else if (aCls == 0x06 && aId == 0x08) {
      output.println(F(" CFG_RATE"));
    } else if (aCls == 0x06 && aId == 0x00) {
      output.println(F(" CFG_PRT"));
    } else if (aCls == 0x06 && aId == 0x23) {
      output.println(F(" CFG_NAVX5"));
    } else if (aCls == 0x06 && aId == 0x24) {
      output.println(F(" CFG_NAV5"));
    } else {
      output.print(F(" class: "));
      output.print(aCls, HEX);
      output.print(F(", id: "));
      output.print(aId, HEX);
      output.println();
    }
  } else {
    output.print(F("Pkt class: "));
    output.print(class, HEX);
    output.print(F(", id: "));
    output.print(id, HEX);
    output.print(F(", len: "));
    output.print(pktLen, DEC);
    output.println();
    output.print(F("    "));
    for (int ii = 0; ii < pktLen; ii++) {
      unsigned jj = UBX_PKTS.buf[ii];
      if (jj < 16)
        output.print('0');
      output.print(jj, HEX);
      if ((ii & 0x0F) == 0x0F) {
        output.println();
        output.print(F("    "));
      } else {
        output.print(F(", "));
      }
    }
    output.println();
  }
}
#endif

bool ubloxPoll() {
  buffer_descriptor d;

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);

  bool doneSomething = process(buffers[d.id]);

  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

  return doneSomething;
}

/***********************************************************************/
/* INITIALIZATION                                                      */
/***********************************************************************/

#ifdef OBSOLETE
//2C_Params      i2cParams;

void ubloxInitOld() {
  I2C_init();
  I2C_Params      params;
  I2C_Params_init(&params);
  params.transferMode  = I2C_MODE_BLOCKING;
  params.bitRate       = I2C_100kHz;
  i2c = I2C_open(0 /* peripheral index */, &params);
  if (!i2c) {
    System_printf("I2C did not open");
    return;
  }

  return; // for testing UART mode

  buffer_descriptor d;

  Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
  enableProtocols(buffers[d.id], false, true );

  if (ubloxState == UBLOX_STATE_PORT_FAILURE) {
    System_printf("ublox no response to command, hardware problem!\n");
    return;
  } else {
    System_printf("ublox acked command\n");
  }


#if 0
  enableMsg(buffers[d.id], 0x01, 0x02, 0);            // NAV-POSLLH - Geodetic Position Solution
  enableMsg(buffers[d.id], 0x01, 0x06, 0);            // NAV-SOL - Navigation Solution Information (ECEF)
  enableMsg(buffers[d.id], 0x01, 0x04, 0);            // NAV_DOP - Dilution of precision
  enableMsg(buffers[d.id], 0x01, 0x01, 0);            // NAV-POSECEF - Position Solution in ECEF
  enableMsg(buffers[d.id], 0x01, 0x32, 0);            // NAV-SBAS - SBAS Status Data
  enableMsg(buffers[d.id], 0x01, 0x03, 0);            // NAV-STATUS - Receiver Navigation Status
  enableMsg(buffers[d.id], 0x01, 0x30, 0);            // NAV-SVINFO - Space Vehicle Information
  enableMsg(buffers[d.id], 0x02, 0x10, 0);            // RXM-RAW - Raw Measurement Data

  enableMsg(buffers[d.id], 0xF0, 0x04, 0);            // RMC
  enableMsg(buffers[d.id], 0xF0, 0x02, 0);            // GSA
  enableMsg(buffers[d.id], 0xF0, 0x00, 0);            // GGA

  enableProtocols(buffers[d.id], false, true );
  setUpdateRate(buffers[d.id], 10000);

  ubloxCommand(buffers[d.id], 0x0A, 0x04, 0); // poll for MON-VER
#endif

  Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);

  //while (true) ubloxPoll();
#if 0
  while (true) {
    I2C_Transaction i2cTransaction;
    uint16_t len;

    uint8_t regAddress = 0xFD;
    uint8_t buf[2];

    /*
    i2cTransaction.slaveAddress = 0x42;
    i2cTransaction.writeBuf = &regAddress;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = buf;
    i2cTransaction.readCount = 2;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
      System_printf("I2C transaction failure (NACK?)\n");
    }
*/
    len = 256 * (uint16_t) (buf[0]) + buf[1];

    if (len>0 && len!=0xFFFF) {
      buffer_descriptor d;
      Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
      i2cTransaction.writeCount = 0;
      i2cTransaction.readBuf = buffers[ d.id ];
      i2cTransaction.readCount = len >= 256 ? 256 : len;
      if (!I2C_transfer(i2c, &i2cTransaction)) {
        System_printf("I2C data transaction failure\n");
      } else {
        int i;
        System_printf("GPS(%d): ",len);
        for (i=0; i<i2cTransaction.readCount; i++) System_printf("%c",buffers[d.id][i]);
        System_printf("\n");
      }
      Mailbox_post(freeMailbox, &d, BIOS_WAIT_FOREVER);
    }

    System_printf("DDC buffer len = %d\n",len);
  }
#endif
}
#endif

void ubloxParseMessage(uint8_t* buffer, uint16_t length) {
  uint8_t class = buffer[2];
  uint8_t id = buffer[3];

  if (class==0x01 && id==0x02) handle_NAV_POSLLH(buffer+6);
  else if (class==0x01 && id==0x21) handle_NAV_TIMEUTC(buffer+6);
  else if (class==0x05 && id==0x00) { System_printf("ubx NACK for %02x %02x\n",buffer[6],buffer[7]); }
  else if (class==0x05 && id==0x01) { System_printf("ubx ACK for %02x %02x\n",buffer[6],buffer[7]);  }
  else System_printf("ublox class=%02x id=%02x len=%d\n",class,id,length);
}



/***********************************************************************/
/* END OF FILE                                                         */
/***********************************************************************/


