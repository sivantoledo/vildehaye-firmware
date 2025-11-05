/*
 * Sensor scheduler
 */

#include "config.h"

#if defined(USE_VH_SENSORS)

#include <stdint.h>
#include <limits.h>
#include <time.h>

#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/sysbios/hal/Seconds.h>

#include "sensors.h"
#include "logger.h"
#include "logger_catalog.h"
#include "leds.h"
#include "tag.h"
#include "i2c.h"

uint8_t message[32];

/**************************************************************************************/
/* schedule data structure                                                            */
/**************************************************************************************/

#define MAX_SENSOR_COUNT 8
#define ONE_SHOT -1
#define FOREVER  -2

typedef struct {
    //uint16_t type;
    uint16_t interval; // how often to start sensing, in periods;
     int16_t duration; // for how long to sense; -1 means one-shot sensing every interval, -2 forever
    uint16_t rate;     // sampling frequency (Hz), during the sensing duration; use centi-Hertz?
    uint16_t config;   // configuration of the sensor (over sampling, filtering, etc; sensor specific)

    uint16_t nextStart; // when to start sensing again
    uint16_t nextLog;   // when to log or buffer sensor data
    uint16_t nextStop;  // when to stop sensing

     int16_t (*handler)(uint8_t);
} sensor_schedule_t;

static sensor_schedule_t schedule[MAX_SENSOR_COUNT];

#define ACTION_START 0x01
#define ACTION_STOP  0x02
#define ACTION_LOG   0x04

/**************************************************************************************/
/* batmon sensor                                                                      */
/**************************************************************************************/

#include "sensors_batmon.h"

static uint32_t batmonBuffer[1 + 32];
static uint32_t batmonBufferPointer = 0;

static int16_t sensorsBatmonInit(uint8_t scheduleIndex) {
    return 0; // all okay
}

static int16_t sensorsBatmonHandler(uint8_t action) {
    System_printf("batmon action %02x\n",action);
    if (action == ACTION_START) {
        if (batmonBufferPointer==0) {
            batmonBuffer[ batmonBufferPointer++ ] = Seconds_get();
        }
        batmonBuffer[ batmonBufferPointer++ ] = batmonTemp();
        batmonBuffer[ batmonBufferPointer++ ] = batmonVoltage();
        if (batmonBufferPointer==33) {
            loggerLog(CATALOG_DATATYPE_SENSOR_BATMON_DATA, (uint8_t*) batmonBuffer, 33*sizeof(uint32_t));
            batmonBufferPointer = 0;
        }
    }

    return -1; // no need to call in the near future to log/buffer
}

/**************************************************************************************/
/* BMP3XX sensor (for tags with BMP384)                                               */
/**************************************************************************************/

#ifdef USE_VH_SENSOR_BMP3XX

#include "bmp3.h"
#include "bmp3_defs.h"

#include <ti/drivers/I2C.h>

extern    I2C_Handle      i2c;
extern    I2C_Transaction i2cTransaction;

static uint8_t         txBuffer[10];
static uint8_t         rxBuffer[30];

static uint8_t bmp3_addr = 0;

BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void* intf) {
    //System_printf(">> bmp3 i2c read i2c 0x%02x reg 0x%02x len %d intf %d\n",bmp3_addr, reg_addr, len, (int32_t) intf );
    //consoleFlush();
    //Task_sleep( 100*1000 / Clock_tickPeriod);

    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    //Fill TX Buffer with the register address
    txBuffer[0] = reg_addr;

    //Point I2C parameters to correct values. Set up to read "cnt" number of bytes.
    i2cTransaction.slaveAddress = bmp3_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = len;

    //Perform I2C read
    I2C_transfer(i2c, &i2cTransaction);

    return rslt;
}

BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void* intf) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    int i = 1;

    //Fill TX Buffer with the register address and subsequent data bytes
    txBuffer[0] = reg_addr;
    for(i=1; i<len+1;i++)
        txBuffer[i] = reg_data[i-1];

    //Point TI-RTOS I2C parameters to correct values
    i2cTransaction.slaveAddress = bmp3_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = len+1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    //Perform I2C write
    I2C_transfer(i2c, &i2cTransaction);

    return rslt;
}

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

static void delay_ms(uint32_t period) {
  Task_sleep( period*1000 / Clock_tickPeriod);
}

static void delay_us(uint32_t period, void *intf_ptr) {
  Task_sleep( period / Clock_tickPeriod);
}

struct bmp3_dev      bmp3;
struct bmp3_settings bmp3_s;

uint8_t bmp3XXSensorInit(uint16_t rate, uint16_t config){
    //System_printf(">> bmp3 init\n" );
    //consoleFlush();
    //Task_sleep( 100*1000 / Clock_tickPeriod);

    bmp3.intf  = BMP3_I2C_INTF;
    bmp3.read  = bmp3_i2c_read;
    bmp3.write = bmp3_i2c_write;
    bmp3.delay_us = delay_us;
    bmp3.intf_ptr = (void*) 1; // cannot be null but I don't think it is ever used
    bmp3_addr = BMP3_ADDR_I2C_PRIM;
    uint8_t rslt = bmp3_init(&bmp3);
    //System_printf(">> bmp3 addr 0x%02x result %d\n",bmp3_addr, rslt );
    //consoleFlush();
    //Task_sleep( 100*1000 / Clock_tickPeriod);

    if (rslt != BMP3_OK) {
        bmp3_addr = BMP3_ADDR_I2C_SEC;
        rslt = bmp3_init(&bmp3);
        //System_printf(">> bmp3 addr 0x%02x result %d\n",bmp3_addr, rslt );
        //consoleFlush();
    }
    if (rslt != BMP3_OK) return rslt;

    //rslt = bmp3_get_sensor_settings(&bmp3_s, &bmp3);

    //uint8_t settings_sel;
    //if (config == 7) {
    System_printf("BMP3XX sense temp & pressure\n");
    bmp3_s.odr_filter.temp_os    = BMP3_NO_OVERSAMPLING;
    bmp3_s.odr_filter.press_os   = BMP3_NO_OVERSAMPLING;
    bmp3_s.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;
    bmp3_s.odr_filter.odr        = BMP3_ODR_0_001_HZ;  // TODO no idea what this means in forced mode
    bmp3_s.press_en              = TRUE;
    bmp3_s.temp_en               = TRUE;
    bmp3_s.op_mode               = BMP3_MODE_FORCED;

    rslt &= bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS, &bmp3_s, &bmp3);
    delay_ms(40); // copied from BM280; needed?
    rslt &= bmp3_set_op_mode(&bmp3_s, &bmp3);
    return rslt;
}

uint8_t bmp3XXStop(){
    bmp3_s.op_mode               = BMP3_MODE_SLEEP;
    return bmp3_set_op_mode(&bmp3_s, &bmp3);
}

uint8_t bmp3XXStart(){
    bmp3_s.op_mode               = BMP3_MODE_FORCED;
    return bmp3_set_op_mode(&bmp3_s, &bmp3);
}

uint16_t bmp3XXGetData(uint8_t* buf, size_t buf_size, size_t* filled){
    struct bmp3_data comp_data;
    if (buf_size < sizeof(uint32_t)){ // bmp3 API returns uint64_t, but we cast to uint32_t
        *filled = 0;
        return 0;
    }

    uint8_t rslt = bmp3_get_sensor_data(BMP3_PRESS, &comp_data, &bmp3);
    uint32_t pressure = (uint32_t) comp_data.pressure;
    System_printf(">> pressure %d\n",pressure );
    memcpy(buf, &(pressure), sizeof(pressure));
    *filled = sizeof(pressure);
    return rslt;
}

uint16_t bmp3XXGetDataAll(uint8_t* buf, size_t buf_size, size_t* filled){
    struct bmp3_data comp_data;
    if (buf_size < (sizeof(uint32_t) + sizeof(int32_t))) {
        *filled = 0;
        return 0;
    }

    uint8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &comp_data, &bmp3);
    uint32_t pressure = (uint32_t) comp_data.pressure;
     int32_t temp     = ( int32_t) comp_data.temperature;
    System_printf(">> pressure %d temp %d\n",pressure,temp);
    memcpy(buf,                                                &(pressure), sizeof(pressure));
    memcpy(buf+sizeof(pressure),                               &(temp    ), sizeof(temp));
    *filled = sizeof(pressure) + sizeof(temp);
    return rslt;
}


static uint8_t  BMP3XXBuffer[4 + 120];
static uint8_t  BMP3XXBufferPointer = 0xFF; // a sign that we have not started sensing yet.
static uint32_t bmp3XXLastSenseTime;

static int16_t sensorsBMP3XXInit(uint8_t scheduleIndex) {
  if (bmp3XXSensorInit(0,schedule[ scheduleIndex ].config) == BMP3_OK) {
    System_printf("bmp3 init ok\n");
    return 0;
  }
  System_printf("bmp3 init failed!\n");
  return 1; // not found, etc
}

static int16_t sensorsBMP3XXHandler(uint8_t action) {
  //System_printf("bmp3 action %d\n",action);
  if (action == ACTION_START) {
    if (BMP3XXBufferPointer == 0xFF) { // we never used the sensor yet
      *((uint32_t*) BMP3XXBuffer) = Seconds_get(); // sensing starts now
      BMP3XXBufferPointer = 4; // next time will be the first in the buffer
      bmp3XXStart();
      return -1;
    }

    size_t filled;
    uint16_t rc = bmp3XXGetData( BMP3XXBuffer+BMP3XXBufferPointer, (4+120)-BMP3XXBufferPointer, &filled);
    if (filled == 0) {
      // nothing filled, we assume that this is because there is no more space in the buffer, so log it
      //System_printf("BMP3XX data logging!\n");
      loggerLog(CATALOG_DATATYPE_SENSOR_BMP3XX_DATA, (uint8_t*) BMP3XXBuffer, BMP3XXBufferPointer); // TODO: SAME DATA FORMAT?
      *((uint32_t*) BMP3XXBuffer) = bmp3XXLastSenseTime;
      BMP3XXBufferPointer = 4;
      // read data again, now there should be enough space
      rc = bmp3XXGetData( BMP3XXBuffer+BMP3XXBufferPointer, (4+120)-BMP3XXBufferPointer, &filled);
    }
    //System_printf("BMP3XX data %d %d\n",filled,BMP3XXBufferPointer);
    BMP3XXBufferPointer += filled;
    bmp3XXLastSenseTime = Seconds_get();
    bmp3XXStart();
  }
  return -1; // no need to call in the near future to log/buffer
}

#endif // BMP3XX

/**************************************************************************************/
/* BME280 sensor                                                                      */
/**************************************************************************************/

#ifdef USE_VH_SENSOR_BME280

#include "bme280.h"

static uint8_t  BME280Buffer[4 + 120];
static uint8_t  BME280BufferPointer = 0xFF; // a sign that we have not started sensing yet.
static uint32_t bme280LastSenseTime;

static int16_t sensorsBME280Init(uint8_t scheduleIndex) {
  if (bme280SensorInit(0,schedule[ scheduleIndex ].config) == BME280_OK) {
    System_printf("bme280 init ok\n");
    return 0;
  }
  System_printf("bme280 init failed!\n");
  return 1; // not found, etc
}

static int16_t sensorsBME280Handler(uint8_t action) {
  System_printf("bme280 action %d\n",action);
  if (action == ACTION_START) {
    if (BME280BufferPointer == 0xFF) { // we never used the sensor yet
      *((uint32_t*) BME280Buffer) = Seconds_get(); // sensing starts now
      BME280BufferPointer = 4; // next time will be the first in the buffer
      bme280Start();
      return -1;
    }

    size_t filled;
    uint16_t rc = bme280GetData( BME280Buffer+BME280BufferPointer, (4+120)-BME280BufferPointer, &filled);
    if (filled == 0) {
      // nothing filled, we assume that this is because there is no more space in the buffer, so log it
      System_printf("bme280 data logging!\n");
      loggerLog(CATALOG_DATATYPE_SENSOR_BME280_DATA, (uint8_t*) BME280Buffer, BME280BufferPointer);
      *((uint32_t*) BME280Buffer) = bme280LastSenseTime;
      BME280BufferPointer = 4;
      // read data again, now there should be enough space
      rc = bme280GetData( BME280Buffer+BME280BufferPointer, (4+120)-BME280BufferPointer, &filled);
    }
    System_printf("bme280 data %d %d\n",filled,BME280BufferPointer);
    BME280BufferPointer += filled;
    bme280LastSenseTime = Seconds_get();
    bme280Start();
  }
  return -1; // no need to call in the near future to log/buffer
}

#endif // BME280

/**************************************************************************************/
/* BMI160 sensor                                                                      */
/**************************************************************************************/

#ifdef USE_VH_SENSOR_BMI160

#include "bmi160.h"

/*
 * Nir Zaidman's code
 */
static uint8_t txBuffer[10];
static uint8_t rxBuffer[30];

//int8_t bmi160_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
//int8_t bmi160_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
//void bmi160_delay_ms(uint32_t period);

/*********************** User function definitions ****************************/
#include <xdc/runtime/System.h>

#include <ti/drivers/I2C.h>

extern    I2C_Handle      i2c;
extern    I2C_Transaction i2cTransaction;

int8_t bmi160_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    //Fill TX Buffer with the register address
    txBuffer[0] = reg_addr;

    //Point I2C parameters to correct values. Set up to read "cnt" number of bytes.
    i2cTransaction.slaveAddress = dev_id;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = len;

    //Perform I2C read
    I2C_transfer(i2c, &i2cTransaction);


    return rslt;
}

int8_t bmi160_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    int i = 1;

    //Fill TX Buffer with the register address and subsequent data bytes
    txBuffer[0] = reg_addr;
    for(i=1; i<len+1;i++)
        txBuffer[i] = reg_data[i-1];

    //Point TI-RTOS I2C parameters to correct values
    i2cTransaction.slaveAddress = dev_id;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = len+1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    //Perform I2C write
    I2C_transfer(i2c, &i2cTransaction);

    return rslt;
}

void bmi160_delay_ms(uint32_t period)
{
    Task_sleep( period*1000 / Clock_tickPeriod);
}

struct bmi160_dev sensor;
struct bmi160_fifo_frame fifo_frame;

static uint8_t  BMI160Buffer[4 + 1 + 120];
static uint8_t  BMI160BufferPointer;
static uint8_t  bmi160SingleMode = 0;
static uint16_t bmi160LogStep;

uint8_t bmi160SensorInit(uint16_t rate, uint16_t config){
  sensor.fifo = &fifo_frame;

  sensor.id = BMI160_I2C_ADDR;
  sensor.interface = BMI160_I2C_INTF;
  sensor.read = bmi160_i2c_read;
  sensor.write = bmi160_i2c_write;
  sensor.delay_ms = bmi160_delay_ms;

  int8_t rslt = BMI160_OK;
  rslt = bmi160_init(&sensor);
  if (rslt != BMI160_OK) return rslt;

  /* Select the Output data rate, range of accelerometer sensor */
  sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  if (rate < 1.56)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_0_78HZ;
  else if (rate < 3.12)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1_56HZ;
  else if (rate < 6.25)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_3_12HZ;
  else if (rate < 12.5)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_6_25HZ;
  else if (rate < 25)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_12_5HZ;
  else if (rate < 50) {
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
    System_printf("bmi160 25Hz\n");
  }
  else if (rate < 100)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
  else if (rate < 200)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
  else if (rate < 400)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
  else if (rate < 800)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_400HZ;
  else if (rate < 1600)
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;

  sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G; // default
  uint8_t gRange = config & 0xFF;
  if (gRange == 1) sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  if (gRange == 2) sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
  if (gRange == 3) sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
  if (gRange == 4) sensor.accel_cfg.range = BMI160_ACCEL_RANGE_16G;

  //sensor.accel_cfg.range = (gRange == 4) ? BMI160_ACCEL_RANGE_8G : (gRange == 8) ? BMI160_ACCEL_RANGE_16G : (gRange == 2) ? BMI160_ACCEL_RANGE_4G : BMI160_ACCEL_RANGE_2G;
  sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  sensor.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  sensor.gyro_cfg.odr   = BMI160_GYRO_ODR_3200HZ;
  sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  sensor.gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;

  /* Select the power mode of Gyroscope sensor */
  sensor.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

  /* Set the sensor configuration */
  rslt = bmi160_set_sens_conf(&sensor);

  // disable previous configuration
  rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &sensor);
  sensor.delay_ms(420);

  // set new configuration
  //rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL, BMI160_ENABLE, &sensor);
  //rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_HEADER, BMI160_DISABLE, &sensor);

#if 0
  uint8_t tst;
  sensor.read(sensor.id, BMI160_ACCEL_CONFIG_ADDR, &tst, 1);
  System_printf("! accel config %02x\n",tst);
  sensor.read(sensor.id, BMI160_GYRO_CONFIG_ADDR, &tst, 1);
  System_printf("! gyro config %02x\n",tst);
  sensor.read(sensor.id, BMI160_FIFO_DOWN_ADDR, &tst, 1);
  System_printf("! fifo downs %02x\n",tst);
  sensor.read(sensor.id, BMI160_ERROR_REG_ADDR, &tst, 1);
  System_printf("! err reg %02x\n",tst);
  sensor.read(sensor.id, BMI160_FIFO_CONFIG_0_ADDR, &tst, 1);
  System_printf("! fifo config 0 %02x\n",tst);
  sensor.read(sensor.id, BMI160_FIFO_CONFIG_1_ADDR, &tst, 1);
  System_printf("! fifo config 1 %02x\n",tst);
#endif

  return rslt;
}

int16_t bmi160GetData(uint8_t* buf, size_t buf_size, size_t* filled){
  if (buf_size < 7) return BMI160_E_INVALID_INPUT; // not enough space

  //sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
  //bmi160_set_sens_conf(&sensor);

  fifo_frame.data = buf;
  fifo_frame.length = buf_size;
  uint8_t res = bmi160_get_fifo_data(&sensor);
  *filled = fifo_frame.length;
//  /fifo_frame.data = NULL;

  // System_printf("! res=%d size %d filled %d\n",res,buf_size, *filled);

  //sensor.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
  //bmi160_set_sens_conf(&sensor);

  return BMI160_OK;
}

uint8_t bmi160Stop(){
  uint8_t rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_HEADER, BMI160_DISABLE, &sensor);
  /*
   * If we are in single mode (fifo is filled once in the burst), switch to normal mode to read fifo.
   * Otherwise, the fifo is already in normal mode.
   */
  if (bmi160SingleMode) {
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    bmi160_set_sens_conf(&sensor);
  }
  //sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
  //return bmi160_set_sens_conf(&sensor);

  return rslt;
}

uint8_t bmi160Suspend(){
  sensor.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
  return bmi160_set_sens_conf(&sensor);
}

uint8_t bmi160Start(){
  //sensor.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
  if (bmi160SingleMode)
    sensor.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
  else
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
  bmi160_set_sens_conf(&sensor);
  bmi160_set_fifo_config(BMI160_FIFO_ACCEL | BMI160_FIFO_HEADER, BMI160_ENABLE, &sensor);
  return bmi160_set_fifo_flush(&sensor);
}

static int16_t sensorsBMI160Init(uint8_t scheduleIndex) {
  uint32_t rate        = schedule[ scheduleIndex ].rate;
  uint32_t duration_ms = tagPeriodMs * schedule[ scheduleIndex ].duration;
  uint32_t nsamples = ( duration_ms * rate ) / 1000;

  if (nsamples * 7 < 10) { // for testing // 1024) {
    bmi160SingleMode = 1;
    bmi160LogStep = schedule[ scheduleIndex ].duration;
    System_printf("! bmi160 single mode %d bytes\n",nsamples * 7);
  } else {
    bmi160LogStep = 1; // log at every period; may improve later
    System_printf("! bmi160 continuous normal mode\n");
  }


  if (bmi160SensorInit(schedule[ scheduleIndex ].rate,schedule[ scheduleIndex ].config) == BMI160_OK) {
    System_printf("bmi160 init ok\n");
    return 0;
  }
  System_printf("bmi160 init failed!\n");
  return 1; // not found, etc
}

static int16_t sensorsBMI160Handler(uint8_t action) {
  System_printf("bmi160 action %d\n",action);
  if (action & ACTION_START) {
    *((uint32_t*) BMI160Buffer) = Seconds_get(); // sensing starts now
    //bmi160Fragment = 0;
    BMI160Buffer[4] = 0; // fragement number
    BMI160BufferPointer = 5; // next time will be the first in the buffer
    bmi160Start();

    System_printf("! bmi160 start, log in %d periods\n",bmi160LogStep);

    return bmi160LogStep;
  }

  if (action & ACTION_STOP) {
    bmi160Stop(); // we probably still need to log stored data
  }

  if (action & ACTION_LOG) {
    size_t filled;
    size_t total = 0;
    do {
      int16_t rc = bmi160GetData( BMI160Buffer+BMI160BufferPointer, (4+1+120)-BMI160BufferPointer, &filled);
      if (rc == BMI160_E_INVALID_INPUT) { // not enough space; our buffer is full
        // nothing filled, we assume that this is because there is no more space in the buffer, so log it
        loggerLog(CATALOG_DATATYPE_SENSOR_BMI160_DATA, (uint8_t*) BMI160Buffer, BMI160BufferPointer);
        // time stamp remains the same ...
        BMI160Buffer[4]++; // increment fragement number
        BMI160BufferPointer = 5;
        // read data again, now there should be enough space
        rc = bmi160GetData( BMI160Buffer+BMI160BufferPointer, (4+1+120)-BMI160BufferPointer, &filled);
      }
      //System_printf("! bme160 getdata filled=%d\n",filled);
      /*
      System_printf("bme160 data %d %d %d filled=%d\n",
          *((int16_t*) BME280Buffer+BME280BufferPointer),
          *((int16_t*) BME280Buffer+BME280BufferPointer+2),
          *((int16_t*) BME280Buffer+BME280BufferPointer+4),
          filled);
          */
      BMI160BufferPointer += filled;
      total += filled;
    } while (filled != 0);
    System_printf("! bme160 getdata filled=%d\n",total);
  }

  if (action & ACTION_STOP) {
    bmi160Suspend(); // now we can put the sensor in suspend mode (in which the FIFO is disabled)
    // if we still have some data in the buffer, log it
    if (BMI160BufferPointer >= 7)
      loggerLog(CATALOG_DATATYPE_SENSOR_BMI160_DATA, (uint8_t*) BMI160Buffer, BMI160BufferPointer);
  }

  if (action & ACTION_STOP) return -1; // we are done, don't call next to log
  else                      return bmi160LogStep; // call again to log
}

#endif // BMI160

/**************************************************************************************/
/* map of sensor types                                                                */
/**************************************************************************************/

typedef struct {
    uint16_t type;
    int16_t (*handler)(uint8_t action);
    int16_t (*initializer)(uint8_t scheduleIndex);
} sensor_dictionary_t;

static const sensor_dictionary_t sensorTypes[] = {
    { 51, sensorsBatmonHandler, sensorsBatmonInit},

    #ifdef USE_VH_SENSOR_BME280
    { 52, sensorsBME280Handler, sensorsBME280Init}, // BME280
#endif

#ifdef USE_VH_SENSOR_BMI160
    { 53, sensorsBMI160Handler, sensorsBMI160Init}, // BMI160
#endif

#ifdef USE_VH_SENSOR_BMP3XX
    { 55, sensorsBMP3XXHandler, sensorsBMP3XXInit}, // BMP384
#endif
    { 0, NULL, NULL} // end marker
};

/**************************************************************************************/
/* initialization                                                                     */
/**************************************************************************************/

#include <ti/drivers/I2C.h>

#ifdef MOVED_TO_I2C
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;
#endif

static uint8_t initialized = 0;


void sensorsInit() {
  initialized = 1;

#ifdef MOVED_TO_I2C
  I2C_init();
  I2C_Params      params;
  I2C_Params_init(&params);
  params.transferMode  = I2C_MODE_BLOCKING;
  i2c = I2C_open(0 /* peripheral index */, &params);
  if (!i2c) {
    System_printf("I2C did not open");
  }
#endif

    int i;
    for (i=0; i<MAX_SENSOR_COUNT; i++) {
        schedule[i].handler = NULL;
    }

    //schedule[0].interval =  2;
    //schedule[0].duration = -1;
    //schedule[0].rate     = 0;
    //schedule[0].config   = 0;
    //schedule[0].nextStart = 0;
    //schedule[0].handler   = sensorsBatmonHandler;

}

/*
 * this function configures one sensor.
 * The input is an array that describes the tag's period and the
 * type, interval, duration, rate, and
 * configuration of one sensor.
 */
uint16_t sensorsConfigure(uint16_t* configData, uint8_t sizeOfConfigArray) {
    System_printf("\r\nsensorsConfigure 0\r\n");
    consoleFlush();
    Task_sleep( 100*1000 / Clock_tickPeriod);

    if (!initialized) sensorsInit();

    //System_printf("sensorConfig %d  %d  %d  %d %d size %d\n",
    //    configData[0],configData[1],configData[2],configData[3],configData[4],
    //    sizeOfConfigArray);

    System_printf("\r\nsensorsConfigure 1\r\n");
    consoleFlush();
    Task_sleep( 10*1000 / Clock_tickPeriod);


    int i,j;
    for (i=0; i<MAX_SENSOR_COUNT; i++) {
        if (schedule[i].handler == NULL) break;
    }

    System_printf("sensorsConfigure 2\r\n");
    consoleFlush();

    if (i>=MAX_SENSOR_COUNT) return SENSORS_ERR_TOO_MANY_SENSORS;

    System_printf("sensorsConfigure 3\r\n");
    consoleFlush();

    uint16_t type = configData[0];

    for (j=0; sensorTypes[j].type != 0; j++) {
        if (sensorTypes[j].type == type) break;
    }

    if (sensorTypes[j].type == 0) return SENSORS_ERR_UNKNOWN_SENSOR;

    System_printf("sensorsConfigure 4\r\n");
    consoleFlush();

    // interval in seconds, convert to ms and divide by period
    schedule[i].interval  = (uint16_t) ((1000*(uint32_t) configData[1]) / (uint32_t) tagPeriodMs);

    schedule[i].duration  = configData[2] / tagPeriodMs; // burst duration in ms
    schedule[i].rate      = configData[3];
    schedule[i].config    = configData[4];
    schedule[i].nextStart = 0;
    schedule[i].handler   = sensorTypes[j].handler;

    // change duration of 0 to -1, to make scheduling a bit easier
    if (schedule[i].duration == 0) schedule[i].duration = ONE_SHOT;
    if (schedule[i].duration >= schedule[i].interval) schedule[i].duration = FOREVER;

    System_printf("sensor %d type %d interval %d duration %d rate %d config %d size %d\n",
                   i,type,schedule[i].interval, schedule[i].duration, schedule[i].rate, schedule[i].config,
                   sizeOfConfigArray);
    consoleFlush();
    Task_sleep( 500*1000 / Clock_tickPeriod);

    // perform sensor specific intialization, if needed
    if (sensorTypes[j].initializer != NULL) {
      if ((sensorTypes[j].initializer)(i) != 0) { // initialization error, remove from schedule
        schedule[i].handler = NULL;
        loggerLog(CATALOG_DATATYPE_SENSOR_CONFIG_FAILURE, (uint8_t*) configData, sizeof(uint16_t)*sizeOfConfigArray);
        System_printf("sensorsConfigure no response from type %d, deleting\n",sensorTypes[j].type);
        sprintf(message, "err sensor type %d",sensorTypes[j].type);
        loggerLog(LOGGER_DATATYPE_LOG_MESSAGE,message,strlen(message));
        return SENSORS_ERR_SENSOR_NO_RESPONSE;
      }
    }

    System_printf("\r\nsensorsConfigure 8\r\n");
    consoleFlush();
    Task_sleep( 10*1000 / Clock_tickPeriod);

    //System_printf("sensor period %d interval_ms %d %d\n",tagPeriodMs,(1000*(uint32_t) configData[1]));
    System_printf("sensorsConfigure schedule slot %d for type %d\n",i,sensorTypes[j].type);
    sprintf(message, "ok sensor type %d",sensorTypes[j].type);
    loggerLog(LOGGER_DATATYPE_LOG_MESSAGE,message,strlen(message));

    loggerLog(CATALOG_DATATYPE_SENSOR_CONFIG, (uint8_t*) configData, sizeof(uint16_t)*sizeOfConfigArray);

    System_printf("\r\nsensorsConfigure 9\r\n");
    consoleFlush();
    Task_sleep( 10*1000 / Clock_tickPeriod);

    return SENSORS_SUCCESS;
}

/**************************************************************************************/
/* scheduling                                                                         */
/**************************************************************************************/

uint16_t sensorsSense(uint16_t periodCounter) {
  if (!initialized) sensorsInit();

  int i;

  uint16_t d = 0xFFFF; // distance in periods to next invocation

  for (i=0; i<MAX_SENSOR_COUNT; i++) {
    if (schedule[i].handler==NULL) continue;
    if (periodCounter == schedule[i].nextStart) {
      System_printf("sensorsSense START on %d\n",i);
      schedule[i].nextLog   = periodCounter + (schedule[i].handler)(ACTION_START);
      schedule[i].nextStart = periodCounter + schedule[i].interval;
      //if (schedule[i].duration==0) continue;
      schedule[i].nextStop = periodCounter + schedule[i].duration;
    } else {
      uint8_t action = 0;
      if (periodCounter == schedule[i].nextLog )
        action |= ACTION_LOG;
      if ((schedule[i].duration != FOREVER) && (periodCounter == schedule[i].nextStop)) {
        action |= ACTION_STOP;
        schedule[i].nextStop = periodCounter -1; // we will set nextStop at the next start slot; for now, invalidate so we don't invoke next NOW.
      }

      if (action != 0) {
        schedule[i].nextLog = periodCounter + (schedule[i].handler)(action);
        System_printf("sensorsSense STOP/LOG (%02x) on %d\n",action,i);
      }
    }

    // tricky unsigned arithmetic ahead!!!

    uint16_t dStart = schedule[i].nextStart - periodCounter;
    uint16_t dStop  = schedule[i].nextStop  - periodCounter;
    uint16_t dLog   = schedule[i].nextLog   - periodCounter;

    System_printf("sensorsSense %d period %d start %u log %u stop %u\n",i,periodCounter,
        schedule[i].nextStart,schedule[i].nextLog,schedule[i].nextStop);

    if (dStart < d) d = dStart;
    if (dLog   < d) d = dLog;
    if (dStop  < d && schedule[i].duration != FOREVER) d = dStop;
  }
  //leds_blink(LEDS_RX, 2);
  System_printf("sensorsSense again in %d periods\n",d);

  return d;
}

#endif // VH_LOGGER
