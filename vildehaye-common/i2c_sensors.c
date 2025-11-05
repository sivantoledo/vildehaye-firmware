#include "config.h"

#if 0
/*
 * Nir Zaidman 2018
 */

#include <stdlib.h>
#include <stdint.h>
//#include <boolean.h>
#include <time.h>

#include <xdc/std.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>

//#include "Board.h"
#include "config.h"
#include "buffers.h"
#include "uart.h"
#include "spi_flash.h"
#include "bmi160_support.h"

I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

#define SENSORS_TASK_STACK_SIZE 1024

// Globals
static Task_Params sensorsTaskParams;
Task_Struct sensorsTask;    /* not static so you can see in ROV */
static uint8_t sensorsTaskStack[SENSORS_TASK_STACK_SIZE];
Task_Struct sensorsUartTask;    /* not static so you can see in ROV */
uint8_t accelGRange;
uint8_t accelSampleDuration;
uint32_t accelFactor;
float accelSampleRate;

Semaphore_Struct semStruct;
Semaphore_Handle semHandle = NULL;

// Declarations
static void sensorsFunction(UArg arg0, UArg arg1);
void readDataFunction();
void readAccel(uint32_t count, uint32_t ticks, int* pageCounter, Types_FreqHz freq);

void i2cSensors_init() {
    I2C_init();
	I2C_Params      params;
	I2C_Params_init(&params);
	params.transferMode  = I2C_MODE_BLOCKING;
	i2c = I2C_open(0 /* peripheral index */, &params);
	if (!i2c) {
	    System_printf("I2C did not open");
	}

	Task_Params_init(&sensorsTaskParams);
	sensorsTaskParams.stackSize = SENSORS_TASK_STACK_SIZE;
	sensorsTaskParams.priority  = 5;
	sensorsTaskParams.stack     = &sensorsTaskStack;
	sensorsTaskParams.arg0       = (UInt)1000000;

    Task_construct(&sensorsTask, sensorsFunction, &sensorsTaskParams, NULL);
}

Semaphore_Handle getSemaphore(){
    if (semHandle != NULL){
        return semHandle;
    }

    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 0, &semParams);
    semHandle = Semaphore_handle(&semStruct);
    return semHandle;
}
void bmi160SetupAccelTicksFactor(const uint8_t* sensorData, uint16_t dataLength) {
    accelFactor=0;
    memcpy(&accelFactor, sensorData, dataLength);
}

void bmi160SetupAccelGRange(const uint8_t* sensorData) {
    accelGRange = *sensorData;
}

void bmi160SetupAccelSampleRate(const uint8_t* sensorData, uint16_t dataLength){
    accelSampleRate = 0;
    memcpy(&accelSampleRate, sensorData, dataLength);
}

void bmi160SetupAccelSampleDuration(const uint8_t* sensorData){
    accelSampleDuration = *sensorData;
}

void I2CSensorsSemaphore_post(){
    Semaphore_post(getSemaphore());
}

static void sensorsFunction(UArg arg0, UArg arg1) {

    int pageCounter=0;
    //uint8_t pageBaro[PAGE_SIZE];
    Types_FreqHz freq;
    buffer_descriptor d;

    // check commands - long wait for flash boot and enough time for user command
    if (Mailbox_pend(uartRxMailbox, &d, 10000000)){
        if (buffers[d.id][0] == 0xa){
            readDataFunction();
            return;
        }
    }

#ifdef putty_debug
    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
    char* string = buffers[ d.id ];
    sprintf(string, "BMI160 waiting for semaphore\r\n");
    d.length = strlen(string);
    Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
#endif

    // write mode
    spiFlashEraseChip(); // can't start writing without clearing the flash

    // wait for tag init to read configuration
    Semaphore_pend(getSemaphore(), BIOS_WAIT_FOREVER);

#ifdef putty_debug
    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
    string = buffers[ d.id ];
    sprintf(string, "semaphore done factor=%d,dur=%d,G=%d,rate=%f\r\n",accelFactor,accelSampleDuration,accelGRange,accelSampleRate);
    d.length = strlen(string);
    Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
#endif

    Semaphore_delete(semHandle);
    Timestamp_getFreq(&freq);
    bmi160_initialize_sensor(accelGRange, accelSampleRate);
    uint32_t ticksAccel = accelFactor * 1000000 / Clock_tickPeriod;

    while (true){
        readAccel(accelSampleDuration*accelSampleRate, (1000000 / Clock_tickPeriod)/accelSampleRate, &pageCounter, freq);
/*#if 1
        Task_sleep(100000);
        while (1){
        buffer_descriptor d;

        Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
        char* string = buffers[ d.id ];
		I2C_Transaction i2cTransaction;
		i2cTransaction.slaveAddress = (0x76 | 0x00); // r with address pin tied to VDD

		uint8_t value;
		uint8_t writeBuffer[2];

		writeBuffer[0] = 0xD0; // ID
		i2cTransaction.writeBuf = writeBuffer; // register id
		i2cTransaction.writeCount = 1;
		i2cTransaction.readBuf = &value;
		i2cTransaction.readCount = 1;
		bool ret = I2C_transfer(i2c, &i2cTransaction);
		sprintf(string+strlen(string), "ret=%d id=0x%02x\r\n",ret, value);

		writeBuffer[0] = 0xF2; // CTRL_HUM
		writeBuffer[1] = 0x01; // measure humidity, no oversampling
		i2cTransaction.writeBuf = writeBuffer; // register id
		i2cTransaction.writeCount = 2;
		i2cTransaction.readBuf = &value;
		i2cTransaction.readCount = 0;
		ret = I2C_transfer(i2c, &i2cTransaction);

		writeBuffer[0] = 0xF4; // CTRL_MEAS
		writeBuffer[1] = (1 << 5) // measure temp, no oversampling
				       | (1 << 2) // measure pressure, no oversampling
					   | 3;       // formced (one-shot) mode
		i2cTransaction.writeBuf = writeBuffer; // register id
		i2cTransaction.writeCount = 2;
		i2cTransaction.readBuf = &value;
		i2cTransaction.readCount = 0;
		ret = I2C_transfer(i2c, &i2cTransaction);

		do {
			writeBuffer[0] = 0xF3; // status
			i2cTransaction.writeBuf = writeBuffer; // register id
			i2cTransaction.writeCount = 1;
			i2cTransaction.readBuf = &value;
			i2cTransaction.readCount = 1;
			ret = I2C_transfer(i2c, &i2cTransaction);
		} while ((value & 0x04) != 0); // while measuring, continue to poll

		uint8_t data[8];
		writeBuffer[0] = 0xF7; // all measurements
		i2cTransaction.writeBuf = writeBuffer; // register id
		i2cTransaction.writeCount = 1;
		i2cTransaction.readBuf = data;
		i2cTransaction.readCount = 8;
		ret = I2C_transfer(i2c, &i2cTransaction);
		int i;
		for (i=0; i<8; i++) {
			sprintf(string + strlen(string), "%02x ",data[i]);
		}
		d.length = strlen(string);
        Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
#endif*/


        Task_sleep(ticksAccel);
	}
}

void readAccel(uint32_t count, uint32_t ticks, int* pageCounter, Types_FreqHz freq){
    buffer_descriptor d;
    struct bmi160_accel_t accelxyz;
    static uint8_t byteCounter = 18;
    static uint8_t pageAccel[PAGE_SIZE];
    BMI160_RETURN_FUNCTION_TYPE com_rslt;
    uint8_t check[PAGE_SIZE];
    uint32_t time;

    int i,j;
    for (i=0; i<count; i++){
        Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
        char* string = buffers[ d.id ];

        if (bmi160_read_accel_xyz(&accelxyz) != 0){
            sprintf(string, "BMI160 ret=%d error reading\r\n",com_rslt);
        }else{
            if (byteCounter == 18)
                time = Timestamp_get32()/freq.lo;

            sprintf(string, "BMI160 x=%d y=%d z=%d count=%d count=%d ticks=%d\r\n", (uint16_t)accelxyz.x, (uint16_t)accelxyz.y, (uint16_t)accelxyz.z, byteCounter, i, ticks);
            if (PAGE_SIZE - byteCounter > 6){
                *((struct bmi160_accel_t*)(pageAccel + byteCounter)) = accelxyz;
                byteCounter+=6;
            }else{
                //uint32_t time = Timestamp_get32()/freq.lo;
                sprintf(string, "BMI160 freq.lo=%d now=%d factor=%d G=%x\r\n", freq.lo, time, accelFactor, accelGRange);
                pageAccel[0] = BMI_160_PAGE;

                memcpy(pageAccel+1, &time, 4);
                memcpy(pageAccel+5, &accelSampleDuration, 4);
                memcpy(pageAccel+9, &accelFactor, 4);
                memcpy(pageAccel+13, &accelSampleRate, 4);
                pageAccel[17] = accelGRange;
                spiFlashWritePage(*pageCounter, pageAccel);

                /*// check that the page was written correctly; for debug/test
                spiFlashReadPage(*pageCounter, check);
                bool b = true;

                for (j=0;j<256;j++){
                    if (check[j] != pageAccel[j]){
                        b = false;
                        break;
                    }
                }

                if (b){
                    sprintf(string+strlen(string), "CHECK page=%d\r\n", *pageCounter);

                }*/

                (*pageCounter)++;
                byteCounter = 18;
            }
        }
        d.length = strlen(string);
        Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
        Task_sleep(ticks);
    }
}

void readDataFunction(){
    uint32_t i=0;
    uint8_t page[PAGE_SIZE];
    spiFlashReadPage(i, page);
    while (page[0] != EMPTY_PAGE){
        i++;
        buffer_descriptor d;
        Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
        uint8_t* string = buffers[ d.id ];
        memcpy(string, page, PAGE_SIZE);
        d.length = PAGE_SIZE;
        Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
        spiFlashReadPage(i, page);
    }

    // end packet
    buffer_descriptor d;
    Mailbox_pend(freeMailbox, &d, BIOS_WAIT_FOREVER);
    buffers[ d.id ][0] = DONE_READ_PAGE;
    d.length = 1;
    Mailbox_post(uartTxMailbox, &d, BIOS_WAIT_FOREVER);
}



/* value should be  0110 1000 = 0x68 */
/*uint8_t mpu9150WhoAmI(uint8_t* value, uint8_t address0) {
	I2C_Transaction i2cTransaction;
	uint8_t regAddress[] = { 0x75 };
	i2cTransaction.writeBuf = regAddress;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = value;
	i2cTransaction.readCount = 1;
	i2cTransaction.slaveAddress = (0x68 | address0);
	bool ret = I2C_transfer(i2c, &i2cTransaction);
	if (!ret) return 0;
	else      return 1;
}
*/
#endif
