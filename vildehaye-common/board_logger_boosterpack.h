/*
 * board_logger_boosterpack.h
 *
 * Author: stoledo
 */

#ifndef BOARD_LOGGER_BOOSTERPACK_H
#define BOARD_LOGGER_BOOSTERPACK_H

#define USE_VH_SENSOR_BME280

#define BOOSTERPACK_LED_SDCARD     BOOSTERPACK_J2_4  /* pin 12 on cc1350 launchpads */
#define BOOSTERPACK_LED_SDCARD_ON  1
#define BOOSTERPACK_LED_SDCARD_OFF 0
#define BOOSTERPACK_LED_CLOCK      BOOSTERPACK_J2_20 /* pin 15 on cc1350 launchpads */
#define BOOSTERPACK_LED_CLOCK_ON   1
#define BOOSTERPACK_LED_CLOCK_OFF  0
#define BOOSTERPACK_LED_TX         BOOSTERPACK_J1_15 /* pin 21 on cc1350 launchpads */
#define BOOSTERPACK_LED_TX_ON      1
#define BOOSTERPACK_LED_TX_OFF     0
#define BOOSTERPACK_LED_RX         BOOSTERPACK_J1_3  /* pin 23 on cc1350 launchpads */
#define BOOSTERPACK_LED_RX_ON      1
#define BOOSTERPACK_LED_RX_OFF     0

#define BOARD_BUTTON2             BOOSTERPACK_J2_9  /* pin 18 on cc1350 launchpads */
#define BOARD_BUTTON2_PRESSED     0
#define BOARD_BUTTON3             BOOSTERPACK_J1_9  /* pin 22 on cc1350 launchpads */
// pressed is 0 in my boosterpck, 1 in the sparkfun breakout board
//#define BOARD_BUTTON3_PRESSED     0
#define BOARD_BUTTON3_PRESSED     1

#define BOOSTERPACK_BUTTON              BOARD_BUTTON2
#define BOOSTERPACK_BUTTON_PRESSED      BOARD_BUTTON2_PRESSED
#define BOOSTERPACK_CARD_DETECT         BOARD_BUTTON3
#define BOOSTERPACK_CARD_DETECT_PRESSED BOARD_BUTTON3_PRESSED

#define BOOSTERPACK_DISPLAY_POWER       BOOSTERPACK_J2_7    /* TPS2553 switch for display                                   */
#define BOOSTERPACK_DISPLAY_POWER_ON    1

#define BOOSTERPACK_GPS_TX              BOOSTERPACK_J1_6    /* pin  */
#define BOOSTERPACK_GPS_RX              BOOSTERPACK_J1_8
#define BOOSTERPACK_GPS_PPS             BOOSTERPACK_J1_11

#define BOOSTERPACK_SDCARD_CS           BOOSTERPACK_J2_6   /* pin 11 on cc1350 launchpads */
#define BOOSTERPACK_SPI_SCLK            BOOSTERPACK_J1_13
#define BOOSTERPACK_SPI_MISO            BOOSTERPACK_J2_14
#define BOOSTERPACK_SPI_MOSI            BOOSTERPACK_J2_12
#define BOOSTERPACK_I2C_SCL             BOOSTERPACK_J1_17  /* for BME280 sensor */
#define BOOSTERPACK_I2C_SDA             BOOSTERPACK_J1_19  /* for BME280 sensor */

#endif /* BOARD_LOGGER_BOOSTERPACK_H */
