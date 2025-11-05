/*
 * board_logger_boosterpack.h
 *
 * Author: stoledo
 */

#ifndef BOARD_AX5031_BOOSTERPACK_H
#define BOARD_AX5031_BOOSTERPACK_H

#define AX5031_CS                       BOOSTERPACK_J2_6   /* pin 11 on cc1350 launchpads */

#define BOOSTERPACK_AX5031_CS           BOOSTERPACK_J2_6   /* pin 11 on cc1350 launchpads */
#define BOOSTERPACK_SPI_SCLK            BOOSTERPACK_J2_4
#define BOOSTERPACK_SPI_MISO            BOOSTERPACK_J2_14
#define BOOSTERPACK_SPI_MOSI            BOOSTERPACK_J2_12
#define BOOSTERPACK_SYSCLK              BOOSTERPACK_J2_8
#define BOOSTERPACK_IRQ                 BOOSTERPACK_J2_10  /* for BME280 sensor */

#define BOARD_SPI0_SCLK                 BOOSTERPACK_SPI_SCLK
#define BOARD_SPI0_MISO                 BOOSTERPACK_SPI_MISO
#define BOARD_SPI0_MOSI                 BOOSTERPACK_SPI_MOSI

#define AX5031_XTAL_FREQ           16000000

#endif /* BOARD_AX5031_BOOSTERPACK_H */
