/*
 * ssd1306_i2c.h
 *
 *  Created on: Aug 28, 2018
 *      Author: Nir Zaidman
 */

#ifndef SSD1306_I2C_H_
#define SSD1306_I2C_H_
#include <ti/drivers/I2C.h>

void ssd1306_clearScreen();
void ssd1306_printBuffer(uint8_t* buffer);
void ssd1306_init();
void ssd1306_printText(char* text, uint8_t row);


#endif /* SSD1306_I2C_H_ */
