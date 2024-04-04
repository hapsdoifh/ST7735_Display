/*
 * ST7735_Driver.h
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */

#ifndef INC_ST7735_DRIVER_H_
#define INC_ST7735_DRIVER_H_

#include <stm32f4xx_hal.h>


#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_6
#define RES_PORT GPIOC
#define RES_PIN GPIO_PIN_7
#define DC_PORT GPIOA
#define DC_PIN GPIO_PIN_9

#define D_WIDTH 160
#define D_HEIGHT 128

#define SWRESET 0x01
#define SLEEPOUT 0x11
#define DISPON 0x29
#define INVOFF 0x20
#define COLMOD 0x3A
#define MADCTL 0x36
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C

void DisplayInit(SPI_HandleTypeDef *hspi);

void WriteCommand(unsigned char Command, unsigned char* Params, unsigned char NumParams);

#endif /* INC_ST7735_DRIVER_H_ */
