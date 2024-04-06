/*
 * ST7735_Driver.h
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */

#ifndef INC_ST7735_DRIVER_H_
#define INC_ST7735_DRIVER_H_

#include "stm32l4xx_hal.h"
#include "ST7735_Fonts.h"
#include <math.h>


#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_6
#define RES_PORT GPIOB
#define RES_PIN GPIO_PIN_7
#define DC_PORT GPIOB
#define DC_PIN GPIO_PIN_0

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
//#define abs(a) (a > 0 ? a : (-a))
#define max(a,b) (abs(a) > abs(b) ? abs(a) : abs(b))

void DisplayInit(SPI_HandleTypeDef *hspi);

void WriteCommand(unsigned char Command, unsigned char* Params, unsigned char NumParams);

unsigned int GenColor(unsigned int R, unsigned int G, unsigned int B);

unsigned int ColorRatio(float R, float G, float B);

void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color);

void DrawRect(int StartX, int StartY, int EndX, int EndY, int Color);

void DrawEllipse(int StartX, int StartY, int EndX, int EndY, int Color, int mapping=-1);

void DrawCharacter(char Character, int StartX, int StartY);
#endif /* INC_ST7735_DRIVER_H_ */



