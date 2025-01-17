/*
 * ST7735_Driver.h
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */

#ifndef INC_ST7735_DRIVER_H_
#define INC_ST7735_DRIVER_H_

#include <driver/spi_master.h>
#include "ST7735_Fonts.h"
// #include "ImageOut.h"
#include <math.h>

#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

#define CS_PORT GPIOB
#define CS_PIN 4
#define DC_PORT GPIOB
#define DC_PIN 3
#define RES_PORT GPIOB
#define RES_PIN 2

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

void DisplayInit(spi_device_handle_t* hspi);

void WriteCommand(unsigned char Command, unsigned char* Params, unsigned char NumParams);

unsigned int GenColor(unsigned int R, unsigned int G, unsigned int B);

unsigned int ColorRatio(float R, float G, float B);

void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color);

void DrawRect(int StartX, int StartY, int EndX, int EndY, int Color);

void DrawEllipse(int StartX, int StartY, int EndX, int EndY, int Color, int mapping=-1);

void DrawCharacter(char Character, int StartX, int StartY, uint16_t ForegndColor = 0xffff, uint16_t BckgndColor = 0);

void WriteText(char* text, int length, int StartX, int StartY, uint16_t ForegndColor = 0xffff, uint16_t BckgndColor = 0);

void DrawImage(uint16_t image[]);
#endif /* INC_ST7735_DRIVER_H_ */



