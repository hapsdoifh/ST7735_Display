/*
 * DisplayDriver.h
 *
 *  Created on: Apr 16, 2024
 *      Author: zuhan
 */

#ifndef INC_SERIALDISPLAYDRIVER_H_
#define INC_SERIALDISPLAYDRIVER_H_

#include <iostream>
#include "stm32l4xx_hal.h"

struct DriverParams{
	uint16_t DataRate;
	uint16_t* Pins;
};

class SerialDisplayDriver {
public: //TODO add templates
	struct ShapeParams{
		uint16_t StartX;
		uint16_t StartY;
		uint16_t EndX;
		uint16_t EndY;
		uint16_t Color;
		uint8_t Thickness;
	};
	SerialDisplayDriver(DriverParams InitParams);
	~SerialDisplayDriver();
	virtual void SetAddr(ShapeParams DrawParams);
	virtual void WriteCommand(unsigned char Command, unsigned char* Params, unsigned char NumParams);
	virtual uint16_t GenColor(unsigned int R, unsigned int G, unsigned int B);
	virtual uint16_t ColorRatio(float R, float G, float B);
	virtual void DrawRectangle(ShapeParams DrawParams);
	virtual void DrawLine(ShapeParams DrawParams);
	virtual void DrawEllipse(ShapeParams DrawParams);
	virtual void DrawPixel(ShapeParams DrawParams);
	virtual void DrawLetter(ShapeParams DrawParams, char Letter);
	virtual void DrawText(ShapeParams DrawParams, std::string Text);
	virtual void DrawImage(uint16_t image[]);
};

//TODO:Make it so you can take control of shapes and manipulate them

#endif /* INC_SERIALDISPLAYDRIVER_H_ */

