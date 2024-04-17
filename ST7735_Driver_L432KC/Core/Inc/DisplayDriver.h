/*
 * DisplayDriver.h
 *
 *  Created on: Apr 16, 2024
 *      Author: zuhan
 */

#ifndef INC_DISPLAYDRIVER_H_
#define INC_DISPLAYDRIVER_H_

#include <iostream>

struct DriverParams{
	uint16_t DataRate;
	uint16_t Pins[];
};

class DisplayDriver {
public: //TODO add templates
	struct ShapeParams{
		uint8_t StartX;
		uint8_t StartY;
		uint8_t EndX;
		uint8_t EndY;
		uint16_t Color;
		uint8_t Thickness;
	};
	virtual DisplayDriver(DriverParams InitParams);
	virtual ~DisplayDriver();
	virtual void DrawPixel(uint8_t Xpos, uint8_t Ypos, uint16_t Color, uint8_t Thickness);
	virtual void DrawRectangle(ShapeParams);
	virtual void DrawLine(ShapeParams);
	virtual void DrawEllipse(ShapeParams);
	virtual void DrawLetter(char Letter, uint8_t Xpos, uint8_t Ypos, uint16_t Color);
	virtual void DrawText(std::string Letter, uint8_t Xpos, uint8_t Ypos, uint16_t Color);
};

//TODO:Make it so you can take control of shapes and manipulate them

#endif /* INC_DISPLAYDRIVER_H_ */
