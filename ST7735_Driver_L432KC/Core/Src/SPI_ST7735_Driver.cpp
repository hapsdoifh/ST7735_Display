/*
 * SPI_ST7735_Driver.cpp
 *
 *  Created on: Apr 25, 2024
 *      Author: zuhan
 */


#include <SerialDisplayDriver.h>

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

#define max(a,b) (abs(a) > abs(b) ? abs(a) : abs(b))

class ST7735_Driver : public SerialDisplayDriver{
private:
	void WriteParams(){};
	static SPI_HandleTypeDef *DisplayHandle;

public:
	ST7735_Driver(DriverParams InitParams);
	~ST7735_Driver();
	void HWReset(){};
	template<typename type>
	void Swap(type &a, type &b);
	void SetAddr(ShapeParams DrawParams);
	void WriteCommand(unsigned char Command, unsigned char* Params, unsigned char NumParams);
	uint16_t GenColor(unsigned int R, unsigned int G, unsigned int B);
	uint16_t ColorRatio(float R, float G, float B);
	void DrawRectangle(ShapeParams DrawParams);
	void DrawLine(ShapeParams DrawParams);
	void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color);
	void DrawEllipse(ShapeParams DrawParams);
	void DrawPixel(ShapeParams DrawParams);
	void DrawLetter(ShapeParams DrawParams, char Letter);
	void DrawText(ShapeParams DrawParams, std::string Text);
	void DrawImage(uint16_t image[]);

	template<typename command, typename... VarArgs>
	void WriteCommandVargs(command Command, VarArgs... Args);
	template<typename First, typename... VarArgs>
	void WriteParams(First first, VarArgs... Args);
};


template<typename type>
void ST7735_Driver::Swap(type &a, type &b){
    type c = a;
    a = b;
    b = c;
}

//This is a funciton called by other functions to write command to the display
void ST7735_Driver::WriteCommand(unsigned char Command, unsigned char* Params = NULL, unsigned char NumParams = 0){
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Command, 1, 100);
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
    for(int i = 0; i < NumParams; i++){
    	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Params[i], 1, 100);
    }
}

uint16_t ST7735_Driver::GenColor(unsigned int R, unsigned int G, unsigned int B){
    R = R <= 0x1F ? R : 0x1F;
    G = G <= 0x3F ? G : 0x3F;
    B = B <= 0x1F ? B : 0x1F;
    return((R<<11) + (G<<5) + B);
}

uint16_t ST7735_Driver::ColorRatio(float R, float G, float B){
    R = R <= 1.0 ? R : 1.0;
    G = G <= 1.0 ? G : 1.0;
    B = B <= 1.0 ? B : 1.0;
    return GenColor(int(R * 31), int(G * 63), int(B * 31));
}

void ST7735_Driver::DrawRectangle(ShapeParams DrawParams){
    DrawLine(DrawParams.StartX, DrawParams.StartY, DrawParams.EndX, DrawParams.StartY, DrawParams.Color);
    DrawLine(DrawParams.StartX, DrawParams.StartY, DrawParams.StartX, DrawParams.EndY, DrawParams.Color);
    DrawLine(DrawParams.StartX, DrawParams.EndY, DrawParams.EndX, DrawParams.EndY, DrawParams.Color);
    DrawLine(DrawParams.EndX, DrawParams.StartY, DrawParams.EndX, DrawParams.EndY, DrawParams.Color);
}

void ST7735_Driver::DrawLine(ShapeParams DrawParams){
	uint16_t StartX = DrawParams.StartX;
	uint16_t EndX = DrawParams.EndX;
	uint16_t StartY = DrawParams.StartY;
	uint16_t EndY = DrawParams.EndY;
    if(( (abs(EndX - StartX) >= abs(EndY-StartY)) && (EndX < StartX) ) || ( ((abs(EndX - StartX) < abs(EndY-StartY)) && (EndY < StartY))) ){
      Swap<uint16_t>(StartX, EndX);
      Swap<uint16_t>(StartY, EndY);
    }
    int Cycles = max((EndX - StartX),(EndY - StartY));
    float Slope = 0;
    uint16_t StartMapping[] = {StartX, StartY};
    uint16_t CoordMapping[] = {0,0}, defau_map = 1;

    if (abs(EndY - StartY) > abs(EndX - StartX)){ //if line is longer vertically rise > run
        if (EndY-StartY != 0){
            Slope = float((EndX - StartX))/(EndY-StartY);
        }
    }else{
        if (EndX - StartX != 0){
            Slope = float((EndY-StartY))/(EndX - StartX);
        }
        defau_map = 0;
    }
    for(int i = 0; i <= Cycles; i++){
        int opp_map = 1 - defau_map;
        CoordMapping[defau_map] = int(StartMapping[defau_map] + i); //default: CoordMapping[x,y] modified:CoordMapping[y,x]
        CoordMapping[opp_map] = int(StartMapping[opp_map] + i*Slope);

        if(CoordMapping[0] >= 0 && CoordMapping[0] < 160 && CoordMapping[1] >= 0 && CoordMapping[1] < 128){
        	struct ShapeParams a = {.StartX = CoordMapping[0], .StartY = CoordMapping[1], .Color = DrawParams.Color, DrawParams.Thickness};
        	DrawPixel(a);
        }
    }
}

template<typename First, typename... VarArgs>
void ST7735_Driver::WriteParams(First first, VarArgs... Args){
	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&first, 1, 100);
    WriteParams(Args...);
}

template<typename command, typename... VarArgs>
void ST7735_Driver::WriteCommandVargs(command Command, VarArgs... Args){
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Command, 1, 100);
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
    WriteParams(Args...);
}

void ST7735_Driver::SetAddr(ShapeParams DrawParams){
    WriteCommandVargs(CASET, 0x00, DrawParams.StartY, 0x00, DrawParams.EndY);
    WriteCommandVargs(RASET, 0x00, DrawParams.StartX, 0x00, DrawParams.EndX);
}

void ST7735_Driver::DrawPixel(ShapeParams DrawParams){
	uint16_t x = DrawParams.StartX;
	uint16_t y = DrawParams.StartY;
	uint16_t Color = DrawParams.Color;
    for(uint16_t Xoff = 0; Xoff < DrawParams.Thickness; Xoff++){
        for(uint16_t Yoff = 0; Yoff < DrawParams.Thickness; Yoff++){
            SetAddr((struct ShapeParams){x+Xoff,y+Yoff});
            WriteCommand(RAMWR);
            uint8_t Transfer[] = {(Color & 0xFF00) >> 8,  Color & 0xFF};
            HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Transfer, 2, 100);
        }
    }
}

