/*
 * ST7735_Driver.c
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */


#include "stm32l4xx_hal.h"
#include "ST7735_Driver.h"

static SPI_HandleTypeDef *DisplayHandle;

template<typename type>
void Swap(type &a, type &b){
    type c = a;
    a = b;
    b = c;
}

void WriteParams(){}

template<typename First, typename... VarArgs>
void WriteParams(First first, VarArgs... Args){
	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&first, 1, 100);
    WriteParams(Args...);
}

template<typename command, typename... VarArgs>
void WriteCommandVargs(command Command, VarArgs... Args){
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Command, 1, 100);
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
    WriteParams(Args...);
}

void WriteCommand(unsigned char Command, unsigned char* Params = NULL, unsigned char NumParams = 0){
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Command, 1, 100);
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
    for(int i = 0; i < NumParams; i++){
    	HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Params[i], 1, 100);
    }
}

void HWReset(){
	HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_SET);
}

void DisplayInit(SPI_HandleTypeDef *hspi){
	DisplayHandle = hspi;

	HWReset();
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	unsigned char InitSequence[] = {
	1, SWRESET,
	1, SLEEPOUT,
	1, DISPON,
	1, INVOFF,
	2, COLMOD, 0x05,
	2, MADCTL, 0b01100000,
	5, CASET, 0x00,0x00, 0x00, 0x9F,
	5, RASET, 0x00,0x00, 0x00, 0x7F,
	};
	unsigned char Instruction = 0;
	while(Instruction < sizeof(InitSequence)/sizeof(char)){
	WriteCommand(InitSequence[Instruction + 1], &InitSequence[Instruction + 2], InitSequence[Instruction]-1);
	Instruction += InitSequence[Instruction] + 1;
	}
	WriteCommand(RAMWR);
	for(int i = 0; i<D_WIDTH*D_HEIGHT; i++){
		unsigned int data = 0b0000000000000000;
		HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&data, 2, 100);
	}

}

void SetAddr(int RowStart, int ColStart, int RowEnd = 0x9F, int ColEnd = 0x7F){
    WriteCommandVargs(CASET, 0x00, RowStart, 0x00, RowEnd);
    WriteCommandVargs(RASET, 0x00, ColStart, 0x00, ColEnd);
}

void DrawPixel(int x, int y, unsigned int Color, int thickness = 2){
    for(int Xoff = 0; Xoff < thickness; Xoff++){
        for(int Yoff = 0; Yoff < thickness; Yoff++){
            SetAddr(x+Xoff,y+Yoff);
            WriteCommand(RAMWR);
            uint8_t Transfer[] = {(Color & 0xFF00) >> 8,  Color & 0xFF};
            HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Transfer, 2, 100);
        }
    }
}

void DrawLine(int StartX, int StartY, int EndX, int EndY, unsigned int Color){
    if(( (abs(EndX - StartX) >= abs(EndY-StartY)) && (EndX < StartX) ) || ( ((abs(EndX - StartX) < abs(EndY-StartY)) && (EndY < StartY))) ){
      Swap<int>(StartX, EndX);
      Swap<int>(StartY, EndY);
    }
    int Cycles = max((EndX - StartX),(EndY - StartY));
    float Slope = 0;
    int StartMapping[] = {StartX, StartY};
    int CoordMapping[] = {0,0}, defau_map = 1;

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

        if(CoordMapping[0] >= 0 && CoordMapping[0] < 160 && CoordMapping[1] >= 0 && CoordMapping[1] < 128)
        DrawPixel(CoordMapping[0], CoordMapping[1], Color);
    }
}

void DrawRect(int StartX, int StartY, int EndX, int EndY, int Color){
    DrawLine(StartX, StartY, EndX, StartY, Color);
    DrawLine(StartX, StartY, StartX, EndY, Color);
    DrawLine(StartX, EndY, EndX,EndY, Color);
    DrawLine(EndX, StartY, EndX, EndY, Color);
}

void DrawEllipse(int StartX, int StartY, int EndX, int EndY, int Color, int mapping){
    if((abs(EndX - StartX) >= abs(EndY-StartY) && EndX < StartX) || (abs(EndX - StartX) < abs(EndY-StartY) && EndY < StartY)){
        Swap<int>(StartX, EndX);
        Swap<int>(StartY, EndY);
    }
    int Cycles[2] = {(EndX - StartX),(EndY - StartY)};
    int Center[2] = {(StartX+EndX)/2,(StartY+EndY)/2};
    int Offset[2] = {abs(StartX-EndX)/2,abs(StartY-EndY)/2};
    int VertexA = abs(EndX-StartX)/2, VertexB = abs(EndY-StartY)/2;
    int Verticies[2] = {VertexA, VertexB};
    int CoordMapping[2] = {0,0}, defau_map = 0;
    if (abs(EndY - StartY) > abs(EndX - StartX)){ //if line is longer vertically rise > run
        defau_map = 1;
    }
    if(mapping != -1){
        defau_map = mapping;
    }else{
        DrawEllipse(StartX, StartY, EndX, EndY, Color, 1 - defau_map);
    }
    int opp_map = 1 - defau_map;
    for(int i = 0; i <= Cycles[defau_map]; i++){
        CoordMapping[defau_map] = int(i); //default: CoordMapping[x,y] modified:CoordMapping[y,x]
        int X = CoordMapping[defau_map];
        CoordMapping[opp_map] = sqrt(((1.0 - float( pow(X-Verticies[defau_map],2)) /(  float(pow(Verticies[defau_map],2)) ))) * pow(float(Verticies[opp_map]),2));

        for(int j = 1; j > -2; j-=2){
            int OffsetMapping[2] = {0, 0};
            OffsetMapping[defau_map] = -Offset[defau_map];
            int OutputMapping[] = {0,0};
            CoordMapping[opp_map] *= j;
            OutputMapping[defau_map] = Center[defau_map] + CoordMapping[defau_map] + OffsetMapping[defau_map];
            OutputMapping[opp_map] = Center[opp_map] + CoordMapping[opp_map] + OffsetMapping[opp_map];
            int XCoord = OutputMapping[0], YCoord = OutputMapping[1];
            if(XCoord >= 0 && XCoord < 160 && YCoord >= 0 && YCoord < 128)
              DrawPixel(XCoord, YCoord, Color, 2);
        }
    }
}

void DrawCharacter(char Character, int StartX, int StartY, uint16_t ForegndColor, uint16_t BckgndColor){
	SetAddr(StartX, StartY, StartX + 9, StartY + 9);
	WriteCommand(RAMWR);
	for(int i = 0; i < 100; i++){
		uint16_t data = ForegndColor;
		if(CapFont[Character - 65][i] == 0){
			data = BckgndColor;
		}
		HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&data, 2, 100);
	}
}

void WriteText(char* text, int length, int StartX, int StartY, uint16_t ForegndColor, uint16_t BckgndColor){
	const int TextGap = 3;
	for(int i = 0; i < length - 1; i++){
		DrawCharacter(text[i], StartX, StartY, ForegndColor, BckgndColor);
		StartX += 9;
	}
}

void DrawImage(uint16_t image[]){
	SetAddr(0, 0);
	WriteCommand(RAMWR);
	for(int i = 0; i < D_HEIGHT * D_WIDTH; i++){
		uint8_t Data = ((image[i] & 0xFF00) >> 8);
		HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Data, 1, 100);
		Data = (image[i] & 0xFF);
		HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&Data, 1, 100);
	}
}

unsigned int GenColor(unsigned int R, unsigned int G, unsigned int B){
    R = R <= 0x1F ? R : 0x1F;
    G = G <= 0x3F ? G : 0x3F;
    B = B <= 0x1F ? B : 0x1F;
    return((R<<11) + (G<<5) + B);
}

unsigned int ColorRatio(float R, float G, float B){
    R = R <= 1.0 ? R : 1.0;
    G = G <= 1.0 ? G : 1.0;
    B = B <= 1.0 ? B : 1.0;
    return GenColor(int(R * 31), int(G * 63), int(B * 31));
}


