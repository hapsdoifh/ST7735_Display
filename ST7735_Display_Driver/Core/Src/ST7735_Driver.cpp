/*
 * ST7735_Driver.c
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */


#include <stm32f4xx_hal.h>
#include "ST7735_Driver.h"

static SPI_HandleTypeDef *DisplayHandle;

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
	2, MADCTL, 0b10100000,
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
		unsigned char data = 0b0000000000000000;
		HAL_SPI_Transmit(DisplayHandle, (uint8_t*)&data, 2, 100);
	}

}
