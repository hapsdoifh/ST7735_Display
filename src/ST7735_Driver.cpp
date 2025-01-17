/*
 * ST7735_Driver.c
 *
 *  Created on: Apr 2, 2024
 *      Author: zuhan
 */


// #include "stm32l4xx_hal.h"
#include <driver/spi_master.h>
#include "ST7735_Driver.h"
#include "driver/gpio.h"

// static SPI_HandleTypeDef *DisplayHandle;

spi_device_handle_t DisplayHandle;

template<typename type>
void Swap(type &a, type &b){
    type c = a;
    a = b;
    b = c;
}

void WriteParams(){}

void spi_transmit_wrapper(uint8_t* data, int length){
    spi_transaction_t t = {
        .length = static_cast<size_t>(length * 8),       // Data length in bits
        .tx_buffer = data
    };    
    t.user = 0;
	spi_device_transmit(DisplayHandle, &t);
}

template<typename First, typename... VarArgs>
void WriteParams(First first, VarArgs... Args){
	spi_transmit_wrapper((uint8_t*)&first, 1);
    WriteParams(Args...);
}

template<typename command, typename... VarArgs>
void WriteCommandVargs(command Command, VarArgs... Args){
	gpio_set_level((gpio_num_t)DC_PIN, GPIO_PIN_RESET);
	spi_transmit_wrapper((uint8_t*)&Command, 1);
	gpio_set_level((gpio_num_t)DC_PIN, GPIO_PIN_SET);
    WriteParams(Args...);
}

void WriteCommand(unsigned char Command, unsigned char* Params = NULL, unsigned char NumParams = 0){
	gpio_set_level((gpio_num_t)DC_PIN, GPIO_PIN_RESET);
    spi_transmit_wrapper((uint8_t*)&Command, 1);
	gpio_set_level((gpio_num_t)DC_PIN, GPIO_PIN_SET);
    for(int i = 0; i < NumParams; i++){
    	spi_transmit_wrapper((uint8_t*)&Params[i], 1);
    }
}

void HWReset(){
	gpio_set_level((gpio_num_t)RES_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level((gpio_num_t)RES_PIN, GPIO_PIN_SET);
}

void esp32_specific_setup(){
    spi_bus_config_t buscfg = {
        .mosi_io_num = 10,
        .sclk_io_num = 8,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = D_WIDTH * D_HEIGHT * 2 + 8
    };
    // buscfg.mosi_io_num = 10;
    // buscfg.sclk_io_num = 8;
    buscfg.miso_io_num = -1;
    // buscfg.quadwp_io_num = -1;
    // buscfg.quadhd_io_num = -1;
    // buscfg.max_transfer_sz = D_WIDTH * D_HEIGHT * 2 + 8;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configure SPI device for the LCD
    spi_device_interface_config_t devcfg = {
        .mode = 0,                          // SPI mode 0
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .queue_size = 7,                    // Transaction queue size
        .pre_cb = NULL,                     // Pre-transaction callback
        .post_cb = NULL,                     // Post-transaction callback
    };
    devcfg.spics_io_num = CS_PIN;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &DisplayHandle));

    // Configure DC pin as GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DC_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure Reset pin as GPIO (if used)
    if (RES_PIN >= 0) {
        io_conf.pin_bit_mask = (1ULL << RES_PIN);
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}

void DisplayInit(spi_device_handle_t* hspi){
	DisplayHandle = *hspi;

	HWReset();
    esp32_specific_setup();
	gpio_set_level((gpio_num_t)CS_PIN, GPIO_PIN_RESET);
	gpio_set_level((gpio_num_t)DC_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
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
		spi_transmit_wrapper((uint8_t*)&data, 2);
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
            uint8_t Transfer[] = {static_cast<uint8_t>((Color & 0xFF00) >> 8),  static_cast<uint8_t>(Color & 0xFF)};
            spi_transmit_wrapper((uint8_t*)&Transfer, 2);
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
		uint16_t data[] = {static_cast<uint16_t>((ForegndColor & 0xFF00) >> 8), static_cast<uint16_t>(ForegndColor & 0xFF)};
		if(CapFont[Character - 65][i] == 0){
			data[0] = (BckgndColor & 0xFF00) >> 8;
			data[1] = BckgndColor & 0xFF;
		}
		spi_transmit_wrapper((uint8_t*)&data, 2);
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
		spi_transmit_wrapper((uint8_t*)&Data, 1);
		Data = (image[i] & 0xFF);
		spi_transmit_wrapper((uint8_t*)&Data, 1);
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


