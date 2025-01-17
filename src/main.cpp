
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
// #include <esp_rom_gpio.h>
#include "driver/gpio.h"
#include <driver/spi_master.h>
#include "ST7735_Driver.h"
#include <semphr.h>

#define BLINK_GPIO 9


void hello_task(void *pvParameter)
{
	while(1)
	{
	    // printf("Hello world!\n");
	    vTaskDelay(100 / 5);
	}
}
void blinky(void *pvParameter)
{
    // gpio_config(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction((gpio_num_t)BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level((gpio_num_t)BLINK_GPIO, 0);
        vTaskDelay(1000 / 5);
        /* Blink on (output high) */
        gpio_set_level((gpio_num_t)BLINK_GPIO, 1);
        vTaskDelay(1000 / 5);
    }
}
void do_display(void *pvParameter)
{
    // gpio_config(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    spi_device_handle_t tempSPI;
    DisplayInit(&tempSPI);
    DrawLine(0,64,159,64,ColorRatio(0.2, 0.3, 0.4));
    DrawLine(80,0,80,127,ColorRatio(0.2, 0.3, 0.4));
    DrawEllipse(0,0,159,127,ColorRatio(1, 0, 0));
    int gap = 0;
    //  for(int i = 0; i < 10; i++){
    //	  DrawCharacter(i + 65, i * 10 + gap, 10);
    //	  gap++;
    //  }
    WriteText("HELLO", sizeof("HELLO")/sizeof(char), 60, 40, ColorRatio(0,0,1), ColorRatio(0,1,0));
    vTaskDelay(1000);
    // DrawImage(ImageData);
    while(1) {
        printf("%d:%d\n", gpio_get_level((gpio_num_t)21), gpio_get_level((gpio_num_t)5));
	    vTaskDelay(100 / 5);
        /* Blink off (output low) */
        // gpio_set_level((gpio_num_t)BLINK_GPIO, 0);
        // vTaskDelay(1000 / 5);
        // /* Blink on (output high) */
        // gpio_set_level((gpio_num_t)BLINK_GPIO, 1);
        // vTaskDelay(1000 / 5);
    }
}

extern "C" void app_main()
{       
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 21) | (1ULL << 5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    xTaskCreate(&hello_task, "hello_task", 2048, NULL, 5, NULL);
    xTaskCreate(&blinky, "blinky", 512,NULL,5,NULL );
    xTaskCreate(&do_display, "do disp", 2048,NULL,5,NULL);
}
// void app_main(){

// }

// void xPortStartFirstTask(){
    
// }
