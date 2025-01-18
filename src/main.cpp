
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
// #include <esp_rom_gpio.h>
#include "driver/gpio.h"
#include <driver/spi_master.h>
#include "ST7735_Driver.h"
#include <semphr.h>
#include <esp_bt_main.h>


#define BLINK_GPIO 9

SemaphoreHandle_t buttonEvt;

void IRAM_ATTR onButton(void *pvParams){
    uint32_t gio_num = (uint32_t)pvParams;

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

    WriteText("HELLO", sizeof("HELLO")/sizeof(char), 60, 40, ColorRatio(0,0,1), ColorRatio(0,1,0));
    vTaskDelay(1000);
    while(1) {
        printf("%d:%d\n", gpio_get_level((gpio_num_t)21), gpio_get_level((gpio_num_t)5));
	    vTaskDelay(100 / 5);
    }
}

extern "C" void app_main()
{       
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 21) | (1ULL << 5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL
    };
    buttonEvt = xSemaphoreCreateMutex();
    gpio_config(&io_conf);
    xTaskCreate(&do_display, "do disp", 2048,NULL,5,NULL);
    gpio_isr_handler_add(GPIO_NUM_5,onButton,(void*)5);
}

