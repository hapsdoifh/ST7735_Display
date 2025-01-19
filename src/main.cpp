
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>

#include "driver/gpio.h"
#include <driver/uart.h>
#include <driver/spi_master.h>
#include "ST7735_Driver.h"

#include <esp_timer.h>


#include "esp32c3/rom/ets_sys.h"
#include <esp_console.h>


#define BLINK_GPIO 9


static portMUX_TYPE mySpinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t commandQueue;

SemaphoreHandle_t buttonEvt;

bool button_5_pressed = false;
bool button_21_pressed = false;

int64_t last_time = esp_timer_get_time()/1000;

void IRAM_ATTR onButton(void *pvParams){
    
    *(bool*)pvParams = true;
    int64_t current_time = esp_timer_get_time()/1000;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(current_time - last_time > 100){
        //this means that the button handling function is on
        ets_printf("Hello?");
        last_time = current_time;
        xSemaphoreGiveFromISR(buttonEvt, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void checkButtonDeferred(void* pvParams){
    while(true){
        xSemaphoreTake(buttonEvt, portMAX_DELAY);
        if(button_21_pressed){
            printf("button is 21\n");
            button_21_pressed = false;
        }
        if(button_5_pressed){
            printf("button is 5\n");
            button_5_pressed = false;
        }
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

    WriteText("HELLO", sizeof("HELLO")/sizeof(char), 60, 40, ColorRatio(0,0,1), ColorRatio(0,1,0));
    
    while(1) {
        portENTER_CRITICAL(&mySpinlock);
        // ets_printf("%d:%d\n", gpio_get_level((gpio_num_t)21), gpio_get_level((gpio_num_t)5));
        portEXIT_CRITICAL(&mySpinlock);
        
        vTaskDelay(100 / 5);
    }
}

void recieveUartData(void *pvParameter){
    // uart_event_t uartEvent;
    // char data[100];
    // while(1){
        
    //     if(xQueueReceive(commandQueue, (void*)&uartEvent, pdMS_TO_TICKS(1000)) == pdTRUE){
    //         if(uartEvent.type == UART_DATA){
    //             int readLen = uart_read_bytes(UART_NUM_0, data, uartEvent.size, pdMS_TO_TICKS(1000));
    //             data[readLen] = 0;
    //             ets_printf("input data is: %s", data);
    //         }
    //     }
    // }
    int num = 0;
    uint8_t* data = (uint8_t*) malloc(30);
    while (1) {
    	int len = sprintf ((char*)data, "Hello world %d\n", num++);
        uart_write_bytes(UART_NUM_0, data, len);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    free (data);
}

void uart_setup(){
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Configure UART parameters
    uart_driver_install(uart_num, 1024, 1024,10,&commandQueue,0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	// uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
}

void gpio_setup(){

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 21) | (1ULL << 5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    buttonEvt = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(GPIO_NUM_21,onButton,(void*)&button_21_pressed);
    gpio_isr_handler_add(GPIO_NUM_5, onButton,(void*)&button_5_pressed);
}

extern "C" void app_main()
{       

    gpio_setup();
    uart_setup();
    // esp_reset_reason_t reason = esp_reset_reason();
    // printf("Reset reason: %d\n", reason);

    // gpio_isr_handler_add(GPIO_NUM_21,onButton,(void*)21);
    xTaskCreate(&do_display, "do disp", 2048,NULL,5,NULL);
    xTaskCreate(&checkButtonDeferred, "check button", 2048,NULL,6,NULL);
    xTaskCreate(&recieveUartData, "uart recieve", 4096, NULL, 5, NULL);
    
}


