#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define TAG "ST7735"

// Updated SPI and GPIO pin configuration
#define LCD_PIN_MOSI 10
#define LCD_PIN_SCLK 8
#define LCD_PIN_CS   4
#define LCD_PIN_DC   3
#define LCD_PIN_RST  2

#define LCD_WIDTH    128
#define LCD_HEIGHT   160

// SPI handle
spi_device_handle_t lcd_handle;

// Function to send a command to the LCD
void lcd_send_command(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0
    };
    spi_device_transmit(lcd_handle, &t);
}

// Function to send data to the LCD
void lcd_send_data(const uint8_t *data, size_t len) {
    if (len == 0) return;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1
    };
    spi_device_transmit(lcd_handle, &t);
}

// Function to initialize the LCD
void lcd_init() {
    ESP_LOGI(TAG, "Initializing LCD...");

    // Reset the LCD
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // ST7735 initialization sequence (partial, adjust as needed)
    lcd_send_command(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_send_command(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_send_command(0x29); // Display on
    ESP_LOGI(TAG, "LCD Initialized");
}

// Function to clear the screen with a solid color
void lcd_clear_screen(uint16_t color) {
    uint8_t color_high = (color >> 8) & 0xFF;
    uint8_t color_low = color & 0xFF;

    lcd_send_command(0x2A); // Column address set
    uint8_t col_data[] = {0x00, 0x00, 0x00, (LCD_WIDTH - 1)};
    lcd_send_data(col_data, 4);

    lcd_send_command(0x2B); // Row address set
    uint8_t row_data[] = {0x00, 0x00, 0x00, (LCD_HEIGHT - 1)};
    lcd_send_data(row_data, 4);

    lcd_send_command(0x2C); // Memory write
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        uint8_t color_data[] = {color_high, color_low};
        lcd_send_data(color_data, 2);
    }
}

// Main entry point
void ain() {
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_SCLK,
        .miso_io_num = -1, // Not used
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Configure SPI device for the LCD
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &lcd_handle));

    // Configure GPIOs for DC and RST
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Initialize the LCD
    lcd_init();

    // Clear the screen with a blue color (RGB565: 0x001F)
    lcd_clear_screen(0x001F);
    ESP_LOGI(TAG, "Screen cleared with blue color!");
}
