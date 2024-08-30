/*****************************************************************************
* | File      	:	DEV_Config.c
* | Author      :   Waveshare team
* | Function    :	GPIO Function
* | Info        :
*   Provide the hardware underlying interface	 
*----------------
* |	This version:   V1.0
* | Date        :   2018-01-11
* | Info        :   Basic version
*
******************************************************************************/

#include "DEV_Config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GPIO write function for ESP32-S3
void DEV_Digital_Write(UWORD Pin, UBYTE Value) {
    gpio_set_level((gpio_num_t)Pin, Value);
}

// GPIO read function for ESP32-S3
UBYTE DEV_Digital_Read(UWORD Pin) {
    return gpio_get_level((gpio_num_t)Pin);
}

// GPIO mode configuration for ESP32-S3
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << Pin),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = (Mode == 0) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
}

// Initialize GPIOs for ESP32-S3
void DEV_GPIO_Init(void) {
    DEV_GPIO_Mode(LCD_RST_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(LCD_DC_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(LCD_BKL_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(LCD_CS_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(TP_CS_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(TP_IRQ_PIN, GPIO_MODE_INPUT);
    DEV_GPIO_Mode(SD_CS_PIN, GPIO_MODE_OUTPUT);

    gpio_set_pull_mode(TP_IRQ_PIN, GPIO_PULLUP_ONLY);  // Set pull-up

    DEV_Digital_Write(TP_CS_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 1);
    DEV_Digital_Write(LCD_BKL_PIN, 1);
    DEV_Digital_Write(SD_CS_PIN, 1);
}


static spi_device_handle_t spi;  // Declare the SPI device handle as static

// Initialize GPIOs and SPI for ESP32-S3
uint8_t System_Init(void) {
    DEV_GPIO_Init();

    // SPI Configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = LCD_MISO_PIN,
        .sclk_io_num = LCD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(SPI_PORT, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return 1; // Return an error code if initialization fails
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000,  // Clock out at 4 MHz
        .mode = 0,                  // SPI mode 0
        .spics_io_num = LCD_CS_PIN, // CS pin
        .queue_size = 7,            // We want to be able to queue 7 transactions at a time
    };

    // Initialize the SPI device
    ret = spi_bus_add_device(SPI_PORT, &devcfg, &spi);
    if (ret != ESP_OK) {
        return 1; // Return an error code if initialization fails
    }

    return 0;
}

void System_Exit(void) {
    // You can add clean-up code if needed, but often not necessary for ESP32
}

// Write a byte to SPI for ESP32-S3
uint8_t SPI4W_Write_Byte(uint8_t value) {
    spi_transaction_t t = {
        .length = 8,  // Transaction length in bits
        .tx_buffer = &value,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(spi, &t);  // Transmit
    if (ret != ESP_OK) {
        return 0; // Return 0 or handle the error if transmission fails
    }
    return *(uint8_t *)t.rx_buffer;  // This line may be adjusted depending on your actual needs
}


// Read a byte from SPI for ESP32-S3
uint8_t SPI4W_Read_Byte(uint8_t value) {
    return SPI4W_Write_Byte(value);  // Use the same function for write and read
}

// Delay function in milliseconds for ESP32-S3
void Driver_Delay_ms(uint32_t xms) {
    vTaskDelay(xms / portTICK_PERIOD_MS);
}

// Delay function in microseconds for ESP32-S3
void Driver_Delay_us(uint32_t xus) {
    esp_rom_delay_us(xus);
}
