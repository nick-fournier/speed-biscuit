// main.c
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <string.h>
#include "gps.h"  // Include the GPS header file

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     9600
#define UART_RX_PIN        GPIO_NUM_44
#define UART_TX_PIN        GPIO_NUM_43
#define UART_BUFFER_SIZE   1024

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t data[UART_BUFFER_SIZE];

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(1000));

        if (len > 0) {
            data[len] = '\0';
        }

        GNRMCPacket packet;

        parseGNRMC(data, &packet);
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}
