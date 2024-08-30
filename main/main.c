// main.c
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>
#include "gps.h"  // Include the GPS header file
#include "kalman.h"  // Include the Kalman filter header file

#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     9600
#define UART_RX_PIN        GPIO_NUM_44
#define UART_TX_PIN        GPIO_NUM_43
#define UART_BUFFER_SIZE   1024

void set_gps_update_rate(int rate_ms) {
    // Construct the command
    char command[50];
    snprintf(command, sizeof(command), "$PMTK220,%d", rate_ms);

    // Calculate the checksum
    uint8_t checksum = 0;
    for (int i = 1; i < strlen(command); i++) {
        checksum ^= command[i];
    }

    // Append the checksum to the command
    snprintf(command + strlen(command), sizeof(command) - strlen(command), "*%02X\r\n", checksum);

    // Send the command to the GPS module
    uart_write_bytes(UART_PORT_NUM, command, strlen(command));
}


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
    
    int dt = 200;  // Update rate in milliseconds
    set_gps_update_rate(dt);

    uint8_t data[UART_BUFFER_SIZE];

    // // Measurements: Latitude, Longitude, Time
    // KalmanFilter kf;
    // kalman_init(&kf);
    // float prev_lat = kf.state[0];
    // float prev_lon = kf.state[1];
    // float prev_time = 0;

    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(dt));

        if (len > 0) {
            data[len] = '\0';
        }

        GNRMCPacket packet;

        parseGNRMC(data, &packet, true);
        

        // float measurement[DIM] = {packet.latitude, packet.longitude};
        // float current_time = packet.epoch_time;

        // // Kalman filter prediction and update steps
        // kalman_predict(&kf);
        // kalman_update(&kf, measurement);

        // // Calculate distance and speed between consecutive measurements
        // float distance = haversine_distance(prev_lat, prev_lon, kf.state[0], kf.state[1]);
        // float delta_time = current_time - prev_time;
        // float speed = calculate_speed(distance, delta_time);
        // printf("Updated state: Latitude = %f, Longitude = %f, Speed = %f m/s\n", kf.state[0], kf.state[1], speed);

        // // Update previous state
        // prev_lat = kf.state[0];
        // prev_lon = kf.state[1];
        // prev_time = current_time;
        vTaskDelay(100);
    }
}
