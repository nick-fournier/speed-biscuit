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

#define UART_PORT_NUM      UART_NUM_0   // Use UART1, or UART_NUM_2 if preferred
#define UART_BAUD_RATE     9600         // Typical baud rate for L76B
#define UART_RX_PIN        GPIO_NUM_44   // Define the GPIO pin for RX (change to your actual pin)
#define UART_TX_PIN        GPIO_NUM_43   // Define the GPIO pin for TX (change to your actual pin)
#define UART_BUFFER_SIZE   1024         // Buffer size for incoming data


#include <stdbool.h>

typedef struct {
    char time[10];
    char status;
    double latitude;
    char latitude_dir;
    double longitude;
    char longitude_dir;
    float speed;
    float track_angle;
    char date[7];
    float magnetic_variation;

} GNRMCPacket;

bool parseGNRMC(const uint8_t* data, GNRMCPacket* packet) {
    // Find the GNRMC protocol
    const char* gnrmc = strstr((const char*)data, "$GNRMC");
    if (gnrmc == NULL) {
        return false; // GNRMC protocol not found
    }

    // Parse the GNRMC protocol
    sscanf(
        gnrmc, "$GNRMC,%[^,],%c,%lf,%c,%lf,%c,%f,%f,%[^,],%f",
        packet->time, &packet->status,
        &packet->latitude, &packet->latitude_dir,
        &packet->longitude, &packet->longitude_dir,
        &packet->speed, &packet->track_angle,
        packet->date, &packet->magnetic_variation
    );

    return true;
}

void formatGNRMC(const GNRMCPacket* packet, char* output) {
    sprintf(
        output, "Time: %s Date: %s Status: %c Magnetic Variation: %.2f\nLatitude: %lf %c Longitude: %lf %c Speed: %.2f Track Angle: %.2f",
        packet->time, packet->date, packet->status, 
        packet->magnetic_variation,
        packet->latitude, packet->latitude_dir,
        packet->longitude, packet->longitude_dir,
        packet->speed, packet->track_angle
        );
}


// Convert the latitude and longitude from the format ddmm.mmmm to decimal degrees
double convertToDecimalDegrees(double ddmmmm, char dir) {
    int dd = (int)(ddmmmm / 100); // Extract the degrees
    double mm = ddmmmm - (dd * 100); // Extract the minutes
    double decimalDegrees = dd + (mm / 60); // Calculate the decimal degrees
    if (dir == 'S' || dir == 'W') {
        decimalDegrees *= -1; // Apply negative sign for South and West directions
    }
    return decimalDegrees;
}


void app_main(void) {
    // Configuration of the UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install the driver and set the UART parameters
    uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t data[UART_BUFFER_SIZE];

    while (true) {
        // Read data from the UART
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(100));

        if (len > 0) {
            data[len] = '\0';  // Null-terminate the data
        }

        // Parse the GNRMC protocol
        GNRMCPacket packet;
        static int cycle_count = 0;
        if (parseGNRMC(data, &packet)) {
            if (cycle_count % 10 == 0) {
                char output[256];
                formatGNRMC(&packet, output);
                ESP_LOGI("GPS", "%s", output);
            }
            cycle_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms to avoid blocking
    }
}

