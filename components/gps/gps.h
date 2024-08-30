// gps.h
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

// Structure to store GNRMC packet data
typedef struct {
    char time[10];
    char status;
    double latitude;
    char latitude_dir;
    double longitude;
    char longitude_dir;
    float speed;
    float track_angle;
    char date[6];
    float magnetic_variation;
    long epoch_time; // New field for epoch time
} GNRMCPacket;

// Function prototypes
void parseGNRMC(const uint8_t* data, GNRMCPacket* packet, bool verbose);
void formatGNRMC(const GNRMCPacket* packet, char* output);
void formatTime(const char* input, char* output);
void formatDate(const char* input, char* output);
double convertToDecimalDegrees(double ddmmmm, char dir);
long convertToEpoch(const char* date, const char* time);


#endif // GPS_H
