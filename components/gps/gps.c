// gps.c
#include "gps.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <time.h>

void parseGNRMC(const uint8_t* data, GNRMCPacket* packet, bool verbose) {
    const char* gnrmc = strstr((const char*)data, "$GNRMC");
    long last_epoch_time = packet->epoch_time;

    if (gnrmc == NULL) {
        return;
    }
    
    // <0> $GNRMC
    // <1> UTC time, the format is hhmmss.sss
    // <2> Positioning status, A=effective positioning, V=invalid positioning
    // <3> Latitude, the format is ddmm.mmmmmmm
    // <4> Latitude hemisphere, N or S (north latitude or south latitude)
    // <5> Longitude, the format is dddmm.mmmmmmm
    // <6> Longitude hemisphere, E or W (east longitude or west longitude)
    // <7> Ground speed
    // <8> Ground heading (take true north as the reference datum)
    // <9> UTC date, the format is ddmmyy (day, month, year)
    // <10> Magnetic declination (000.0~180.0 degrees)
    // <11> Magnetic declination direction, E (east) or W (west)
    // <12> Mode indication (A=autonomous positioning, D=differential, E=estimation, N=invalid data)

    // %d: Reads an integer.
    // %f: Reads a floating-point number.
    // %s: Reads a string.
    // %c: Reads a character.
    // %x: Reads a hexadecimal integer.
    // %o: Reads an octal integer.
    // %u: Reads an unsigned integer.

    // $GNRMC,043316.000,A,3755.3217,N,12218.6623,W,0.12,285.62,270824,,,A
    
    sscanf(
        gnrmc, "$GNRMC,%10[^,],%c,%lf,%c,%lf,%c,%f,%f,%6[^,],%f",
        packet->time,
        &packet->status,
        &packet->latitude,
        &packet->latitude_dir,
        &packet->longitude,
        &packet->longitude_dir,
        &packet->speed,
        &packet->track_angle,
        packet->date,
        &packet->magnetic_variation
    );

    // Update latitude and longitude to decimal degrees
    packet->latitude = convertToDecimalDegrees(packet->latitude, packet->latitude_dir);
    packet->longitude = convertToDecimalDegrees(packet->longitude, packet->longitude_dir);

    // Convert date and time to epoch
    packet->epoch_time = convertToEpoch(packet->date, packet->time);

    // Format time and date
    formatTime(packet->time, packet->time);
    formatDate(packet->date, packet->date);

    int tdelta = packet->epoch_time - last_epoch_time;

    if (tdelta > 0 && packet->epoch_time % 5 == 0 && verbose) {
        char output[256];
        formatGNRMC(packet, output);
        ESP_LOGI("GPS", "%s", output);
    }

}

void formatGNRMC(const GNRMCPacket* packet, char* output) {
    sprintf(
        output, "Timestamp: %s %s LatLon: %lf %c %lf %c Speed: %.2f Bearing: %.2f Epoch: %ld",
        packet->date,
        packet->time,
        packet->latitude,
        packet->latitude_dir,
        packet->longitude,
        packet->longitude_dir,
        packet->speed,
        packet->track_angle,
        packet->epoch_time
    );
}

double convertToDecimalDegrees(double ddmmmm, char dir) {
    int dd = (int)(ddmmmm / 100);
    double mm = ddmmmm - (dd * 100);
    double decimalDegrees = dd + (mm / 60);
    if (dir == 'S' || dir == 'W') {
        decimalDegrees *= -1;
    }
    return decimalDegrees;
}

long convertToEpoch(const char* date, const char* time) {
    struct tm tm;
    memset(&tm, 0, sizeof(struct tm));

    // Parse date
    sscanf(date, "%2d%2d%2d", &tm.tm_mday, &tm.tm_mon, &tm.tm_year);
    tm.tm_mon -= 1; // Adjust month to range 0-11
    tm.tm_year += 100; // Adjust year to range 1900-1999

    // Parse time
    sscanf(time, "%2d%2d%2d", &tm.tm_hour, &tm.tm_min, &tm.tm_sec);

    // Convert to epoch as long integer
    long epoch = mktime(&tm);

    return epoch;
}

void formatTime(const char* input, char* output) {
    int hours, minutes;
    double seconds;
    sscanf(input, "%2d%2d%lf", &hours, &minutes, &seconds);

    sprintf(output, "%02d:%02d:%.3lf", hours, minutes, seconds);
}

void formatDate(const char* input, char* output) {
    int day, month, year;
    sscanf(input, "%2d%2d%2d", &day, &month, &year);
    sprintf(output, "%02d/%02d/%02d", day, month, year);
}