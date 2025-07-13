#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// GPS Data Structure
typedef struct {
    // Raw NMEA data
    char nmea_buffer[256];
    uint8_t nmea_index;

    // Parsed data
    double latitude;      // Degrees
    double longitude;     // Degrees
    float altitude;       // Meters
    float speed;          // Knots
    float course;         // Degrees
    uint8_t satellites;   // Number of satellites
    uint8_t fix_quality;  // 0=no fix, 1=GPS fix, 2=DGPS fix
    uint8_t fix_type;     // 1=no fix, 2=2D, 3=3D

    // Time
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // Date
    uint8_t day;
    uint8_t month;
    uint16_t year;

    // Status flags
    bool data_valid;
    bool new_data;
} GPS_Data;

// Function prototypes
void GPS_Init(UART_HandleTypeDef *huart);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
bool GPS_ParseNMEA(GPS_Data *gps);
void GPS_ProcessData(GPS_Data *gps);
float GPS_DistanceBetween(double lat1, double lon1, double lat2, double lon2);
float GPS_BearingTo(double lat1, double lon1, double lat2, double lon2);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */
