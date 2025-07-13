#include "gps.h"

// Private variables
static UART_HandleTypeDef *gps_huart;
static volatile bool gps_rx_complete = false;

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    // Start UART reception in interrupt mode
    HAL_UART_Receive_IT(gps_huart, (uint8_t*)&gps_data.nmea_buffer[gps_data.nmea_index], 1);
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == gps_huart) {
        // Check for end of NMEA sentence
        if (gps_data.nmea_buffer[gps_data.nmea_index] == '\n') {
            gps_data.nmea_buffer[gps_data.nmea_index + 1] = '\0'; // Null-terminate
            gps_rx_complete = true;
            gps_data.nmea_index = 0;
        } else {
            gps_data.nmea_index++;
            if (gps_data.nmea_index >= sizeof(gps_data.nmea_buffer) - 1) {
                gps_data.nmea_index = 0; // Prevent buffer overflow
            }
        }
        // Restart reception
        HAL_UART_Receive_IT(gps_huart, (uint8_t*)&gps_data.nmea_buffer[gps_data.nmea_index], 1);
    }
}

bool GPS_ParseNMEA(GPS_Data *gps) {
    if (!gps_rx_complete) return false;

    gps_rx_complete = false;
    gps->new_data = false;

    // Check for valid NMEA sentence
    if (strncmp(gps->nmea_buffer, "$GPGGA", 6) == 0) {
        // Parse GGA sentence (Global Positioning System Fix Data)
        char *token = strtok(gps->nmea_buffer, ",");
        uint8_t field = 0;
        while (token != NULL) {
            switch (field) {
                case 1: // UTC Time
                    if (strlen(token) >= 6) {
                        gps->hour = (token[0] - '0') * 10 + (token[1] - '0');
                        gps->minute = (token[2] - '0') * 10 + (token[3] - '0');
                        gps->second = (token[4] - '0') * 10 + (token[5] - '0');
                    }
                    break;
                case 2: // Latitude
                    if (strlen(token) >= 4) {
                        double lat = atof(token);
                        gps->latitude = floor(lat / 100) + fmod(lat, 100) / 60;
                    }
                    break;
                case 3: // Latitude direction (N/S)
                    if (token[0] == 'S') gps->latitude *= -1;
                    break;
                case 4: // Longitude
                    if (strlen(token) >= 5) {
                        double lon = atof(token);
                        gps->longitude = floor(lon / 100) + fmod(lon, 100) / 60;
                    }
                    break;
                case 5: // Longitude direction (E/W)
                    if (token[0] == 'W') gps->longitude *= -1;
                    break;
                case 6: // Fix quality
                    gps->fix_quality = atoi(token);
                    break;
                case 7: // Number of satellites
                    gps->satellites = atoi(token);
                    break;
                case 9: // Altitude
                    gps->altitude = atof(token);
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
        gps->data_valid = (gps->fix_quality > 0);
        gps->new_data = true;
        return true;
    }
    else if (strncmp(gps->nmea_buffer, "$GPRMC", 6) == 0) {
        // Parse RMC sentence (Recommended Minimum Navigation Information)
        char *token = strtok(gps->nmea_buffer, ",");
        uint8_t field = 0;
        while (token != NULL) {
            switch (field) {
                case 1: // UTC Time
                    if (strlen(token) >= 6) {
                        gps->hour = (token[0] - '0') * 10 + (token[1] - '0');
                        gps->minute = (token[2] - '0') * 10 + (token[3] - '0');
                        gps->second = (token[4] - '0') * 10 + (token[5] - '0');
                    }
                    break;
                case 2: // Status (A=active, V=void)
                    gps->data_valid = (token[0] == 'A');
                    break;
                case 3: // Latitude
                    if (strlen(token) >= 4) {
                        double lat = atof(token);
                        gps->latitude = floor(lat / 100) + fmod(lat, 100) / 60;
                    }
                    break;
                case 4: // Latitude direction (N/S)
                    if (token[0] == 'S') gps->latitude *= -1;
                    break;
                case 5: // Longitude
                    if (strlen(token) >= 5) {
                        double lon = atof(token);
                        gps->longitude = floor(lon / 100) + fmod(lon, 100) / 60;
                    }
                    break;
                case 6: // Longitude direction (E/W)
                    if (token[0] == 'W') gps->longitude *= -1;
                    break;
                case 7: // Speed over ground (knots)
                    gps->speed = atof(token);
                    break;
                case 8: // Course over ground (degrees)
                    gps->course = atof(token);
                    break;
                case 9: // Date
                    if (strlen(token) >= 6) {
                        gps->day = (token[0] - '0') * 10 + (token[1] - '0');
                        gps->month = (token[2] - '0') * 10 + (token[3] - '0');
                        gps->year = (token[4] - '0') * 10 + (token[5] - '0') + 2000;
                    }
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
        gps->new_data = true;
        return true;
    }
    return false;
}

float GPS_DistanceBetween(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula for distance calculation
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = sin(dLat/2) * sin(dLat/2) +
               sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return 6371000 * c; // Earth radius in meters
}

float GPS_BearingTo(double lat1, double lon1, double lat2, double lon2) {
    // Calculate bearing between two points
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / M_PI;

    // Normalize to 0-360
    if (bearing < 0) bearing += 360;
    return bearing;
}
