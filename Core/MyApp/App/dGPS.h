/*
 * dGPS.h
 *
 *  Created on: Oct 8, 2025
 *      Author: braml
 */

#include "main.h"
#include "cmsis_os.h"


#ifndef MYAPP_APP_dGPS_H_
#define MYAPP_APP_dGPS_H_

/* Enable dGPS output and uncorrected GPS output */
#define enable_dGPS_out
#define enable_uncorrectedGPS_out

typedef struct {
    double latitude;
    double longitude;
    uint32_t timestamp; // Timestamp in HHMMSS format (from NMEA time field)
} dGPS_errorData_t, *PdGPS_errorData_t;

typedef struct {
    double latitude;
    double longitude;
    uint32_t timestamp; // Timestamp in HHMMSS format (from NMEA time field)
} dGPS_decimalData_t, *PdGPS_decimalData_t;

extern double convert_decimal_degrees(char *nmea_coordinate, char* ns);
void GPS_getlatest_ringbuffer(dGPS_decimalData_t *dest);

#endif /* MYAPP_APP_dGPS_H_ */
