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

typedef struct {
    double latitude;
    double longitude;
    uint32_t timestamp; // Unix timestamp of the GPS data
} dGPS_errorData_t, *PdGPS_errorData_t;

typedef struct {
    double latitude;
    double longitude;
    uint32_t timestamp; // Unix timestamp of the GPS data
} dGPS_decimalData_t, *PdGPS_decimalData_t;

extern double convert_decimal_degrees(char *nmea_coordinate, char* ns);

#endif /* MYAPP_APP_dGPS_H_ */
