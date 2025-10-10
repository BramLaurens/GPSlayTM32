/*
 * GPS_Route_Setter.h
 *
 *  Created on: Oct 1, 2025
 *      Author: SebeB
 */

#ifndef MYAPP_APP_GPS_ROUTE_SETTER_H_
#define MYAPP_APP_GPS_ROUTE_SETTER_H_

/**
 * @brief Struct voor het opslaan van GPS coördinaten in een linked list.
 * 
 */
typedef struct _GPS_Route{
    double    latitude;
    double   longitude;
    uint8_t nodeNumber;
    struct _GPS_Route *Next_point;
    uint8_t Point_Passed; // maybe unnecessary but let it be for now
} GPS_Route;

uint8_t GPS_Route_Maker();
double convert_decimal_degrees(char *nmea_coordinate, char* ns);

GPS_Route *Route_Pointer_Request(); // request the newest route
#endif /* MYAPP_APP_GPS_ROUTE_SETTER_H_ */
