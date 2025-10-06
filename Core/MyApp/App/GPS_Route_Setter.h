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
} GPS_Route;

/**
 * @brief Struct to hold GPS coordinates in decimal degrees.
 */
typedef struct {
	double latitude;    // Latitude in decimal degrees
	double longitude;   // Longitude in decimal degrees
} GPS_decimal_degrees_t, *PGPS_decimal_degrees_t;

uint8_t GPS_Route_Maker();



#endif /* MYAPP_APP_GPS_ROUTE_SETTER_H_ */
