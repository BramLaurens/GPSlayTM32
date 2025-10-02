/*
 * GPS_Route_Setter.h
 *
 *  Created on: Oct 1, 2025
 *      Author: SebeB
 */

#ifndef MYAPP_APP_GPS_ROUTE_SETTER_H_
#define MYAPP_APP_GPS_ROUTE_SETTER_H_

typedef struct _GPS_Route{
    double    latitude;
    double   longitude;
    uint8_t nodeNumber;
    struct _GPS_Route *Next_point;
} GPS_Route;

uint8_t GPS_Route_Maker();



#endif /* MYAPP_APP_GPS_ROUTE_SETTER_H_ */
