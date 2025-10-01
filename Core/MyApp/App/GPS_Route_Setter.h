/*
 * GPS_Route_Setter.h
 *
 *  Created on: Oct 1, 2025
 *      Author: SebeB
 */

#ifndef MYAPP_APP_GPS_ROUTE_SETTER_H_
#define MYAPP_APP_GPS_ROUTE_SETTER_H_

typedef struct _GPS_Route{
    double    latitude[10];
    double   longitude[10];
    struct _GPS_Route *Next_point;
} GPS_Route;


GPS_Route *GPS_Route_Maker(GPS_Route *Route);



#endif /* MYAPP_APP_GPS_ROUTE_SETTER_H_ */
