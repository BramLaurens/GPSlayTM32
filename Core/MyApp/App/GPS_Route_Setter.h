/*
 * GPS_Route_Setter.h
 *
 *  Created on: Oct 1, 2025
 *      Author: SebeB
 */

#ifndef MYAPP_APP_GPS_ROUTE_SETTER_H_
#define MYAPP_APP_GPS_ROUTE_SETTER_H_



typedef struct _GPS_Route{
int Lat;
int Long;
struct _GPS_Route *Next_point;
} GPS_Route;



#endif /* MYAPP_APP_GPS_ROUTE_SETTER_H_ */
