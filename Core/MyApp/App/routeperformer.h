/*
 * Routeperformer.h
 *
 *  Created on: okt 9, 2025
 *      Author: SebeB
 */
#ifndef ROUTEPERFORMER_H
#define ROUTEPERFORMER_H
#include "gps.h"
#include "GPS_Route_Setter.h" // for GPS_Route declaration if needed
#include <stdbool.h>

int Give_NodeNumber(void *argument);
double Distance_Till_Waypoint(int Next_routing_point);
void Route_performer(void *argument);
void getlatestCourse(double *dest);
void set_RP_algoState(bool state);
void get_waypointhold(bool *dest);
void set_waypointhold(bool state);
void get_RP_distance(double *dest);
void get_RP_AlgoState(bool *dest);

#endif // ROUTEPERFORMER_H
