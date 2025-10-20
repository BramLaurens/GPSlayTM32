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

double Deg_Heading(int Next_routing_point);
int Give_NodeNumber(void *argument);
double Distance_Till_Waypoint(int Next_routing_point);
void Route_performer(void *argument);

#endif // ROUTEPERFORMER_H
