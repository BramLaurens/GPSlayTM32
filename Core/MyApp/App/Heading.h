/*
 * Heading.h
 *
 *  Created on: okt 9, 2025
 *      Author: SebeB
 */
#ifndef HEADING_H
#define HEADING_H
#include "gps.h"
#include "GPS_Route_Setter.h" // for GPS_Route declaration if needed

double Deg_Heading(int Next_routing_point);
int Give_NodeNumber(void *argument);
double Distance_Till_Waypoint(int Next_routing_point);
void PID_Controller();

#endif // HEADING_H