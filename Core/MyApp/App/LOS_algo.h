/*
 * LOS_algo.h
 *
 *  Created on: Nov 3, 2025
 *      Author: BramL
 */

#ifndef MYAPP_APP_LOS_ALGO_H_
#define MYAPP_APP_LOS_ALGO_H_

typedef struct {
    double desired_bearing;    // deg: bearing toward lookahead point
    double heading_error;      // deg: difference between desired and compass heading
    double distance_to_wp;     // meters: distance to current waypoint
    bool waypoint_reached;     // true if close enough to current waypoint
    bool route_complete;       // true if no waypoints left
} LOS_Output_t;

void set_LOS_algoState(bool state);
void LOS_getdesiredheading(double *dest);
void LOS_getwaypointhold_status(bool *dest);
void LOS_getcurrentWPnumber(int *dest);
void LOS_getWPbearing(double *dest);
void LOS_getWPdistance(double *dest);

#endif /* MYAPP_APP_LOS_ALGO_H_ */