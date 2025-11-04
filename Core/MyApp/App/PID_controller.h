#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "routeperformer.h"
#include "admin.h"

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

void PID_setObstacleFlag(bool flag);
void PID_getObstacleFlag(bool *flag);
void PID_getObstacleAvoidanceState(bool *state);

#endif // PID_CONTROLLER_H
