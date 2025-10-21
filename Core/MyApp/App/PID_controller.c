/**
 * @file PID_controller.c
 * @author Bram Laurens
 * @brief PID controller implementation
 * @version 0.1
 * @date 2025-10-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "gps.h"
#include <math.h>
#include <stdbool.h>   
#include "routeperformer.h"
#include "dGPS.h"

dGPS_decimalData_t latest_dGPS_data;

void PID_Controller(void *argument)
{
    osDelay(200); // wait a second to make sure everything is started

    while (1)
    {
        // PID control logic would go here
        GPS_getlatest_uncorrected(&latest_dGPS_data);
        osDelay(100); // Control loop delay
    }
}