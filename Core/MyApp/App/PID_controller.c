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
#include "motordriver.h"
#include "compass_driver.h"
#include "LOS_algo.h"

// #define DEBUG_PID_CONTROLLER

// ==== PID constants ====
#define KP  2.5
#define KI  0.01
#define KD  0.1

/*Good base tuning
Kp = 4.5
Ki = 0.01
Kd = 0.1
 */

// ==== Control parameters ====
#define BASE_SPEED 100     // normalized 0–1 (or map to PWM)
#define MAX_SPEED  255
// Minimum effective PWM value to actually move the tracks. Commands with
// absolute value below this will be clamped to this value so the motors
// overcome static friction. When the desired action is effectively zero
// (error near zero and BASE_SPEED == 0) the motors remain off.
#define MIN_SPEED 0


// ==== Shared variables (updated by GPS task) ====
volatile double desiredHeading = 0.0;   // degrees
volatile double currentHeading = 0.0;   // degrees

// Waypoint hold status
volatile bool pid_waypoint_hold = false;    // from route performer

// ==== Internal PID state ====
static double integral = 0.0;
static double prevError = 0.0;
double lastTime = 0.0;

// ==== Function prototypes ====
double headingError(double target, double current);
double pidCompute(double error, double dt);

bool enablePID = false;
unsigned int key = 0;
bool obstacleFlag = false;

dGPS_decimalData_t latest_dGPS_data;

void PID_setObstacleFlag(bool flag)
{
    obstacleFlag = flag;
}

double headingError(double target, double current) {
    double error = target - current;
    while (error > 180.0)  error -= 360.0;
    while (error < -180.0) error += 360.0;
    return error;
}

double pidCompute(double error, double dt) {
    integral += error * dt;
    double derivative = (error - prevError) / dt;
    prevError = error;

    double output = (KP * error) + (KI * integral) + (KD * derivative);

    // Normalize output range (e.g., -1 to 1)
    if (output > 255) output = 255;
    if (output < -255) output = -255;

    return output;
}

void obstacleAvoidance(){

    Motor_Set_Speed(0, 0); // stop motors
    osDelay(300); // wait 300 ms
    Motor_Set_Speed(-100, 100); // turn left
    osDelay(500); // turn for 500 ms
    Motor_Set_Speed(0, 0); // stop motors
    osDelay(300); // wait 300 ms
    obstacleFlag = false; // reset obstacle flag

}

void PID_trigger(){

    // Get latest heading and course to calculate error
    getlatestHeading(&currentHeading);
    LOS_getdesiredheading(&desiredHeading);

    double dt;
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTime) / 1000.0; // convert to seconds
    lastTime = now;

    double error = headingError(desiredHeading, currentHeading);
    double steering = pidCompute(error, dt);

    double leftSpeed = BASE_SPEED + steering;
    double rightSpeed = BASE_SPEED - steering;

    // If the desired action is effectively zero (no base speed and very
    // small heading error), keep motors off so they don't waste power.
    const double EPS_ERROR = 0.5; // degrees (tolerance for considering error zero)
    bool keepMotorsOff = (fabs(error) < EPS_ERROR) && (BASE_SPEED == 0);

    // Clamp to max limits first
    if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
    if (leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;
    if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
    if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;

    // Apply minimum effective speed to overcome stiction when a non-zero
    // command is requested. Preserve exact zero when keepMotorsOff is true.
    if (!keepMotorsOff) {
        if ((leftSpeed > 0) && (leftSpeed < MIN_SPEED)) leftSpeed = MIN_SPEED;
        if ((leftSpeed < 0) && (leftSpeed > -MIN_SPEED)) leftSpeed = -MIN_SPEED;

        if ((rightSpeed > 0) && (rightSpeed < MIN_SPEED)) rightSpeed = MIN_SPEED;
        if ((rightSpeed < 0) && (rightSpeed > -MIN_SPEED)) rightSpeed = -MIN_SPEED;
    } else {
        leftSpeed = 0;
        rightSpeed = 0;
    }

    Motor_Set_Speed((int16_t)leftSpeed, (int16_t)rightSpeed);

    #ifdef DEBUG_PID_CONTROLLER
    char msg[128];
    sprintf(msg, "PID Debug: DH=%.2f CH=%.2f Err=%.2f LSpd=%.2f RSpd=%.2f\r\n",
            desiredHeading, currentHeading, error, leftSpeed, rightSpeed);
    UART_puts(msg);
    #endif
}

void PID_Controller(void *argument)
{
    osDelay(200); // wait a second to make sure everything is started

    while (1)
    {
        // Non-blocking: Try to read a key from the queue. If none available, continue doing other work.

        LOS_getwaypointhold_status(&pid_waypoint_hold);

        if (hKeyPID_Queue != NULL)
        {
            if (xQueueReceive(hKeyPID_Queue, &key, 0) == pdTRUE)
            {
                // Process key
                switch(key)
                {
                    case 5:
                        UART_puts("\r\n Toggle received in PID_Controller\r\n");
                        // Toggle PID controller state and inform Route Performer with same state
                        enablePID = !enablePID;
                        // set_RP_algoState(enablePID);
                        set_LOS_algoState(enablePID);

                        if (enablePID)
                            UART_puts("PID Controller enabled\r\n");
                        else
                            UART_puts("PID Controller disabled\r\n");   
                        break;
                    default:
                        UART_puts("\r\nInvalid key pressed for PID_Controller\r\n");
                        break; // continue loop
                }
            }
        }

        if (enablePID && !pid_waypoint_hold && !obstacleFlag)
        {
            PID_trigger();
            osDelay(10); // Control loop delay

        }
        else if(!enablePID)
        {
            Motor_Set_Speed(0, 0); // Stop motors when PID is disabled
        }
        else if(pid_waypoint_hold)
        {
            Motor_Set_Speed(0, 0); // Stop motors when in waypoint hold
        }
        else if(obstacleFlag)
        {
            obstacleAvoidance();
        }
        
        osDelay(10); // Idle delay when PID is disabled
    }
}