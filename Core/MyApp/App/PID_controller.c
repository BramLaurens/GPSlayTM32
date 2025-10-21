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

// ==== PID constants ====
#define KP  0.35
#define KI  0.001
#define KD  0.05

// ==== Control parameters ====
#define BASE_SPEED 100     // normalized 0–1 (or map to PWM)
#define MAX_SPEED  255


// ==== Shared variables (updated by GPS task) ====
volatile double desiredHeading = 0.0;   // degrees
volatile double currentHeading = 0.0;   // degrees

// ==== Internal PID state ====
static double integral = 0.0;
static double prevError = 0.0;
double lastTime = 0.0;

// ==== Function prototypes ====
double headingError(double target, double current);
double pidCompute(double error, double dt);

bool enablePID = false;
unsigned int key = 0;

dGPS_decimalData_t latest_dGPS_data;

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

void PID_check(){

    // Get latest GPS data
    GPS_getlatest_uncorrected(&latest_dGPS_data);
    currentHeading = latest_dGPS_data.course;
    getlatestAngle(&desiredHeading);

    double dt;
    TickType_t now = xTaskGetTickCount();
    dt = (now - lastTime) / 1000.0; // convert to seconds
    lastTime = now;

    double error = headingError(desiredHeading, currentHeading);
    double steering = pidCompute(error, dt);

    double leftSpeed = BASE_SPEED + steering;
    double rightSpeed = BASE_SPEED - steering;

    // Clamp and apply
    if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
    if (leftSpeed < -MAX_SPEED) leftSpeed = -MAX_SPEED;
    if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
    if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;

    Motor_Set_Speed((int16_t)leftSpeed, (int16_t)rightSpeed);

    osDelay(100); // Control loop delay
}

void PID_Controller(void *argument)
{
    osDelay(200); // wait a second to make sure everything is started

    while (1)
    {
        // Non-blocking: Try to read a key from the queue. If none available, continue doing other work.
        if (hKeyRP_Queue != NULL)
        {
            if (xQueueReceive(hKeyRP_Queue, &key, 0) == pdTRUE)
            {
                // Process key
                switch(key)
                {
                    case 5:
                        UART_puts("\r\n Toggle received in PID_Controller\r\n");
                        enablePID = !enablePID;
                        break;
                    default:
                        UART_puts("\r\nInvalid key pressed for PID_Controller\r\n");
                        break; // continue loop
                }
            }
        }

        if (enablePID)
        {
            PID_check();
        }
        else
        {
            Motor_Set_Speed(0, 0); // Stop motors when PID is disabled
        }
        osDelay(10); // Idle delay when PID is disabled
    }
}