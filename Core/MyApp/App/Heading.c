/**
 * @file Heading.c
 * @brief 
 
 * @author Sebe Buitenkamp
 * @version 0.1
 * @date 2025-10-09
 * 
 * used https://rijksdriehoekscoordinaten.nl/ calculation of Lat, Long to X, Y. and converted it to C
 */


#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "GPS_Route_Setter.h"
#include "gps.h"
#include <math.h>
#include <stdbool.h>   
#include "dGPS.h"

#define M_PI 3.14159265358979323846

#define Error_marge_completed_waypoint 3   //error margin for when the leaphy is within x meters of the waypoint (+ or -) in meters


dGPS_decimalData_t dGPS_localcopy4;
GPS_Route *pRoute_copy;
int *pWorking_Waypoint;

int update_GPS_loc()
{
    GPS_getlatest_uncorrected(&dGPS_localcopy4); // get the latest uncorrected gps data
    return 0;
}

/**
 * @brief Returns the current waypoint number the leaphy is heading to
 * 
 * @return int 
 */
int Give_NodeNumber()
{
    return *pWorking_Waypoint; // gives the value the pointer is pointed at
}

/**
 * @brief checks if the structs exist and have valid data, also returns the correct linked list element
 * if the data is not valid or structs dont excist corretly then NULL
 * 
 * @param Next_routing_point The working next waypoint
 * @return GPS_Route* 
 */
GPS_Route *check_structs(int Next_routing_point)
{
    pRoute_copy = Route_Pointer_Request(); // returns the pointer to the waypoints

    if(pRoute_copy == NULL) //struct is empty
    {
        UART_puts("Error no data in waypoint structs received (check_structs) \r \n");
        return NULL;
    }

    GPS_Route *temp = pRoute_copy;

    // Find the correct waypoint in the linked list
    while(Next_routing_point != temp->nodeNumber) 
    { // goes through the struct searching for the node number which it is supposed to be going to
        temp = temp->Next_point;

        if((temp == NULL)) // error the end of the struct should be caught before the struct walkthrough
        { 
            UART_puts("Error end of struct reached without catch or excisting node nr (check_structs) \r \n");
            return NULL; 
        }
    }
    return temp;
}


/**
 * @brief Calculates the true heading of the next waypoint, corrected for curvature of the earth
 * -1 for errors and only positif angles
 *  dont call this function unless you give it the correct node yourself a
 * @param Route pointer to the node in the linked list that needs to be visited
 * @return the angle as a double 
 */
double Calc_Angle(GPS_Route *pRoute)
{
    // Convert all positions to radians
    double lat1 = dGPS_localcopy4.latitude * M_PI / 180.0;
    double lon1 = dGPS_localcopy4.longitude * M_PI / 180.0;
    double lat2 = pRoute->latitude * M_PI / 180.0;
    double lon2 = pRoute->longitude * M_PI / 180.0;

    double dLon = lon2 - lon1;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double angle = atan2(y, x) * 180.0 / M_PI;

    if (angle < 0) angle += 360.0;
    return angle;
}



/**
 * @brief checks for errors and gives the Calc_Angle() the correct struct
 * 
 * @param int Next_routing_point, 
 * @return double (angle)
 */
double GET_workingHeading(int Working_routing_point)
{
    UART_puts("\r\n Starting angle calculation \r \n");
    double Angle = -2; // 0 is valid angle, -1 is error so -2 so other errors will still be visible
    
    // Check de struct validity, and get the right element
    UART_puts("Starting to view the next point \r \n");
    GPS_Route *temp = check_structs(Working_routing_point);

    // Update GPS location
    if(update_GPS_loc() < 0) // update the gps location and check for errors
    {
        return -1; // error updating gps location
    }

    if(temp == NULL){
        UART_puts("Error one of the structs is not filled or correct (GET_workingHeading)\r \n");
        return -1;
    }

    char Buffer[100]; // buffer for sprintf for debugging the float

    Angle = Calc_Angle(temp);
    if(Angle < 0) // error in calculation
    {
        UART_puts("Error calculating the angle (GET_workingHeading) \r \n");
        return -1;
    }
    sprintf(Buffer, "Angle is: %0.4f \r \n", Angle);
    UART_puts(Buffer);

    return Angle;
}

double distance_tillwaypoint_FE(int Working_routing_point)
{
    UART_puts("\r\n Starting distance calculation using flat earth (FE) model\r \n");
    GPS_Route *temp = pRoute_copy;

    if(update_GPS_loc() < 0) // update the gps location and check for errors
    {
        return -1; // error updating gps location
    }

    //Check if the structs are valid, and get pointer to the correct element
    temp = check_structs(Working_routing_point); 

    if(temp == NULL)
    {
        UART_puts("Error one of the structs is not filled or correct (Distance_Till_Waypoint_FE)\r \n");
        return -1;
    }

    const double lat_to_m = 111320.0; // meters per degree latitude
    const double lon_to_m = 111320.0 * cos(dGPS_localcopy4.latitude * M_PI / 180.0); // meters per degree longitude, adjusted for latitude

    double dLat = (dGPS_localcopy4.latitude - temp->latitude);
    double dLong = (dGPS_localcopy4.longitude - temp->longitude);

    double dx = dLong * lon_to_m;
    double dy = dLat * lat_to_m;

    double distance = sqrt(dx * dx + dy * dy);
    char Buffer[100]; // buffer for sprintf for debugging the float
    sprintf(Buffer, "Distance calculated (FE model): %2.4f \r \n", distance);
    UART_puts(Buffer);
    return distance;
}

/**
 * @brief gives the distance to a waypoint or the current waypoint heading by calling: Give_NodeNumber()
 * 
 * @param Next_routing_point 
 * @return double 
 */
double Distance_Till_Waypoint(int Working_routing_point)
{
    UART_puts("\r\n Starting distance calculation \r \n");
    GPS_Route *temp = pRoute_copy;

    if(update_GPS_loc() < 0) // update the gps location and check for errors
    {
        return -1; // error updating gps location
    }
    //Check if the structs are valid
    temp = check_structs(Working_routing_point); 

    if(temp == NULL){
        UART_puts("Error one of the structs is not filled or correct (Distance_Till_Waypoint)\r \n");
        return -1;
    }


    // Calculate the distance between the current location and the waypoint
    double dLat = (dGPS_localcopy4.latitude - temp->latitude);
    double dLong = (dGPS_localcopy4.longitude - temp->longitude);


    // calc from lat, long to x, y using Rijksdriehoeks conversion 
    // Reference point (Amersfoort) for now this will change to our Ref gps
    const double phi0 = 52.15517440; // Lat
    const double lam0 = 5.38720621; // Long

    double phi = (dGPS_localcopy4.latitude - phi0) * 0.36; //Lat
    double lambda = (dGPS_localcopy4.longitude - lam0) * 0.36; // Long

    // Base coordinates of amersfoort change this to our using amersfoort as ref or dont and see the distance in X, Y from amersfoort
    const double x0 = 155000.0;
    const double y0 = 463000.0;

    // Compute RD X
    double x_rd =
        (190094.945 * lambda) +
        (-11832.228 * phi * lambda) +
        (-114.221 * phi * lambda * lambda) +
        (-32.391 * pow(lambda, 3)) +
        (-0.705 * phi) +
        (-2.340 * phi * phi * lambda) +
        (-0.608 * phi * phi) +
        (-0.008 * lambda) +
        (0.148 * pow(lambda, 4));

    // Compute RD Y
    double y_rd =
        (309056.544 * phi) +
        (3638.893 * lambda * lambda) +
        (73.077 * phi * phi) +
        (-157.984 * phi * lambda * lambda) +
        (59.788 * pow(phi, 3)) +
        (0.433 * lambda) +
        (-6.439 * phi * phi * lambda) +
        (-0.032 * lambda * lambda) +
        (0.092 * pow(phi, 4)) +
        (-0.054 * phi * pow(lambda, 3));

    // Now calculate the distance between the two points in cartesian coordinates
    double x = x0 + x_rd;
    double y = y0 + y_rd;
    double z = pow(x,2) + pow(y,2); // now in cartesian you can triangulate with pytharogrian theorem for you distance
    double distance = sqrt(z);
    
    UART_puts("Distance calculated: ");
    char Buffer[100]; // buffer for sprintf for debugging the float
    sprintf(Buffer, "%2.4f \r \n", distance);
    UART_puts(Buffer);
    return distance;
}



/**
 * @brief Checks if waypoint has been reached, and returns the current next waypoint
 * 
 * @param Distance_to_point 
 * @return INT The working waypoint if not reached yet, if reached then the next waypoint, -1 for errors
 */
int Completed_waypoint(double Distance_to_point)
{
    if(update_GPS_loc() < 0) // update the gps location and check for errors
    {
        return -1; // error updating gps location
    }
    pRoute_copy = Route_Pointer_Request(); // returns pointer to the head of the waypoint linked list

    if(pRoute_copy == NULL) //struct is empty
    {
        UART_puts("Error no data in waypoint structs received (Completed_waypoint) \r \n");
        return -1;
    }

    // Check if close enough to the next waypoint
    if(Distance_to_point < Error_marge_completed_waypoint ) // Check if the leaphy is close enough to the waypoint see header for #define
    {
        int Working_routing_point = Give_NodeNumber();
        UART_puts("Waypoint reached! Calculating next waypoint...\r \n");
        UART_puts(" Current waypoint got: ");
        UART_putint(Working_routing_point); // shows the nr of the waypoint got
        UART_puts("\r \n");

        GPS_Route *temp = check_structs(Working_routing_point); // get the correct struct

        if(temp == NULL) {
            UART_puts("Error: current waypoint not found in list (Completed_waypoint)\r \n");
            return -1;
        }

        if((temp->Next_point == NULL)) // end of the list
        { 
            UART_puts("Last waypoint achieved! \r \n");
            return -2 ; // indicates last waypoint reached
        }

        UART_puts("Found current waypoint in linked list, returning next waypoint's nodeNumber \r \n");
        /* Return the actual nodeNumber of the next waypoint instead of assuming sequential numbering */
        return temp->Next_point->nodeNumber;
    }
    // Not close enough to the waypoint just return the current waypoint
    return Give_NodeNumber(); 
}

int run_RP_algo(double Distance, int Working_routing_point)
{
    // This function can be used to run the route planning algorithm if needed
    // For now, it does nothing

    if(Completed_waypoint(Distance) < 0) // error check if structs are valid
    {
        return -1; // skip rest of loop and try again
    }

    Working_routing_point = Completed_waypoint(Distance); // Check if waypoint is completed and get next waypoint if so
    return 0; // Success
}

void PID_Controller()
{
    osDelay(200); // wait a second to make sure everything is started
    double Angle=-2, Distance=999;
    static int Working_routing_point = 0; // static so pointer remains valid
    pWorking_Waypoint = &Working_routing_point;

    volatile bool EnableRP_algo = false; // Set to true to enable route planning algorithm, false to disable

    uint32_t key=0;

    UART_puts("PID_Controller started, polling hKey_Queue for key events\r\n");

    while(1)
    {
        // Non-blocking: Try to read a key from the queue. If none available, continue doing other work.
        if (hKeyRP_Queue != NULL)
        {
            if (xQueueReceive(hKeyRP_Queue, &key, 0) == pdTRUE)
            {
                // Process key
                switch(key)
                {
                    case 0x0D: // Get and print heading to next WP
                        Angle = GET_workingHeading(Working_routing_point); // Get angle to working waypoint
                        break;
                    case 0x0E: // Get and print distance to next WP
                        Distance = distance_tillwaypoint_FE(Working_routing_point); // Get distance to working waypoint
                        break;
                    case 0x0F: // Reset route to WP 0
                        Working_routing_point = 0; // Reset to first waypoint
                        break;
                    case 0x10: // Run route planning algorithm
                        EnableRP_algo = !EnableRP_algo; // Toggle RP algo
                        UART_puts(EnableRP_algo ? "Route Planning Algorithm Enabled\r\n" : "Route Planning Algorithm Disabled\r\n");
                        EnableRP_algo ? HAL_GPIO_WritePin(GPIOD, LEDORANGE, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, LEDORANGE, GPIO_PIN_RESET); // Indicate RP algo status on LED
                        break;
                    default:
                        UART_puts("\r\nInvalid key pressed for PID_Controller\r\n");
                        break; // continue loop
                }
            }
        }

        // If RP algo is enabled, run it periodically
        if (EnableRP_algo)
        {
            Distance = distance_tillwaypoint_FE(Working_routing_point); // Update distance to working waypoint
            Angle = GET_workingHeading(Working_routing_point); // Update angle to working waypoint
            Working_routing_point = Completed_waypoint(Distance); // Check if waypoint is completed and get next waypoint if so
        }
        // Do other periodic PID work here, if any
        osDelay(100); // small sleep so this task isn't busy-waiting
    }
}





