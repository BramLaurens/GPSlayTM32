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

#define Error_marge_completed_waypoint 1   //error marge for when the leaphy is within x meters of the waypoint (+ or -) in meters


GNRMC GNRMC_localcopy4;
GPS_Route *pRoute_copy;
int *pNext_Waypoint;



/**
 * @brief gives the current waypoint nr the leaphy is going to
 * 
 * @return int 
 */
int Give_NodeNumber(void *argument)
{
    return *pNext_Waypoint; // gives the value the pointer is pointed at
}



/**
 * @brief checks if the structs exist and have valid data
 * if the data is not valid or structs dont excist corretly then NULL
 * 
 * @param Next_routing_point 
 * @return GPS_Route* 
 */
GPS_Route *check_structs(int Next_routing_point)
{
    getlatest_GNRMC(&GNRMC_localcopy4);
    pRoute_copy = Route_Pointer_Request(); // returns the pointer to the waypoints

    if(GNRMC_localcopy4.status != 'a') // invalid data or no data is received
    {
        UART_puts("Error no Valid data recieved form gps (GNRMC) (check_structs) \r \n");
        return NULL;
    }

    if(pRoute_copy == NULL) //struct is empty
    {
        UART_puts("Error no data in waypoint structs received (check_structs) \r \n");
        return NULL;
    }

    GPS_Route *temp = pRoute_copy;
    while(Next_routing_point != temp->nodeNumber) // maybe change to for loop 
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
 * @brief 
 * calculates the angle for the leaphy to turn to in degrees 
 * -1 for errors and only positif angles
 *  dont call this function unless you give it the correct node yourself a
 * @param Route pointer to the node in the linked list that needs to be visited
 * @return the angle as a double 
 */
double Calc_Angle(GPS_Route *pRoute)
{
    double Angle=0;
    double dLat=0,dLong=0;
    dLat = convert_decimal_degrees(GNRMC_localcopy4.latitude,  &GNRMC_localcopy4.EW_ind) - pRoute->latitude;
    dLong = convert_decimal_degrees(GNRMC_localcopy4.longitude,  &GNRMC_localcopy4.NS_ind) - pRoute->longitude;
    // ordening is important!
    if(dLat <0 && dLong <0) return (atan(dLat/dLong) + 180);  // the angle is in the third quater of the cirkel
    if(dLong == 0 && dLat < 0) return (270.0);                  // the angle is straight left   (arctan(-dLat/0))
    if(dLat < 0)  return (atan(dLat/dLong)+270);              // the angle is in the fourth quater of the cirkel
    if(dLong < 0)  return (atan(dLat/dLong)+90);              // the angle is in the second quater of the cirkel
    if(dLong == 0 && dLat > 0) return (90.0);                   // the angle is straight right  (arctan(dLat/0))
    if(dLong == 0 && dLat == 0) return 0.0;                     // the angle has no deviation of the current heading
    UART_puts("error something went wrong in receiving the data or calculating the error (Calc_Angle) \r \n");
    return -1.0;
}


/**
 * @brief checks for errors and gives the Calc_Angle() the correct struct
 * 
 * @param int Next_routing_point, 
 * @return double (angle)
 */
double Deg_Heading(int Next_routing_point)
{
    UART_puts("Starting angle calculation \r \n");
    double Angle = -2; // 0 is valid angle, -1 is error so -2 so other errors will still be visible
    
    // zorg dat de struct wel afgegaan wordt.
    UART_puts("Starting to view the next point \r \n");
    GPS_Route *temp = check_structs(Next_routing_point);

    char Buffer[100]; // buffer for sprintf for debugging the float
    if((temp->Next_point != NULL))
    { // finish after this
        UART_puts("End of struct reachted this is the last point  \r \n");
        Angle = Calc_Angle(temp);

        sprintf(Buffer, "Angle is: %0.4f \r \n", Angle);
        UART_puts(Buffer);

        return Angle;
    }
    // print out the angle 
    Angle = Calc_Angle(temp);
    
    sprintf(Buffer, "Angle is: %0.4f \r \n", Angle);
    UART_puts(Buffer);

    return Angle;
}




/**
 * @brief gives the distance to a waypoint or the current waypoint heading by calling: Give_NodeNumber()
 * 
 * @param Next_routing_point 
 * @return double 
 */
double Distance_Till_Waypoint(int Next_routing_point)
{
    GPS_Route *temp = pRoute_copy;
    temp = check_structs(Next_routing_point); 

    if(temp == NULL){
        UART_puts("Error one of the structs is not filled or correct (Distance_Till_Waypoint)\r \n");
        return -1;
    }
    double dLat = convert_decimal_degrees(GNRMC_localcopy4.latitude,  &GNRMC_localcopy4.EW_ind) - temp->latitude;
    double dLong = convert_decimal_degrees(GNRMC_localcopy4.longitude,  &GNRMC_localcopy4.NS_ind) - temp->longitude;
    // calc from lat, long to x, y using Rijksdriehoeks conversion 


    // Reference point (Amersfoort) for now this will change to our Ref gps
    const double phi0 = 52.15517440; // Lat
    const double lam0 = 5.38720621; // Long

    double phi = (convert_decimal_degrees(GNRMC_localcopy4.latitude,  &GNRMC_localcopy4.EW_ind) - phi0) * 0.36; //Lat
    double lambda = (convert_decimal_degrees(GNRMC_localcopy4.longitude,  &GNRMC_localcopy4.NS_ind) - lam0) * 0.36; // Long

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

    double x = x0 + x_rd;
    double y = y0 + y_rd;
    double z = pow(x,2) + pow(y,2); // now in cartesian you can triangulate with pytharogrian theorem for you distance
    return sqrt(z);
}



/**
 * @brief returns the waypoint nr or -1 for error and shows if a waypoint or the end has been reached
 * 
 * @param Distance_to_point 
 * @return int 
 */
int Completed_waypoint(double  Distance_to_point)
{
    getlatest_GNRMC(&GNRMC_localcopy4);
    pRoute_copy = Route_Pointer_Request(); // returns the pointer to the waypoints

    if(GNRMC_localcopy4.status != 'a') // invalid data or no data is received
    {
        UART_puts("Error no Valid data recieved form gps (GNRMC) (Completed_waypoint) \r \n");
        return -1;
    }

    if(pRoute_copy == NULL) //struct is empty
    {
        UART_puts("Error no data in waypoint structs received (Completed_waypoint) \r \n");
        return -1;
    }

    // give area to check point or car 
    if(Distance_to_point < Error_marge_completed_waypoint ) // see if the leaphy is close enough to the waypoint see header for #define
    {
        int Next_routing_point = Give_NodeNumber(NULL);
        UART_putint(Next_routing_point); // shows the nr of the waypoint got
        UART_puts(" Waypoint got \r \n");

        GPS_Route *temp = pRoute_copy;
        while(Next_routing_point != temp->nodeNumber) // maybe change to for loop 
        { // goes through the struct searching for the node number which it is supposed to be going to
            temp = temp->Next_point;

            if((temp->Next_point == NULL)) // error the end of the struct should be caught before the struct walkthrough
            { 
                UART_puts("Last waypoint achieved! \r \n");
                return Give_NodeNumber(NULL); 
            }
        }
        UART_puts("Going to next waypoint \r \n");
        return Give_NodeNumber(NULL)+1; // gives back current waypoint if got then +1 
    }
    return Give_NodeNumber(NULL); 
}








void PID_Controller()
{
    double Angle=-2, Distance=0;
    int Next_routing_point=-1; // -1 till init aka button press or other function calls this
    pNext_Waypoint = &Next_routing_point;

    while(1)
    {
        Angle = Deg_Heading(Next_routing_point); // returns the angle or error code
        Distance = Distance_Till_Waypoint(Next_routing_point); // returns the distance in m 
        Next_routing_point = Completed_waypoint(Distance);
        osDelay(1);
    }
}





