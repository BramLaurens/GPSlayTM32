/**
 * @file Heading.c
 * @brief 
 
 * @author Sebe Buitenkamp
 * @version WIP
 * @date 2025-10-09
 * 
 * 
 */


#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "GPS_Route_Setter.h"
#include "gps.h"
#include <math.h>

#define Error_marge_completed_waypoint      //error marge for when the leaphy is within x meters of the waypoint 

/* Todo
Curr waypoint call 
distance call
when waypoint is achieved with error marge

*/

GNRMC GNRMC_localcopy4;
GPS_Route *pRoute_copy;

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
    UART_puts("error something went wrong in receiving the data or calculating the error \r \n");
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

    getlatest_GNRMC(&GNRMC_localcopy4);
    pRoute_copy = Route_Pointer_Request(); // returns the pointer to the waypoints

    if(GNRMC_localcopy4.status != 'a') // invalid data or no data is received
    {
        UART_puts("Error no Valid data recieved form gps (GNRMC) \r \n");
        return -1.0;
    }

    if(pRoute_copy == NULL) //struct is empty
    {
        UART_puts("Error no data in waypoint structs received \r \n");
        return -1.0;
    }

    double Angle = -2; // 0 is valid angle, -1 is error so -2 so other errors will still be visible 
    
    // zorg dat de struct wel afgegaan wordt.
    UART_puts("Starting to view the next point \r \n");
    GPS_Route *temp = pRoute_copy;
    while(Next_routing_point != temp->nodeNumber) // maybe change to for loop 
    { // goes through the struct searching for the node number which it is supposed to be going to
        temp = temp->Next_point;

        if((temp == NULL)) // error the end of the struct should be caught before the struct walkthrough
        { 
            UART_puts("Error end of struct reached without catch \r \n");
            return -1; 
        }
    }

    char Buffer[100]; // buffer for sprintf for debugging the float
    if((temp->Next_point != NULL))
    { // finish after this
        UART_puts("End of struct reachted this is the last point \r \n");
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

double Distance_Till_Waypoint(){

    return -1;
}







void PID_Controller()
{
    double Angle=-2, Distance=0;
    int Next_routing_point=-1; // -1 till init aka button press or other function calls this


    while(1)
    {
        Angle = Deg_Heading(Next_routing_point); // returns the angle or error code

        osDelay(1);
    }
}





