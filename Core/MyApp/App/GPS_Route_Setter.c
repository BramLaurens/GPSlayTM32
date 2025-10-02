/*
 * GPS_Route_Setter.c
 *
 *  Created on: Sep 30, 2025
 *      Author: SebeB
 */
#include <admin.h>
#include "main.h"
#include "cmsis_os.h"
#include "GPS_Route_Setter.h"
#include "gps.h"

#define debug_routesetter


// Struct to hold latest GNRMC data from gps.c
GNRMC GNRMC_localcopy3;
GNRMC *pt_Route_Parser = &GNRMC_localcopy3; // Pointer to the struct holding the latest GNRMC data

// Create a pointer of type GPS_Route
GPS_Route *pt_Route=NULL;

/**
 * @brief Converts NMEA coordinate format (ddmm.mmmm) to decimal degrees. (+ for N/E, - for S/W)
 * 
 * @param nmea_coordinate NMEA coordinate string
 * @param ns North south or East west indicator ('N', 'S', 'E', 'W')
 * @return Decimal degrees as double 
 */
double convert_decimal_degrees(char *nmea_coordinate, char* ns)
{
    double raw = atof(nmea_coordinate); // Convert string to double

    int degrees = (int)(raw / 100); // Get the degrees part
    double minutes = raw - (degrees * 100); // Get the minutes part

    double decimal_degrees = degrees + (minutes / 60.0); // Convert to decimal degrees

    if (ns[0] == 'S' || ns[0] == 'W') // Check if the coordinate is South or West
    {
        decimal_degrees = -decimal_degrees; // Make it negative
    }

    return decimal_degrees; // Return the converted value
}

/**
 * @brief creates a node for the linked list containing coordinates in Long, Lapt_Route_Parser 
 * which are pulled from the global gps data struct: "pt_Route_Parser" if the data recieved is valid.
 * 
 * @param pt_Route pointer to the head of the linked list, if there is no head it will create one
 * @return GPS_Route* returns a pointer to the head of the linked list
 */
uint8_t GPS_Route_Maker()
{
	char Float_buffer[100]; // char buffer so the floats can be made visible for the user

	// 'V' is invalid 'A' is valid 
	if(pt_Route_Parser->status == 'V')
	{ 
		UART_puts("Data from GPS is not valid or is currently busy locking");
		// posibility for lcd screen debuging
		return 1; 
	}

 	// free memory so a struct can be added returning a pointer to that memory address
	GPS_Route *Node= malloc(sizeof(GPS_Route));

	// error malloc failed
	if(Node==NULL)
	{ 
		UART_puts("Malloc failed!");
		return 1;
	}

	// if there is no head(first node of a linked list)  
	if(pt_Route == NULL)
	{ 
		Node->longitude= convert_decimal_degrees(GNRMC_localcopy3.longitude, &GNRMC_localcopy3.EW_ind);	// make a float out of the ascii char of the gps data
		Node->latitude= convert_decimal_degrees(GNRMC_localcopy3.latitude, &GNRMC_localcopy3.NS_ind);	// make a float out of the ascii char of the gps data
		Node->nodeNumber = 0;
		Node->Next_point = NULL; 
		UART_puts("Head created");
		UART_puts("\r\n");

		UART_puts("	Waypoint number: "); UART_putint(Node->nodeNumber); UART_puts("\r\n");

		sprintf(Float_buffer, "	Long:%2.9f",Node->longitude); // So the float can read by the user in terminal 
		UART_puts(Float_buffer); UART_puts("\r\n");
		// posibility for lcd screen debuging and showing user the coordinates
		sprintf(Float_buffer, "	Lat:%2.9f",Node->latitude);	// So the float can read by the user in terminal 
		UART_puts(Float_buffer); UART_puts("\r\n");

		pt_Route = Node;
		return 0;
	}

	// adding a node other then the head(first)
	GPS_Route *temp = pt_Route; // creating a temp so the original pointer does not get lost when searching for a next point
	int i=1;
	UART_puts("begin search for next point. \r\n");

	// searching for the next node with no next point so one can be added
	while(temp->Next_point != NULL)
	{ 
		i++;
		temp = temp->Next_point; 
	}
	UART_puts("\r\n");
	UART_puts("Node in linked list:"); UART_putint(i); UART_puts("\r\n");

	// Filling node elements
	Node->longitude= convert_decimal_degrees(GNRMC_localcopy3.longitude, &GNRMC_localcopy3.EW_ind);	// make a float out of the ascii char of the gps data
	Node->latitude= convert_decimal_degrees(GNRMC_localcopy3.latitude, &GNRMC_localcopy3.NS_ind);	// make a float out of the ascii char of the gps data
	Node->nodeNumber = i;
	Node->Next_point = NULL;
	temp->Next_point = Node;

	UART_puts("	Waypoint number: "); UART_putint(Node->nodeNumber); UART_puts("\r\n");

	sprintf(Float_buffer, "	Long:%2.9f",Node->longitude);
	UART_puts(Float_buffer); UART_puts("\r\n");

	sprintf(Float_buffer, "	Lat:%2.9f",Node->latitude);
	UART_puts(Float_buffer); UART_puts("\r\n");

	UART_puts("New node created succesfully");
	UART_puts("\r\n");

	return 0;
}

/**
 * @brief Task that waits for a ARM key press to set a waypoint by calling the GPS_Route_Maker function, making a linkedlist of waypoints.
 * 
 * @param argument 
 */
void Route_Setter(void *argument)
{
	// Just filling GPS data struct with dummy data for testing
	// pt_Route_Parser = malloc(sizeof(GNRMC));
	// strncpy(pt_Route_Parser->latitude, "51.3654",10);
	// strncpy(pt_Route_Parser->longitude, "50.97652",10);
	// pt_Route_Parser->status = 'A';

	//Create a key variable to store the key pressed by the user
	uint32_t key=0;

	while(TRUE){
		// Wait for a notification from the ARM key handler task instead of waiting on the
		// shared event group. Using the event group caused a race where either the
		// key-IRQ task or this task consumed the bits, so only one saw the key press.
		// xTaskNotifyWait blocks until ARM_keys_task (or IRQ task) notifies us.
		xTaskNotifyWait(0x00,            // Don't clear any notification bits on entry
						0xffffffff,      // Clear the notification value on exit
						&key,            // Notified value
						portMAX_DELAY);  // Block indefinitely
		UART_puts("\r\n");

		switch(key){
		case 0x01: // key 1 pressed (upper left)
			#ifdef debug_routesetter
				UART_puts("\r\n");
				UART_puts("Trying to set a waypoint");
				UART_puts("\r\n");
			#endif
			getlatest_GNRMC(&GNRMC_localcopy3);
			GPS_Route_Maker(); // Creates a node for a linked list everytime the ARM key is pressed
			break;


		default: // incease a unassigned key is pressed
			UART_puts("Current key: "); UART_putint(key);
			UART_puts(" is not in use by GPS_Route_Setter.c");
			UART_puts("\r\n");
			break;
		}
		osDelay(1);
	}
}


