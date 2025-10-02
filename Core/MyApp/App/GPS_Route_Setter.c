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


GNRMC *pt_Route_Parser; 

// a function to init and call other functions necessary for the Route Setter
void Route_Setter(void *argument)
{
	GPS_Route *pt_Route=NULL;
	// testing instead of gps data
	pt_Route_Parser = malloc(sizeof(GNRMC));
	strncpy(pt_Route_Parser->latitude, "51.3654",10);
	strncpy(pt_Route_Parser->longitude, "50.97652",10);
	pt_Route_Parser->status = 'A';

	uint32_t key=0;
	while(TRUE){
		key = xEventGroupWaitBits(hKEY_Event, 0xffff, pdTRUE, pdFALSE, portMAX_DELAY ); // wait for a ARM key press
		UART_puts("\r\n");
		switch(key){
		case 0x01: // key 1 pressed
			UART_puts("\r\n");
			UART_puts("Trying to set a waypoint");
			UART_puts("\r\n");
			// fill_GNRMC(&pt_Route_Parser);	// data from the gps filled in pt_Route_Parser
			pt_Route = GPS_Route_Maker(pt_Route); // Creates a node for a linked list everytime the ARM key is pressed
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






// creates a node for the linked list containing coordinates in Long, Lapt_Route_Parser which are pulled from 
// the global gps data struct: "pt_Route_Parser" if the data recieved is valid.
GPS_Route *GPS_Route_Maker(GPS_Route *pt_Route)
{
	char Float_buffer[100]; // char buffer so the floats can be made visible for the user
	if(pt_Route_Parser->status == 'V'){ // 'V' is invalid 'A' is valid 
		UART_puts("Data from GPS is not valid or is currently busy locking");
		// posibility for lcd screen debuging
		return NULL; 
	}

	GPS_Route *Node= malloc(sizeof(GPS_Route)); // free memory so a struct can be added returning a pointer to that memory address
	if(Node==NULL){ // error malloc failed
		UART_puts("Malloc failed");
		return NULL;
	}

	if(pt_Route == NULL){ // if there is no head(first node of a linked list)  
		Node->longitude= atof(pt_Route_Parser->longitude);	// make a float out of the ascii char of the gps data
		Node->latitude= atof(pt_Route_Parser->latitude);	// make a float out of the ascii char of the gps data
		Node->Next_point = NULL; 
		UART_puts("Head created");
		UART_puts("\r\n");
		sprintf(Float_buffer, "Long:%2.9f",Node->longitude); // So the float can read by the user in terminal 
		UART_puts(Float_buffer); UART_puts("\r\n");
		// posibility for lcd screen debuging and showing user the coordinates
		sprintf(Float_buffer, "Lat:%2.9f",Node->latitude);	// So the float can read by the user in terminal 
		UART_puts(Float_buffer); UART_puts("\r\n");
		return Node;
	}

	// adding a node other then the head(first)
	GPS_Route *temp = pt_Route; // creating a temp so the original pointer does not get lost when searching for a next point
	int i=1;
	UART_puts("begin search for next point. \r\n");
	while(temp->Next_point != NULL){ // searching for the next node with no next point so one can be added
		i++;
		temp = temp->Next_point; 
	}
	UART_puts("\r\n");
	UART_puts("Node in linked list:"); UART_putint(i); UART_puts("\r\n");
	Node->longitude= atof(pt_Route_Parser->longitude);
	Node->latitude= atof(pt_Route_Parser->latitude);
	Node->Next_point = NULL;
	temp->Next_point = Node;
	sprintf(Float_buffer, "Long:%2.9f",Node->longitude);
	UART_puts(Float_buffer); UART_puts("\r\n");
	// LCD_clear();						// bedenk iets om dit goed te doen zonder dat node aanmaken verkeerd gaat of blocked is.
	// LCD_puts(Float_buffer);				// dit ook
	sprintf(Float_buffer, "Lat:%2.9f",Node->latitude);
	UART_puts(Float_buffer); UART_puts("\r\n");
	UART_puts("New node created");
	UART_puts("\r\n");
	return pt_Route;
}

