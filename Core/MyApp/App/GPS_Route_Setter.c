/**
 * @file GPS_Route_Setter.c
 * @brief Verwerkt de GPS data en zet deze om in een linked list met waypoints. GPS data wordt gehaald uit gps.c via de functie getlatest_GNRMC()
 * Er zijn drie acties mogelijk:
 * 1. Een waypoint toevoegen aan de linked list met de functie GPS_Route_Maker
 * 2. De linked list bekijken met de functie View_Linkedlist
 * 3. De linked list wissen met de functie Remove_Last_Node
 * Ook is er een functie om de NMEA coordinaten om te zetten naar decimale graden: convert_decimal_degrees()
 * @author Sebe Buitenkamp & Bram Laurens
 * @version 0.1
 * @date 2025-10-04
 * 
 * 
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
// pt_Route_Parser is redonedend maybe remove for final build

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
 * @brief Views all nodes in the linked list and prints them using UART
 * 
 * @param not in use
 * @return void
 */
void View_Linkedlist()
{
	if(pt_Route == NULL)
		{	// if the linked list is empty then return
			UART_puts("Linked list is empty \r\n");
			return; 
		}

	char buffer[100];
	GPS_Route *temp = pt_Route;
	if(temp->Next_point == NULL)
	{
		sprintf(buffer,"Long:%2.9f\n Lat:%2.9f ", temp->longitude, temp->latitude);
		UART_puts(buffer);
		UART_puts("\r\n");
		return;
	}

	while(temp!= NULL) // go through and print all nodes while there is a node
	{
		sprintf(buffer,"Long:%2.9f\n Lat:%2.9f ", temp->longitude, temp->latitude);
		UART_puts(buffer);
		UART_puts("\r\n");
		temp=temp->Next_point;
	}

	return;
}



/**
 * @brief creates a node for the linked list containing coordinates in Long, Lapt_Route_Parser 
 * which are pulled from the global gps data struct: "pt_Route_Parser" if the data recieved is valid.
 * 
 * @param Void *arg not in use
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


// return float if error -1
// temp hierin wordt de Lat of Long gegeven zodat we weten welke we terug moeten geven
double Error_calc_LongLat(char temp[10])
{
	if(!strcmp(temp,"Lat")) // if Lat it will return the latitude 
	{
		// then check if the data received from the gps over nrf is new and/or availible 

		

		
	}
	else if(!strcmp(temp,"Long"))
	{



	}

	UART_puts("Unknown param given expected 'Lat or 'Long'. \r\n");
	return -1;
}







/**
 * @brief Removes the last node from the linked list if the linked list is 
 * 
 * @param not in use
 * @return returns an int based on the state of removal 1: linked list is empty, 0: The last node is removed, -1: The linked list is empty when this function was called 
 */
uint8_t Remove_Last_Node()
{

	if(pt_Route == NULL)
	{	// if the linked list is empty then return -1
		UART_puts("Linked list is empty \r\n");
		return 2; // error code maybe other nr
	}

	if(pt_Route->Next_point == NULL)
	{ // the last node of the linked list
		free(pt_Route); // this frees the memory for the kernel to be used again
		pt_Route = NULL; // the pointer will still point to the spot in memory so make it NULL so no exidental read write operation is done
		UART_puts("Linked list is now cleared completly \r\n");
		return 1; // returns 1 when the linked list is completly empty 
	}

	GPS_Route *temp = pt_Route;
    GPS_Route *prev = NULL;

    while(temp->Next_point != NULL) {
        prev = temp;
        temp = temp->Next_point;
    }

    // temp is now the last node, prev is the second-to-last

    free(temp);
    prev->Next_point = NULL;

    UART_puts("Removed last node \r\n ");
    return 0;
}

/**
 * @brief Task that waits for a ARM key press to set a waypoint by calling the GPS_Route_Maker function, making a linkedlist of waypoints.
 * 
 * @param argument 
 */
void Route_Setter(void *argument)
{
	//Create a key variable to store the key pressed by the user
	uint32_t key=0;

	while(TRUE){
		// Wait for a notification from the ARM key handler.
		xTaskNotifyWait(0x00,            // Don't clear any notification bits on entry
						0xffffffff,      // Clear the notification value on exit
						&key,            // Notified value
						portMAX_DELAY);  // Block indefinitely
		UART_puts("\r\n");

		switch(key){
		case 0x01: // key 1 pressed (upper left)
			#ifdef debug_routesetter
				UART_puts("\r\n");
				UART_puts("Trying to set a waypoint...");
				UART_puts("\r\n");
			#endif
			getlatest_GNRMC(&GNRMC_localcopy3);
			GPS_Route_Maker(); // Creates a node for a linked list everytime the ARM key is pressed
			break;

		case 0x02: // remove last node 
			UART_puts("Removing last node...");
			Remove_Last_Node();
			break;

		case 0x03: // remove all nodes 
			UART_puts("Removing all nodes...");
			while(Remove_Last_Node() == 0);
			break;

		case 0x04:
		UART_puts("Viewing all items in linked list...");
			View_Linkedlist();
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


