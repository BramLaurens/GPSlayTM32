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


GNRMC *Route_Parser;

// bool Verifieer_Data();
// bool of int of iets waarmee je kan zien of de data correct is


//struct struct_functie
//geef terug pointer struct.


// task maken die constant kijkt of een key gepressed is. en dan actie uitvoerd als ja. dan functie Verifieer_Data() dan struct functie.
// Dan zetten op lcd dat het correct is gegaan.


// data die ontvangen is wordt in een pointer opgeslagen die ik kan callen.


// gebruik @brief @PARAM @RETURN ever om functies te documenteren.

//		key = xEventGroupWaitBits(hKEY_Event, 0xffff, pdTRUE, pdFALSE, portMAX_DELAY );
// deze callen want hieruit kan je de key presses halen met key gelijk aan de knop de

// 	LCD_puts(screen_text); // max 16 chars , boven 8 onder 8 denk erom.
// void fill_GNRMC(char *message); pointer wordt de pointer naar de struct met data



//

void Route_Setter(void *argument)
{
	GPS_Route *Route=NULL;
	// testing instead of gps data
	Route_Parser = malloc(sizeof(GNRMC));
	memcpy(Route_Parser->longitude[10],"5.8",8);
	Route_Parser->latitude[10] = "23.698421";
	Route_Parser->status = 'A';

	uint32_t key=0;
	while(TRUE){
		key = xEventGroupWaitBits(hKEY_Event, 0xffff, pdTRUE, pdFALSE, portMAX_DELAY );
		// UART_puts("\r\n");
		// UART_puts(key);
		UART_puts("\r\n");
		switch(key){
		case 0x01: // key 1 pressed
			UART_puts("Trying to set a waypoint");
			UART_puts("\r\n");
			// fill_GNRMC(&Route_Parser);
			Route = GPS_Route_Maker(Route);
			break;


		default:
			UART_puts("Current key: "); UART_putint(key);
			UART_puts(" is not in use by GPS_Route_Setter.c");
			UART_puts("\r\n");
			break;
		}
		osDelay(1);
	}
}





GPS_Route *GPS_Route_Maker(GPS_Route *Route)
{
	if(Route_Parser->status == 'V'){ // 'V' is invalid 'A' is valid
		UART_puts("Data from GPS is not valid or is currently busy locking");
		// hier nog iets met lcd screen doen later for debuging
		return NULL;
	}

	GPS_Route *temp= malloc(sizeof(GPS_Route));
	if(temp==NULL){ // error malloc failed
		UART_puts("Malloc failed");
		return NULL;
	}

	if(Route == NULL){
		strncpy(temp->longitude, Route_Parser->longitude, 10);
		strncpy(temp->latitude, Route_Parser->latitude, 10);
		temp->Next_point = NULL;
		UART_puts("Head created");
		UART_puts("\r\n");
		UART_puts("Longitude in head:"); UART_puts(temp->longitude);
		UART_puts("latitude in head:"); UART_puts(temp->latitude);
		return temp;
	}


	UART_puts("begin search to next point. \r\n");
	while(temp->Next_point != NULL){
		
		UART_puts("Longitude in part:"); UART_puts(temp->longitude);
		UART_puts("latitude in part:"); UART_puts(temp->latitude);
			UART_puts("\r\n");
	temp = temp->Next_point; 
	}
		strncpy(temp->longitude, Route_Parser->longitude, 10);
		strncpy(temp->latitude, Route_Parser->latitude, 10);
		temp->Next_point = NULL;
		UART_puts("point created");
		UART_puts("\r\n");
	return Route;
}









