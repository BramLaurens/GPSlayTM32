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


void Route_Setter(void *argument){
	GNRMC *Route_Parser;
	GPS_Route *Route;
	uint32_t key=0;
	while(1){
		key = xEventGroupWaitBits(hKEY_Event, 0xffff, pdTRUE, pdFALSE, portMAX_DELAY );
		UART_puts("\r\n");
		UART_putint(key);
			switch(key){
			case'1':
				UART_puts("\r\n");
				UART_puts("Key 1 pressed");
				break;


			default:
//				UART_puts("Current key:%i is not in use by GPS_Route_Setter.c",key);
				break;
			}
	}
}





//Void GPS_Route_Maker(char *Route){
//
//
//}









