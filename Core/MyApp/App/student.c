/**
* @file student.c
* @brief Hier kunnen studenten hun eigen tasks aanmaken
*
* @author MSC

* @date 5/5/2022
*/
#include <admin.h>
#include "main.h"
#include "cmsis_os.h"


/**
* @brief Oefentask voor studenten
* @param argument, kan evt vanuit tasks gebruikt worden
* @return void
*/
void Student_task1 (void *argument)
{
	UART_puts((char *)__func__); UART_puts(" started\r\n");

	while(TRUE)
	{
       	osDelay(1000);
		

	}
}
