/*
Author: 	J.F. van der Bent
			W Pielage & E Helmond

Date:		24-11-2021
Revision:	5

    buzzer.c:
          HAL_ Buzzer-driver for ARM-board v5

    pin-info:
           PC8 - Buzzer

*/

#include "main.h"
#include "buzzer.h"

void BUZZER_init(void)
{
		// kept in place for downward compatibility
}

void BUZZER_set(int counter)
{
	while(counter--)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
}

void Buzzer_put(unsigned int counter)
{
	BUZZER_set(counter);
	HAL_Delay(150);
}

// Maak een piepje met meegegeven lengte
// toevoeging om compatibel te blijven met ARM v4.2
void BUZZER_put(unsigned int time)
{
    volatile unsigned int i;
    volatile unsigned int k=0;

	while (k++<100)
    {
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		for (i=0;i<(time+k);i++);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		for (i=0;(i<time+k);i++);
	}
}

