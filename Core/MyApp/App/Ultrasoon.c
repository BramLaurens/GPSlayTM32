/**
@brief Function for Ultrasonic sensor
01/11/2025

(Reading the Echo pin in core/src/stm32f4xx_it.c)
 */
#include <Ultrasoon.h>
#include "cmsis_os.h"
#include "admin.h"

void Ultrasoon_trig(void) // 10us pulse for Trigger pin
{
	HAL_GPIO_WritePin(GPIOC, Trigger_Pin, GPIO_PIN_SET);
	int delay = 10;

	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while (__HAL_TIM_GET_COUNTER(&htim8)  < delay); // wait 10us

	HAL_GPIO_WritePin(GPIOC, Trigger_Pin, GPIO_PIN_RESET);
}


void Echo_sign_task(void *argument) // calculations for distance
{
	float Distance;
	int Echo_time = 0;
	char Buffer[70];

	while(TRUE)
	{
		Ultrasoon_trig();
		xEventGroupWaitBits(hEcho_Event, 1, pdTRUE, pdFALSE, HAL_MAX_DELAY);

		Echo_time = __HAL_TIM_GetCounter(&htim12); // amount of time receiving pulse
		Distance = Echo_time*0.034;

		sprintf(Buffer, "Afstand is: %.2f.", Distance);
		UART_puts(Buffer);
		UART_puts("\r\n");
		UART_putint(Echo_time);
		UART_puts("\r\n");
		osDelay(300);
	}
}
