/**
@brief Function for Ultrasonic sensor
01/11/2025

(Reading the Echo pin in core/src/stm32f4xx_it.c)
 */
#include <Ultrasoon.h>
#include "cmsis_os.h"
#include "admin.h"

#define ultrasoon_debug

float ObjectDistance;

/**
  * @brief  Returns distance value
  * @param  None
  * @retval Distance value in cm
  */
double Afstand() // returns distance value with Afstand function
{
	return ObjectDistance;
}

/**
  * @brief  Generates a 10us pulse on Trigger pin
  * @param  None
  * @retval None
  */

void Ultrasoon_trig(void) // 10us pulse for Trigger pin
{
	HAL_GPIO_WritePin(GPIOC, Trigger_Pin, GPIO_PIN_SET);
	int delay = 10;

	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while (__HAL_TIM_GET_COUNTER(&htim8)  < delay); // wait 10us

	HAL_GPIO_WritePin(GPIOC, Trigger_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Task for calculating distance (cm) based on Echo time
  * @param  argument: Not used
  * @retval None
  */
void Echo_sign_task(void *argument) // calculations for distance
{

	int Echo_time = 0;
	char Buffer[70];
	osDelay(1000); // wait for system to stabilize
	UART_puts("Echo task started \r\n");

	while(TRUE)
	{
		Ultrasoon_trig();
		xEventGroupWaitBits(hEcho_Event, 1, pdTRUE, pdFALSE, HAL_MAX_DELAY);

		Echo_time = __HAL_TIM_GetCounter(&htim9); // amount of time receiving pulse
		ObjectDistance = Echo_time*0.034;

		#ifdef ultrasoon_debug
			sprintf(Buffer, "Afstand is: %.2f.", ObjectDistance);
			UART_puts(Buffer);
			UART_puts("\r\n");
			UART_putint(Echo_time);
			UART_puts("\r\n");
		#endif
		osDelay(1000);
	}
}
