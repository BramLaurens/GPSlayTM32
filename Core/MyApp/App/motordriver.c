#include "main.h"

uint32_t dutyCycle = 0;
uint32_t freq = 1000;

void PWM_SIGNAL(uint32_t freq, uint32_t dutyCycle)
{
	uint32_t timer_clk = HAL_RCC_GetSysClockFreq();
	uint32_t prsc = htim9.Instance->PSC;
	uint32_t arr = (timer_clk / ((prsc + 1) * freq)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim9, arr);

	uint32_t duty = (arr + 1) * dutyCycle / 100;
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, duty);
}