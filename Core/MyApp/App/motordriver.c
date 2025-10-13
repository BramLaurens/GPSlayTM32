#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "motordriver.h"

#define PWM_FREQUENCY 100 // 100 Hz
#define MAX_DUTY_CYCLE 255
#define CLOCK_DIVISION 2 	// APB1 prescaler is set to 2 in SystemClock_Config

//temp
uint32_t speed = 0;

enum {BACKWARD, FORWARD} directionLeft = FORWARD, directionRight = FORWARD;

/**
 * @brief Function to initialize PWM signals relevant channels.
 * @note TIM12 CHANNEL_2 on pin PB15 (PWM A1)
 * @note TIM12 CHANNEL_1 on pin PB14 (PWM A2)
 * @note TIM4  CHANNEL_4 on pin PB9  (PWM B1)
 * @note TIM4  CHANNEL_1 on pin PB6  (PWM B2)
 */
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_1);  
	HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_4);	
}

/**
 * @brief Function to calculate and set frequency and duty cycle for PWM channel A1.
 * 
 * @param freq in Hz (usually 100 - 10000 Hz)
 * @param dutyCycle range 0 - 255
 * @param htim pointer to the timer handler (eg &htim12)
 * @param channel timer channel (eg TIM_CHANNEL_1)
 */
void PWM_Set(uint32_t freq, uint32_t dutyCycle, TIM_HandleTypeDef *htim, uint32_t channel)
{
	uint32_t timer_clk = HAL_RCC_GetSysClockFreq() / CLOCK_DIVISION; 	//Calculate timer clock by dividing system clock by 2 (APB1 prescaler)
	uint32_t prsc = htim->Instance->PSC; 								//Get prescaler value from TIM12
	uint32_t arr = (timer_clk / ((prsc + 1) * freq)) - 1;				//Calculate auto-reload register value to achieve desired frequency
	__HAL_TIM_SET_AUTORELOAD(htim, arr);								//Set the auto-reload register with the calculated value

	uint32_t duty = (arr + 1) * dutyCycle / MAX_DUTY_CYCLE;				//Calculate compare value to achieve desired duty cycle
	__HAL_TIM_SET_COMPARE(htim, channel, duty);							//Set the compare register with the calculated value
}

/**
 * @brief Function to set all motor driver inputs to HIGH, causing a brake.
 */
void Motor_Brake(void)
{
	PWM_Set(PWM_FREQUENCY, 0, &htim12, TIM_CHANNEL_1); // A2
	PWM_Set(PWM_FREQUENCY, 0, &htim12, TIM_CHANNEL_2); // A1
	PWM_Set(PWM_FREQUENCY, 0, &htim4,  TIM_CHANNEL_1); // B2
	PWM_Set(PWM_FREQUENCY, 0, &htim4,  TIM_CHANNEL_4); // B1
}

/**
 * @brief Function to set speed and direction of both motors.
 * 
 * @param speedLeft  range -255 - 255
 * @param speedRight range -255 - 255
 * @note Positive values indicate forward direction, negative values indicate backward direction.
 */
void Motor_Set_Speed(int16_t speedLeft, int16_t speedRight)
{	
	//Clamp speed values to be within the allowed range
	if (speedLeft  >  MAX_DUTY_CYCLE) speedLeft  =  MAX_DUTY_CYCLE;
	if (speedLeft  < -MAX_DUTY_CYCLE) speedLeft  = -MAX_DUTY_CYCLE;
	if (speedRight >  MAX_DUTY_CYCLE) speedRight =  MAX_DUTY_CYCLE;
	if (speedRight < -MAX_DUTY_CYCLE) speedRight = -MAX_DUTY_CYCLE;

	//Determine direction based on positive or negative speed values
	directionLeft  = (speedLeft  >= 0)  ? FORWARD : BACKWARD;
	directionRight = (speedRight >= 0)  ? FORWARD : BACKWARD;
	
	//Set correct PWM channels based on speed and direction
	if (directionLeft == FORWARD)
	{
		PWM_Set(PWM_FREQUENCY, abs(speedLeft), &htim12, TIM_CHANNEL_2); // A1
		PWM_Set(PWM_FREQUENCY, 0, 			   &htim12, TIM_CHANNEL_1); // A2
	}
	else
	{
		PWM_Set(PWM_FREQUENCY, 0,			   &htim12, TIM_CHANNEL_2); // A1
		PWM_Set(PWM_FREQUENCY, abs(speedLeft), &htim12, TIM_CHANNEL_1); // A2
	}

	if (directionRight == FORWARD)
	{
		PWM_Set(PWM_FREQUENCY, abs(speedRight), &htim4, TIM_CHANNEL_4); // B1
		PWM_Set(PWM_FREQUENCY, 0,	 			&htim4, TIM_CHANNEL_1); // B2
	}
	else
	{
		PWM_Set(PWM_FREQUENCY, 0,				&htim4, TIM_CHANNEL_4); // B1
		PWM_Set(PWM_FREQUENCY, abs(speedRight), &htim4, TIM_CHANNEL_1); // B2
	}
}


/**
 * @brief Task function to initialize motor driver and periodically update PWM signals.
 * 
 * @param argument 
 */
void Motor_Driver(void *argument)
{
	Motor_Init();
	PWM_Set(PWM_FREQUENCY, 0, &htim12, TIM_CHANNEL_1); // A1
	PWM_Set(PWM_FREQUENCY, 0, &htim12, TIM_CHANNEL_2); // A2
	PWM_Set(PWM_FREQUENCY, 0, &htim4,  TIM_CHANNEL_1); // B1
	PWM_Set(PWM_FREQUENCY, 0, &htim4,  TIM_CHANNEL_4); // B2	
	while(1)
	{
		osDelay(100); // adjust delay as needed
	}
}