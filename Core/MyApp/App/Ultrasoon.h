/** 
    * @file    Ultrasoon.h
    * @author  Anne Kamphuis
    * @version V1.0
    * @date    09-10-2025
    * @brief   Header file for Ultrasoon.c
*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#ifndef ULTRASOON_H
#define ULTRASOON_H

void Ultrasoon_trig(void);
void Echo_sign_task(void *argument);

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
#endif // ULTRASOON_H
