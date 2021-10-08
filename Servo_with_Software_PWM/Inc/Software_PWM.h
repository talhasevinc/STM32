/*
 * Software_PWM.c
 *
 *  Created on: Aug 13, 2021
 *      Author: Talha SEVİNÇ
 */

#ifndef SOFTWARE_PWM_C_
#define SOFTWARE_PWM_C_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
extern TIM_HandleTypeDef htim2;
GPIO_TypeDef* PWM;
uint16_t PWM_Pin;

uint32_t PCLK1TIM(void) ;
void set_PWM_Frequency(uint16_t frequency);
void PWM_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void servoMove(int degree);
void set_PWM_Signal(uint8_t dutyCycle);

#endif /* SOFTWARE_PWM_C_ */
