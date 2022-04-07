/*
 * Sofware_PWM.c
 *
 *  Created on: Aug 13, 2021
 *      Author: Talha SEVINC
 */

#include "Software_PWM.h"
#include "stm32f4xx_hal.h"

uint32_t time=0;
uint32_t micro=0;
uint32_t second=0;
uint32_t timerMax=0;
uint32_t Frequency=0;
bool start=0;
bool finish=0;

extern uint32_t ms;

void TIM2_IRQHandler(void)
{

	  time++;
	  if(time==10000)
	  {
		  time=0;
	  }

  HAL_TIM_IRQHandler(&htim2);

}

uint32_t PCLK1TIM()
{

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  if((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
  {
    return (pclk1);
  }
  else
  {
    return(2 * pclk1);
  }
}



void set_PWM_Frequency(uint16_t frequency)
{

  HAL_TIM_Base_Stop_IT(&htim2);

  uint16_t Prescelar=0;
  uint16_t period=0;
  uint32_t systemClockFreq=0;
  uint32_t timerFrequency=0;

  timerFrequency=PCLK1TIM();
  Prescelar= timerFrequency/( 2*frequency*100);
  timerMax =(systemClockFreq/(Prescelar));

  Frequency=frequency;

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = Prescelar-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }


}

void PWM_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  PWM=GPIOx;
	  PWM_Pin=GPIO_Pin;

	  if(GPIOx==GPIOA  )
		  __HAL_RCC_GPIOA_CLK_ENABLE();
	  else if(GPIOx==GPIOB )
		  __HAL_RCC_GPIOB_CLK_ENABLE();
	  else if(GPIOx==GPIOC  )
		  __HAL_RCC_GPIOC_CLK_ENABLE();
	  else if(GPIOx==GPIOD  )
		  __HAL_RCC_GPIOD_CLK_ENABLE();
	  else if(GPIOx==GPIOE   )
		  __HAL_RCC_GPIOE_CLK_ENABLE();

	  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

}


void set_PWM_Signal(uint8_t dutyCycle)
{
	 HAL_TIM_Base_Start_IT(&htim2);

	 for(int i=0;i<Frequency;i++)
	 {
			  time=0;
			  micro=0;
			  while(time<dutyCycle )
			  {
               if(!start)
               {
                 start=1;
				  HAL_GPIO_WritePin(PWM, PWM_Pin, SET);
               }

			  }
			  while(time<100)
			  {
				 if(!finish)
				 {
					 finish=1;
					 HAL_GPIO_WritePin(PWM, PWM_Pin, GPIO_PIN_RESET);
				 }
			  }
			  start=0;
			  finish=0;
	 }

    HAL_TIM_Base_Stop_IT(&htim2);


}

void servoMove(int degree)
{

	 HAL_TIM_Base_Start_IT(&htim2);

	 int value=(int)(25+(100*degree/180));
	 if(value>125)
		 value=125;
	 for(int i=0;i<50;i++)
	 {
			  time=0;
			  micro=0;
			  while(time<value )
			  {
                if(!start)
                {
                  start=1;
				  HAL_GPIO_WritePin(PWM, PWM_Pin, SET);
                }

			  }
			  while(time<1000)
			  {
				 if(!finish)
				 {
					 finish=1;
					 HAL_GPIO_WritePin(PWM, PWM_Pin, GPIO_PIN_RESET);
				 }
			  }
			  start=0;
			  finish=0;
	 }

     HAL_TIM_Base_Stop_IT(&htim2);


}
