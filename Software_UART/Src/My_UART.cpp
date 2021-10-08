/*
 * My_UART.cpp
 *
 *  Created on: Aug 22, 2021
 *      Author: Talha SEVİNÇ
 */

/*


My test conditions are these:

    MicroController:STM32F407VGT6
	Clock Frequency:100 MHz

	Timer2 Basic Timer is on and fed by internal clock.
	Timer2 interrupt is enable but not used.

	Delay time counter adjusted to clock frequency and it is dinamic.
	You don't have to use Stm32 at 100 MHz. But This situations didn't tested.

	Cofficient values have to change according to speed of Uart.
	For example When Uart speed is in range of 0-57600, Optimal value for cofficient1
	and coefficient2 is 646. But in higher speed of uart, This value should be reduced.


	For Write Function:
		Baud Rate             Optimal Value(coefficient1-coefficient2)
		 9600	              			  646-646
		38400              			 	  636-636
		57600              			  	  628-615
		115200              			  602-578
        230400							  573-510
    For Read Function:

		Baud Rate             Optimal Value(coefficient1-coefficient2)
		 9600              			      646-646
		38400              			      636-636
		57600              			      628-615
		115200              			  590-586
		230400              			  ------


     NOTE: For 230400 Baud Rate, Transmit process tested succesfully, but no suitable values ​​
     found for receive process.
*/

#include "My_UART.h"
#include "stdbool.h"

extern TIM_HandleTypeDef htim2;
extern uint32_t seconds;
extern uint32_t miliseconds;

GPIO_InitTypeDef TxPinInit = {0};
GPIO_InitTypeDef RxPinInit = {0};


uint16_t cofficients[5][4]=
{
  {646,646,646,646},
  {636,636,636,636},
  {628,615,628,615},
  {602,578,590,586},
  {573,510,510,500}
};


uint32_t time=0;
uint32_t loopCounter=0;
uint32_t delayTime=624;
uint32_t delayTimeStart=0;
uint32_t ReaddelayTime=624;
uint32_t ReaddelayTimeStart=0;
uint32_t systemClockFrequency=0;



uint16_t WriteCoefficient1=573;  //For 0-57600 Baud Rate
uint16_t WriteCoefficient2=510;  //For 0-57600 Baud Rate


uint16_t ReadCoefficient1=560;  //For 0-57600 Baud Rate
uint16_t ReadCoefficient2=555;  //For 0-57600 Baud Rate


bool debug=0;



#define IDLEState           0
#define startConditionState 1
#define receiveData         2
#define stopConditionState  3

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




UART::UART(GPIO_TypeDef *TxPort,uint32_t TxPinn,GPIO_TypeDef *RxPort,uint32_t RxPinn)
:Tx(TxPort),Rx(RxPort),TxPin(TxPinn),RxPin(RxPinn),startBit(0),stopBit(1)
{
	  HAL_TIM_Base_Stop_IT(&htim2);
	  if(TxPort==GPIOA || RxPort==GPIOA  )
		  __HAL_RCC_GPIOA_CLK_ENABLE();
	  else if(TxPort==GPIOB || RxPort==GPIOB  )
		  __HAL_RCC_GPIOB_CLK_ENABLE();
	  else if(TxPort==GPIOC || RxPort==GPIOC  )
		  __HAL_RCC_GPIOC_CLK_ENABLE();
	  else if(TxPort==GPIOD || RxPort==GPIOD  )
		  __HAL_RCC_GPIOD_CLK_ENABLE();
	  else if(TxPort==GPIOE || RxPort==GPIOE  )
		  __HAL_RCC_GPIOE_CLK_ENABLE();



	  HAL_GPIO_WritePin(TxPort, TxPinn|RxPinn, GPIO_PIN_RESET);

	  TxPinInit.Pin = TxPinn;
	  TxPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	  TxPinInit.Pull = GPIO_NOPULL;
	  TxPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(TxPort, &TxPinInit);

	  HAL_GPIO_WritePin(TxPort, TxPinn, GPIO_PIN_SET);

	  RxPinInit.Pin =RxPinn;
	  RxPinInit.Mode = GPIO_MODE_INPUT;
	  RxPinInit.Pull = GPIO_NOPULL;
	  RxPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(RxPort, &RxPinInit);



}


void UART::setTimer(uint16_t frequency)
{

  HAL_TIM_Base_Stop_IT(&htim2);
  uint16_t Prescelar=0;
  uint32_t timerFrequency=0;
  timerFrequency=PCLK1TIM();
  Prescelar= timerFrequency/( 2*frequency);

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

  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {

  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {

  }


}



void UART::setBaudRate(uint32_t baudRate)
{
	uint8_t coffSelection=0;
	if(baudRate<=9600)
		coffSelection=0;
	else if(baudRate<=38400)
		coffSelection=1;
	else if(baudRate<=57600)
		coffSelection=2;
	else if(baudRate<=115200)
		coffSelection=3;
	else if(baudRate<=230400)
		coffSelection=4;

	WriteCoefficient1=cofficients[coffSelection][0];
	WriteCoefficient2=cofficients[coffSelection][1];
	ReadCoefficient1 =cofficients[coffSelection][2];
	ReadCoefficient2 =cofficients[coffSelection][3];

	systemClockFrequency= HAL_RCC_GetSysClockFreq();

    delayTime=     ( systemClockFrequency/100000000)*WriteCoefficient1 /(baudRate/9600);
    delayTimeStart=( systemClockFrequency/100000000)*WriteCoefficient2 /(baudRate/9600);

    ReaddelayTime     = ( systemClockFrequency/100000000)*ReadCoefficient1 /(baudRate/9600);
    ReaddelayTimeStart= ( systemClockFrequency/100000000)*ReadCoefficient2 /(baudRate/9600);
    debug=1;
}




void UART::testOutput()
{
    Rx->BSRR = RxPin  ;
    Tx->BSRR = TxPin  ;

	HAL_Delay(1000);

	Tx->BSRR = (uint32_t)TxPin << 16U ;
	Rx->BSRR = (uint32_t)RxPin << 16U ;

}

void UART::testTimer()
{
   miliseconds=0;
   while(miliseconds<100)
   {
	   loopCounter++;
   }

}

void UART::delay()
{
   uint32_t delayCounter=0;
   while(delayCounter<delayTime)
   {
	   delayCounter++;

   }

}

void UART::delay_start()
{
   uint32_t delayCounter=0;
   while(delayCounter<delayTimeStart)
   {
	   delayCounter++;

   }

}

void UART::readDelay()
{
   uint32_t delayCounter=0;
   while(delayCounter<ReaddelayTime)
   {
	   delayCounter++;

   }

}

void UART::readDelay_start()
{
   uint32_t delayCounter=0;
   while(delayCounter<ReaddelayTimeStart)
   {
	   delayCounter++;

   }

}


void UART::firstSampleDelay()
{
   uint32_t delayCounter=0;
   uint32_t delayTimeFirst=ReaddelayTimeStart/2;

   while(delayCounter<delayTimeFirst)
   {
	   delayCounter++;

   }

}
void UART::IDLE()
{
    Tx->BSRR = TxPin  ;
}

void UART::startCondition()
{
	Tx->BSRR = (uint32_t)TxPin << 16U ;
	delay_start();
}

void UART::stopCondition()
{
    Tx->BSRR = TxPin  ;
    delay();
}

void UART::sendData(uint8_t value)
{
     for(int i=0;i<8;i++)
     {
    	 if( (value>>i) & 0x01  )
    		 Tx->BSRR = TxPin  ;
    	 else
    		 Tx->BSRR = (uint32_t)TxPin << 16U ;

    	 delay();



     }
}

void UART::Uart_Transmit(uint8_t *sendArray,uint32_t dataSize)
{
	for(int i=0;i<dataSize;i++)
	{
		startCondition();
		sendData(sendArray[i]);
		stopCondition();

	}
	IDLE();
}

uint8_t UART::receiveByte()
{
	 uint8_t data=0;
	 delay();
     for(int i=0;i<8;i++)
     {
    	 data |= (HAL_GPIO_ReadPin(Rx, RxPin)<<7);

    	 if(i!=7)
    	 {	 data=data>>1; readDelay();}

     }




     return data;

}
bool UART::Uart_Receive(uint8_t *receiveArray,uint8_t receiveSize,uint16_t timeOut)
{
	 miliseconds=0;
	 uint8_t state=0;
	 uint8_t count=0;
	 bool startCame=0;
	 bool findStartCondition=0;
	 bool dataReceived=false;

	 while(miliseconds<timeOut)
	 {
		 if( (Rx->IDR & RxPin) && state == IDLEState  )
			 state=IDLEState;
		 else if( !findStartCondition )
		 {
			 state=startConditionState;

		 }

		 switch(state)
		 {
		 	 case startConditionState:
				 dataReceived=true;
		 		 firstSampleDelay();
		 		 if(HAL_GPIO_ReadPin(Rx, RxPin) == startBit)
		 		 { state=receiveData; findStartCondition=1; startCame=1;}
		 		 else
		 			 state=IDLEState;

		 		 break;

		 	 case receiveData:
		 		receiveArray[count++]=receiveByte();
		 		if(count >=receiveSize)
		 			miliseconds=timeOut;
		 		state=stopConditionState;
		 		break;

		 	 case stopConditionState:
		 		 delay();
		 		 if(! (HAL_GPIO_ReadPin(Rx, RxPin) == stopBit) )
		 			receiveArray[count-1] = 0;
		 		 state=IDLEState;
		 		 startCame = 0;
		 	     findStartCondition=0;
		 		 break;

		 }

	 }
	 debug=1;
	 return dataReceived;



}

