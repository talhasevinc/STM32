/*
 * I2C.c
 *
 *  Created on: Jun 26, 2021
 *      Author: Talha SEVİNÇ
 */
#include "I2C.h"

GPIO_InitTypeDef SDAPinInit = {0};
GPIO_InitTypeDef SCLPinInit = {0};
GPIO_InitTypeDef SDAPinInput = {0};


I2C::I2C(GPIO_TypeDef *SDAPort,uint32_t SDAPin,GPIO_TypeDef *SCLPort,uint32_t SCLPin)
	:SDA(SDAPort),SCL(SCLPort),SDAP(SDAPin),SCLP(SCLPin)
{

	  if(SDAPort==GPIOA || SCLPort==GPIOA  )
		  __HAL_RCC_GPIOA_CLK_ENABLE();
	  else if(SDAPort==GPIOB || SCLPort==GPIOB  )
		  __HAL_RCC_GPIOB_CLK_ENABLE();
	  else if(SDAPort==GPIOC || SCLPort==GPIOC  )
		  __HAL_RCC_GPIOC_CLK_ENABLE();
	  else if(SDAPort==GPIOD || SCLPort==GPIOD  )
		  __HAL_RCC_GPIOD_CLK_ENABLE();
	  else if(SDAPort==GPIOE || SCLPort==GPIOE  )
		  __HAL_RCC_GPIOE_CLK_ENABLE();



	  HAL_GPIO_WritePin(SDA, SDAPin|SCLPin, GPIO_PIN_RESET);

	  SDAPinInit.Pin = SDAPin;
	  SDAPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	  SDAPinInit.Pull = GPIO_NOPULL;
	  SDAPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(SDA, &SDAPinInit);
	  HAL_GPIO_WritePin(SDA, SDAP, GPIO_PIN_SET);

	  SDAPinInput.Pin  = SDAPin;
	  SDAPinInput.Mode = GPIO_MODE_INPUT;
	  SDAPinInput.Pull = GPIO_NOPULL;

	  SCLPinInit.Pin =SCLPin;
	  SCLPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	  SCLPinInit.Pull = GPIO_NOPULL;
	  SCLPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(SCL, &SCLPinInit);

	  HAL_GPIO_WritePin(SCL, SCLPin, GPIO_PIN_SET);


}

void I2C::testOutput()
{
    SCL->BSRR = SCLP  ;
	SDA->BSRR = SDAP  ;

	HAL_Delay(1000);

	SDA->BSRR = (uint32_t)SDAP << 16U ;
	SCL->BSRR = (uint32_t)SCLP << 16U ;

}

void I2C::resetIO()
{
    changeSDAState(1) ;
    SCL->BSRR = SCLP  ;
	SDA->BSRR = SDAP  ;

}

void I2C::delay()
{
   for(int i=0;i<delayCount;i++);
}


void I2C::startCondition()
{


	SDA->BSRR = (uint32_t)SDAP << 16U ;
	delay();
	SCL->BSRR = (uint32_t)SCLP << 16U ;


}

void I2C::stopCondition()
{

	changeSDAState(1);
	SCL->BSRR = SCLP;
	delay();
	SDA->BSRR = SDAP;
	delay();

}


void I2C::repeatedStartCondition()
{
	changeSDAState(1);
	SDA->BSRR = SDAP;
	delay();
	SCL->BSRR = SCLP;
	delay();
	SDA->BSRR = (uint32_t)SDAP << 16U ;

}


void I2C::changeSDAState(int selection)
{


	if(selection==0)
	{
		SDAPinInit.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(SDA, &SDAPinInit);
	}
	else if(selection==1)
	{
		SCL->BSRR = (uint32_t)SCLP << 16U ;
		SDAPinInit.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(SDA, &SDAPinInit);
		SDA->BSRR = (uint32_t)SDAP << 16U ;
		SCL->BSRR = (uint32_t)SCLP << 16U ;


	}

	for(int i=0;i<5;i++)
		delay();

}



bool I2C::waitACK(uint32_t timeOut)
{
   changeSDAState(0);

   int time=HAL_GetTick();
   bool ACK=0;

	   SCL->BSRR = SCLP;
	   delay();
	   if(!(HAL_GPIO_ReadPin(SDA, SDAP)))
	   {
		   ACK=1;
		   changeSDAState(1);

	   }

	   SCL->BSRR = (uint32_t)SCLP << 16U ;
	   delay();

   return ACK;
}

bool I2C::writeReadBitSet(uint8_t address,bool selection,int32_t timeOut)
{
	bool successfullACK=0;
	int writeAddress=0;
	if(selection)
	    writeAddress=(address<<1) & 0xFE;
	else
		writeAddress=(address<<1) | 0x01;

	startCondition();

	for(int i=7;i>=0;i--)
	{
		if( (writeAddress>>i)&0x01)
			SDA->BSRR = SDAP;
		else
			SDA->BSRR = (uint32_t)SDAP << 16U ;


		delay();
		SCL->BSRR = SCLP;
		delay();
		SCL->BSRR = (uint32_t)SCLP << 16U ;


	}

	if(waitACK(timeOut))
		successfullACK=1;
    if(selection == 1)
    	changeSDAState(1);
	return successfullACK;

}

bool I2C::writeBytes(uint8_t value,int32_t timeOut)
{
	bool successfullACK=0;

	SCL->BSRR = (uint32_t)SCLP << 16U ;

	for(int i=7;i>=0;i--)
	{
		if( (value>>i)&0x01)
			SDA->BSRR = SDAP;
		else
			SDA->BSRR = (uint32_t)SDAP << 16U ;

		delay();

		SCL->BSRR = SCLP;
		delay();
		SCL->BSRR = (uint32_t)SCLP << 16U ;

	}

	if(waitACK(timeOut))
		successfullACK=1;

	//changeSDAState(1);
	return successfullACK;
}

uint8_t I2C::readLine()
{
	 changeSDAState(0);
	 uint8_t data=0;
	 uint8_t counter=0;
	 bool ACK=0;
	 while(1)
	 {

		 SCL->BSRR = SCLP;
         delay();
		 data |= HAL_GPIO_ReadPin(SDA, SDAP);
		 counter++;

		 SCL->BSRR = (uint32_t)SCLP << 16U ;
		 delay();


		 if(counter==8)
			 return data;

         data=data<<1;



	 }



}

void I2C::sendACK(int select)
{
	 changeSDAState(1);

	 if(select == 1)
		 SDA->BSRR = (uint32_t)SDAP << 16U ;
	 else
		 SDA->BSRR = SDAP  ;

     delay();

     SCL->BSRR = SCLP;

	 delay();

	 SCL->BSRR = (uint32_t)SCLP << 16U ;


}
bool I2C::I2C_Write(uint8_t deviceAddress,uint8_t registerAddress,uint8_t *sendData,uint8_t dataNumber,uint32_t timeOut)
{

	if(!writeReadBitSet(deviceAddress,1,500))
	{
		resetIO();
		return false;
	}

	if(!writeBytes(registerAddress,timeOut))
	{
		resetIO();
		return false;
	}

	for(int i=0;i<dataNumber;i++)
	{
		if(!writeBytes(sendData[i],timeOut))
		{
			resetIO();
			return false;
		}

	}

	stopCondition();
	return true;

}

bool I2C::I2C_Read (uint8_t deviceAddress,uint8_t registerAddress,uint8_t *receiveBuffer,uint8_t dataNumber,uint32_t timeOut)
{

	if(!writeReadBitSet(deviceAddress,1,500))
	{
		resetIO();
		return false;
	}

	if(!writeBytes(registerAddress,timeOut))
	{
		resetIO();
		return false;
	}

	//stopCondition();
	repeatedStartCondition();

	if(!writeReadBitSet(deviceAddress,0,500))
	{
			resetIO();
			return false;
	}

    for(int i=0;i<dataNumber;i++)
    {
    	receiveBuffer[i]=readLine();
    	if(i != dataNumber-1)
    	{
    		sendACK(1);
    		changeSDAState(0);
    	}

    }

    sendACK(0);
	stopCondition();
	int debug=0;



}


