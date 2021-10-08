/*
* XbeeInit.c
*
*  Created on: 14 May 2021
*      Author: TALHA SEVINC
*/
/*
*/

#include "Xbee_Init.h"

void changeUartBaud(uint32_t BaudRate)
{


	  huart4.Instance = UART4;
	  huart4.Init.BaudRate = BaudRate;
	  huart4.Init.WordLength = UART_WORDLENGTH_8B;
	  huart4.Init.StopBits = UART_STOPBITS_1;
	  huart4.Init.Parity = UART_PARITY_NONE;
	  huart4.Init.Mode = UART_MODE_TX_RX;
	  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart4.Init.OverSampling = UART_OVERSAMPLING_16;

	  if (HAL_UART_Init(&huart4) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

bool waitResponse()
{
     memset(recBuffer,'\0',10);

     HAL_UART_Receive(&huart4, (uint8_t *)recBuffer,10, 100);
     for(int i=0;i<9;i++)
     {
		  if(recBuffer[i]=='O' && recBuffer[i+1]=='K')
		  {
			  HAL_Delay(100);
          	  return 1;
		  }
     }

     return 0;

}

bool AtModeactive()
{
  sprintf(Buffer,"+++");
  HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 3, 400);
  memset(Buffer,'\0',20);
  return waitResponse();
}

bool setID(char *ID)
{
      sprintf(Buffer,"ATID%s\r",ID);
      HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
      memset(Buffer,'\0',20);
      return waitResponse();

}
bool  setDestinationAddress(char *DA)
{
   sprintf(Buffer,"ATDT%s\r",DA);
   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
   memset(Buffer,'\0',20);
   return waitResponse();


}


bool  setSourceAddress(char *DA)
{
   sprintf(Buffer,"ATMY%s\r",DA);
   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
   memset(Buffer,'\0',20);
   return waitResponse();


}
bool setRFMode(char *mode)
{
	   sprintf(Buffer,"ATMY%s\r",mode);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 6, 100);
	   memset(Buffer,'\0',20);
	   return waitResponse();

}

bool setStreamingLimit(char *stLimit)
{
	   sprintf(Buffer,"ATTT%s\r",stLimit);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 7, 100);
	   memset(Buffer,'\0',20);
	   return waitResponse();
}

/*
 0 = 1200 bps
 1 = 2400
 2 = 4800
 3 = 9600
 4 = 19200
 5 = 38400
 6 = 57600
 7 = 115200
 */
bool setBaudRate(char *baudRate)
{
	   sprintf(Buffer,"ATBD%s\r",baudRate);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 6, 100);
	   memset(Buffer,'\0',20);
	   if( waitResponse())
	   {
		   uint32_t baud=0;

		   if(baudRate[0]=='0')
			   baud=1200;
		   else if(baudRate[0]=='1')
			   baud=2400;
		   else if(baudRate[0]=='2')
			   baud=4800;
		   else if(baudRate[0]=='3')
			   baud=9600;
		   else if(baudRate[0]=='4')
			   baud=19200;
		   else if(baudRate[0]=='5')
			   baud=38400;
		   else if(baudRate[0]=='6')
			   baud=57600;
		   else if(baudRate[0]=='7')
			   baud=115200;
		   else
			   baud=38400;

		   changeUartBaud(baud);
		   return 1;

	   }

	   return 0;
}

bool setPacketSize(char *packetSize)
{
	   sprintf(Buffer,"ATPK%s\r",packetSize);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 7, 100);
	   memset(Buffer,'\0',20);
	   return waitResponse();
}

bool setGuardTime(char *GuartTime)
{
	   sprintf(Buffer,"ATBT%s\r",GuartTime);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
	   memset(Buffer,'\0',20);
	   return waitResponse();
}

bool setAtTimeout(char *TimeOut)
{
	   sprintf(Buffer,"ATCT%s\r",TimeOut);
	   HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 9, 100);
	   memset(Buffer,'\0',20);
	   return waitResponse();

}

bool endAtMode()
{

		 sprintf(Buffer,"ATWR\r");
		 HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
		 memset(Buffer,'\0',20);
		 if(!waitResponse())
			 return 0;

		 sprintf(Buffer,"ATAC\r");
		 HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
		 memset(Buffer,'\0',20);

		 HAL_Delay(1000);

		 sprintf(Buffer,"ATCN\r");
		 HAL_UART_Transmit(&huart4, (uint8_t *)Buffer, 5, 100);
		 memset(Buffer,'\0',20);
		 if(!waitResponse())
			 return 0;


         return 1;
}

bool XbeeInit(char *ID,char *SourceAddress,char *DestinationAddress,char *BaudRate,char *maxPacketSize)
{
  HAL_Delay(100);
  if(!AtModeactive())
	  return 0;
  if(!setID(ID))
	  return 0;
  if(!setDestinationAddress(DestinationAddress))
	  return 0;
  if(!setSourceAddress(SourceAddress))
	  return 0;
  if(!setBaudRate(BaudRate))
	  return 0;
  if(!setPacketSize(maxPacketSize))
	  return 0;
  if(!endAtMode())
	  return 0;

  HAL_Delay(100);
  return 1;
}

