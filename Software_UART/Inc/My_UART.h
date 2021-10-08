/*
 * My_UART.h
 *
 *  Created on: Aug 22, 2021
 *      Author: Talha SEVİNÇ
 */

#ifndef MY_UART_H_
#define MY_UART_H_


#include"stm32f4xx_hal.h"



class UART
{

	public:

		UART(GPIO_TypeDef *TxPort,uint32_t TxPinn,GPIO_TypeDef *RxPort,uint32_t RxPinn);
		void testOutput();
		void testTimer();
		void delay();
		void delay_start();
		void readDelay();
		void readDelay_start();
		void firstSampleDelay();
		void setBaudRate(uint32_t baudRate);
		void IDLE();
		void startCondition();
		void stopCondition();
		void sendData(uint8_t value);
		void Uart_Transmit(uint8_t *sendArray,uint32_t dataSize);
		void Uart_Receive(uint8_t *receiveArray,uint8_t receiveSize,uint16_t timeOut);
		uint8_t receiveByte();
		void resetIO();
		void setTimer(uint16_t frequency);
		uint8_t readLine();


	private:

		GPIO_TypeDef *Tx;
		GPIO_TypeDef *Rx;
		uint32_t TxPin;
		uint32_t RxPin;

		bool startBit;
		bool stopBit;

};




#endif /* MY_UART_H_ */
