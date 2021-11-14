/*
 * I2C.c
 *
 *  Created on: Jun 26, 2021
 *      Author: Talha SEVİNÇ
 */

#include"stm32f4xx_hal.h"

#define delayCount 5


class I2C
{

	public:

		I2C(GPIO_TypeDef *SDAPort,uint32_t SDAPin,GPIO_TypeDef *SCLPort,uint32_t SCLPin);
		void testOutput();
		void delay();
		void startCondition();
		void stopCondition();
		void repeatedStartCondition();
		bool waitACK(uint32_t timeOut);
		void changeSDAState(int selection);
		bool writeReadBitSet(uint8_t address,bool selection,int32_t timeOut);
		void resetIO();
		bool writeBytes(uint8_t value,int32_t timeOut);
		uint8_t readLine();
		void sendACK(int select);
		bool I2C_Write(uint8_t deviceAddress,uint8_t registerAddress,uint8_t *sendData,uint8_t dataNumber,uint32_t timeOut);
		bool I2C_Read (uint8_t deviceAddress,uint8_t registerAddress,uint8_t *receiveBuffer,uint8_t dataNumber,uint32_t timeOut);

	private:

		uint8_t i2cAddress;
		GPIO_TypeDef *SDA;
		GPIO_TypeDef *SCL;
		uint32_t SDAP;
		uint32_t SCLP;

};
