/*
 * SoftI2C.h
 *
 *  Created on: Dec 22, 2021
 *      Author: Talha Sevinc
 */

#ifndef INC_SOFTI2C_H_
#define INC_SOFTI2C_H_

#include"stm32f4xx_hal.h"
#include "stdbool.h"

#define delayCount 9

uint8_t i2cAddress;
GPIO_TypeDef *SDA;
GPIO_TypeDef *SCL;
uint32_t SDAP;
uint32_t SCLP;


void I2C_Init(GPIO_TypeDef *SDAPort,uint32_t SDAPin,GPIO_TypeDef *SCLPort,uint32_t SCLPin);

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
HAL_StatusTypeDef I2C_DeviceReady(uint8_t deviceAddress);
bool I2C_Write(uint8_t deviceAddress,uint16_t registerAddress,uint8_t regLengt,uint8_t *sendData,uint8_t dataNumber,uint32_t timeOut);
bool I2C_Read (uint8_t deviceAddress,uint16_t registerAddress,uint8_t regLengt,uint8_t *receiveBuffer,uint8_t dataNumber,uint32_t timeOut);

#endif /* INC_SOFTI2C_H_ */
