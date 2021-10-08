/*

		Author: TALHA SEVİNÇ

*/


#include "MPL3115A2.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"

extern int32_t timeoutSecond;
extern int32_t timeoutmiliSecond;

void i2cWrite(uint8_t Address, uint8_t Value)
{

	HAL_I2C_Mem_Write(&hi2c2, I2C_WRITE_REGISTER, Address, 1, &Value, 1, 100000);

}

bool Mpl3115A2_Init()
{

	if(HAL_I2C_IsDeviceReady(&hi2c2, I2C_WRITE_REGISTER, 1, 100000) !=HAL_OK){

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
		return false;

	}
	return true;

}



bool Mpl3115A2_Begin()
{
	 uint8_t whoami[1];
	 uint8_t readValue;
     uint8_t setValue;
	 HAL_I2C_Mem_Read(&hi2c2,I2C_READ_REGISTER ,MPL3115A2_WHOAMI,(uint16_t)1, whoami,(uint16_t)1, 100);

	 if (whoami[0] != 0xC4)
	 {

		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		 return false;

	 }

	 i2cWrite(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);

	 HAL_Delay(10);
	 HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1,1,&readValue, 1, 100);

	 while (readValue & MPL3115A2_CTRL_REG1_RST)
	 {
		 HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1,1,&readValue, 1, 100);
         HAL_Delay(10);

	 }

	 //setValue=MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
     _ctrl_reg1.reg=MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
	 i2cWrite(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

	 setValue= MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM;
     i2cWrite(MPL3115A2_PT_DATA_CFG, setValue);

     return true;
}


float MPL3115A2_ReadPressure(){

	uint32_t pressure;
	uint8_t readValue[1];
    uint8_t Status[1]={0};
	uint8_t readPressure[3];
	float realPressure=0;
	timeoutSecond=0;
	timeoutmiliSecond=0;

	HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1, 1 ,readValue , 1, 100);
	while( readValue[0] & MPL3115A2_CTRL_REG1_OST)
	{
		HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1, 1 ,readValue , 1, 100);
		HAL_Delay(10);
		if(timeoutSecond>1)
		{
			return 0.00;

		}
	}
	_ctrl_reg1.bit.ALT = 0;

	i2cWrite(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

	_ctrl_reg1.bit.OST = 1;
	i2cWrite(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

	while (!(Status[0] & MPL3115A2_REGISTER_STATUS_PDR))
	{
	    HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_REGISTER_STATUS, 1, &Status, 1, 100);
	    HAL_Delay(10);
	    if(timeoutSecond>1)
	    {
	    			return 0.00;

	    }
	  }

	//MPL3115A2_REGISTER_PRESSURE_MSB
	HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, 0x01, 1, readPressure, 3, 100000);

	pressure=readPressure[0];
	pressure=pressure<<8;
	pressure|=readPressure[1];
	pressure=pressure<<8;
	pressure|=readPressure[2];
    pressure=pressure>>4;

    realPressure=pressure;
    realPressure/=4;
    return realPressure;

}


float MPL3115A2_ReadTemperature(){

	uint8_t temp[2];
	uint8_t ms,ls;
	uint8_t Status = 0;
	uint32_t temperature;
	float realTemperature=0;
	timeoutSecond=0;
	timeoutmiliSecond=0;

	_ctrl_reg1.bit.OST = 1;

	i2cWrite(MPL3115A2_CTRL_REG1,  _ctrl_reg1.reg);

	while (!(Status & MPL3115A2_REGISTER_STATUS_TDR)) {

		HAL_I2C_Mem_Read(&hi2c2,I2C_READ_REGISTER,MPL3115A2_REGISTER_STATUS,1,&Status,1,100000);

	    HAL_Delay(10);

	    if(timeoutSecond>1)
	    {
	    	return 0.00;

	    }
	  }


	HAL_I2C_Mem_Read(&hi2c2,I2C_READ_REGISTER,0x04,1,temp,2,100000);

	ms=temp[0];
	ls=temp[1];

	temperature=((uint32_t) (ms<<8)) | ((uint32_t) (ls));
	temperature=temperature >>4;


	  if (temperature & 0x800)
	  {
	    temperature |= 0xF000;
	  }

	  realTemperature=(float)temperature;

	  realTemperature=(float)realTemperature/ 16.0;
	  return realTemperature;

}


float MPL3115A2_ReadAltitude()
{

	int32_t altitude;
	uint8_t read[1];
	uint8_t alt[3];

	float realAltitude=0;
	timeoutSecond=0;
	timeoutmiliSecond=0;

	HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1, 1, read, 1, 1000);

	while (read[0] & MPL3115A2_CTRL_REG1_OST)
	{
		HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_CTRL_REG1, 1, read, 1, 1000);
	    HAL_Delay(10);

	    if(timeoutSecond>1)
	    {
	    			return 0.00;


	    }
	}
	  _ctrl_reg1.bit.ALT = 1;

      i2cWrite(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

	  _ctrl_reg1.bit.OST = 1;

	  i2cWrite(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

	  uint8_t Status = 0;

	  while (!(Status & MPL3115A2_REGISTER_STATUS_PDR)) {

		HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_REGISTER_STATUS, 1, &Status, 1, 1000);

	    HAL_Delay(10);
	    if(timeoutSecond>1)
	    {
	    	return 0.00;


	    }
	  }

	  HAL_I2C_Mem_Read(&hi2c2, I2C_READ_REGISTER, MPL3115A2_REGISTER_PRESSURE_MSB, 1, alt, 3, 1000);


	  altitude = ((uint32_t) alt[0])<<24;
	  altitude |= ((uint32_t)alt[1])<<16;
	  altitude |= ((uint32_t) alt[2])<<8;

	  realAltitude=(float)altitude/(float)65536.0;

	  return realAltitude;

}

