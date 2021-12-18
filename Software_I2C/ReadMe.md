## What is I2C? 
The Inter-Integrated Circuit (I2C) Protocol is a protocol intended to allow multiple "peripheral" digital integrated circuits ("chips") to communicate with one or more "controller" chips. Like the Serial Peripheral Interface (SPI), it is only intended for short distance communications within a single device. Like Asynchronous Serial Interfaces (such as RS-232 or UARTs), it only requires two signal wires to exchange information.

## What is purpose of this project?
Certain pins can be used as hardware for i2c on stm. However, in some cases, those pins may be full or the i2c pins may be incorrectly selected on a designed card. In such cases, the desired pins can be selected as SDA and SCL pins through this library, and communication processing, data reading or writing can be performed with any module.

## How is it working

Usage of library is very simple. Firstly, you have to declare SDA and SCL ports and pins.

**I2C myI2C(GPIOB,GPIO_PIN_11,GPIOB,GPIO_PIN_10);**   // PinB11 ==>SDA , PinB10 ===>SCL

For above example, firstly define an object from I2C class. Its name is myI2C. Then sets required pins. GPIOB port Pin11 is used for SDA , and GPIOB Pin10 is used for SCL bus.

For sending data, you can use this function;

***myI2C.I2C_Write(0x60, 0x26, (uint8_t *)sendingData, 3, 1000);***

The parameter is that;

I2C_Write(uint8_t deviceAddress,uint8_t registerAddress,uint8_t *sendData,uint8_t dataNumber,uint32_t timeOut)

<br />**deviceAddress:** 7-bit module address. Last bit is setted by library according to write/read process.
<br />**registerAddress:** desired register value of module to write.
<br />**sendData:**  Array sending.
<br />**dataNumber:** How many bytes sending?
<br />**timeOut:** If module return NACK, how many times function try to send datas?

For receiving data, you can use this function;

***myI2C.I2C_Read(0x60, 0x26, (uint8_t *)sendingData, 3, 1000);***

Parameters are same as write function.



