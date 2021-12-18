
# What is UART?
<br />UART stands for Universal Asynchronous Receiver/Transmitter. It’s not a communication protocol like SPI and I2C, but a physical circuit in a microcontroller, or a stand-alone IC. A UART’s main purpose is to transmit and receive serial data.

One of the best things about UART is that it only uses two wires to transmit data between devices. The principles behind UART are easy to understand.

# How is this library work?

<br />Firsty, you have define an object from UART class.
<br /> **UART myUart(GPIOC, GPIO_PIN_6,GPIOC, GPIO_PIN_7);**
<br />  Object name is myUart. And GPIOC Pin6 is Tx, GPIOC Pin7 Rx pins.
<br />  Then, you have to set Baud rate.
<br />  myUart.setBaudRate(57600);
<br /> In this example, Baud rate:57600

**SEND PROCESS:**
<br /> For sending data, use this function;
<br /> **myUart.Uart_Transmit(sendDatas,strlen(sendDatas));**
<br />  **-sendDatas:** Array will be sent
<br />  **-strlen(sendDatas):** How many bytes will be sent.
  
![UartSend](https://user-images.githubusercontent.com/34924065/136582927-659c3d04-9e4b-4d55-a816-22394356abaf.JPG)


     
 **RECEIVE PROCESS:**
 <br /> For receiving data, use this function;
 <br /> **myUart.Uart_Receive(receiveDatas,20,500);**
 <br /> **-receiveDatas: Array that will store datas.**
 <br /> **-20:How many bytes will receive.**
 <br /> **-500:Timeout**
     
 ![UartReceive115200](https://user-images.githubusercontent.com/34924065/136582985-4ef3754e-f358-4980-ba2b-44cbe09b9300.JPG)
 
 
    
![57600Baud](https://user-images.githubusercontent.com/34924065/136583027-20c60148-2063-414c-b192-4a7894470a0c.JPG)
     



     
