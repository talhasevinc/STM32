
**My test conditions are these:**

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
     
     
     # EXAMPLE:
     
     SEND:
     
![UartSend](https://user-images.githubusercontent.com/34924065/136582927-659c3d04-9e4b-4d55-a816-22394356abaf.JPG)


     
     RECEIVE:
     
 ![UartReceive115200](https://user-images.githubusercontent.com/34924065/136582985-4ef3754e-f358-4980-ba2b-44cbe09b9300.JPG)
 
 
    
![57600Baud](https://user-images.githubusercontent.com/34924065/136583027-20c60148-2063-414c-b192-4a7894470a0c.JPG)
     



     
