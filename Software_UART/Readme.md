
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
     
     
     EXAMPLE:
     
     SEND:
     
     ![UartSend_550x350](https://user-images.githubusercontent.com/34924065/136581845-a91890b3-8026-405d-91a7-41611d7aa212.jpg)
     
     RECEIVE:
     
     ![UartReceive_550x350](https://user-images.githubusercontent.com/34924065/136581883-c85985eb-9c5b-4ab9-935b-e78ceb1050dc.jpg)


     
