# Module Features

This module has this specification:


  * -165 dBm sensitivity, 10 Hz updates, 66 channels <br/>
  * 5V friendly design and only 20mA current draw <br/>
  * Breadboard friendly + two mounting holes <br/>
  * RTC battery-compatible <br/>
  * Built-in datalogging <br/>
  * PPS output on fix <br/>
  * Internal patch antenna + u.FL connector for external active antenna <br/>
  * Fix status LED <br/>
  
![AdafruitUltimateGps](https://user-images.githubusercontent.com/34924065/136550701-1ba07bb2-5a91-4aed-8afe-f7e17c3fcd8d.jpg)

# Working Principle

The module communicates with the uart interface. NMEA sentences are received over the Uart and the information in the desired format is filtered. Position information is obtained from the obtained array. DMA feature of STM is used to receive data from the UART interface.The purpose of this operation is to prevent the processor from interrupting other work. After the necessary data transfer is completed, the 'GPSDataCame' flag is set and the process of controlling the location values begins.


