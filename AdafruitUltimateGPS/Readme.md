This module has this specification:
  .-165 dBm sensitivity, 10 Hz updates, 66 channels
  .5V friendly design and only 20mA current draw
  .Breadboard friendly + two mounting holes
  .RTC battery-compatible
  .Built-in datalogging
  .PPS output on fix
  .Internal patch antenna + u.FL connector for external active antenna
  .Fix status LED
  
The module communicates with the uart interface. NMEA sentences are received over the Uart and the information in the desired format is filtered. Position information is obtained from the obtained array. DMA feature of STM is used to receive data from the UART interface.The purpose of this operation is to prevent the processor from interrupting other work. After the necessary data transfer is completed, the 'GPSDataCame' flag is set and the process of controlling the location values begins.


