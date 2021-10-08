This library is for XBee Pro 900 XSC to configure module settings and send/receive data via XBee.
For configuration, this function used.

![XBee-Pro_900_XSC_S3B_Wire](https://user-images.githubusercontent.com/34924065/136598218-7d90e0d4-734f-48aa-b903-c06e6ad69006.jpg)

XbeeInitFlag=XbeeInit("1071","AAAA", "BBBB", "5" , "40");

Module has UART interface.The parameters mean, respectively,
ID, SourceAddress, DestinationAddress, BaudRate, maxPacketSize

In order to communicate between two modules, the parameters must be set so that the source address and destination addresses have to be cross, and the modules must have the same ID.


