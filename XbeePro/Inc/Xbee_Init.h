/*
 * Xbee_Init.h
 *
 *  Created on: 14 May 2021
 *      Author: TALHA
 */

#ifndef XBEE_INIT_H_
#define XBEE_INIT_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
extern UART_HandleTypeDef huart4;
char Buffer[20];
char recBuffer[10];

bool waitResponse();
bool AtModeactive();
bool setID(char *ID);
bool setDestinationAddress(char *DA);
bool setSourceAddress(char *DA);
bool setRFMode(char *mode);
bool setStreamingLimit(char *stLimit);
bool setBaudRate(char *baudRate);
bool setPacketSize(char *packetSize);
bool setGuardTime(char *GuartTime);
bool setAtTimeout(char *TimeOut);
bool endAtMode();
bool XbeeInit(char *ID,char *SourceAddress,char *DestinationAddress,char *BaudRate,char *maxPacketSize);

#endif /* XBEE_INIT_H_ */
