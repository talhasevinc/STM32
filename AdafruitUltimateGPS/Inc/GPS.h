/*

     Author: TALHA SEVİNÇ

*/



#include "stm32f4xx_hal.h"
#include "stdbool.h"

#ifndef GPS_H_
#define GPS_H_

#define ArrayLenght 500

char GpsData[ArrayLenght];
char GprMc[100];
char hours[11];


void GPSPinInit(void);
void GPSUARTInit(void);

void WaitGps();
bool TakeGpsGprmc(void);
bool TakeGpsGPGGA(void);

float GPSLongitude(void);
float GPSLatit(void);
void GpsHours(void);


float GPSLongitudeGGA(void);
float GPSLatitGGA(void);
void GpsHoursGGA(void);
int GpsSatellite(void);
float GpsAltitude(void);



#endif

