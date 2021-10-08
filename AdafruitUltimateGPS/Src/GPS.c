/*

     Author: TALHA SEVİNÇ

*/

#include "stm32f4xx_hal.h"
#include "GPS.h"

extern UART_HandleTypeDef huart2;
extern void Error_Handler(void);
extern char Hours[11];

int gprmc=0;
int cntGprMc=0;
int satelliteNum=0;

double longitude=0;
double latitude=0;
float altitudeValue=0.0;
//char hours[11];

char latit[20];
char lng[20];
char satellite[2];
char altitude[6];

bool valid=false;
bool findValue=false;


bool TakeGpsValue(void)
{
      valid=false;
	  int count=0;
	  bool gprmcbool=false;
	  cntGprMc=0;
      findValue=false;

	  while(!findValue)
	  {

	    for(int i=0;i<495;i++)
	    {
	     	if(GpsData[i]=='G' &&GpsData[i+1]=='P' &&GpsData[i+2]=='R' && GpsData[i+3]=='M' && GpsData[i+4]=='C')
	     	{

	     		gprmc=i+6;
	     		while(GpsData[i] !='\r')
	     		{

	     			i++;
	     			if(i==494)
	     			{
	     			  	return false;
	     			}

	     		}


	     		findValue=true;
	     		gprmcbool=true;
	     		break;

	     	}
         }

	     findValue=true;
	    }

	    count=gprmc;

	    if(gprmcbool)
	    {

	      while(GpsData[count] != '\r')

	      {

		     GprMc[cntGprMc]=GpsData[count];

		     ++count;
		     ++cntGprMc;

	       }

	     }

	     int commaCount=0;
	     int i=0;

	     while(i<cntGprMc-1)
	  	 {

	  		if(GprMc[i]==',')
	  		{
	  			commaCount++;
	          	i++;

	          	if(commaCount==1)
	          	{
	          		if(GprMc[i]=='A')
	          		{
	          			valid=true;
	          			break;

	          		}

	          		else if(GprMc[i]=='V')
	          		{

	          			valid=false;
	          			break;
	          		}
	          		++i;

	          	}
	  		}

	  		else
	  			i++;
	  	  }

	      return valid;

}

float GPSLatit()
{

	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<3)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;


            if(commaCount==2)
            {

            	while(GprMc[i] != ',')
				{
            		latit[cntLong]=GprMc[i];

            		cntLong++;

            		i++;

				}
    			commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong >= 8)   ////////////////////FOR NEO6MV2 This value must be 10...
    {

        float extra=0;

        latitude=(float)((latit[0]-48)*10 + (latit[1]-48));

    	latitude+=(float)((latit[2]-48)*10 + (latit[3]-48)/(float)60);
    	extra=(float)((latit[5]-48)*10+(latit[6]-48)+(float)((latit[7]-48)/10)+(float)((latit[8]-48)/100));
    	latitude+=(float)((float)extra/((float)6000));


    	founded=true;
    }

    if(founded)

    	return latitude;

    return 0.0;

}



float GPSLongitude()

{

	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<5)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;

            if(commaCount==4)
            {

            	while(GprMc[i] != ',')
				{

					lng[cntLong]=GprMc[i];
					cntLong++;
				    i++;

				}
    			commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong >= 7)
    {

    	longitude=(float)(((lng[0]-48)*100 + (lng[1]-48)*10 + (lng[2]-48)));
    	longitude+=(float)((float)((lng[3]-48)*10+(lng[4]-48))/(float)60);

    	longitude += (float)( (float)(((float)(lng[6]-48)/10)+(float)((float)(lng[7]-48)/(float)100))/(float)60);
    	founded=true;
    }

    if(founded)
    	return longitude;

    return 0.0;


}

void GpsHours(void)

{

    int i=0;
    int cntLong=0;

    while(GprMc[cntLong] != ',')
    {

        	if(GprMc[cntLong] != '.')
    		     hours[i]=GprMc[cntLong];

        	else
   		         hours[i]='0';


    		if(cntLong % 2 == 1 && cntLong != 7)
    		{
    			i++;
    			hours[i]=':';
    		}
    		i++;
    		cntLong++;

        }

        for(int i=0;i<11;i++)
         {

       	  Hours[i]=hours[i];

         }

        Hours[11]='\n';
 }





bool TakeGpsGPGGA(void)
{


    valid=false;
	int count=0;
	bool gprmcbool=false;
    cntGprMc=0;
    findValue=false;

	while(!findValue)
    {

	  for(int i=0;i<495;i++)
	  {
	     if(GpsData[i]=='G' &&GpsData[i+1]=='P' &&GpsData[i+2]=='G' && GpsData[i+3]=='G' && GpsData[i+4]=='A')
	     {
	     	gprmc=i+6;

	     	while(GpsData[i] !='\r')
	     	{
	     		i++;
	     		if(i==500)
	     		{
	     		  return false;
	     		}
	     	}

	     	findValue=true;
	     	gprmcbool=true;
	     	break;

	      }
       }

	   findValue=true;

	  }

	  count=gprmc;

	  if(gprmcbool)
	  {

		  while(GpsData[count] != '\r')
		  {

			  GprMc[cntGprMc]=GpsData[count];

			  ++count;
			  ++cntGprMc;

		  }
	  }


       if(GprMc[0] == ',')
       {

    	   valid=false;
       }

       else
    	   valid=true;


	      return valid;

}


float GPSLatitGGA(void)
{


	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<2)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;


            if(commaCount==1)
            {

            	while(GprMc[i] != ',')
				{
            		latit[cntLong]=GprMc[i];

            		cntLong++;

            		i++;

				}

    			commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong >= 8)
    {
        float extra=0;
    	latitude=(float)((latit[0]-48)*10 + (latit[1]-48));

    	latitude+=(float)((float)((latit[2]-48)*10 + (latit[3]-48))/(float)60);
    	extra=(float)((latit[5]-48)*10+(latit[6]-48)+(float)((float)(latit[7]-48)/(float)10)+(float)((float)(latit[8]-48)/(float)100));
    	latitude+=(float)((float)extra/((float)6000));


    	founded=true;
    }

    if(founded)
    	return latitude;

    return 0.0;



}

float GPSLongitudeGGA(void)
{

	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<4)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;

            if(commaCount==3)
            {

            	while(GprMc[i] != ',')
				{

					lng[cntLong]=GprMc[i];
					cntLong++;
				    i++;

				}
    			commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong >= 7)
    {


    	longitude=(float)(((lng[0]-48)*100 + (lng[1]-48)*10 + (lng[2]-48)));
    	longitude+=(float)((float)((lng[3]-48)*10+(lng[4]-48))/(float)60);

    	longitude += (float)( (float)(((float)(lng[6]-48)/10)+(float)((float)(lng[7]-48)/(float)100))/(float)60);



    	founded=true;
    }

    if(founded)
    	return longitude;

    return 0.0;

}


void GpsHoursGGA(void)
{

    int i=0;
    int cntLong=0;

        while(GprMc[cntLong] != ',')
    	{

        	if(GprMc[cntLong] != '.')
    		     hours[i]=GprMc[cntLong];

        	else
   		         hours[i]='0';


    		if(cntLong % 2 == 1 && cntLong != 7)
    		{
    			i++;
    			hours[i]=':';
    		}
    		i++;
    		cntLong++;

        }

        for(int i=0;i<11;i++)
         {

       	  Hours[i]=hours[i];

         }

        Hours[11]='\n';

}


int GpsSatellite(void)
{

	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<7)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;


            if(commaCount==6)
            {

            	while(GprMc[i] != ',')
				{
            		satellite[cntLong]=GprMc[i];

            		cntLong++;

            		i++;

				}
            	commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong == 2)
    {

    	satelliteNum=(int)((satellite[0]-48)*10 + (satellite[1]-48)) ;

    	founded=true;
    }

    if(founded)
    	return satelliteNum;

    return 0;


}

float GpsAltitude(void)
{


	int commaCount=0;
    int i=0;
    int cntLong=0;

    while(commaCount<9)
	{

		if(GprMc[i]==',')
		{
			commaCount++;
        	i++;


            if(commaCount==8)
            {

            	while(GprMc[i] != ',')
				{
            		altitude[cntLong]=GprMc[i];

            		cntLong++;

            		i++;

				}
            	commaCount++;


           }
		}

		else
		    i++;

    }

    bool founded=false;

    if(cntLong == 6)
    {

    	altitudeValue=(float)((altitude[0]-48)*1000 + (altitude[1]-48)*100 + (altitude[2]-48)*10 + (altitude[3]-48) +(float)(altitude[5]-48)/10)  ;
    	founded=true;
    }

    if(founded)
    	return altitudeValue;

    return 0;

}





