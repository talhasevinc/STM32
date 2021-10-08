/*


     AUTHOR: TALHA SEVİNÇ

 */


#include "MPU6050.h"
#include "stm32f4xx_hal.h"
/******************************************************/
/* This variables holds previous degree values*/
uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

double KalmanAngleX;
double KalmanAngleY;
/******************************************************/

float Calib_X,Calib_Y,Calib_Z;           //Calibration value for X-Y-Z Axis
float calibrationGyro[3]={0.0,0.0,0.0};  //0 ===>X  1====>Y   2====>Z
float sampleTime=0.005;

uint16_t XRaw,YRaw,ZRaw;                 //Acceleration Raw Data
float XRawReal,YRawReal,ZRawReal;        //Acceleration Calibrated Value

uint16_t G_XRaw,G_YRaw,G_ZRaw;           //Gyro Raw Data
float G_X,G_Y,G_Z;                       //Gyro calibrated Value



/*Temperature */
uint8_t temp[2];
float temperature=0.0;

/*Calibration Flags*/
bool GyroSet=0;
bool calibration=false;
float ROLL=0.0;
float PITCH=0.00;


/******************************************************/
/*They are not used.*/
float ap_gyro=0;
float ar_gyro=0;
float ap_acc, ar_acc,az_acc;
float angle_Pitch,angle_Roll;   //Pitch and Roll
float total_Acc=0;
float C_X,C_Y,C_Z;
/******************************************************/


extern uint32_t SystickMs;


void i2cWrite(uint8_t Address, uint8_t Value)
{

	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, Address, 1, &Value, 1, 100);

}



bool MPU6050Init()
{
    uint8_t control=0;
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &control, 1, 100);
	if(control==0x68)
	 return true;
	else
	 return false;
}

void MPU6050Initialization()
{
     i2cWrite(PWR_MGMT_1,0x00);   //Sensor Wake-Up
     i2cWrite(SMPLRT_DIV,0x07);
     i2cWrite(GYRO_CONFIG,0x00);  //Gyro ==>250 degree/sc  /131
     i2cWrite(ACCEL_CONFIG,0x00); //ACC==> -+2 degree/sc   /16384

}

void setSampleTime(uint32_t ms)
{
    sampleTime= (float )ms/(float)1000.00;
}


/*This function are used to read raw acceleration datas and convert real datas...*/
void getAccelerationRawData()
{
     uint8_t RawDatas[6];
     HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, (uint16_t) 1, RawDatas, (uint16_t) 3 , 100);//3----6

     XRaw=(RawDatas[0]<<8) | RawDatas[1];
     YRaw=(RawDatas[2]<<8) | RawDatas[3];
     ZRaw=(RawDatas[4]<<8) | RawDatas[5];

     XRawReal=(float)XRaw/16384.00;
     YRawReal=(float)YRaw/16384.00;
     ZRawReal=(float)ZRaw/16384.00;




}

/*This function are used to read raw gyro datas and convert real datas...*/
void getGyroRawData()
{
    uint8_t RawDatas[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, (uint16_t) 1, RawDatas, (uint16_t) 6 , 100);

    G_XRaw=(RawDatas[0]<<8) | RawDatas[1];
    G_YRaw=(RawDatas[2]<<8) | RawDatas[3];
    G_ZRaw=(RawDatas[4]<<8) | RawDatas[5];
    if(calibration)
    {
		G_XRaw-=calibrationGyro[0];
		G_YRaw-=calibrationGyro[1];
		G_ZRaw-=calibrationGyro[2];
    }

	G_X=(float) G_XRaw/ (float)131.0;
	G_Y=(float) G_YRaw/ (float)131.0;
	G_Z=(float) G_ZRaw/ (float)131.0;
}

/*Read all datas and convert them to real values. Then, find degree*/

void getAllDatas()
{
	getAccelerationRawData();
	getGyroRawData();

	double dt = (double) (HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	double roll;
	double roll_sqrt = sqrt(XRaw *XRaw + ZRaw * ZRaw);
	roll = atan(YRaw / roll_sqrt) * RAD_TO_DEG;
	if (roll_sqrt != 0.0) {}
	else { roll = 0.0; }

    double pitch = atan2(-XRaw, ZRaw) * RAD_TO_DEG;


	if ((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90))
	{
	        KalmanY.angle = pitch;
	        KalmanAngleY = pitch;
	}
	else
	{
	        KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, YRawReal, dt);
	}
	if (fabs(KalmanAngleY) > 90)
	        XRawReal = -XRawReal;

	KalmanAngleX = Kalman_getAngle(&KalmanX, roll, YRawReal, dt);

    ROLL=roll;
    PITCH=pitch;
}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

float getRoll()
{

    return ROLL;
}

float getPitch()
{
    return PITCH;
}


float getTemperature()
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_REG, (uint16_t) 1, temp, (uint16_t) 2 , 100);
	temperature=(float) ( (temp[1]<<8 |temp[1])/340.0 + 36.53);
	return temperature;
}


void MPU6050Calibration(int time)
{

     sampleTime=(float) time/1000;
     for(int i=0;i<50;i++)
     {
    	 __HAL_TIM_SET_COUNTER(&htim1,0);
    	 getGyroRawData();
    	 calibrationGyro[0]+=G_XRaw;
    	 calibrationGyro[1]+=G_YRaw;
    	 calibrationGyro[2]+=G_ZRaw;

    	 while(__HAL_TIM_GET_COUNTER(&htim1)<4000);
     }

     for(int i=0;i<3;i++) calibrationGyro[i]/=50.0;
     calibration=1;
     Calib_X=calibrationGyro[0];
     Calib_Y=calibrationGyro[1];
     Calib_Z=calibrationGyro[2];



}



void GetGyroValue()
{
	getGyroRawData();

    ap_gyro+=(G_XRaw*0.0000611);
    ar_gyro+=(G_YRaw*0.0000611);

    ap_gyro+=ar_gyro*sin(G_ZRaw * (sampleTime*0.000001066));
    ar_gyro-=ap_gyro*sin(G_ZRaw * (sampleTime*0.000001066));
}



void getACCAngle()
{

	getAccelerationRawData();
	GetGyroValue();

	total_Acc=sqrt(XRawReal*XRawReal+YRawReal*YRawReal+ZRawReal*ZRawReal);
	ap_acc=asin((float) YRawReal/total_Acc) * 57.296;
	ar_acc=asin((float) XRawReal/total_Acc) * -57.296;


    if(GyroSet)
    {
    	angle_Pitch=ap_gyro*0.9996 + ap_acc*0,0004;
    	angle_Roll=ar_gyro *0.9996 + ar_acc*0.0004;

    }

    else
    {
    	GyroSet=1;
    	angle_Pitch=ap_acc;
    }

}

float x_Acc()
{
	return angle_Pitch;
}
float y_Acc()
{
    return 	angle_Roll;
}


void setCalibrationValue(uint32_t X,uint32_t Y,uint32_t Z)
{
	calibrationGyro[0]=X;
	calibrationGyro[1]=Y;
	calibrationGyro[2]=Z;
}

float X_calib()
{
   return Calib_X;
}


float Y_calib()
{
   return Calib_Y;
}


float Z_calib()
{
   return Calib_Z;
}






