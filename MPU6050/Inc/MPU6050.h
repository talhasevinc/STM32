#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#ifndef MPU6050_H_
#define MPU6050_H_


#define MPU6050_ADDR        0xD0     //0x68 (0110 1000) sifht left (1101 0000)=0xD0
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1          0x6B
#define SMPLRT_DIV          0x19
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define TEMP_REG            0x41
#define RAD_TO_DEG 57.295779513082320876798154814105
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;


typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;



bool MPU6050Init();
void i2cWrite(uint8_t Address, uint8_t Value);
void MPU6050Initialization();
void getGyroRawData();
void getAccelerationRawData();
void getAllDatas();
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
float getRoll();
float getPitch();
void MPU6050Calibration(int time);
float getTemperature();



/*Unused*/
void GetAccelerationValue();
void setSampleTime(uint32_t ms);
void setCalibrationValue(uint32_t X,uint32_t Y,uint32_t Z);
void GetGyroValue();
void getACCAngle();
float x_Acc();
float y_Acc();
float z_Acc();
float x_Slope();
float y_Slope();
float z_Slope();
float X_calib();
float Y_calib();
float Z_calib();


#endif
