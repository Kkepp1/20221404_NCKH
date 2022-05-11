#ifndef _MPU6050_H_
#define _MPU6050_H_
/*
- MPU6050 return pitch angle
- kalman filter
*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"

void MPU6050_Init(I2C_HandleTypeDef *);
void MPU6050_Read_Accel(I2C_HandleTypeDef *);
void MPU6050_Read_Gyro (I2C_HandleTypeDef *);
float Kalman_getAngle( float, float, float);
float MPU6050_GET_ANGLE(I2C_HandleTypeDef *,float*,float*,float, float);
void calib_MPU6050(I2C_HandleTypeDef *hi2c,int loop, float *Error_gyroZ);
#endif
