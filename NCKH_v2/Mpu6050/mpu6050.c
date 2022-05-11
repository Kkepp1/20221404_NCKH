#include "mpu6050.h"
//---------------define address register mpu6050--------
#define WHO_AM_I_REG 0X75
#define MPU6050_ADDR 0XD0
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG 0x1B

#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43
#define TEMP_OUT_H_REG 0x41
#define PWR_MGMT_1_REG 0x6B

#define PI 3.1415926559

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax=0;
float Ay=0;
float Az=0;
float Gx=0;
float Gy=0;
float Gz=0;

//kalman filter
float	bias =0.003;
float	angle = 0.0;
float	Q_angle = 0.001;
float	Q_bias = 0.003;
float	R_measure = 0.03;
float P[2][2];

float pitch=0.0;
float yaw=0.0;
float roll=0.0;
//float pitch_Kalman=0;

static int16_t offset_gyx = 0, offset_gyy = 0, offset_gyz = 0;


void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t check, Data;
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG,1,&check,1,100);
		if(check == 104) // if the device is present
		{
			Data =0;
			HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG,1,&Data, 1, 100); 
      Data= 0x07;
			HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG,1,&Data,1,100);
      Data = 0x00;
			HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG,1,&Data,1,100);			
      Data = 0x00;
			HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG,1,&Data,1,100); 
		}
}
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c){
 uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG,1,Rec_Data,6,100);
	
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	
	Ax = Accel_X_RAW/16384.0f;
	Ay = Accel_Y_RAW/16384.0f;
	Az = Accel_Z_RAW/16384.0f;
	}

void MPU6050_Read_Gyro (I2C_HandleTypeDef *hi2c)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6,100);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}
float Kalman_getAngle( float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
void calib_MPU6050(I2C_HandleTypeDef *hi2c,int loop, float *Error_gyroZ){
	 *Error_gyroZ=0;
		for(int i=0;i<loop;i++){
			MPU6050_Read_Gyro(hi2c);
			(*Error_gyroZ) += Gz;
		
		}
		(*Error_gyroZ)/=loop;
}
float MPU6050_GET_ANGLE(I2C_HandleTypeDef *hi2c,float *pitch_Kalman,float* yaw,float Error_gyroZ, float timeSample){
					MPU6050_Read_Accel(hi2c);
					MPU6050_Read_Gyro(hi2c);
//					float e=0;
//					e=0.2;//-0.15
					
						 Gz = Gz-Error_gyroZ;
					
						
					
					(*yaw)=(*yaw)+Gz*(0.0009+timeSample);
						//*yaw=*yaw+Gz*timeSample;
					pitch = (atan2f((- Ax), Az)+PI)*180/PI;	
					(*pitch_Kalman) = Kalman_getAngle(pitch,Gy,timeSample);
	
}
