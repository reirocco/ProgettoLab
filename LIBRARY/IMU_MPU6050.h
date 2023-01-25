#ifndef INC_IMU_MPU6050_H_
#define INC_IMU_MPU6050_H_

#include "main.h"

//sensor I2C adress
#define MPU6050_ADDR 0xD0
//sensor register
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define INT_PIN_CFG_REG 0x37
#define INT_ENABLE_REG 0x38
#define FIFO_EN_REG 0x23
#define FIFO_R_w_REG 0x74
#define FIFO_COUNT_H_REG 0x72
#define FIFO_COUNT_L_REG 0x73
#define USER_CTRL_REG 0x6A
#define WHO_AM_I_REG 0x75
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

extern I2C_HandleTypeDef hi2c1;

typedef struct MPU6050_Data {
	float Ax_raw, Ay_raw, Az_raw;
	float Wx_raw, Wy_raw, Wz_raw;
}MPU6050_Data;

int8_t IMU_MPU6050_Init (void);
void IMU_MPU6050_Read_Acc_Gyro(MPU6050_Data* y);
uint16_t Read_FIFO_Count();
void Reset_Reable_FIFO();

#endif
