/*
 * IMU_MPU6050.h
 *
 *  Created on: Nov 15, 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_IMU_MPU6050_H_
#define INC_IMU_MPU6050_H_

#include "main.h"
//************************************************************************
//************************** DEFINE **************************************

//Sensor's I2C address
#define MPU6050_ADDR 0xD0

//Sensor's registers (partial list)
#define SMPLRT_DIV_REG 0x19 //Sample Rate Divider
#define CONFIG_REG 0x1A //Configuration
#define PWR_MGMT_1_REG 0x6B //Power Management 1
#define ACCEL_CONFIG_REG 0x1C //Accelerometer Configuration
#define ACCEL_XOUT_H_REG 0x3B //Accelerometer Measurements
#define INT_PIN_CFG_REG 0x37 //INT Pin / Bypass Enable Configuration
#define INT_ENABLE_REG 0x38 //Interrupt Enable
#define FIFO_EN_REG 0x23 //FIFO Enable
#define FIFO_R_W_REG 0x74 //FIFO Read Write
#define FIFO_COUNT_H_REG 0x72 //FIFO Count H
#define FIFO_COUNT_L_REG 0x73 //FIFO Count L
#define USER_CTRL_REG 0x6A //User Control
#define WHO_AM_I_REG 0x75 //Who am I
#define GYRO_CONFIG_REG 0x1B //Gyroscope Configuration
#define GYRO_XOUT_H_REG 0x43 //Gyroscope Measurements


//************************** EXTERNAL VARIABLES **************************
//Main
extern I2C_HandleTypeDef hi2c1;

typedef struct MPU6050_Data {
	float Ax_raw, Ay_raw, Az_raw; // m/s^2 (not g)
	float Wx_raw, Wy_raw, Wz_raw; // °/s
}MPU6050_Data;


//************************************************************************
//************************** FUNCTIONS ***********************************
int8_t IMU_MPU6050_Init (void);
void IMU_MPU6050_Read_Acc_Gyro(MPU6050_Data* y);
uint16_t Read_FIFO_Count();
void Reset_Reable_FIFO();

#endif /* INC_IMU_MPU6050_H_ */
