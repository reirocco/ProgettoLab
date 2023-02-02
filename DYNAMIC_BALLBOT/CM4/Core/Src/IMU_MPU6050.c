/*
 * IMU_MPU6050.c
 *
 *  Created on: Nov 15, 2022
 *      Author: M.Cristina Giannini
 */

#include "IMU_MPU6050.h"
#include "main.h"
#include "stdio.h"

//Gravity acceleration
float g = 9.80665;

//MPU6050's sensitivity (depends on full scale range selected)
//8192 LSB/g for ± 4g
//65.5 LSB/°/s for ± 500 °/s
float Acc_LSB_Sensitivity = 8192.0;
float Gyro_LSB_Sensitivity = 65.5;


int8_t IMU_MPU6050_Init (void){
	uint8_t Data;
	HAL_StatusTypeDef ret;

	//Double check I2C communication
	ret = HAL_I2C_IsDeviceReady (&hi2c1,MPU6050_ADDR, 2, 1000); //2 trials, 1000ms each
	if(ret!=HAL_OK){
		return -1;
	}
	uint8_t check;
	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if((ret!=HAL_OK) || (check!=0x68)){
		return -1;
	}

	//Initialize

	//Power Management 1 register
	//sensor's normal operation
	Data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

	//Accelerometer Configuration register
	//set accelerometer's full scale range to ± 4g (AFS_SEL=1)
	//no self-test
	Data = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	//Gyroscope Configuration register
	//set gyroscope's full scale range to ± 500 °/s (FS_SEL=1)
	//no self-test
	Data = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

	//Configuration register
	//enable Digital Low Pass Filter(DLPF) with the highest bandwidth (DLPF_CFG=1):
	//Acc: 184Hz bandwidth, 2.0ms delay
	//Gyro: 188Hz bandwidth, 1.9ms delay
	//Note with DLPF gyroscope frequency reduces to 1kHz
	//Note acc max frequency is 1KHz
	Data = 0x01;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

	//Sample Rate Divider register
	//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	//set sample rate to 20Hz, 50ms (SMPLRT_DIV=49)
	Data = 0x31;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	//User Control register
	//enable FIFO buffer
	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	//FIFO Enable register
	//select accelerometer's and gyroscope's data for FIFO buffer
	Data = 0x78;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);

	//INT Pin / Bypass Enable Configuration register
	//configure interrupt signal (default)
	Data = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);

	//Interrupt Enable register
	//enable Data Ready interrupt (DATA_RDY_EN=1)
	//(write operation to all of the sensor registers has been completed)
	Data = 0x01;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);

	return 0;
}

void IMU_MPU6050_Read_Acc_Gyro(MPU6050_Data* y){
	uint8_t Data[12]; //3acc,3vel, 2byte each

	uint16_t counts = Read_FIFO_Count();
	//printf("counts %d\r\n",counts);
	int16_t app = 0;

	if(counts>=12){
		//Read 3acc, 3vel from FIFO buffer
		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_R_W_REG, 1, Data, 12, 1000);
		if(ret==HAL_OK){
			//Store acquisitions in MPU6050_Data
			app = (int16_t) Data[0] << 8 | (int16_t) Data[1];
			y->Ax_raw = (app / Acc_LSB_Sensitivity) * g;
			app = (int16_t) Data[2] << 8 | (int16_t) Data[3];
			y->Ay_raw = (app / Acc_LSB_Sensitivity) * g;
			app = (int16_t) Data[4] << 8 | (int16_t) Data[5];
			y->Az_raw = (app / Acc_LSB_Sensitivity) * g;

			app = (int16_t) Data[6] << 8 | (int16_t) Data[7];
			y->Wx_raw = app / Gyro_LSB_Sensitivity;
			app = (int16_t) Data[8] << 8 | (int16_t) Data[9];
			y->Wy_raw = app / Gyro_LSB_Sensitivity;
			app = (int16_t) Data[10] << 8 | (int16_t) Data[11];
			y->Wz_raw = app / Gyro_LSB_Sensitivity;
		} else {
			printf("Error I2C\r\n");
			switch(ret){
			case HAL_ERROR: printf("HAL_ERROR\r\n");break;
			case HAL_BUSY: printf("HAL_BUSY\r\n");break;
			case HAL_TIMEOUT: printf("HAL_TIMEOUT\r\n");break;
			}
		}
		/*
		if(counts>12){
			printf("Delay\r\n");
			printf("Reset FIFO buffer\r\n");
			Reset_Reable_FIFO();
		}*/
	}
}


//Read FIFO Count register
//Output : FIFO Count value (n byte)
uint16_t Read_FIFO_Count(){

	HAL_StatusTypeDef ret;
	uint16_t fifo_count = 0;
	uint8_t Rec_Data[2];

	//Read FIFO_COUNT_H
	ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_H_REG, 1, Rec_Data, 1, 1000);

	//Read FIFO_COUNT_L
	ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_L_REG, 1, Rec_Data + 1, 1, 1000);

	//Get FIFO Count value
	fifo_count = (uint16_t) (Rec_Data[0] << 8 | Rec_Data[1]);

	return fifo_count;
}

//Reset and reable FIFO buffer
void Reset_Reable_FIFO(){
	HAL_StatusTypeDef ret;

	//Reset FIFO
	uint8_t Data = 0x04;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	//Reable FIFO
	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

}

