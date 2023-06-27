#include "PWM_Motor3.h"
float CtoD_M3(float u){
	if(u <= 0)
		u = -u;
	float duty = 100*u/7.68;
	if(duty > 100){
			duty = 100;
	}else if(duty < 0){
		duty = 0;
	}
	return duty;

}
float VtoD_M3(float u){
	if(u <= 0)
		u = -u;
	float duty = 100*u/12;

	if(duty > 100){
		duty = 100;
	}else if(duty < 0){
		duty = 0;
	}
	return duty;
}


uint8_t ReftoDir_M3(float u){
	uint8_t dir;
	if(u>=0){
		dir=0;//senso orario
	}else{
		dir=1;//senso antiorario
	}
	return dir;

}

void set_PWM_dir_M3(uint32_t duty,uint8_t dir){
	TIM1->CCR3 = ((float)duty/100)*TIM1->ARR;
	//printf("%d \r\n", dir);

	if(dir==1){
		HAL_GPIO_WritePin(GPIOE, DIR3_Pin,GPIO_PIN_SET);//cambia senso di rotazione
	}else{
		HAL_GPIO_WritePin(GPIOE, DIR3_Pin,GPIO_PIN_RESET);
	}
}
