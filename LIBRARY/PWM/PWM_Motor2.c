#include "PWM_Motor2.h"

float VtoD_M2(float u){
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


uint8_t ReftoDir_M2(float y_ref){
	uint8_t dir;
	if(y_ref>=0){
		dir=0;//senso orario
	}else{
		dir=1;//senso antiorario
	}
	return dir;

}

void set_PWM_dir_M2(uint32_t duty,uint8_t dir){
	TIM1->CCR2 = ((float)duty/100)*TIM1->ARR;
	/*
	uint8_t current_dir = (TIM4->CR1 & 0x0010);

	if(dir != current_dir)
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);//cambia senso di rotazione
	*/
}
