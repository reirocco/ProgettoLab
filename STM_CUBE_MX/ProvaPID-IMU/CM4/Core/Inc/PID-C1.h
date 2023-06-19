#ifndef pidc1
#define pidc1

#include "main.h"
typedef struct PID_C1{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_C1;
void init_tune_PID_C1(PID_C1*,float,float,float,float,float,float);

float PID_controller_C1(PID_C1*,float,float);


#endif
