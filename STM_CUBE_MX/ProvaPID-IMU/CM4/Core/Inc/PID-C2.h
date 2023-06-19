#ifndef pidc2
#define pidc2

#include "main.h"
typedef struct PID_C2{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_C2;
void init_tune_PID_C2(PID_C2*,float,float,float,float,float,float);

float PID_controller_C2(PID_C2*,float,float);


#endif
