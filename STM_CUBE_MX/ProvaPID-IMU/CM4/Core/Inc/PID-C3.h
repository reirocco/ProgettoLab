#ifndef pidc3
#define pidc3

#include "main.h"
typedef struct PID_C3{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_C3;
void init_tune_PID_C3(PID_C3*,float,float,float,float,float,float);

float PID_controller_C3(PID_C3*,float,float);


#endif
