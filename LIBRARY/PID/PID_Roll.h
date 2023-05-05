#ifndef pidroll
#define pidroll

#include "main.h"

typedef struct PID_Roll{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_Roll;
void init_tune_PID_Roll(PID_Roll*,float,float,float,float);

float PID_controller_Roll(PID_Roll*,float,float);

#endif
