#ifndef pidpitch
#define pidpitch

#include "main.h"

typedef struct PID_Pitch{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_Pitch;
void init_tune_PID_Pitch(PID_Pitch*,float,float,float,float);

float PID_controller_Pitch(PID_Pitch*,float,float);

#endif
