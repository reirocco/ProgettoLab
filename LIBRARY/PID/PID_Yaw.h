#ifndef pidyaw
#define pidyaw

#include "main.h"

typedef struct PID_Yaw{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID_Yaw;
void init_tune_PID_Yaw(PID_Yaw*,float,float,float,float);

float PID_controller_Pitch(PID_Yaw*,float,float);

#endif
