#ifndef INC_PID_H_
#define INC_PID_H_

#include <main.h>

typedef struct PID{
//tuning param
	float Kp;//Proportional gain
	float Ki;//Integral gain
	float Kd;//Derivative gain
//altri parametri
	float Tc;//sampling period
	float u_max;//pid output upper limit
	float u_min;//pid output lower limit

}PID;
//function declaration
void init_tune_PID(PID*,float,float,float,float);
//void tune_PID(PID*,float,float,float);
float PID_controller(PID*,float,float);

#endif /* INC_PID_H_ */
