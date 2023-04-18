#ifndef pwmyaw
#define pwmyaw

#include "main.h"
#include "PID.h"

float VtoD_Yaw(float);
uint8_t ReftoDir_Yaw(float);
void set_PWM_dir_Yaw(uint32_t,uint8_t);

#endif pidyaw
