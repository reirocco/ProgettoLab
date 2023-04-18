#ifndef pwmpitch
#define pwmpitch

#include "main.h"
#include "PID.h"

float VtoD_Pitch(float);
uint8_t ReftoDir_Pitch(float);
void set_PWM_dir_Pitch(uint32_t,uint8_t);

#endif pidpitch
