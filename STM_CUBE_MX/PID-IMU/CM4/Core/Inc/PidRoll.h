#ifndef pidroll
#define pidroll

#include "main.h"
#include "PID.h"

float VtoD_Roll(float);
uint8_t ReftoDir_Roll(float);
void set_PWM_dir_Roll(uint32_t,uint8_t);

#endif pidroll
