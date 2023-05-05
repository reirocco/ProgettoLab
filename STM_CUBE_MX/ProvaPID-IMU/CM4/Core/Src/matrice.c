#include "matrice.h"
#include "math.h"
float* matriceT(float u_roll,float u_pitch,float u_yaw){
	float *Tout = malloc(sizeof(int)*3);

	Tout[0] = 0.229*u_yaw - 0.3844*u_roll;
	Tout[1] = 0.1922*u_roll - 0.3329*u_pitch + 0.229*u_yaw;
	Tout[2] = 0.3329*u_pitch + 0.1922*u_roll + 0.229*u_yaw;

	return Tout;

}
