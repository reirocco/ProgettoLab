#include "matrice.h"
#include "math.h"
float* matriceT(float u_roll,float u_pitch,float u_yaw){
	float *Tout = malloc(sizeof(int)*3);

	Tout[0] = 0.4304*u_yaw - 0.3133*u_roll;
	Tout[1] = 0.1567*u_roll - 0.2714*u_pitch + 0.4304*u_yaw;
	Tout[2] = 0.2714*u_pitch + 0.1567*u_roll + 0.4304*u_yaw;

	return Tout;

}
