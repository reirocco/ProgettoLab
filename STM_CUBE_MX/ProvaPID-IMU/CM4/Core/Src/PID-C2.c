#include "PID-C2.h"

void init_tune_PID_C2(PID_C2* p,float Tc,float Kp,float Ki,float Kd,float u_max,float u_min){

	p->Tc=Tc;
	p->u_max= u_max; // coppia massima
	p->u_min= u_min;  // coppia minima

	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
}
float PID_controller_C2(PID_C2* p,float y,float r){

	static float e_old=0,Iterm=0;
	float u;
	float newIterm;
	float e=r-y;
	float Pterm = p->Kp*e;
	newIterm=Iterm+(p->Ki)*p->Tc*e_old;
	float Dterm=(p->Kd/p->Tc)*(e-e_old);
	e_old=e;
	u=Pterm+newIterm+Dterm;

	if(u>p->u_max){
		u=p->u_max;

	}else if(u<0.0){
		u= -u;

	}else if(u<p->u_min){
		u=p->u_min;
	}else if(u == 0.0){

		u = 0;
	}else{
		Iterm= newIterm;
	}
	return u;

}
