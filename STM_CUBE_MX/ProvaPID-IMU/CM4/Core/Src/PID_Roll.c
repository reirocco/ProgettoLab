#include "PID_Roll.h"
void init_tune_PID_Roll(PID_Roll* p,float Tc,float Kp,float Ki,float Kd){
	p->Tc=Tc;
	p->u_max= 28.3017; // coppia massima
	p->u_min= -28.3017;  // coppia minima

	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
}
// metodo per il calcolo del PID
float PID_controller_Roll(PID_Roll* p,float y,float r){

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

	}else if(u<p->u_min){
		u=p->u_min;
	}else{
		Iterm= newIterm;
	}
	return u;

}
