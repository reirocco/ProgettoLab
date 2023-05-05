#include "PID_Pitch.h"

void init_tune_PID_Pitch(PID_Pitch* p,float Tc,float Kp,float Ki,float Kd){
	p->Tc=Tc;
	p->u_max= 7.68; // coppia massima
	p->u_min= -7.68;  // coppia minima

	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
}
/*
void tune_PID(PID* p,float Kp,float Ki,float Kd){
	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
}*/

float PID_controller_Pitch(PID_Pitch* p,float y,float r){
	//printf("Ingresso: %f\r\n",y);
	static float e_old=0,Iterm=0;
	float u;
	float newIterm;
	float e=r-y;
	float Pterm = p->Kp*e;
	newIterm=Iterm+(p->Ki)*p->Tc*e_old;
	float Dterm=(p->Kd/p->Tc)*(e-e_old);
	e_old=e;
	u=Pterm+newIterm+Dterm;
	//printf("Uscita reale: %f\r\n",u);
	if(u>p->u_max){
		u=p->u_max;
		//printf("Uscita approssimata: %f\r\n",u);
	}else if(u<0.0){
		u= -1*u;/*p->u_min;*/
		//printf("Uscita approssimata: %f\r\n",u);
	}else if(u<p->u_min){
		u=p->u_min;
	}else{
		Iterm= newIterm;
	}
	return u;

}
