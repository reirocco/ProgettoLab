/******************************************************************************
* 					PID torque/pos control
* Description  : Used to stabilize the CC motor.
******************************************************************************/

/*******************************************************************************
* History : DD.MM.YYYY     Version     Description       Author
*         : 27.04.2016     2.00        Second release    Jualo
*******************************************************************************/

#define M_PI 3.14159265359
#define PID_Volt_Limit 11.76 // Voltaggio massimo: 12V -> PID_Volt_Limit (98% Voltaggio massimo): 11.76V

/* IMPLEMENTAZIONE CON STRUTTURA */

// Definizione Struttura PID
typedef struct
{
	float Kp;		// Guadagno Proporzionale
    float Ki;		// Guadagno Integrale
    float Kd;		// Guadagno Derivativo
	float ti;		// Tempo di Integrazione
    float td;		// Tempo di Derivazione
    float tt;		// Reset Time
    float n;		// Massimo Guadagno Derivativo (Coefficiente di Filtro)
    float b;		// Set Point in k
    float uhigh;	// Limite Superiore dell' Uscita
    float ulow;		// Limite Inferiore dell' Uscita
    float h;		// Periodo di Campionamento

    float partI;	// Termine Integrale I(kh)
    float partD;	// Termine Derivativo D(kh)
    float yold;		// Uscita del PID all'iterazione precedente
    float uc;    	// Ingresso di riferimento
    float y;        // Ingresso della Retroazione
    float output;   // Uscita PID
} PIDSt_Type;

// Prototipi delle funzioni
int init_pid(PIDSt_Type * PID_P, int Set_param);
#define SET_PARAM_PID_ROLL 0
#define SET_PARAM_PID_PITCH 1
#define SET_PARAM_PID_YAW 2
#define SET_PARAM_PID_TORQUE 3
#define SET_PARAM_PID_SPEED 4

int calcPID (PIDSt_Type * PID_P);



/* End of file PID.h */
