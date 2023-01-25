/******************************************************************************
* File Name     : PID.c
* Version       : 1.0
* Description   : This file contain a PID struct and calculation method,
*				  using struct to define the PID's coefficents.
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description       Author
*         : 05.04.2016     1.00        First release     Jualo
*******************************************************************************/

#include "PID.h"
#include <math.h>


/********************************************************************************
* Function name: calcPID_curr & calcPID_pos
* Description  : Perform the calculation in order to provide the correction
*				 signal, this function is used for all PID control step.
* Arguments    : Reference signal (float uc), actual signal (float y), PID struct
*				 pointer
* Return value : Correction signal.
********************************************************************************/
int calcPID (PIDSt_Type * PID_P)
{
    float P, v, u; // P = Termine Proporzionale, v = Uscita pura del PID, u = Uscita finale del PID (filtrata con Antiwindup)

    /* Implementazione PID */

	// Calcolo P
	P = PID_P -> Kp * ((PID_P ->  b * PID_P ->uc) - PID_P ->y);

	// Calcolo D (Non è utilizzata nel controllo in corrente)
	/*PID_P -> partD = (PID_P -> td) / ((PID_P -> td) + ((PID_P -> n) * (PID_P -> h))) * (PID_P -> partD) - ((PID_P -> Kp) *
					 (PID_P -> n) * (PID_P -> td)) / ((PID_P -> td) + (PID_P -> n) * (PID_P -> h)) * (PID_P -> y -  PID_P -> yold);*/

	// Uscita pura del PID
	v = P + (PID_P -> partI); // Per l'uscita del PID in corrente non è stata considerata la parte derivativa (PID_P -> partD)

	// Antiwindup
	if (v < PID_P -> ulow)
	{
		u = PID_P -> ulow;
	}
	else
	{
		if (v > PID_P -> uhigh)
		{
			u = PID_P -> uhigh;
		}
		else
		{
			u = v;
		}
	}

	// Calcolo I
	(PID_P -> partI) = (PID_P -> partI) + ((PID_P -> Kp) * (PID_P -> h) / (PID_P -> ti)) * (PID_P ->uc - PID_P -> y) + ((PID_P -> h) / (PID_P -> tt))*(u - v);

	// Aggiornamento yold (Non serve per il controllo in coppia)
	//PID_P -> yold = PID_P ->y;

	//uscita PID
	PID_P ->output=u;
	return 1;
}

/**********************************************************************************
* Function name: init_pid
* Description  : Initialize the struct of the Position PID with the desired
*				 coefficient. It also evaluate the terms based on them.
* Arguments    : PID struct pointer (PIDSt_Type * PID_P), Curret angle (int angle).
* Return value : none
**********************************************************************************/
/* Il PID in posizione ha bisogno di parametri diversi per ogni angolo;
* inizializzando la struttura con l'int angle è possibile distinguere i diversi casi:
* 0 = Roll, 1 = Pitch, 2 = Yaw
*/
int init_pid(PIDSt_Type * PID_P, int Set_param)
{
	switch(Set_param)
	{
		case SET_PARAM_PID_ROLL:
			PID_P -> Kp  = 2;
			PID_P -> Ki = 1;
			PID_P -> ti = (PID_P -> Kp) / (PID_P -> Ki);
			PID_P -> Kd = 0.2;
			PID_P -> td = (PID_P -> Kd) / (PID_P -> Ki);
			PID_P -> tt = sqrt(PID_P -> ti * PID_P -> td);
			PID_P -> n = 20;
			PID_P -> b = 1;
			PID_P -> ulow = -M_PI/2;
			PID_P -> uhigh = M_PI/2;
			PID_P -> h = 0.01; // Tempo di campionamento: 10ms
			PID_P -> partI = 0;
			PID_P -> partD = 0;
			PID_P -> yold = 0;
			break;

		case SET_PARAM_PID_PITCH:
			PID_P -> Kp  = 2;
			PID_P -> Ki = 1;
			PID_P -> ti = (PID_P -> Kp) / (PID_P -> Ki);
			PID_P -> Kd = 0.2;
			PID_P -> td = (PID_P -> Kd) / (PID_P -> Ki);
			PID_P -> tt = sqrt(PID_P -> ti * PID_P -> td);
			PID_P -> n = 20;
			PID_P -> b = 1;
			PID_P -> ulow = -M_PI/2;
			PID_P -> uhigh = M_PI/2;
			PID_P -> h = 0.01;
			PID_P -> partI = 0;
			PID_P -> partD = 0;
			PID_P -> yold = 0;
			break;

		case SET_PARAM_PID_YAW:
			PID_P -> Kp  = 2;
			PID_P -> Ki = 1;
			PID_P -> ti = (PID_P -> Kp) / (PID_P -> Ki);
			PID_P -> Kd = 0.2;
			PID_P -> td = (PID_P -> Kd) / (PID_P -> Ki);
			PID_P -> tt = sqrt(PID_P -> ti * PID_P -> td);
			PID_P -> n = 20;
			PID_P -> b = 1;
			PID_P -> ulow = -M_PI/2;
			PID_P -> uhigh = M_PI/2;
			PID_P -> h = 0.01;
			PID_P -> partI = 0;
			PID_P -> partD = 0;
			PID_P -> yold = 0;
			break;

        case SET_PARAM_PID_TORQUE:
		    PID_P -> Kp = 0.8;
			PID_P -> Ki = 10;
			PID_P -> ti = (PID_P -> Kp) / (PID_P -> Ki);
			PID_P -> Kd = 1;
			PID_P -> td = (PID_P -> Kd) / (PID_P -> Ki);
			PID_P -> tt = sqrt(PID_P -> ti * PID_P -> td);
			PID_P -> n = 3;
			PID_P -> b = 1;
			PID_P -> ulow = -PID_Volt_Limit;
			PID_P -> uhigh = PID_Volt_Limit;
			PID_P -> h = 0.002; // Tempo di campionamento: 2ms
			PID_P -> partI = 0;
			PID_P -> partD = 0;
			PID_P -> yold = 0;
			break;

        case SET_PARAM_PID_SPEED:
			PID_P -> Kp  = 2;
			PID_P -> Ki = 1;
			PID_P -> ti = (PID_P -> Kp) / (PID_P -> Ki);
			PID_P -> Kd = 0.2;
			PID_P -> td = (PID_P -> Kd) / (PID_P -> Ki);
			PID_P -> tt = sqrt(PID_P -> ti * PID_P -> td);
			PID_P -> n = 20;
			PID_P -> b = 1;
			PID_P -> ulow = -M_PI/2;
			PID_P -> uhigh = M_PI/2;
			PID_P -> h = 0.01; // Tempo di campionamento: 10ms
			PID_P -> partI = 0;
			PID_P -> partD = 0;
			PID_P -> yold = 0;
			break;

		default:
			return 0;
			break;
	}
	return 1;
}

/* End of file PID.c */
