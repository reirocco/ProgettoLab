/**********************************************************************************
 * File name	: sensore.c
 * Created on	: 11/12/2017
 * H/W Platform : YRDKRX63N
 * Description  : Driver for the current sensor.
 * 				  Implementazione per la calibrazione del sensore di corrente
 * 				  e per la lettura dei dati dal sensore di corrente.
 **********************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "platform.h"
#include "sensore.h"
#include "ADb10.h"

//Dichiarazione delle variabili
float bias1 = 0, bias2 = 0, bias3 = 0;				// Fattori di correzione per i sensori di corrente
float vout1 = 0, vout2 = 0, vout3 = 0;				// Tensione in uscita dai sensori di corrente
float sens_curr_1 = 0, sens_curr_2 = 0, sens_curr_3 = 0;				// Corrente in ingresso ai sensori di corrente
int cont_calibration = 0;							// Contatore che tiene il numero di elementi da inserire in vettore_vout
float sum_vout1 = 0, sum_vout2 = 0, sum_vout3 = 0; 	// Somma delle tensioni lette dai sensori di corrente e inserite in vettore_vout

/*******************************************************************************
* Function name: sens_calibration_init
* Description  : Current sensors calibration
* Arguments    : none
* Return value : none
*******************************************************************************/
void sens_calibration_init (void)
{
	if(Gestisci_ADb10() == 1)
	{
		vout1 = ADb10_read(1);
		vout2 = ADb10_read(2);
		vout3 = ADb10_read(3);

		sum_vout1 += vout1;
		sum_vout2 += vout2;
		sum_vout3 += vout3;

		cont_calibration++;
	}
} /* Fine della funzione calibration_init */

/*******************************************************************************
* Function name: sens_calibration_bias
* Description  : Calculation function for current sensors' bias
* Arguments    : none
* Return value : none
*******************************************************************************/
void sens_calibration_bias (void)
{
	bias1 = (sum_vout1/calibration_length) - offset_vout;
	bias2 = (sum_vout2/calibration_length) - offset_vout;
	bias3 = (sum_vout3/calibration_length) - offset_vout;

	cont_calibration++;
} /* Fine della funzione calibration_bias */

/*******************************************************************************
* Function name: sens_read
* Description  : Acquisition function for voltage values proportional to the current
*				 from sensors
* Arguments    : none
* Return value : none
*******************************************************************************/
void sens_read (void)
{
	if (Gestisci_ADb10() == 1)
	{
		// Lettura dei valori di tensione legati alla corrente inviata ai motori
		vout1 = ADb10_read(1) - bias1;
		vout2 = ADb10_read(2) - bias2;
		vout3 = ADb10_read(3) - bias3;

		// Conversione dei valori di tensione letti dal sensore in valori di corrente
		sens_curr_1 = (vout1 - offset_vout)/sensitivity;
		sens_curr_2 = (vout2 - offset_vout)/sensitivity;
		sens_curr_3 = (vout3 - offset_vout)/sensitivity;
	}
} /* Fine della funzione sens_read */

/* End of file sensore.c */
