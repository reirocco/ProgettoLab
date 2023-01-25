
/*******************************************************************************
Includes   <System Includes> , "Project Includes"
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <machine.h>
#include "platform.h"
#include "ADb10.h"
#include "cmt.h"
#include "pwm.h"
#include "sensore.h"
#include "PID.h"

//Variabili esterne
//extern unsigned int general_timer_mS;
extern int timer_2mS;
extern int cont_calibration;
extern float sens_curr_1, sens_curr_2, sens_curr_3;
//extern float vout1, vout2, vout3; //variabili dichiarate per la stampa a video
extern float volt_signal_1, volt_signal_2, volt_signal_3;

//vettore prova sensore
const int vettP[10]={10, 20 , 35, 20, 5, -10, -15, -20, -5, 15};

int ptvettP=0;
int oldtimer=0;
int timerCont=0;

/* Creazione variabile di tipo struttura PID_Struct che verra'  passata alle funzioni di gestione del PID */
PIDSt_Type PID_P_curr_1;
PIDSt_Type PID_P_curr_2;
PIDSt_Type PID_P_curr_3;

/* Declare display buffer */
uint8_t  lcd_buffer[13];

/*******************************************************************************
* Function name: DEMO
* Description  : Main program function.
* Arguments    : none
* Return value : none
*******************************************************************************/
void main(void)
{
    /* Initialize LCD */
    lcd_initialize();
    
    /* Clear LCD */
    lcd_clear();
    
    /* Display message on LCD */
    lcd_display(LCD_LINE1, "Ballbot 3.0");
    lcd_display(LCD_LINE2, "PID COPPIA ");
    
	CMT_init(); // Settaggio del tempo di clock

	// Inizializzazione della MTU2 per generare la PWM sul primo canale (MTIOC3A)
	PWM_Init(2); // PWM motor 2 J8-PIN 15
	PWM_Init(3); // PWM motor 3 JN2-PIN 23
	PWM_Init(4); // PWM motor 1 JN2-PIN 22

	/*Inizializzazione dei parametri per il controllo PID in coppia*/
	init_pid(&PID_P_curr_1,SET_PARAM_PID_TORQUE);
	init_pid(&PID_P_curr_2,SET_PARAM_PID_TORQUE);
	init_pid(&PID_P_curr_3,SET_PARAM_PID_TORQUE);

	//valori per la prova espressi in mA
	/*PID_P_curr_1.uc=-100;
	PID_P_curr_2.uc=200;
	PID_P_curr_3.uc=150;*/

	// Inizializzazione del convertitore A/D a 10-bit
	ADb10_init();

	Init_Port_Dir(); // Inizializzazione delle porte di Direzione

	vettore_vout_init(); // Inizializzazione a 0 dei vettori per le tensioni lette dai sensori di corrente
    
	//Variabili dichiarate per vedere la durata di un ciclo
	/*unsigned int timer_init=0;
	unsigned int tempo=0;*/

    //This is the main loop.
	while (1)
    {
    	// Ciclo di acquisizione dati e controllo dei motori
    	// E' stato implementato un timer da 2ms per l'acquisizione della corrente assorbita dai motori
		//timer_init=general_timer_mS;
		if(timer_2mS!=oldtimer){
			oldtimer = timer_2mS;
			if (++timerCont>3){
				timerCont=0;
				if(++ptvettP>9)
					ptvettP=0;
				PID_P_curr_1.uc=vettP[ptvettP];
				PID_P_curr_2.uc=vettP[ptvettP];
				PID_P_curr_3.uc=vettP[ptvettP];
			}
		}

    	// Ciclo che scandisce l'acquisizione dei dati dai sensori di corrente
    	if(timer_2mS)
    		{
    			timer_2mS = 0;
    			if(cont_calibration < calibration_length)
    			{
    					// Stampa a video dell'avviso di calibrazione dei sensori di corrente
    					sprintf((char *)lcd_buffer, "Calibrating");
    					lcd_display(LCD_LINE4, lcd_buffer);
    					sprintf((char *)lcd_buffer, "Sensors...");
    					lcd_display(LCD_LINE5, lcd_buffer);

    				sens_calibration_init();	// Funzione di calibrazione dei sensori di corrente
    			}
    			else if (cont_calibration == calibration_length)
    				sens_calibration_bias();   	/*Calcolo del bias calcolato su 1000 elementi da sottrare
    											  al valore letto dai sensori*/
    			else
    			{
    				//Lettura dei valori di tensione legati alla corrente inviata ai motori e relativa conversione in corrente
    				sens_read();

    				sprintf((char *)lcd_buffer, "C1=%f",sens_curr_1);
    				lcd_display(LCD_LINE4, lcd_buffer);

    				sprintf((char *)lcd_buffer, "RIF=%d",vettP[ptvettP]);
    				lcd_display(LCD_LINE5, lcd_buffer);

    				/*Assegnazione dei valori letti dal sensore di corrente alla struttura attuale del PID
    			  	  per il controllo in corrente*/
    				PID_P_curr_1.y=sens_curr_1;
    				PID_P_curr_2.y=sens_curr_2;
    				PID_P_curr_3.y=sens_curr_3;

    				// Calcolo del segnale di controllo da inviare ai motori
    				// Inviamo il segnale di riferimento curr_ref al PID in corrente che provvedera' a generare
    				// il segnale di controllo che invieremo ai motori.
    				calcPID(&PID_P_curr_1);
    				calcPID(&PID_P_curr_2);
    				calcPID(&PID_P_curr_3);
    				volt_signal_1 = PID_P_curr_1.output;
    				volt_signal_2 = PID_P_curr_2.output;
    				volt_signal_3 = PID_P_curr_3.output;

    				/*sprintf((char *)lcd_buffer, "vs2=%f",volt_signal_2);
    				lcd_display(LCD_LINE6, lcd_buffer);*/

    				/* Controllo del verso di rotazione */
    				motor_direction();

    				/* Calcolo del Duty-Cycle da inviare ai motori */
    				DutyCycle_to_Motor();
    			}
    		}/* Fine ciclo acquisizione dati e controllo (2ms)*/

    	 	 /*tempo = general_timer_mS - timer_init;
    	     sprintf((char *)lcd_buffer, "tempo=%d", tempo);
    	     lcd_display(LCD_LINE7, lcd_buffer);*/
   }

} /* End function main() */
