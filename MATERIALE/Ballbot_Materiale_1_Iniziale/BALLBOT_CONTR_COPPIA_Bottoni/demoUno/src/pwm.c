
/* standard types */
#include <stdint.h>
#include <stdbool.h>
/* provide access to on-chip peripherals */
#include "platform.h"
#include "pwm.h"
volatile float duty1, duty2, duty3;
int val1, val2, val3;
float volt_signal_1;
float volt_signal_2;
float volt_signal_3;

/*
*********************************************************************************************************
*                                   PWM (MTU2) INITIALIZATION (PWM mode 1)
*
* Description   : This function initialize MTU2.
*
* Argument      : Numero del canale, 1,2,3,4,5
*
* Returns       : None
*********************************************************************************************************
*/

void PWM_Init(unsigned char channel_number)
{		
	#ifdef PLATFORM_BOARD_RDKRX63N
		SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
	#endif

	/* Enable MTU2 module operation */
	if(channel_number == 1 || channel_number == 2)
		MSTP(MTU3) = 0;      /* Cancel MTU peripheral stop state. */
	if(channel_number == 3 || channel_number == 4)
		MSTP(MTU4) = 0;      /* Cancel MTU peripheral stop state. */

	/*  PWM   mode  1, MTU2 channels 1, 2, 3, 4 */
	/* da qui in poi le scritture sui registri della mtu sono abilitate */
	MTU.TRWER.BIT.RWE = 1; /* Enable read/write access to the write-protected MTU3 and MTU4 registers. */

	if(channel_number == 1 || channel_number == 2)
	{  
		MTU.TSTR.BIT.CST3 = 0;     /* Stop MTU 3 */

		/*  1. Select counter clock  */
		/* Select the counter clock  with bits  TPSC2 to  TPSC0  in TCR, 
	   		select the input clock edge with bits CKEG1 and CKEG0 in TCR 	*/
		MTU3.TCR.BIT.TPSC = 1;  /* Internal clock, Peripheral clock/4=> Periodo dell'onda PWM=(TGRA_3_VAL*4)/(40*10^6) [s]
	                               In frequenza:TGRA_3_VAL=(40*10^6)/4/f
	                               con f = PWM_FREQ impostata nella #define iniziale
								 */

		MTU3.TCR.BIT.CKEG = 0;  /* Count at rising edge  */

		/*   2. Select counter clearing source  */
		/* Select the TGR to be used as the TCNT_3 clearing source with bits CCLR2 to CCLR0 in TCR */
		MTU3.TCR.BIT.CCLR = 1;  /* TCNT_3 cleared by TGRA_3 compare match */

		/*   3. Select waveform output level  */
		/* Designate TGR as an output compare register, select initial value and output value, with byte TIORH/L */
		if(channel_number == 1)
		{
			/* Quando TCNT match TGRB l'onda da 0 va a 1, quando ho il match con TGRA l'onda da 1 va ad 0*/
			MTU3.TIORH.BIT.IOA = 5;  /* Initial output 1,and 0 output at compare match TGRA_3 */
			MTU3.TIORH.BIT.IOB = 2;  /* Initial output 0,and 1 output at compare match TGRB_3 */
		}
		if(channel_number == 2)
		{
			/* Quando match TGRC l'onda da 1 va a 0, quando ho il match TGRD l'onda da 0 va ad 1*/
			MTU3.TIORL.BIT.IOC = 5;  /* Initial output 1,and 0 output at compare match TGRC_3 */
			MTU3.TIORL.BIT.IOD = 2;  /* Initial output 0,and 1 output at compare match TGRD_3 */
		}

	   /*   3.a Select count direction   */
		MTU3.TSR.BIT.TCFD = 1; /* TCNT counts up */

#if(PWM_FREQ < 160)
#error "MIN PWM FREQ is 160 Hz"
#endif
#if(PWM_FREQ > 20000)
#error "MAX PWM FREQ is 20kHz"
#endif

		/*   4. Set TGR   */
		if(channel_number == 1)
		{
			/* Set the cycle in the TGR selected in 2., and set the duty cycle in the other TGR */
			MTU3.TGRA = TGRA_3_VAL;  	/* Tp = TGRA_3_VAL * 16/Peripheral clock */
			MTU3.TGRB = 0xFFFF;  		/*  initially stopped  */
		}

		if(channel_number == 2)
		{
			/* Le due forme d'onda hanno lo stesso periodo */
			MTU3.TGRC = TGRA_3_VAL;  	/* Tp = TGRA_3_VAL * 16/Peripheral clock */
			MTU3.TGRD = 0xFFFF;  		/* initially stopped */
		}

		/*   5. Set PWM mode  */
		/* Select the PWM mode with bits MD3 to MD0 in TMDR */
		MTU3.TMDR.BIT.MD = 2;  /* Select PWM mode 1 */

		/*   5.a Start count  */
		/* Set the CST bit in TSTR to 1 to start the count operation  */
		MTU.TSTR.BIT.CST3 = 1;  			/* Start count TCNT_3 */

		//ms_delay(30);
	
		/*   6. Pin Function Controller (PFC) initialization, v. 12.8.2, SH7201 HW Manual  */
		if(channel_number == 1)
		{
		    /*  Enable writing to the port PFS register with PWPR. (PFS write protect register) */
		    MPC.PWPR.BIT.B0WI = 0;
		    MPC.PWPR.BIT.PFSWE = 1;

			/* Set P17 MTIOC3A as peripheral function bit */
			PORT1.PMR.BIT.B7  = 1;

			/* Port E2 Pin Function Select Register (PE2PFS)
			    b7      Reserved: This bit is always read as 0. The write value should always be 0.
			    b6      ISEL:     Interrupt Input Function Select, 0 = not used as IRQn input pin
			    b5      Reserved: This bit is always read as 0. The write value should always be 0.
			    b4:b0   PSEL:     These bits select the peripheral function.
			    */
			MPC.P17PFS.BYTE  = 0x01; /* 1 defines P17 to be MTIOC3A, with no IRQ. */

			/* Multiplexed pin, accesso per bit, pin P17 -> MTIOC3A (I/O) 									*/
			PORT1.PDR.BIT.B7  = 1;    /* output direction for P17: MTIOC3A => output PWM waveform 			*/
	                             	  /* pin 29 (AUD_R), il pin 1 sul connettore JP17 						*/

#if defined(MCU_RX630) || defined(MCU_RX63N)
			/* Done with port function settings so lock the port function register */
			MPC.PWPR.BIT.PFSWE = 0; /* Disable writing to PFS */
			MPC.PWPR.BIT.B0WI = 1; 	/* Disable writing to PFSWE. */
#endif
		}

		if(channel_number == 2)
		{
		    /*  Enable writing to the port PFS register with PWPR. (PFS write protect register) */
		    MPC.PWPR.BIT.B0WI = 0;
		    MPC.PWPR.BIT.PFSWE = 1;

			/* Set P17 MTIOC3A as peripheral function bit */
			PORTJ.PMR.BIT.B3  = 1;

			/* Port E2 Pin Function Select Register (PE2PFS)
			    b7      Reserved: This bit is always read as 0. The write value should always be 0.
			    b6      ISEL:     Interrupt Input Function Select, 0 = not used as IRQn input pin
			    b5      Reserved: This bit is always read as 0. The write value should always be 0.
			    b4:b0   PSEL:     These bits select the peripheral function.
			    */
			MPC.PJ3PFS.BYTE  = 0x01; /* 1 defines PJ3 to be MTIOC3C, with no IRQ. */

			/* Multiplexed pin, accesso per bit, pin PJ3 -> MTIOC3C (I/O) 								 	*/
			PORTJ.PDR.BIT.B3  = 1;    /* output direction for PJ3: MTIOC3C => output PWM waveform 			*/
	                             	  /* il pin 15 sul connettore J8 						*/

#if defined(MCU_RX630) || defined(MCU_RX63N)
			/* Done with port function settings so lock the port function register */
			MPC.PWPR.BIT.PFSWE = 0; /* Disable writing to PFS */
			MPC.PWPR.BIT.B0WI = 1; 	/* Disable writing to PFSWE. */
#endif
		}
	}

	if(channel_number == 3 || channel_number == 4)
	{ 
	    /*  Enable writing to the port PFS register with PWPR. (PFS write protect register) */
	    MPC.PWPR.BIT.B0WI = 0;
	    MPC.PWPR.BIT.PFSWE = 1;

		MTU.TSTR.BIT.CST4 = 0;  /* Stop MTU 4 */

		MTU4.TCR.BIT.TPSC = 1;  /* Internal clock, Peripheral clock/4=> Periodo dell'onda PWM=(TGRA_4_VAL*4)/(40*10^6) [s]
	                               		In frequenza:	TGRA_4_VAL=(40*10^6)/4/f
										con f = PWM_FREQ impostata nella #define iniziale
								 */
		MTU4.TCR.BIT.CKEG = 0;  /* Count at rising edge  */

		/*   2. Select counter clearing source  */
		/* Select the TGR to be used as the TCNT_3 clearing source with
	   		bits CCLR2 to CCLR0 in TCR */
		MTU4.TCR.BIT.CCLR = 1;  /* TCNT_4 cleared by TGRA_4 compare match */

		/*   5. Set PWM mode  */
		MTU4.TMDR.BIT.MD = 2;  /* Select PWM mode 1 */

		/*   3.0 Set TOER of channels 3 and 4 prior to setting TIOR */
		MTU.TOER.BIT.OE4A = 1;
		MTU.TOER.BIT.OE4C = 1;

		if(channel_number == 3)
		{
			/*   3.0 Set TOER of channels 3 and 4 prior to setting TIOR */
			MTU.TOER.BIT.OE4A = 1;

			/* Quando match TGRA l'onda da 1 va a 0, quando c'� il match TGRB l'onda da 0 va ad 1*/
			MTU4.TIORH.BIT.IOA = 5;  /* Initial output 1,and 0 output at compare match TGRA_3 */
			MTU4.TIORH.BIT.IOB = 2;  /* Initial output 0,and 1 output at compare match TGRB_3 */
		}

		if(channel_number == 4)
		{
			/*   3.0 Set TOER of channels 3 and 4 prior to setting TIOR */
			MTU.TOER.BIT.OE4C = 1;

			/* Quando match TGRC l'onda da 1 va a 0, quando c'� il match TGRD l'onda da 0 va ad 1*/
			MTU4.TIORL.BIT.IOC = 5;  /* Initial output 1,and 0 output at compare match TGRC_3 */
			MTU4.TIORL.BIT.IOD = 2;  /* Initial output 0,and 1 output at compare match TGRD_3 */
		}

		/*   3.a Select count direction   */
		MTU4.TSR.BIT.TCFD = 1; /* TCNT counts up */

		#if(PWM_FREQ < 160)
		#error "MIN PWM FREQ is 160 Hz"
		#endif
		#if(PWM_FREQ > 20000)
		#error "MAX PWM FREQ is 20kHz"
		#endif

		/*   4. Set TGR   */
		if(channel_number == 3)
		{
			/* Set the cycle in the TGR selected in 2., and set the duty cycle in the other TGR */
			MTU4.TGRA = TGRA_4_VAL;	/* Tp = TGRA_4_VAL * 16/Peripheral clock */
			MTU4.TGRB = 0xFFFF;  		/*  initially stopped  */
		}
		if(channel_number == 4)
		{
			/* Le due forme d'onda hanno lo stesso periodo */
			MTU4.TGRC = TGRA_4_VAL;  	/* Tp = TGRA_4_VAL * 16/Peripheral clock */
			MTU4.TGRD = 0xFFFF;  		/* initially stopped */
		}

		/*   5.a Start count  */
		MTU.TSTR.BIT.CST4 = 1;  /* Start count TCNT_4 */

		//ms_delay(30);
				
		/*   6. Pin Function Controller (PFC) initialization, v. 12.8.2, SH7201 HW Manual  */
		if(channel_number == 3)
		{
		    /*  Enable writing to the port PFS register with PWPR. (PFS write protect register) */
		    MPC.PWPR.BIT.B0WI = 0;
		    MPC.PWPR.BIT.PFSWE = 1;

			/* Port E2 Pin Function Select Register (PE2PFS)
		    b7      Reserved: This bit is always read as 0. The write value should always be 0.
		    b6      ISEL:     Interrupt Input Function Select, 0 = not used as IRQn input pin
		    b5      Reserved: This bit is always read as 0. The write value should always be 0.
		    b4:b0   PSEL:     These bits select the peripheral function.
		    */
		    MPC.PE2PFS.BYTE  = 0x01; /* 1 defines PE2 to be MTIOC4A, with no IRQ. */

		    PORTE.PDR.BIT.B2 = 1;    /* Set PE2 as output. */

		    PORTE.PMR.BIT.B2 = 1;    /* Set PE2 as peripheral function bit */

#if defined(MCU_RX630) || defined(MCU_RX63N)
			/* Done with port function settings so lock the port function register */
			MPC.PWPR.BIT.PFSWE = 0; /* Disable writing to PFS */
			MPC.PWPR.BIT.B0WI = 1; 	/* Disable writing to PFSWE. */
#endif
		}

		if(channel_number == 4)
		{							   
		    /*  Enable writing to the port PFS register with PWPR. (PFS write protect register) */
		    MPC.PWPR.BIT.B0WI = 0;
		    MPC.PWPR.BIT.PFSWE = 1;

			/* Port E2 Pin Function Select Register (PE2PFS)
		    b7      Reserved: This bit is always read as 0. The write value should always be 0.
		    b6      ISEL:     Interrupt Input Function Select, 0 = not used as IRQn input pin
		    b5      Reserved: This bit is always read as 0. The write value should always be 0.
		    b4:b0   PSEL:     These bits select the peripheral function.
		    */
		    MPC.PE1PFS.BYTE  = 0x01; /* 1 defines PE1 to be MTIOC4C, with no IRQ. */

		    PORTE.PDR.BIT.B1 = 1;    /* Set PE1 as output. */

		    PORTE.PMR.BIT.B1 = 1;    /* Set PE1 as peripheral function bit */

#if defined(MCU_RX630) || defined(MCU_RX63N)
			/* Done with port function settings so lock the port function register */
			MPC.PWPR.BIT.PFSWE = 0; /* Disable writing to PFS */
			MPC.PWPR.BIT.B0WI = 1; 	/* Disable writing to PFSWE. */
#endif
		}
	}
}

/*
*********************************************************************************************************
*                                  PWM SIGNAL GENERATION 
*
* Description   : Converte lo sforzo di controllo medio (in volt) in un valore numerico per il 
*				  registro di comparazione TGR (duty cycle).
*
* Argument      : 'control_output' sforzo di controllo in volt, 'channel' numero del canale 
*
* Returns       : None
*********************************************************************************************************
*/

void Volt_to_duty (float control_output, unsigned char channel)
{
 unsigned short int tgr_reg; 	/* Valore per il registro TGR (B o D) */
 	
	if(channel == 1 || channel == 2)
		tgr_reg = (unsigned short)(((float)TGRA_3_VAL * control_output) / (float)V_MAX);
	else if(channel == 3 || channel == 4)
		tgr_reg = (unsigned short)(((float)TGRA_4_VAL * control_output) / (float)V_MAX);

	if(tgr_reg == 0 || control_output <= 0.0) /* Se lo sforzo di controllo si approssima a zero oltre il minimo valore di precisione */
	  { 
		tgr_reg = 0xFFFF;  /* max value for TGR */
	  }
	else if(control_output >= V_MAX)	/* Max value 3.3 volt */
			{
	    	 tgr_reg = 0x0000;  /* corrisponde al min valore di TGR */
			}
		else if(channel == 1 || channel == 2) 		tgr_reg = TGRA_3_VAL - tgr_reg; /* valore per TGRB_3 or TGRD_3 */
			  else if(channel == 3 || channel == 4) tgr_reg = TGRA_4_VAL - tgr_reg; /* valore per TGRB_4 or TGRD_4 */
 
  if(channel == 1) /* Coppia TGRA-TGRB 3 */
   {
    /* imposta il nuovo valore del duty cycle */
    
    MTU3.TGRB = tgr_reg;  /* new TGRB_3 => change Duty Cycle */
    
   } else if(channel == 2) /* coppia TGRC-TGRD 3 */
   		  {
		    /* imposta il nuovo valore del duty cycle */
		    
		    MTU3.TGRD = tgr_reg;  /* new TGRD_3 => change Duty Cycle */
		    	
		  } else if(channel == 3) /* coppia TGRA-TGRB 4 */
   		  		{
		    	/* imposta il nuovo valore del duty cycle */
		    	
		    	MTU4.TGRB = tgr_reg;  /* new TGRB_4 => change Duty Cycle */
	    		
		  		} else if(channel == 4) /* coppia TGRC-TGRD 4 */
			   		  	{
					    	/* imposta il nuovo valore del duty cycle */
					    	MTU4.TGRD = tgr_reg;  /* new TGRD_4 => change Duty Cycle */
			   		  	}
}

/*
*********************************************************************************************************
*                                  PWM SIGNAL GENERATION 
*
* Description   : Converte il duty cycle in un valore numerico per il registro di comparazione TGR, 
*
* Arguments     : 'duty_cycle' in percentuale (ad esempio 65.5) e numero del canale 'channel' 1,2,3,4
*
* Returns       : None
*********************************************************************************************************
*/
 
void DutyCycle (float duty_cycle, unsigned char channel) /* Duty in percentuale, ad esempio 65.5 */
{
 unsigned short int tgr_reg; /* Valore per il registro TGR-1-3-4 (B o D) */

  if(channel == 1 || channel == 2)
    {
	 /* Puo' essere zero in relazione al valore di TGRA_3_VAL e di duty_cycle */
	 tgr_reg = (unsigned short)(((float)TGRA_3_VAL * (float)duty_cycle) / (float)100);
	}

  else if(channel == 3 || channel == 4)
			 {
	 		  /* Puo' essere zero in relazione al valore di TGRA_4_VAL e di duty_cycle */
	 		  tgr_reg = (unsigned short)(((float)TGRA_4_VAL * (float)duty_cycle) / (float)100);
			 }

	if(tgr_reg == 0 || duty_cycle <= 0)  /* min 0% */
		tgr_reg = 0xFFFF;  /* max value for TGR */

    else  if (duty_cycle >= 100)/* max 100% */
        	tgr_reg = 0x0000;  /* min value for TGR */

		  else if(channel == 1 || channel == 2) tgr_reg = TGRA_3_VAL - tgr_reg; /* valore per TGRB_3 or TGRD_3 */

			   else if(channel == 3 || channel == 4)
				   tgr_reg = TGRA_4_VAL - tgr_reg; /* valore per TGRB_4 or TGRD_4 */

	if(channel == 1)
	{
	 /* imposta il nuovo valore del duty cycle*/
     MTU3.TGRB = tgr_reg;  /* new TGRB_3 => change Duty Cycle */
	}
	else if(channel == 2)
			{
		     /* imposta il nuovo valore del duty cycle */
		     MTU3.TGRD = tgr_reg;  /* new TGRD_3 => change Duty Cycle */
			}
			else if(channel == 3)
					{
		     		 /* imposta il nuovo valore del duty cycle */
		     		 MTU4.TGRB = tgr_reg;  /* new TGRB_4 => change Duty Cycle */
					}
					else if(channel == 4)
							{
		     				 /* imposta il nuovo valore del duty cycle */
		     				 MTU4.TGRD = tgr_reg;  /* new TGRD_4 => change Duty Cycle */
							}
}

/* FUNZIONI DI PILOTAGGIO DELLA DIREZIONE DEI MOTORI */
void Init_Port_Dir(void)	// Inizializzazione delle porte di Direzione
{
	// Motor 1 (Connector JN2 - PIN 19)
	PORTD.PDR.BIT.B6 = 1; //OUTPUT
	PORTD.PMR.BIT.B6 = 0; //porta di io
	PORTD.PODR.BIT.B6 = 0; //orario
	
	// Motor 2 (Connector JN2 - PIN 16)
	PORTD.PDR.BIT.B3 = 1; //OUTPUT
	PORTD.PMR.BIT.B3 = 0; //GPIO
	PORTD.PODR.BIT.B3 = 0; //orario
	
	// Motor 3 (Connector JN2 - PIN 7)
	PORTA.PDR.BIT.B7 = 1; //OUTPUT
	PORTA.PMR.BIT.B7 = 0; //PA7
	PORTA.PODR.BIT.B7 = 0; //orario
}

// Funzione per il cambio di direzione dei motori
void change_motor_dir(int num_motor, int direction)
{
	switch(num_motor)
	{
		case(1):
		PORTD.PODR.BIT.B6 = direction; // DIR Port Motor 1 - PD6 (Connector JN2 - PIN 19)
		break;
		
		case(2):
		PORTD.PODR.BIT.B3 = direction; // DIR Port Motor 2 - PD3 (Connector JN2 - PIN 16)
		break;
		
		case(3):
		PORTA.PODR.BIT.B7 = direction; // DIR Port Motor 3 - PA7 (Connector JN2 - PIN 7)
		break;
	}
}

/*******************************************************************************
* Function name: DutyCycle_to_Motor
* Description  : Duty_Cycle calculation function for each motor PWM
* Arguments    : none
* Return value : none
*******************************************************************************/
void DutyCycle_to_Motor(void)
{
	// Calcolo del Duty-Cycle per il motore 1
	duty1 = ((volt_signal_1/Max_Volt)*100.0);	// Max Voltage : 100% = Read Current : x

	if (duty1 > Max_Duty)
		duty1 = Max_Duty;
	DutyCycle(duty1, 4);	//Il duty1(in percentuale) viene inoltrato nel canale 4 ovvero verso il motore 1

	change_motor_dir(1, val1);	// Determinazione del verso di rotazione del motore
								// (0: verso orario, 1: verso antiorario)

	// Calcolo del Duty-Cycle per il motore 2
	duty2 = ((volt_signal_2/Max_Volt)*100.0);
	if (duty2 > Max_Duty)
		duty2 = Max_Duty;
	DutyCycle(duty2, 2); //Il duty2(in percentuale) viene inoltrato nel canale 2 ovvero verso il motore 2

	change_motor_dir(2, val2);	// Determinazione del verso di rotazione del motore
								// (0: verso orario, 1: verso antiorario)

	// Calcolo del Duty-Cycle per il motore 3
	duty3 = ((volt_signal_3)/Max_Volt)*100.0;
	if (duty3 > Max_Duty)
		duty3 = Max_Duty;
	DutyCycle(duty3, 3); //Il duty3(in percentuale) viene inoltrato nel canale 3 ovvero verso il motore 3

	change_motor_dir(3, val3);	// Determinazione del verso di rotazione del motore
								// (0: verso orario, 1: verso antiorario)
}

/*******************************************************************************
* Function name: motor_direction
* Description  : Control function for motors rotation's direction
* Arguments    : none
* Return value : none
*******************************************************************************/
void motor_direction(void)
{
	// Controllo del verso di rotazione del motore 1
	// Se il valore della tensione è negativo viene reso positivo e impostato un flag a 1 per segnalare
	// la rotazione antioraria del motore
	if (volt_signal_1 < 0)
	{
		volt_signal_1 = volt_signal_1*(-1.0);
		val1 = 1;
	}
	else val1 = 0;

	// Controllo del verso di rotazione del motore 2
	if (volt_signal_2 < 0)
	{
		volt_signal_2 = volt_signal_2*(-1.0);
		val2 = 1;
	}
	else val2 = 0;

	// Controllo del verso di rotazione del motore 3
	if (volt_signal_3 < 0)
	{
		volt_signal_3 = volt_signal_3*(-1.0);
		val3 = 1;
	}
	else val3 = 0;
}
