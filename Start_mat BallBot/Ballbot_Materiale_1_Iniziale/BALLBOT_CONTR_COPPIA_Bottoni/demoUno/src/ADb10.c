
    /*******************************************************************************
    * File Name     : ADb10.C
    * Version       : 1.0
    * H/W Platform  : YRDKRX63N
    * Description   : Driver for the ADb10 (10-bit A/D converter)
    *                 This implementation sets up ADb10 in single-scan mode.
    *                 It scans AN4,AN5.
    *******************************************************************************/

	/******************************************************************************
	* History : DD.MM.YYYY 	 Version 	Description  	 Author
	*     	: xx.12.2014 	 1.00    	First release 	 Giuseppe Antonio Scala
          	: 05.05.2015   	 Comment and fix
          	: 08.09.2016  2.00  Register added  JuaLo
	*******************************************************************************/

	/******************************************************************************
	Includes   <System Includes> , "Project Includes"
	*******************************************************************************/
	#include <stdint.h>
	#include <stdio.h>
	#include <stdbool.h>
	#include "platform.h"
	#include "ADb10.h"

	// Dichiarazione vettori dei valori letti dai sensori di corrente e contatori
    float vettore_vout_1[length];   //vettore di uscita per ogni sensore, dato che abbiamo tre sensori ci sono 3 vettori formati da lenght elementi, cioè lenght misurazioni date da lenght
    float vettore_vout_2[length];
    float vettore_vout_3[length];
    float contatore_1 = 0;    //Contatori per filtro a media mobile.
    float contatore_2 = 0;
    float contatore_3 = 0;

	/*******************************************************************************
	* Function name: ADb10_init
	* Description  : Sets up ADb10 in single scan mode.
	*	      	Used to read AN4,AN5.
	* Arguments	: none
	* Return value : none
	*******************************************************************************/
	void ADb10_init (void)  //Inizializza il sensore di corrente
	{
    	/* Abilita la periferica ADC per consentirne l’uso. (la periferica deve essere obbligatoriamente abilitata prima dell’uso.)
        * Per abilitare l’ADb (Convertitore analogico/digitale)
    	* Il registro MSTP del convertitore A/D dev’essere abilitato, perché
    	* questo registro è protetto dalla scrittura dal registro di protezione PRCR,
    	* prima dell’attivazione del MSTP(ADb) è necessario:
    	* a) settare ad OFF il PRCR,
    	* b) abilitare il registro ADb10
    	* c) settare ad ON il PRCR.
    	*/

	/* Disabilitare la protezione del registro PRCR */
	#ifdef PLATFORM_BOARD_RDKRX63N
    	SYSTEM.PRCR.WORD = 0xA50B; // Protect off (ref. Hardware Manual Chapt.13.1.1)
                               	// write A5 (in hexadecimal) to the eight higher-order
                               	// bits and 0B (in hexadecimal) to the eight lower-order
                               	// where B in hexadecimal is equivalent to 1011 in Binary
                               	// therefore it sets PRC3, PRC1 and PRC0 to 1
	#endif
    	/* Disable Module Stop Function (MSTP) of timer ADb10 */
    	/* Power up the ADb10 */
    	MSTP(AD) = 0;
    	/* SELECTION OF ANALOG PORT OF MCU TO BE USED AS SIGNAL INPUT */
    	/* Set up the I/O pin that will be used as analog input source.
    	*  AN2  PE4  PMOD1 1
    	*  AN4  PE6  PMOD1 2
    	*  AN5  PE7  PMOD1 3
    	*/
    	// Porta AN2 (Connettore JN1 - PIN 16)
    	PORTE.PODR.BIT.B4 = 0;	// Clear I/O pin data register to low output.
    	PORTE.PDR.BIT.B4  = 0;	// Set I/O pin direction to input.
    	PORTE.PMR.BIT.B4  = 0;	// First set I/O pin mode register to GPIO mode.
    	MPC.PE4PFS.BYTE = 0x00;   // Set port function register to analog input, no interrupt.
    	// Porta AN4 (Connettore JN1 - PIN 18)
    	PORTE.PODR.BIT.B6 = 0;	// Clear I/O pin data register to low output.
    	PORTE.PDR.BIT.B6  = 0;	// Set I/O pin direction to input.
    	PORTE.PMR.BIT.B6  = 0;	// First set I/O pin mode register to GPIO mode.
    	MPC.PE6PFS.BYTE = 0x00;   // Set port function register to analog input, no interrupt.
    	// Porta AN5 (Connettore JN1 - PIN 19)
    	PORTE.PODR.BIT.B7 = 0;	// Clear I/O pin data register to low output.
    	PORTE.PDR.BIT.B7  = 0;	// Set I/O pin direction to input.
    	PORTE.PMR.BIT.B7  = 0;	// First set I/O pin mode register to GPIO mode.
    	MPC.PE7PFS.BYTE = 0x00;   // Set port function register to analog input, no interrupt.

	#ifdef PLATFORM_BOARD_RDKRX63N
    	SYSTEM.PRCR.WORD = 0xA500; // Protect on
                               	// write A5 (in hexadecimal) to the eight higher-order
                               	// bits and 00 (in hexadecimal) to the eight lower-order
                               	// where last 0 in hexadecimal is equivalent to 0000 in Binary
                               	// therefore it sets PRC3, PRC1 and PRC0 to 0
	#endif
    	/* INITIALIZATION OPERATIONS ON 10 BIT ADC PERIPHERAL(ref. Hardware Manual Chapt.43) */
    	/* ADCSR: A/D Control Register (ref. Hardware Manual Chapt.43.2.2)
    	*  b7	Reserved 0 This bit is always read as 0. The write value should always be 0.
    	*  b6	ADIE 	0 Interrupt
    	*  b5	ADST 	0 a/d conversion start, Stop a scan conversion process
    	*  b4:b3 Reserved 0 This bit is always read as 0. The write value should always be 0.
    	*  b2:b0 CH   	1 Select channels AN0 to AN5, in scan mode 101
    	*/
    	AD.ADCSR.BYTE = 0x05;
    	/* ADCR: A/D Control Register (ref. Hardware Manual Chapt.43.2.3)
    	*  b7:b5 TRGS 	0 set to software trigger
    	*  b4	Reserved 0 This bit is always read as 0. The write value should always be 0.
    	*  b3:b2 CKS  	0 A/D conversion clock select = PCLK/8
    	*  b1:b0 MODE 	1 Scan mode select, single scan mode 11 (reading enabled from AN0 channel to AN5)
    	*/
    	AD.ADCR.BYTE = 0x03;
    	/* ADCR2: A/D Channel Select Register 0
    	*  b7:b0   43.2.4
    	*/
    	AD.ADCR2.BYTE = 0x00;
    	/* ADDIAGR: A/D Channel Select Register 0
    	*  b7:b0   43.2.6
    	*/
    	AD.ADDIAGR.BYTE = 0x00;
	} /* End of function ADb10_init() */

	/*******************************************************************************
	* Function name: ADb10_start
	* Description  : Starts the ADC converter.  It will run continuously.
	* Arguments	: none
	* Return value : none
	*******************************************************************************/
	void ADb10_start (void)
	{
    	/* Start the A/D converter */
    	AD.ADCSR.BIT.ADST = 1;
	} /* End of function ADb10_start() */

	 /*****************************************************************************
	    * Function name : filtraggio
	    * Description   : Funzione per l' implementazione del filtro FIR sui valori letti
	    * 				  dal sensore di corrente, cioè fa la media pesata mobile continuamente, da 2 valori in su.
	    * 				  Arrivati a 1000 fa lo shift eliminando il primo valore preso,
	    * 				  aggiungendo l’ultimo valore letto, e fa la media.
	    * Arguments     : Valori letti dal registro convertiti in volt
	    * Return_value  : Media mobile dei valori del registro in volt
	    *******************************************************************************/
	float filtraggio (float read,int contatore,float vettore_vout[])
	    {
	      	int i=0;
	      	float somma = 0;
	      	float vout;
	      	if(contatore<length) //Ci dice se abbiamo raggiunto il limite lenght di valori
	      					{
	      						vettore_vout[contatore] = read; /*Nella variabile read sono contenuti i valori
	        	  	  	  	  	  	  	  	  	  	  	  	  	  letti nei vari registri dal sensore di
	        	  	  	  	  	  	  	  	  	  	  	  	  	  corrente opportuno */
	      						for(i=0; i<=contatore; i++)
	      						{
	      							somma += vettore_vout[i]*(i+1); //somma pesata di tutti i valori letti
	      						}

	      						vout =(float) somma / ((contatore+1)*(contatore+2)/2); /*assegniamo a vout
	      																	la media calcolata ponderata */
	      					}
	      					else // Eseguiamo lo shift, dato che abbiamo già esaminato lenght valori
	      					{
	      						for(i=1; i<length; i++)
	      						{
	      							vettore_vout[i-1]=vettore_vout[i]; /*Scambia ultimo valore letto
	  											        con il primo elemento del
	   											        vettore */
	      							somma += vettore_vout[i-1]*i;
	      						}
	      						vettore_vout[length-1] = read;
	      						somma += vettore_vout[length-1]*(i+1);
	      						vout = (float)somma/denom_tot;  //Guardare .h per spiegazione denom tot
	      					}
	      					return vout;
	      }

	/***********************************************************************************************
	* Function name : ADb10_read
	* Description : Legge i valori dai 3 registri dell' ADb10 per filtrarli
	* Arguments	: canale selezionato
	* Return value : il valore letto e filtrato in volt
	***********************************************************************************************/
	float ADb10_read (int sens_channel)
	{
    	float read_register, vout_1, vout_2, vout_3; /*read_register è la variabile nella quale sono inseriti i valori
    												   di tensione letti dai registri*/

        switch(sens_channel) //sens_channel è un intero che assume i valori 1, 2 o 3 a seconda del sensore di corrente considerato
        {
        // AD.ADDRC/E/F sono i registri dove vengono “salvati” i valori letti dal sensore di corrente
    case 1:
				read_register = ((AD.ADDRC*VMAX)/MAX_COUNTS); /*legge il risultato del registro per la porta AN2
																che corrisponde al pin 16 della JN1*/
				vout_1 = filtraggio(read_register, contatore_1, vettore_vout_1);
				contatore_1++;
				return vout_1;
				break;
	case 2:
				read_register = ((AD.ADDRE*VMAX)/MAX_COUNTS); /*legge il risultato del registro per la porta AN4
																che corrisponde al pin 18 della JN1*/
     			vout_2 = filtraggio(read_register, contatore_2, vettore_vout_2);
     			contatore_2++;
     			return vout_2;
     			break;
	case 3:
				read_register = ((AD.ADDRF*VMAX)/MAX_COUNTS); /*legge il risultato del registro per la porta AN5
																che corrisponde al pin 19 della JN1*/
				vout_3 = filtraggio(read_register, contatore_3, vettore_vout_3);
				contatore_3++;
				return vout_3;
				break;

	default: return 0;
				break;
			}
	} /* End of function ADb10_read() */

	/*******************************************************************************
	* Function name: ADb10_conversion_complete
	* Description  : Checks to see if the conversion is complete.
	* Arguments	: none
	* Return value : True -> Conversion is complete
	*            	False -> Otherwise
	*******************************************************************************/
	bool ADb10_conversion_complete()
	{
    	/* The ADST returns to zero when the conversion is complete  */
    	return AD.ADCSR.BIT.ADST == 0;
	} /* End of function ADb10_conversion_complete() */

	/*******************************************************************************
	* Function name: Gestisci_ADb10
	* Description  : Manage conversion without waiting cycle for it to end
	* Arguments	: none
	* Return value : state
	*******************************************************************************/
	int Gestisci_ADb10(void)   //Funzione che restituisce 1 solo quando la conversione A/D è completata.
	{                          //Serve solamente per restituire un livello logico 0 o 1.
    	static int stato = 0;
    	if(stato == 0)
    	{
        	/* Start the A/D converter */
        	ADb10_start();
        	stato = 1;
    	}
    	if(stato == 1)
    	{
        	/* Wait for the conversion to complete */
        	if(false == ADb10_conversion_complete())
        	{
            	stato = 1; // unchange state conversione non completata, lo stato rimane invariato.
        	}else
            	{
             	stato = 0; // initial state
             	return 1;
            	}
    	}
    	return 0;
	}

	/*******************************************************************************
	* Function name: vettore_vout_init
	* Description  : Average voltage values' vector initialization to 0
	* Arguments	: none
	* Return value : none
	*******************************************************************************/
    void vettore_vout_init(void) //inizializzazione dei vettori vout
    {
        for(int i = 0; i < length; i++)
        {
            vettore_vout_1[i] = 0;
            vettore_vout_2[i] = 0;
            vettore_vout_3[i] = 0;
        }
    }

	/*******************************************************************************
	End of file ADb10.c
	*******************************************************************************/
