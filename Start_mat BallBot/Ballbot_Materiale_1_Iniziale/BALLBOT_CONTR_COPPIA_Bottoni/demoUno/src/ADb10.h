
/*******************************************************************************
* File Name    : ADb10.h
* Version      : 1.00
* Description  : Simple driver for ADb10 (10-bit A/D converter)
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 05.05.2015     1.00        First release
*******************************************************************************/
#ifndef _ADb10_H_
#define _ADb10_H_
#endif

/*******************************************************************************
Macro definitions
*******************************************************************************/
/* Values for conversion of AD10 counts to voltage */
#define MAX_COUNTS 1023.0 // Massimo valore in bit del valore restituito dal convertitore
#define VMAX 3.3 // Voltaggio massimo in ingresso alla scheda
#define VMIN 0.0 // Voltaggio minimo in ingresso alla scheda
#define length 1000 // Dimensione del campione per la media dei valori letti dai sensori
#define denom_tot 500500 // Denominatore per il calcolo della media
#include <stdbool.h>

/*******************************************************************************
Prototypes for exported functions
*******************************************************************************/
void ADb10_init (void);
void ADb10_start (void);
float filtraggio (float read,int contatore,float vettore_vout[]);
float ADb10_read(int sens_channel) ;
bool ADb10_conversion_complete (void);
int Gestisci_ADb10 (void);
void vettore_vout_init(void);

/* End of file ADb10.h */
