/*
 * sensore.h
 *
 *  Created on: 11 dic 2017
 *      Author: nicoe
 */

#ifndef SRC_SENSORE_H_
#define SRC_SENSORE_H_
#endif /* SRC_SENSORE_H_ */

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define calibration_length 1000
#define sensitivity 0.000185			// Sensibilita'  del sensore di corrente (V/mA)
#define offset_vout 2.5					// Offset in volt corrispondente a 0A (V)

/*******************************************************************************
Prototypes for exported functions
*******************************************************************************/
void sens_calibration_init (void);
void sens_calibration_bias (void);
void sens_read (void);

/* End of file sensore.h */
