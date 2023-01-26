/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer *
* Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.    
*******************************************************************************/
/******************************************************************************
* File Name     : CMT.h
* Version       : 1.0
* Description   : Header file for CMT.c
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 19.01.2012     1.00        First release
*******************************************************************************/

#ifndef _CMT_H_             /* Multiple inclusion prevention. */
#define _CMT_H_

/*******************************************************************************
 Prototypes for exported functions
*******************************************************************************/
void CMT_init (void) ;
void ms_delay(unsigned int ms);

/*******************************************************************************
 Exported variables
*******************************************************************************/

extern unsigned int general_timer_mS;
extern int timer_10mS;
extern int timer_100mS;
extern int timer_500mS;
extern int timer_1S;

#endif                       /* Multiple inclusion prevention. */