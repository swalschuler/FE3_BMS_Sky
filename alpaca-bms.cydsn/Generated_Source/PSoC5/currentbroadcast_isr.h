/*******************************************************************************
* File Name: currentbroadcast_isr.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_currentbroadcast_isr_H)
#define CY_ISR_currentbroadcast_isr_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void currentbroadcast_isr_Start(void);
void currentbroadcast_isr_StartEx(cyisraddress address);
void currentbroadcast_isr_Stop(void);

CY_ISR_PROTO(currentbroadcast_isr_Interrupt);

void currentbroadcast_isr_SetVector(cyisraddress address);
cyisraddress currentbroadcast_isr_GetVector(void);

void currentbroadcast_isr_SetPriority(uint8 priority);
uint8 currentbroadcast_isr_GetPriority(void);

void currentbroadcast_isr_Enable(void);
uint8 currentbroadcast_isr_GetState(void);
void currentbroadcast_isr_Disable(void);

void currentbroadcast_isr_SetPending(void);
void currentbroadcast_isr_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the currentbroadcast_isr ISR. */
#define currentbroadcast_isr_INTC_VECTOR            ((reg32 *) currentbroadcast_isr__INTC_VECT)

/* Address of the currentbroadcast_isr ISR priority. */
#define currentbroadcast_isr_INTC_PRIOR             ((reg8 *) currentbroadcast_isr__INTC_PRIOR_REG)

/* Priority of the currentbroadcast_isr interrupt. */
#define currentbroadcast_isr_INTC_PRIOR_NUMBER      currentbroadcast_isr__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable currentbroadcast_isr interrupt. */
#define currentbroadcast_isr_INTC_SET_EN            ((reg32 *) currentbroadcast_isr__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the currentbroadcast_isr interrupt. */
#define currentbroadcast_isr_INTC_CLR_EN            ((reg32 *) currentbroadcast_isr__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the currentbroadcast_isr interrupt state to pending. */
#define currentbroadcast_isr_INTC_SET_PD            ((reg32 *) currentbroadcast_isr__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the currentbroadcast_isr interrupt. */
#define currentbroadcast_isr_INTC_CLR_PD            ((reg32 *) currentbroadcast_isr__INTC_CLR_PD_REG)


#endif /* CY_ISR_currentbroadcast_isr_H */


/* [] END OF FILE */
