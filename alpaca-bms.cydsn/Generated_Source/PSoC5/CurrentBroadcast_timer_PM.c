/*******************************************************************************
* File Name: CurrentBroadcast_timer_PM.c
* Version 2.70
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "CurrentBroadcast_timer.h"

static CurrentBroadcast_timer_backupStruct CurrentBroadcast_timer_backup;


/*******************************************************************************
* Function Name: CurrentBroadcast_timer_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  CurrentBroadcast_timer_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void CurrentBroadcast_timer_SaveConfig(void) 
{
    #if (!CurrentBroadcast_timer_UsingFixedFunction)
        CurrentBroadcast_timer_backup.TimerUdb = CurrentBroadcast_timer_ReadCounter();
        CurrentBroadcast_timer_backup.InterruptMaskValue = CurrentBroadcast_timer_STATUS_MASK;
        #if (CurrentBroadcast_timer_UsingHWCaptureCounter)
            CurrentBroadcast_timer_backup.TimerCaptureCounter = CurrentBroadcast_timer_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!CurrentBroadcast_timer_UDB_CONTROL_REG_REMOVED)
            CurrentBroadcast_timer_backup.TimerControlRegister = CurrentBroadcast_timer_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: CurrentBroadcast_timer_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  CurrentBroadcast_timer_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void CurrentBroadcast_timer_RestoreConfig(void) 
{   
    #if (!CurrentBroadcast_timer_UsingFixedFunction)

        CurrentBroadcast_timer_WriteCounter(CurrentBroadcast_timer_backup.TimerUdb);
        CurrentBroadcast_timer_STATUS_MASK =CurrentBroadcast_timer_backup.InterruptMaskValue;
        #if (CurrentBroadcast_timer_UsingHWCaptureCounter)
            CurrentBroadcast_timer_SetCaptureCount(CurrentBroadcast_timer_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!CurrentBroadcast_timer_UDB_CONTROL_REG_REMOVED)
            CurrentBroadcast_timer_WriteControlRegister(CurrentBroadcast_timer_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: CurrentBroadcast_timer_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  CurrentBroadcast_timer_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void CurrentBroadcast_timer_Sleep(void) 
{
    #if(!CurrentBroadcast_timer_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(CurrentBroadcast_timer_CTRL_ENABLE == (CurrentBroadcast_timer_CONTROL & CurrentBroadcast_timer_CTRL_ENABLE))
        {
            /* Timer is enabled */
            CurrentBroadcast_timer_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            CurrentBroadcast_timer_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    CurrentBroadcast_timer_Stop();
    CurrentBroadcast_timer_SaveConfig();
}


/*******************************************************************************
* Function Name: CurrentBroadcast_timer_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  CurrentBroadcast_timer_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void CurrentBroadcast_timer_Wakeup(void) 
{
    CurrentBroadcast_timer_RestoreConfig();
    #if(!CurrentBroadcast_timer_UDB_CONTROL_REG_REMOVED)
        if(CurrentBroadcast_timer_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                CurrentBroadcast_timer_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
