/****************************************************************************
 *
 * MODULE:             JenNet Home Sensor Demo
 *
 * COMPONENT:          $RCSfile: HomeSensorEndDevice.c,v $
 *
 * VERSION:            $Name: $
 *
 * REVISION:           $Revision: 1.10 $
 *
 * DATED:              $Date: 2009-08-06 09:27:46 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             $Author:  $
 *
 * DESCRIPTION:
 *
 * LAST MODIFIED BY:   thayd
 *                     $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2008. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <string.h>
#include "Button.h"
#include "LedControl.h"
#include "Jenie.h"
#include "Utils.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define SLEEP_PERIOD            10      /* Units of 10 mS */
#define REGISTER_FLASH_RATE     10
#define RUNNING_FLASH_RATE      8
#define RUNNING_TRANSMIT_RATE   5
#define DELAY_PERIOD            3200

/* define LED positions  */
#define LED1                        0
#define LED2                        1

/* define if using high power modules */
//#define HIGH_POWER TRUE

/* networking values */
#define DEMO_PAN_ID                       0x0e1c

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* Button values */
typedef enum
{
    E_KEY_0 = BUTTON_0_MASK,
    E_KEY_1 = BUTTON_1_MASK
} teKeyValues;

/* All application data with scope within the entire file is kept here, */
typedef struct
{
    uint64 u64DestAddr;
    uint64 u64ParentAddr;
    bool_t bAppTimerStarted;
    bool_t bStackReady;
    uint8 eAppState;
} tsHomeData;

typedef enum
{
    E_STATE_OFF,
    E_STATE_REGISTER,
    E_STATE_RUNNING
}teAppState;
/* All variables with scope throughout module are in one structure */
typedef struct
{
    /* Transceiver (basically anything TX/RX not covered elsewhere) */
    struct
    {
        uint8   u8CurrentTxHandle;
        uint8   u8PrevRxBsn;
    } sTransceiver;




    /* System (state, assigned address, channel) */
    struct
    {
        teAppState eState;
        uint16  u16ShortAddr;
        uint8   u8ThisNode;
        uint8   u8Channel;
    } sSystem;
} tsDemoData;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* File scope data */
PRIVATE tsHomeData sHomeData;
PRIVATE tsDemoData sDemoData;

PRIVATE bool_t bTimeOut;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/* Stack to application callback functions */
/****************************************************************************
 *
 * NAME: vJenie_ConfigureNetwork
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system.
 *
 * RETURNS:
 * Nothing
 *
 ****************************************************************************/
PUBLIC void vJenie_CbConfigureNetwork(void)
{
    /* Set PAN_ID and other network stuff or defaults will be used */
    gJenie_NetworkApplicationID =   0xdeaddead;
    gJenie_PanID                =   DEMO_PAN_ID;
    gJenie_EndDevicePollPeriod  =   10;
    gJenie_EndDeviceScanSleep   =   100;

    gJenie_RoutingEnabled       = FALSE;
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{

    vUtils_Init();

    if(bWarmStart==FALSE)
    {
        (void)u32AHI_Init();
        sHomeData.bStackReady=FALSE;
        /* Initialise buttons, LEDs and program variables */
        /* Set DIO for buttons and LEDs */
        vLedControl(LED1, FALSE);
        vLedControl(LED2, FALSE);
        vLedInitRfd();
        vButtonInitRfd();

        #ifdef NO_SLEEP
            vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);
        #endif


        sHomeData.eAppState = E_STATE_REGISTER;
        switch(eJenie_Start(E_JENIE_END_DEVICE))        /* Start network as end device */
        {
        case E_JENIE_SUCCESS:
			vUtils_Debug("Jenie Started");
			vUtils_Debug("E_JENIE_SUCCESS");
            #ifdef HIGH_POWER
                /* Set high power mode */
                eJenie_RadioPower(18, TRUE);
            #endif
            break;

        case E_JENIE_ERR_UNKNOWN:
			vUtils_Debug("E_JENIE_ERR_UNKNOWN");
			break;
        case E_JENIE_ERR_INVLD_PARAM:
			vUtils_Debug("E_JENIE_ERR_INVLD_PARAM");
			break;
        case E_JENIE_ERR_STACK_RSRC:
			vUtils_Debug("E_JENIE_ERR_STACK_RSRC");
			break;
        case E_JENIE_ERR_STACK_BUSY:
			vUtils_Debug("E_JENIE_ERR_STACK_BUSY");
			break;

        default:
			vUtils_Debug("Unknown eJenie_Start Status Code");
			break;
        }
    }else{


        switch(eJenie_Start(E_JENIE_END_DEVICE))        /* Start network as end device */
        {
        case E_JENIE_SUCCESS:
			vUtils_Debug("E_JENIE_SUCCESS");
            #ifdef HIGH_POWER
                /* Set high power mode */
                eJenie_RadioPower(18, TRUE);
            #endif
            break;

        case E_JENIE_ERR_UNKNOWN:
			vUtils_Debug("E_JENIE_ERR_UNKNOWN");
			break;
        case E_JENIE_ERR_INVLD_PARAM:
			vUtils_Debug("E_JENIE_ERR_INVLD_PARAM");
			break;
        case E_JENIE_ERR_STACK_RSRC:
			vUtils_Debug("E_JENIE_ERR_STACK_RSRC");
			break;
        case E_JENIE_ERR_STACK_BUSY:
			vUtils_Debug("E_JENIE_ERR_STACK_BUSY");
			break;

        default:
			vUtils_Debug("Unknown eJenie_Start Status Code");
			break;
        }

    }
    /* set watchdog to long timeout - override setting in JenNet startup */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogStart(254);
    #endif

}

/****************************************************************************
 *
 * NAME: vJenie_Main
 *
 * DESCRIPTION:
 * Main user routine. This is called by the Basic Operating System (BOS)
 * at regular intervals.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbMain(void)
{
    static bool phase=0;
    static uint32 loop_count;

    /* regular watchdog reset */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogRestart();
    #endif

    if(sHomeData.bStackReady && bTimeOut)       // Stack up and running and waiting for us to do something
    {
        switch(sHomeData.eAppState)
        {
        case E_STATE_REGISTER:
			vUtils_Debug("E_STATE_REGISTER");
            if(loop_count % REGISTER_FLASH_RATE == 0)
            {
                vLedControl(LED1,phase);
                phase ^= 1;
                #ifdef NO_SLEEP
                    /* Manually poll parent as not sleeping */
                    (void)eJenie_PollParent();
                #endif
            }
            break;

        case E_STATE_RUNNING:
			vUtils_Debug("E_STATE_RUNNING");
            if(loop_count % RUNNING_FLASH_RATE == 0)
            {
                vLedControl(LED1,phase);
                phase ^= 1;
            }
            if(loop_count % RUNNING_TRANSMIT_RATE == 0)
            {
            }
            break;

        default:
			vUtils_Debug("Unknown State");
            break;
        }
        loop_count--;

        #ifdef NO_SLEEP
            vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_1, DELAY_PERIOD);
            bTimeOut = FALSE;
        #else
            eJenie_SetSleepPeriod(SLEEP_PERIOD * 10);
            eJenie_Sleep(E_JENIE_SLEEP_OSCON_RAMON);
        #endif
    }


}

/****************************************************************************
 *
 * NAME: vJenie_StackMgmtEvent
 *
 * DESCRIPTION:
 * Used to receive stack management events
 *
 * PARAMETERS:      Name                    RW  Usage
 *                  *psStackMgmtEvent       R   Pointer to event structure
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbStackMgmtEvent(teEventType eEventType, void *pvEventPrim)
{
    switch(eEventType)
    {
    case E_JENIE_NETWORK_UP:
		vUtils_Debug("E_JENIE_NETWORK_UP");
        sHomeData.u64ParentAddr = ((tsNwkStartUp*)pvEventPrim)->u64ParentAddress;
		vUtils_DisplayMsg("New parent:",(uint32)sHomeData.u64ParentAddr);
		vUtils_Debug("Network Up");
        sHomeData.bStackReady=TRUE;
        bTimeOut=TRUE;
        break;

    case E_JENIE_REG_SVC_RSP:
		vUtils_Debug("E_JENIE_REG_SVC_RSP");
        break;

    case E_JENIE_SVC_REQ_RSP:
		vUtils_Debug("E_JENIE_SVC_REQ_RSP");
        break;

    case E_JENIE_POLL_CMPLT:
		vUtils_Debug("E_JENIE_POLL_CMPLT");
        break;

    case E_JENIE_PACKET_SENT:
		vUtils_Debug("E_JENIE_PACKET_SENT");
        break;

    case E_JENIE_PACKET_FAILED:
		vUtils_Debug("E_JENIE_PACKET_FAILED");
        break;

    case E_JENIE_CHILD_JOINED:
        vUtils_Debug("E_JENIE_CHILD_JOINED");
        vUtils_DisplayMsg("Child Joined: ",(uint32)(((tsChildJoined*)pvEventPrim)->u64SrcAddress));
        tsChildJoined *joinEvent = ((tsChildJoined*) pvEventPrim);
        break;

    case E_JENIE_CHILD_LEAVE:
        vUtils_Debug("E_JENIE_CHILD_LEAVE");
        vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildLeave*)pvEventPrim)->u64SrcAddress));
        break;

    case E_JENIE_CHILD_REJECTED:
        vUtils_Debug("E_JENIE_CHILD_REJECTED");
        vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildRejected*)pvEventPrim)->u64SrcAddress));            
        break;

    case E_JENIE_STACK_RESET:
		vUtils_Debug("E_JENIE_STACK_RESET");
		vUtils_Debug("Stack Reset");
        sHomeData.bStackReady = FALSE;
        sHomeData.eAppState = E_STATE_REGISTER;
        break;

    default:
        /* Unknown management event type */
		vUtils_Debug("Unknown Management Event");
        break;
    }
}





/****************************************************************************
 *
 * NAME: vJenie_StackDataEvent
 *
 * DESCRIPTION:
 * Used to receive stack data events
 *
 * PARAMETERS:      Name                    RW  Usage
 *                  *psStackDataEvent       R   Pointer to data structure
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbStackDataEvent(teEventType eEventType, void *pvEventPrim)
{
    switch(eEventType)
    {
    case E_JENIE_DATA:
		vUtils_Debug("E_JENIE_DATA");
        break;

    case E_JENIE_DATA_TO_SERVICE:
		vUtils_Debug("E_JENIE_DATA_TO_SERVICE");
        break;

    case E_JENIE_DATA_ACK:
		vUtils_Debug("E_JENIE_DATA_ACK");
        /* Update current state on success*/
        if (sHomeData.eAppState == E_STATE_REGISTER)
        {
			vUtils_Debug("Registered");
            sHomeData.eAppState = E_STATE_RUNNING;
        }
    break;

    case E_JENIE_DATA_TO_SERVICE_ACK:
		vUtils_Debug("E_JENIE_DATA_TO_SERVICE_ACK");
        break;

    default:
        /*Unknown data event type */
        #ifdef DEBUG
            vUtils_Debug("Unknown Data Event");
        #endif
        break;
    }
}


/****************************************************************************
 *
 * NAME: vJenie_HwEvent
 *
 * DESCRIPTION:
 * Adds events to the hardware event queue.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32Device       R   Peripheral responsible for interrupt e.g DIO
 *                  u32ItemBitmap   R   Source of interrupt e.g. DIO bit map
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vJenie_CbHwEvent(uint32 u32DeviceId,uint32 u32ItemBitmap)
{
	if ( (u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_SYSCTRL_WK1_MASK) )
    {
        bTimeOut = TRUE;
    }
}

