/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <string.h>
#include "LcdDriver.h"
#include "AlsDriver.h"
#include "HtsDriver.h"
#include "Button.h"
#include "LedControl.h"
#include "Jenie.h"
#include "Utils.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define SLEEP_PERIOD            10      /* Units of 10 mS */
#define DELAY_PERIOD            3200
#define NO_SLEEP TRUE

#define DEMO_PAN_ID                       0x0e1c
/* define LED positions  */
#define LED1                        0
#define LED2                        1
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    E_STATE_OFF,
    E_STATE_REGISTER,
    E_STATE_RUNNING
}teAppState;

/* All application data with scope within the entire file is kept here, */
typedef struct
{
    struct
	{
		uint64 u64LocalAddr;
		uint64 u64ParentAddr;
		bool_t bAppTimerStarted;
		bool_t bStackReady;
		uint8 eAppState;
	} sHome;

    /* System (state, assigned address, channel) */
    struct
    {
        teAppState eState;
        uint16  u16ShortAddr;
        uint8   u8ThisNode;
        uint8   u8Channel;
    } sSystem;
} tsApplicationState;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsApplicationState sAppState;
PRIVATE bool_t bTimeOut;

/****************************************************************************/
/***        Method Declarations                                           ***/
/****************************************************************************/
// JenOS Required Functions:
PUBLIC void vJenie_CbConfigureNetwork(void);
PUBLIC void vJenie_CbInit(bool_t bWarmStart);
PUBLIC void vJenie_CbMain(void);
PUBLIC void vJenie_CbHwEvent(uint32 u32DeviceId,uint32 u32ItemBitmap);
PUBLIC void vJenie_CbStackMgmtEvent(teEventType eEventType, void *pvEventPrim);
PUBLIC void vJenie_CbStackDataEvent(teEventType eEventType, void *pvEventPrim);


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
	vUtils_Init();
	vUtils_Debug("vJenie_CbConfigureNetwork");
    /* Set PAN_ID and other network stuff or defaults will be used */
    gJenie_NetworkApplicationID =   0xdeaddead;
    gJenie_PanID                =   DEMO_PAN_ID;
    gJenie_EndDevicePollPeriod  =   10;
    gJenie_EndDeviceScanSleep   =   100;

    gJenie_RoutingEnabled       = FALSE;
	vUtils_Debug("exiting vJenie_CbConfigureNetwork");
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
    vUtils_Debug("vJenie_CbInit");
    if(bWarmStart==FALSE)
    {
		(void)u32AHI_Init();
		sAppState.sSystem.eState = E_STATE_OFF;
		sAppState.sSystem.u16ShortAddr = 0xffff;
		sAppState.sSystem.u8ThisNode = 0;

        vLedControl(LED1, FALSE);
        vLedControl(LED2, FALSE);
		vLedInitRfd();
	}
	sAppState.sHome.eAppState = E_STATE_REGISTER;
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
		vUtils_Debug("default");
		break;
	}

	/* set watchdog to long timeout - override setting in JenNet startup */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogStart(254);
    #endif
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

    if(sAppState.sHome.bStackReady && bTimeOut)       // Stack up and running and waiting for us to do something
    {
        switch(sAppState.sHome.eAppState)
        {
			case E_STATE_REGISTER:
				vUtils_Debug("E_STATE_REGISTER");
				// if(loop_count % REGISTER_FLASH_RATE == 0)
				// {
					// vTxRegister();
					vLedControl(LED2,phase);
					phase ^= 1;
					#ifdef NO_SLEEP
						/* Manually poll parent as not sleeping */
						(void)eJenie_PollParent();
					#endif
				// }
				break;

			case E_STATE_RUNNING:
				vUtils_Debug("E_STATE_RUNNING");
				// vProcessRead();
				// if(loop_count % RUNNING_FLASH_RATE == 0)
				// {
				// 	vLedControl(LED1,phase);
				// 	phase ^= 1;
				// }
				// if(loop_count % RUNNING_TRANSMIT_RATE == 0)
				// {
				// 	vProcessTxData();
				// }
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
			sAppState.sHome.u64ParentAddr = ((tsNwkStartUp*)pvEventPrim)->u64ParentAddress;
			sAppState.sHome.u64LocalAddr = ((tsNwkStartUp*)pvEventPrim)->u64LocalAddress;
			sAppState.sSystem.u8Channel = ((tsNwkStartUp*)pvEventPrim)->u8Channel;

			vUtils_DisplayMsg("New parent:", (uint32)sAppState.sHome.u64ParentAddr);
			vUtils_DisplayMsg("Local address:", (uint32)sAppState.sHome.u64LocalAddr);
			vUtils_DisplayMsg("Channel:", (uint32)sAppState.sSystem.u8Channel);
			vUtils_Debug("Network Up");
			sAppState.sHome.bStackReady=TRUE;
			sAppState.sHome.eAppState=E_STATE_RUNNING;
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
			break;

		case E_JENIE_CHILD_LEAVE:
			vUtils_Debug("E_JENIE_CHILD_LEAVE");
			vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildLeave*)pvEventPrim)->u64SrcAddress));
			break;
		
		case E_JENIE_STACK_RESET:
			vUtils_Debug("E_JENIE_STACK_RESET");
			break;
		
		case E_JENIE_CHILD_REJECTED:
				vUtils_Debug("E_JENIE_CHILD_REJECTED");
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
    switch (eEventType)
    {
        case E_JENIE_DATA:
            vUtils_Debug("E_JENIE_DATA");    
            // vProcessIncomingData(((tsData*)pvEventPrim));
            break;

        case E_JENIE_DATA_TO_SERVICE:
            vUtils_Debug("E_JENIE_DATA_TO_SERVICE");
            break;

        case E_JENIE_DATA_ACK:
            vUtils_Debug("E_JENIE_DATA_ACK");
            break;

        case E_JENIE_DATA_TO_SERVICE_ACK:
            vUtils_Debug("E_JENIE_DATA_TO_SERVICE_ACK");
            break;

        default:
            vUtils_Debug("default");
            break;
    }
}