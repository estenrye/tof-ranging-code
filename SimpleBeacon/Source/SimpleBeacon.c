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
#define DEMO_PAN_ID                       0x0e1c

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
		uint64 u64DestAddr;
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
	/* to be implemented */
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
	/* to be implemented */
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
	/* to be implemented */
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
	/* to be implemented */
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
	/* to be implemented */
}