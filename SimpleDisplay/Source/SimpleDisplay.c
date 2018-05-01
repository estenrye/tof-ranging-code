/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Block (time slice) values */
#define BLOCK_TIME_IN_32K_PERIODS   1600
#define MAX_BLOCKS                  20
#define BLOCK_WRITE_MAIN_SCREEN     10
#define ONE_MSEC_IN_32KHZ_CYCLES    32
#define BUTTON_DEBOUNCE             (500 * ONE_MSEC_IN_32KHZ_CYCLES)
/* PAN ID on which demo operates */
#define DEMO_PAN_ID                       0x0e1c
#define ENDPOINT_ADDR_BASE                0x0e01
/* Channels available */
#define CHANNEL_MIN                       11
#define CHANNEL_MID                       18
#define CHANNEL_MAX                       26

#define NUMBER_OF_BEACONS 2

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
/***        Method Declarations                                           ***/
/****************************************************************************/
// JenOS Required Functions:
PUBLIC void vJenie_CbConfigureNetwork(void);
PUBLIC void vJenie_CbInit(bool_t bWarmStart);
PUBLIC void vJenie_CbMain(void);
PUBLIC void vJenie_CbHwEvent(uint32 u32DeviceId,uint32 u32ItemBitmap);
PUBLIC void vJenie_CbStackMgmtEvent(teEventType eEventType, void *pvEventPrim);
PUBLIC void vJenie_CbStackDataEvent(teEventType eEventType, void *pvEventPrim);

// Application Code Functions:
PUBLIC void lcd_WriteMainScreen(void);
PRIVATE void task_RegisterBeacon(uint64 u64SrcAddress);

// Application Screens
PUBLIC void lcd_BuildChannelSelectionScreen(void);
PUBLIC void lcd_UpdateChannelSelectionScreen(void);

// Button Handlers
PRIVATE bool_t button_ProcessKeys(uint8 *pu8Keys);
PRIVATE void button_ProcessSetChannelKeyPressHandler(uint8 u8KeyMap);

// System Initialization
PRIVATE void init_System(void);

// Task Management
PRIVATE void vSetTimer(void);
PRIVATE void vSetTimer1(void);
PRIVATE uint8 u8UpdateTimeBlock(uint8 u8TimeBlock);
PRIVATE void vProcessCurrentTimeBlock(uint8 u8TimeBlock);

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* System states with respect to screen display being shown */
typedef enum
{
    E_STATE_NETWORK,
    E_STATE_NODE,
    E_STATE_NODE_CONTROL,
    E_STATE_SET_CHANNEL,
    E_STATE_SETUP_SCREEN,
    E_STATE_SCANNING
} teState;

typedef enum
{
    E_STATE_STARTUP,
    E_STATE_RUNNING,
    E_STATE_WAITING
}teAppState;

/* Button values */
typedef enum
{
    E_KEY_0 = BUTTON_0_MASK,
    E_KEY_1 = BUTTON_1_MASK,
    E_KEY_2 = BUTTON_2_MASK,
    E_KEY_3 = BUTTON_3_MASK,
    E_KEYS_0_AND_3 = (BUTTON_0_MASK | BUTTON_3_MASK)
} teKeyValues;

/* Used to track an association between extended address and short address */
typedef struct
{
    uint64 u64ExtAddr;
    uint16 u16ShortAddr;
} tsAssocNodes;

typedef struct {
    struct
    {
        tsAssocNodes asAssocNodes[NUMBER_OF_BEACONS];
        uint8        u8AssociatedNodes;
    } sNode;

	struct
    {
        teState eState;
        uint8   u8Channel;
        // uint32  u32AppApiVersion;
        // uint32  u32JenieVersion;
        uint32  u32CalibratedTimeout;
    } sSystem;

    struct
    {
        // uint64 u64DestAddr;
        // uint64 u64ParentAddr;
        bool_t bAppTimerStarted;
        bool_t bStackReady;
        uint8 eAppState;
    } sHome;
} tsApplicationState;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsApplicationState sAppState;
PRIVATE bool_t bKeyDebounce = FALSE;
PRIVATE bool_t  bTimer0Fired;
/* Routing table storage */
PRIVATE tsJenieRoutingTable asRoutingTable[100];


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
    uint8 u8Keys = 0;
    volatile uint32 wait;

	/* Starting LCD and buttons here so channel can be set */
	vButtonInitFfd();    // Docs: JN-RM-2003 Page 45
	vLcdResetDefault();  // Docs: JN-RM-2003 Page 53
    vUtils_Init();
    vUtils_Debug("vJenie_CbConfigureNetwork");
	sAppState.sSystem.u8Channel = CHANNEL_MID;

	lcd_BuildChannelSelectionScreen();

	/* Change to channel setting state */
    sAppState.sSystem.eState = E_STATE_SET_CHANNEL;

    /* Loop while on set channel screen */
    while ((sAppState.sSystem.eState == E_STATE_SET_CHANNEL))
    {
        (void)button_ProcessKeys(&u8Keys);
        for (wait = 0; wait < 100000; wait++);
        bKeyDebounce = FALSE;
    }

    /* Set PAN_ID and other network stuff or defaults will be used */
    gJenie_Channel = sAppState.sSystem.u8Channel;
    gJenie_NetworkApplicationID=0xdeaddead;
    gJenie_PanID   = DEMO_PAN_ID;

    /* Configure stack with routing table data */
    gJenie_RoutingEnabled    = TRUE;
    gJenie_RoutingTableSize  = 100;
    gJenie_RoutingTableSpace = (void *)asRoutingTable;
    vUtils_Debug("exiting vJenie_CbConfigureNetwork");
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
    vUtils_Debug("vJenie_CbInit");
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);
    if (bWarmStart==FALSE)
    {
        sAppState.sHome.bStackReady=FALSE;
        sAppState.sHome.eAppState=E_STATE_STARTUP;
        vUtils_Debug("E_STATE_STARTUP");
        vSetTimer();
        switch (eJenie_Start(E_JENIE_COORDINATOR))        /* Start network as coordinator */
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
    }

    init_System();

    vUtils_Debug("exiting vJenie_CbInit");
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
    // vUtils_Debug("vJenie_CbHwEvent");
	if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & (1 << E_AHI_SYSCTRL_WK0)))      /* added for timer 0 interrupt */
    {
        bTimer0Fired = TRUE;

    } else if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_SYSCTRL_WK1_MASK) )
    {
        bKeyDebounce = FALSE;
    }
    // vUtils_Debug("exiting vJenie_CbHwEvent");
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
    // vUtils_Debug("vJenie_CbMain");
    static uint8 u8TimeBlock = MAX_BLOCKS;
    uint8 u8Keys = 0;

    /* regular watchdog reset */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogRestart();
    #endif

    if (sAppState.sHome.bStackReady)
    {
        switch (sAppState.sHome.eAppState)
        {
        case E_STATE_STARTUP:
            vUtils_Debug("E_STATE_STARTUP");
            if (!(bJenie_GetPermitJoin() ))
            {
                eJenie_SetPermitJoin(TRUE);
            }

            sAppState.sHome.eAppState = E_STATE_RUNNING;
            break;

        case E_STATE_RUNNING:
            // vUtils_Debug("E_STATE_RUNNING");
            vSetTimer();
            /* Perform scheduler action */
            vProcessCurrentTimeBlock(u8TimeBlock);
            /* Check keys. Returns TRUE if 'reset' combination has been pressed */
            (void)button_ProcessKeys(&u8Keys);
            /* Increment scheduler time block for next time */
            u8TimeBlock = u8UpdateTimeBlock(u8TimeBlock);

            sAppState.sHome.eAppState = E_STATE_WAITING;
            break;

        case E_STATE_WAITING:
            // vUtils_Debug("E_STATE_WAITING");
            if (bTimer0Fired)
            {
                bTimer0Fired = FALSE;
                sAppState.sHome.eAppState = E_STATE_RUNNING;
            }
            break;

        default:
            vUtils_Debug("default");
            break;
        }
    }
    // vUtils_Debug("exiting vJenie_CbMain");
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
    vUtils_Debug("vJenie_CbStackMgmtEvent");
    switch (eEventType)
    {
        case E_JENIE_NETWORK_UP:
            vUtils_Debug("E_JENIE_NETWORK_UP");
            sAppState.sHome.bStackReady=TRUE;
            vLedControl(3, TRUE);
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
            task_RegisterBeacon(joinEvent->u64SrcAddress);
            break;
        case E_JENIE_CHILD_LEAVE:
            vUtils_Debug("E_JENIE_CHILD_LEAVE");
            vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildLeave*)pvEventPrim)->u64SrcAddress));
            break;
        case E_JENIE_CHILD_REJECTED:
            vUtils_Debug("E_JENIE_CHILD_REJECTED");
            vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildRejected*)pvEventPrim)->u64SrcAddress));            
            break;
        default:
            vUtils_Debug("default");
            /* Unknown data event type */
            break;
    }
    vUtils_Debug("exiting vJenie_CbStackMgmtEvent");
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


PUBLIC void lcd_WriteMainScreen(void)
{
	// vUtils_Debug("lcd_WriteMainScreen");
    vLcdClear();  
	vLcdWriteText("Esten Rye", 1, 0);
	vLcdWriteTextRightJustified("SEIS 740", 1, 127);
    vLcdWriteText("TOF Ranging Project", 2, 0);
    if (sAppState.sHome.bStackReady)
    {
        vLcdWriteText("Network Stack Ready: Y", 3, 0);
    }
    else
    {
        vLcdWriteText("Network Stack Ready: N", 3, 0);
    }
    switch (sAppState.sNode.u8AssociatedNodes)
    {
        case 0:
            vLcdWriteText("Beacon1: Unregistered" , 4, 0);
            vLcdWriteText("Beacon2: Unregistered" , 5, 0);
            break;
        case 1:
            vLcdWriteText("Beacon1: Registered" , 4, 0);
            vLcdWriteText("Beacon2: Unregistered" , 5, 0);
            break;
        case 2:
            vLcdWriteText("Beacon1: Registered" , 4, 0);
            vLcdWriteText("Beacon2: Registered" , 5, 0);
            break;
        default:
            break;
    }
    
    vLcdWriteText("Hello World", 6, 0);
	vLcdWriteText("I am here", 7, 0);
    vLcdRefreshAll();
	// vUtils_Debug("exiting lcd_WriteMainScreen");    
}

/****************************************************************************
 *
 * NAME: lcd_BuildChannelSelectionScreen
 *
 * DESCRIPTION:
 * Creates the Set Channel screen, consisting of a header containing project
 * details and labels for the soft buttons on the bottom row. Uses the related
 * update function to display the current channel and refresh the LCD.
 *
 * JenOS API DOCUMENTATION REFERENCES:
 *   - JN-RM-2003: Page 56: void vLcdClear()
 *   - JN-RM-2003: Page 57: void vLcdWriteText(char *pcString, uint8 u8Row, uint8 u8Column)
 *   - JN-RM-2003: Page 58: void vLcdWriteTextRightJustified(char *pcString, uint8 u8Row, uint8 u8EndColumn)
 * 
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void lcd_BuildChannelSelectionScreen(void)
{
    vLcdClear();  
	vLcdWriteText("Esten Rye", 1, 0);
	vLcdWriteTextRightJustified("SEIS 740", 1, 127);
    vLcdWriteText("TOF Ranging Project", 2, 0);

	vLcdWriteText("Ch", 7, 0);
    vLcdWriteText("\\", 7, 47);
    vLcdWriteText("]", 7, 74);
    vLcdWriteText("Done", 7, 103);

    lcd_UpdateChannelSelectionScreen();
}

/****************************************************************************
 *
 * NAME: lcd_UpdateChannelSelectionScreen
 *
 * DESCRIPTION:
 * Updates the Set Channel screen, when it first appears or when the user
 * changes the channel number.
 *
 * JenOS API DOCUMENTATION REFERENCES:
 *   - JN-RM-2003: Page 57: void vLcdWriteText(char *pcString, uint8 u8Row, uint8 u8Column)
 *   - JN-RM-2003: Page 66: void vLcdRefreshAll(void)
 * 
 * Source References:
 *   - ../../Common/Utils.h: void vUtils_ValToDec(char *pcOutString, uint8 u8Value)
 * 
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void lcd_UpdateChannelSelectionScreen(void)
{
    vUtils_Debug("lcd_UpdateChannelSelectionScreen");
	char displayOutput[5];
	vUtils_ValToDec(displayOutput, sAppState.sSystem.u8Channel);
	vLcdWriteText(displayOutput, 7, 16);
    vUtils_Debug("Channel Selected");
    vUtils_Debug(displayOutput);
    vLcdRefreshAll();
    vUtils_Debug("exiting lcd_UpdateChannelSelectionScreen");    
}

/****************************************************************************
 *
 * NAME: button_ProcessKeys
 *
 * DESCRIPTION:
 * Gets the latest button presses and detects any change since the last time
 * the buttons were checked. If there is a change it is passed to the
 * individual handler for the screen currently being displayed (the buttons
 * are all 'soft' keys so their meaning changes from screen to screen). The
 * exception to this is a button combination that causes the software to
 * shutdown and stop the LCD. There is also a reset combination.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pu8Keys         RW  Persistent value of buttons pressed
 *
 * RETURNS:
 * TRUE if reset combination is pressed
 *
 ****************************************************************************/
PRIVATE bool_t button_ProcessKeys(uint8 *pu8Keys)
{
    //vUtils_Debug("button_ProcessKeys");
    uint8 u8KeysDown;
    uint8 u8NewKeysDown;

    u8KeysDown = *pu8Keys;

    /* Process key press */
    u8NewKeysDown = u8ButtonReadFfd();

    if ((u8NewKeysDown != 0) && (!bKeyDebounce))
    {
        vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_1, BUTTON_DEBOUNCE);
        bKeyDebounce = TRUE;

        if ((u8NewKeysDown | u8KeysDown) != u8KeysDown)
        {
            /* Logical OR values to enable multiple keys at once */
            u8KeysDown |= u8NewKeysDown;

            /* Key presses depend on mode */
            switch (sAppState.sSystem.eState)
            {
				case E_STATE_SET_CHANNEL:
					button_ProcessSetChannelKeyPressHandler(u8KeysDown);
					break;

				default:
					break;
            }
        }
    }
    else
    {
        u8KeysDown = 0;
    }

    /* Store value for use next time */
    *pu8Keys = u8KeysDown;
    //vUtils_Debug("exiting button_ProcessKeys");
    return (u8KeysDown == E_KEYS_0_AND_3);
}

/****************************************************************************
 *
 * NAME: button_ProcessSetChannelKeyPressHandler
 *
 * DESCRIPTION:
 * Handles button presses on the Set Channel screen. There is one parameter
 * that can be adjusted (the channel) and buttons to navigate to two other
 * screens.
 *
 * PARAMETERS:      Name        RW  Usage
 *                  u8KeyMap    R   Current buttons pressed bitmap
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void button_ProcessSetChannelKeyPressHandler(uint8 u8KeyMap)
{
    vUtils_Debug("button_ProcessSetChannelKeyPressHandler");
    switch (u8KeyMap)
    {
    case E_KEY_0:
        /* Further setup button: go to setup screen */
        sAppState.sSystem.eState = E_STATE_SETUP_SCREEN;
        vLcdWriteTextToClearLine("Initializing",7,0);
        vLcdRefreshArea(0,7,128,1);
        break;

    case E_KEY_1:
        /* Plus button: increment value */
    case E_KEY_2:
        /* Minus button: decrement value */

        vUtils_AdjustBoundedValue(&sAppState.sSystem.u8Channel, CHANNEL_MAX, CHANNEL_MIN, u8KeyMap == E_KEY_1);
        lcd_UpdateChannelSelectionScreen();
        break;

    case E_KEY_3:
        /* Done button: start beaconing and go to network screen */
        // vStartBeacon();
        sAppState.sSystem.eState = E_STATE_NETWORK;
        vLcdWriteTextToClearLine("Initializing",7,0);
        vLcdRefreshArea(0,7,128,1);
        break;

    default:
        break;
    }
    vUtils_Debug("exiting button_ProcessSetChannelKeyPressHandler");
}

/****************************************************************************
 *
 * NAME: init_System
 *
 * DESCRIPTION:
 * Initialises stack and hardware. Also sets non-default values in the
 * 802.15.4 PIB.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void init_System(void)
{
    vUtils_Debug("init_System");
    /* Initialise stack and hardware interfaces, and register peripheral
       interrupts with AppQueueApi handler. We aren't using callbacks
       at all, just monitoring the upward queues in a loop */
    /* Set up buttons and LEDs */
    vLedControl(0, FALSE);
    vLedControl(1, FALSE);
    vLedControl(2, FALSE);
    vLedControl(3, FALSE);
    vLedInitFfd();

    /* Set up hardware and splash screen */

    /* Calibrate wake timer */
    sAppState.sSystem.u32CalibratedTimeout = BLOCK_TIME_IN_32K_PERIODS * 10000 / u32AHI_WakeTimerCalibrate();

    /* Enable timer to use for sequencing */
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);
    vUtils_Debug("exiting init_System");
}

/****************************************************************************
 *
 * NAME: vSetTimer
 *
 * DESCRIPTION:
 * Sets wake-up timer 0 for a 50ms time-out. Assumes that timer was
 * previously enabled.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vSetTimer(void)
{
    /* Set timer for next block */
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, sAppState.sSystem.u32CalibratedTimeout);
}


PRIVATE void vSetTimer1(void)
{
    /* Set timer for next block */
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, sAppState.sSystem.u32CalibratedTimeout*3);
}

/****************************************************************************
 *
 * NAME: u8UpdateTimeBlock
 *
 * DESCRIPTION:
 * Moves the state machine time block on by one time period.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8TimeBlock     R   Previous time block
 *
 * RETURNS:
 * uint8 Next time block
 *
 ****************************************************************************/
PRIVATE uint8 u8UpdateTimeBlock(uint8 u8TimeBlock)
{
    /* Update block state for next time, if in a state where regular
       updates should be performed */
    if ((sAppState.sSystem.eState != E_STATE_SET_CHANNEL)
            // && (sAppState.sSystem.eState != E_STATE_SETUP_SCREEN)
            && (sAppState.sSystem.eState != E_STATE_SCANNING))
    {
        u8TimeBlock++;
        if (u8TimeBlock >= MAX_BLOCKS)
        {
            u8TimeBlock = 0;
        }
    }

    return u8TimeBlock;
}

/****************************************************************************
 *
 * NAME: vProcessCurrentTimeBlock
 *
 * DESCRIPTION:
 * Operates a simple state machine. Called 20 times per second, this performs
 * several tasks over u8LocalSensora 1 second period, with the time split into 50ms blocks.
 * In one block it updates the display, in another it starts a reading from
 * the temperature, in another it reads the temperature, etc.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u8TimeBlock     R   Current time block, 0-19
 *u8LocalSensor
 * RETURNS:
 * void
 *
 * NOTES:
 * A value greater than 19 may be used for u8TimeBlock, to ensure that the
 * simple state machine remains idle.
 *
 ****************************************************************************/
PRIVATE void vProcessCurrentTimeBlock(uint8 u8TimeBlock)
{
    /* Process current block scheduled activity */
    switch (u8TimeBlock)
    {
        case BLOCK_WRITE_MAIN_SCREEN:
             /* Time to update the display */
             lcd_WriteMainScreen();
             break;

    }
}

PRIVATE void task_RegisterBeacon(uint64 u64SrcAddress)
{
    uint8              u8Node;
    uint8              u8AssocStatus;
    uint16             u16ShortAddress;
    /* Check if already associated (idiot proofing) */
    u8Node = 0;
    u16ShortAddress = 0xffff;

    while (u8Node < sAppState.sNode.u8AssociatedNodes)
    {
        if (u64SrcAddress == sAppState.sNode.asAssocNodes[u8Node].u64ExtAddr)
        {
            /* Already in system: give it same short address */
            u16ShortAddress = sAppState.sNode.asAssocNodes[u8Node].u16ShortAddr;
        }
        u8Node++;
    }
    /* Assume association succeeded */
    u8AssocStatus = 0;
    if (u16ShortAddress == 0xffff)
    {
        if (sAppState.sNode.u8AssociatedNodes < NUMBER_OF_BEACONS)
        {
            /* Allocate short address as next in list */
            u16ShortAddress = ENDPOINT_ADDR_BASE + sAppState.sNode.u8AssociatedNodes;
            /* Store details for future use */
            sAppState.sNode.asAssocNodes[sAppState.sNode.u8AssociatedNodes].u64ExtAddr = u64SrcAddress;
            sAppState.sNode.asAssocNodes[sAppState.sNode.u8AssociatedNodes].u16ShortAddr = u16ShortAddress;
            sAppState.sNode.u8AssociatedNodes++;
        }
        else
        {
            /* PAN access denied */
            u8AssocStatus = 2;
        }
    }

    /* Update display if necessary */
    if (sAppState.sSystem.eState == E_STATE_NETWORK)
    {
        lcd_WriteMainScreen();
        vLedControl(sAppState.sNode.u8AssociatedNodes, TRUE);
    }

    vSetTimer1();
    sAppState.sHome.eAppState = E_STATE_WAITING;

}