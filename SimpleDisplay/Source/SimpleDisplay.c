
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define MAX_BEACONS                 2
#define MAX_READINGS                10
#define DEBUG_NETWORK_STACK TRUE
#define DEBUG_APPSTATE TRUE
/* Block (time slice) values */
#define BLOCK_TIME_IN_32K_PERIODS   1600
#define BLOCK_REGISTER_BEACONS      0
#define BLOCK_GET_TOF_A             2
#define BLOCK_CALCULATE_DISTANCE_A  4
#define BLOCK_GET_TOF_B             6
#define BLOCK_CALCULATE_DISTANCE_B  8
#define BLOCK_CALCULATE_POSITION    10
#define BLOCK_UPDATE                (BLOCK_CALCULATE_POSITION + 1)
#define MAX_BLOCKS                  20

#define CHANNEL_MIN                       11
#define CHANNEL_MID                       18
#define CHANNEL_MAX                       26
#define DEMO_PAN_ID                       0x0e1c


/* define LED positions  */
#define LED1                        0
#define LED2                        1

#define SLEEP_PERIOD                1000      // Units of 10 mS
#define FLASH_RATE                  100      // Approx 1 sec

#define INACTIVE_PERIOD             24

#define ONE_MSEC_IN_32KHZ_CYCLES    32
#define BUTTON_DEBOUNCE             (500 * ONE_MSEC_IN_32KHZ_CYCLES)

/* define if using high power modules */
/* #define HIGH_POWER */

/* message types */
#define BEACON_ASSIGNMENT                 0xb0

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <string.h>
#include <math.h>
#include "LcdDriver.h"
#include "JennicLogo.h"
#include "Button.h"
#include "LedControl.h"
#include "Jenie.h"
#include "Utils.h"
#include <AppApiTof.h>
#include <mac_sap.h>
#include "Printf.h"

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
	E_BEACON_0 = 0,
	E_BEACON_1 = 1,
    E_BEACON_NOT_ASSIGNED = 2
} teBeaconAssignment;

typedef struct
{
    uint64 u64BeaconAddress;
    MAC_Addr_s sAddrMACBeaconAddress;
    teBeaconAssignment eBeaconRole;
} tsBeaconData;

/* Used to track an association between extended address and short address */
typedef struct
{
    uint64 u64ExtAddr;
    uint16 u16ShortAddr;
}
tsAssocNodes;

/* System states with respect to screen display being shown */
typedef enum
{
    E_STATE_SET_CHANNEL,
    E_STATE_SCANNING,
    E_STATE_STATUS_SCREEN
} teState;

/* Button values */
typedef enum
{
    E_KEY_0 = BUTTON_0_MASK,
    E_KEY_1 = BUTTON_1_MASK,
    E_KEY_2 = BUTTON_2_MASK,
    E_KEY_3 = BUTTON_3_MASK,
    E_KEYS_0_AND_3 = (BUTTON_0_MASK | BUTTON_3_MASK)
} teKeyValues;

/* All application data with scope within the entire file is kept here,
   including all stored node data, GUI settings and current state */
typedef struct
{
    struct
    {
        tsBeaconData asBeacons[MAX_BEACONS];
        uint8 u8ConnectedBeacons;
    } sBeaconState;

    struct
    {
        double dDistanceA;
        double dDistanceB;
        double dDistanceC;
        double dXpos;
        double dYpos;
    } sState;

    struct
    {
        teState eState;
        uint8   u8Channel;
        uint32  u32AppApiVersion;
        uint32  u32JenieVersion;
        uint32  u32CalibratedTimeout;
    }
    sSystem;
}
tsApplicationState;

/* All application data with scope within the entire file is kept here, */
typedef struct
{
    uint64 u64DestAddr;
    uint64 u64ParentAddr;
    bool_t bAppTimerStarted;
    bool_t bStackReady;
    uint8 eAppState;
}
tsHomeData;

typedef enum
{
    E_STATE_STARTUP,
    E_STATE_RUNNING,
    E_STATE_WAITING,
    E_STATE_JOINING_BEACON
}teAppState;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

PRIVATE tsHomeData sHomeData;

PRIVATE tsApplicationState sDemoData;


PRIVATE bool_t bKeyDebounce = FALSE;

/* Routing table storage */
PRIVATE tsJenieRoutingTable asRoutingTable[100];

PRIVATE eTofReturn eTofStatus = -1;
PRIVATE volatile bool_t bTofInProgress = FALSE;
PRIVATE tsAppApiTof_Data asTofDataA[MAX_READINGS];
PRIVATE tsAppApiTof_Data asTofDataB[MAX_READINGS];

/* RSSI to Distance (cm) lookup table. Generated from formula in JN-UG-3063 */
uint32 au32RSSIdistance[] = { 502377, 447744, 399052, 355656, 316979, 282508,
		251785, 224404, 200000, 178250, 158866, 141589, 126191, 112468, 100237,
		89337, 79621, 70963, 63246, 56368, 50238, 44774, 39905, 35566, 31698,
		28251, 25179, 22440, 20000, 17825, 15887, 14159, 12619, 11247, 10024,
		8934, 7962, 7096, 6325, 5637, 5024, 4477, 3991, 3557, 3170, 2825, 2518,
		2244, 2000, 1783, 1589, 1416, 1262, 1125, 1002, 893, 796, 710, 632,
		564, 502, 448, 399, 356, 317, 283, 252, 224, 200, 178, 159, 142, 126,
		112, 100, 89, 80, 71, 63, 56, 50, 45, 40, 36, 32, 28, 25, 22, 20, 18,
		16, 14, 13, 11, 10, 9, 8, 7, 6, 6, 5, 4, 4, 4, 3, 3, 3, 2, 2 };

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE bool_t bProcessKeys(uint8 *pu8Keys);
PRIVATE void vInitCoord(void);
PRIVATE void vInitSystem(void);
PRIVATE void vProcessCurrentTimeBlock(uint8 u8TimeBlock);

PRIVATE void lcd_BuildSetChannelScreen(void);
PRIVATE void lcd_UpdateSetChannelScreen(void);
PRIVATE void lcd_BuildStatusScreen(void);
PRIVATE void lcd_UpdateStatusScreen(void);
PRIVATE void vStringCopy(char *pcFrom,char *pcTo);
PRIVATE void vValToDec(char *pcOutString, uint8 u8Value, char *pcLabel);
PRIVATE void button_adjustChannel(uint8 *pu8Value, uint8 u8MaxValue, uint8 u8OffValue, bool_t bUpNotDown);
PRIVATE uint8 u8UpdateTimeBlock(uint8 u8TimeBlock);
PRIVATE void button_ProcessSetChannelKeyPress(uint8 u8KeyMap);

PRIVATE void vSetTimer(void);
PRIVATE void dataTx_AssignBeaconRole(void);
PRIVATE void interrupt_RegisterBeacon(uint64 beaconAddress);
PRIVATE void interrupt_TofCallback(eTofReturn eStatus);
PRIVATE void task_StartTOF(tsAppApiTof_Data *pTofData, int iBeacon);
PRIVATE void task_CalculateTOFDistance(tsAppApiTof_Data *asTofData, double *distance);
PRIVATE void task_CalculateXYPos(void);
PRIVATE void reverse(char *str, int len);
PRIVATE int intToStr(int x, char str[], int d);
PRIVATE void dtoa(double n, char *res, int afterpoint);
PRIVATE void vPutChar(unsigned char c);

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
    uint8 u8Keys = 0;
    volatile uint32 wait;

    /* Starting LCD and buttons here so channel can be set */
    vButtonInitFfd();
    vLcdResetDefault();

    vUtils_Init();
    #ifdef DEBUG_NETWORK_STACK
        vUtils_Debug("vJenie_CbConfigureNetwork");
    #endif
    sDemoData.sSystem.u8Channel = CHANNEL_MID;

    lcd_BuildSetChannelScreen();
    /* Change to channel setting state */
    sDemoData.sSystem.eState = E_STATE_SET_CHANNEL;

    /* Loop while on set channel screen */
    while ((sDemoData.sSystem.eState == E_STATE_SET_CHANNEL))
    {
        (void)bProcessKeys(&u8Keys);
        for (wait = 0; wait < 100000; wait++);
        bKeyDebounce = FALSE;
    }

    /* Set PAN_ID and other network stuff or defaults will be used */
    gJenie_Channel = sDemoData.sSystem.u8Channel;
    gJenie_NetworkApplicationID=0xdeaddead;
    gJenie_PanID   = DEMO_PAN_ID;

    /* Configure stack with routing table data */
    gJenie_RoutingEnabled    = TRUE;
    gJenie_RoutingTableSize  = 100;
    gJenie_RoutingTableSpace = (void *)asRoutingTable;
    #ifdef DEBUG_NETWORK_STACK
        vUtils_Debug("exiting vJenie_CbConfigureNetwork");
    #endif
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
    vUtils_Debug("vJenie_CbInit");

    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);

    if (bWarmStart==FALSE)
    {
        sHomeData.bStackReady=FALSE;
        sHomeData.eAppState = E_STATE_STARTUP;
        #ifdef DEBUG_APPSTATE
            vUtils_Debug("E_STATE_STARTUP");
        #endif
        vInitSystem();

        vInitCoord();

        vSetTimer();
        vInitPrintf((void *)vPutChar);

        switch (eJenie_Start(E_JENIE_COORDINATOR))        /* Start network as coordinator */
        {
            case E_JENIE_SUCCESS:
                vUtils_Debug("E_JENIE_SUCCESS");
                #ifdef HIGH_POWER
                    /* Set high power mode */
                    eJenie_RadioPower(18, TRUE);
                #endif
                break;
            #ifdef DEBUG_NETWORK_STACK
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
            #endif
            default:
                vUtils_Debug("Unexpected Network Stack Startup Error");
                break;
        }
    }
    vUtils_Debug("exiting vJenie_CbInit");
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
PRIVATE bool_t  bTimer0Fired;

PUBLIC void vJenie_CbMain(void)
{
    static uint8 u8TimeBlock = MAX_BLOCKS;
    uint8 u8Keys = 0;

    /* regular watchdog reset */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogRestart();
    #endif

    if (sHomeData.bStackReady)
    {
        switch (sHomeData.eAppState)
        {
        case E_STATE_STARTUP:
            #ifdef DEBUG_APPSTATE
                vUtils_Debug("E_STATE_STARTUP");
            #endif
            if (!(bJenie_GetPermitJoin() ))
            {
                eJenie_SetPermitJoin(TRUE);
            }
            vLedControl(0, FALSE);
            sHomeData.eAppState = E_STATE_RUNNING;
            break;

        case E_STATE_RUNNING:
            #ifdef DEBUG_APPSTATE
                vUtils_Debug("E_STATE_RUNNING");
            #endif
            vSetTimer();
            /* Perform scheduler action */
            vProcessCurrentTimeBlock(u8TimeBlock);
            /* Check keys. Returns TRUE if 'reset' combination has been pressed */
            (void)bProcessKeys(&u8Keys);
            /* Increment scheduler time block for next time */
            u8TimeBlock = u8UpdateTimeBlock(u8TimeBlock);

            sHomeData.eAppState = E_STATE_WAITING;
            break;

        case E_STATE_WAITING:
            #ifdef DEBUG_APPSTATE
                vUtils_Debug("E_STATE_WAITING");        
            #endif
            if (bTimer0Fired)
            {
                bTimer0Fired = FALSE;
                sHomeData.eAppState = E_STATE_RUNNING;
            }
            break;
        case E_STATE_JOINING_BEACON:
            #ifdef DEBUG_APPSTATE
                vUtils_Debug("E_STATE_JOINING_BEACON");
            #endif
            break;
        default:
            #ifdef DEBUG_APPSTATE
                vUtils_Debug("default");
            #endif
            break;
        }
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
    vUtils_Debug("vJenie_CbStackMgmtEvent");
    switch (eEventType)
    {
        case E_JENIE_NETWORK_UP:
            vUtils_Debug("E_JENIE_NETWORK_UP");
            sHomeData.bStackReady=TRUE;
            break;

        case E_JENIE_CHILD_JOINED:
            sHomeData.eAppState = E_STATE_JOINING_BEACON;
            vPrintf("E_JENIE_CHILD_JOINED\n");
            vUtils_DisplayMsg("Child Joined:",(uint32)(((tsChildJoined*)pvEventPrim)->u64SrcAddress));
            interrupt_RegisterBeacon((((tsChildJoined*)pvEventPrim)->u64SrcAddress));
            vSetTimer();
            sHomeData.eAppState = E_STATE_WAITING;
            break;

        #ifdef DEBUG_NETWORK_STACK
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

            case E_JENIE_CHILD_LEAVE:
                vUtils_Debug("E_JENIE_CHILD_LEAVE");
                vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildLeave*)pvEventPrim)->u64SrcAddress));
                break;
            case E_JENIE_CHILD_REJECTED:
                vUtils_Debug("E_JENIE_CHILD_REJECTED");
                vUtils_DisplayMsg("Child Left: ",(uint32)(((tsChildRejected*)pvEventPrim)->u64SrcAddress));            
                break;
        #endif
        default:
            vUtils_Debug("Unexpected Network Stack Management Event.");
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
        #ifdef DEBUG_NETWORK_STACK
            case E_JENIE_DATA:
                vUtils_Debug("E_JENIE_DATA");    
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
        #endif
        default:
            vUtils_Debug("Unexpected Network Stack Data Event");
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

    if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & (1 << E_AHI_SYSCTRL_WK0)))      /* added for timer 0 interrupt */
    {
        #ifdef DEBUG_APPSTATE
            vUtils_Debug("Timer Fired.");
        #endif
        bTimer0Fired = TRUE;

    } else if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_SYSCTRL_WK1_MASK) )
    {
        bKeyDebounce = FALSE;
    }
}

/****************************************************************************
 *
 * NAME: vInitSystem
 *
 * DESCRIPTION:
 * Initialises stack and hardware. Also sets non-default values in the
 * 802.15.4 PIB and starts the first read of the light sensor. Subsequent
 * reads of this sensor occur automatically.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitSystem(void)
{
    /* Initialise stack and hardware interfaces, and register peripheral
       interrupts with AppQueueApi handler. We aren't using callbacks
       at all, just monitoring the upward queues in a loop */
    /* Set up buttons and LEDs */
    vLedControl(0, TRUE);
    vLedControl(1, TRUE);
    vLedControl(2, TRUE);
    vLedControl(3, TRUE);
    vLedInitFfd();

    /* Calibrate wake timer */
    sDemoData.sSystem.u32CalibratedTimeout = BLOCK_TIME_IN_32K_PERIODS * 10000 / u32AHI_WakeTimerCalibrate();

    /* Enable timer to use for sequencing */
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);


}

/****************************************************************************
 *
 * NAME: vInitCoord
 *
 * DESCRIPTION:
 * Initialises software structures and variables. Endpoint data is reset and
 * the GUI is set to the default condition.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitCoord(void)
{
    int i;
    for (i=0; i<MAX_BEACONS; i++)
    {
        sDemoData.sBeaconState.asBeacons[i].eBeaconRole = E_BEACON_NOT_ASSIGNED;
        sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.u8AddrMode = 3;
        sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.u16PanId = DEMO_PAN_ID;
        sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.uAddr.sExt.u32L = 0ULL & 0x00000000ffffffff;
        sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.uAddr.sExt.u32H = 0ULL >> 32;
        sDemoData.sBeaconState.asBeacons[i].u64BeaconAddress = 0ULL;
    }
    sDemoData.sBeaconState.u8ConnectedBeacons = 0;
    sDemoData.sState.dDistanceC = 1.2; //meters (4 ft)

    // Prep Measurement Arrays.
    int n;
    for(n = 0; n < MAX_READINGS; n++)
    {
        asTofDataA[n].s32Tof       = 0;
        asTofDataA[n].s8LocalRSSI  = 0;
        asTofDataA[n].u8LocalSQI   = 0;
        asTofDataA[n].s8RemoteRSSI = 0;
        asTofDataA[n].u8RemoteSQI  = 0;
        asTofDataA[n].u32Timestamp = 0;
        asTofDataA[n].u8Status     = 0;
    }

    for(n = 0; n < MAX_READINGS; n++)
    {
        asTofDataB[n].s32Tof       = 0;
        asTofDataB[n].s8LocalRSSI  = 0;
        asTofDataB[n].u8LocalSQI   = 0;
        asTofDataB[n].s8RemoteRSSI = 0;
        asTofDataB[n].u8RemoteSQI  = 0;
        asTofDataB[n].u32Timestamp = 0;
        asTofDataB[n].u8Status     = 0;
    }

    /* Enable TOF ranging. */
    vAppApiTofInit(TRUE);

    /* Get software version numbers */
    sDemoData.sSystem.u32AppApiVersion = u32Jenie_GetVersion(E_JENIE_COMPONENT_MAC);
    sDemoData.sSystem.u32JenieVersion = u32Jenie_GetVersion(E_JENIE_COMPONENT_JENIE);

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
        case BLOCK_GET_TOF_A:
            vPrintf("BLOCK_GET_TOF_A\n");
            task_StartTOF(asTofDataA, 0);
            break;
        case BLOCK_CALCULATE_DISTANCE_A:
            vPrintf("BLOCK_CALCULATE_DISTANCE_A\n");
            task_CalculateTOFDistance(asTofDataA, &(sDemoData.sState.dDistanceA));
            break;
        case BLOCK_GET_TOF_B:
            vPrintf("BLOCK_GET_TOF_B\n");
            task_StartTOF(asTofDataB, 1);
            break;
        case BLOCK_CALCULATE_DISTANCE_B:
            vPrintf("BLOCK_CALCULATE_DISTANCE_B\n");
            task_CalculateTOFDistance(asTofDataB, &(sDemoData.sState.dDistanceB));
            break;
        case BLOCK_REGISTER_BEACONS:
            dataTx_AssignBeaconRole();
            break;
        case BLOCK_UPDATE:
            lcd_UpdateStatusScreen();
            vSetTimer();
            break;
        case BLOCK_CALCULATE_POSITION:
            task_CalculateXYPos();
            break;
    }
}



/****************************************************************************
 *
 * NAME: bProcessKeys
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
PRIVATE bool_t bProcessKeys(uint8 *pu8Keys)
{
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
            switch (sDemoData.sSystem.eState)
            {
            case E_STATE_SET_CHANNEL:
                button_ProcessSetChannelKeyPress(u8KeysDown);
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

    return (u8KeysDown == E_KEYS_0_AND_3);
}

/****************************************************************************
 *
 * NAME: lcd_BuildSetChannelScreen
 *
 * DESCRIPTION:
 * Creates the Set Channel screen, consisting of a bitmap of the Jennic logo
 * and labels for the soft buttons on the bottom row. Uses the related update
 * function to display the current channel and refresh the LCD.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void lcd_BuildSetChannelScreen(void)
{
    vLcdClear();

    vLcdWriteBitmap((tsBitmap *)&sJennicLogo, 0, 1);

    vLcdWriteText("Ch", 7, 0);
    vLcdWriteText("\\", 7, 47);
    vLcdWriteText("]", 7, 74);
    vLcdWriteText("Done", 7, 103);

    /* Update to display the data */
    lcd_UpdateSetChannelScreen();
}

/****************************************************************************
 *
 * NAME: lcd_UpdateSetChannelScreen
 *
 * DESCRIPTION:
 * Updates the Set Channel screen, when it first appears or when the user
 * changes the channel number.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void lcd_UpdateSetChannelScreen(void)
{
    char acString[5];

    vValToDec(acString, sDemoData.sSystem.u8Channel, "  ");
    vLcdWriteText(acString, 7, 16);

    vLcdRefreshAll();
}

PRIVATE void lcd_BuildStatusScreen(void)
{
    vUtils_Debug("lcd_BuildStatusScreen");
    vLcdClear();
    vLcdWriteText("Esten Rye", 0, 0);
    vLcdWriteTextRightJustified("SEIS 740", 0, 127);
    vLcdWriteText("TOF Triangulation", 1, 0);
    vLcdWriteText("Node 0:", 2, 0);
    vLcdWriteTextRightJustified("Off", 2, 60);
    vLcdWriteText("Node 1:", 2, 64);
    vLcdWriteTextRightJustified("Off", 2, 123);
    vLcdWriteText("A:", 3, 0);
    vLcdWriteText("B:", 4, 0);
    vLcdWriteText("C:", 5, 0);
    vLcdWriteText("X:", 6, 0);
    vLcdWriteText("Y:", 7, 0);
    lcd_UpdateStatusScreen();
}

PRIVATE void lcd_UpdateStatusScreen(void)
{
    vUtils_Debug("lcd_UpdateStatusScreen");
    int i;
    bool_t beacon0Assigned = FALSE;
    bool_t beacon1Assigned = FALSE;
    vUtils_DisplayMsg("u8ConnectedBeacons", sDemoData.sBeaconState.u8ConnectedBeacons);
    for (i=0; i<sDemoData.sBeaconState.u8ConnectedBeacons; i++)
    {
        switch(sDemoData.sBeaconState.asBeacons[i].eBeaconRole)
        {
            case E_BEACON_0:
                beacon0Assigned = TRUE;
                break;
            case E_BEACON_1:
                beacon1Assigned = TRUE;
                break;
            default:
                break;
        }
    }

    if (beacon0Assigned)
    {
        vLcdWriteTextRightJustified(" On", 2, 60);
        vUtils_Debug("Beacon1");
    }
    if (beacon1Assigned)
    {
        vLcdWriteTextRightJustified(" On", 2, 123);
        vUtils_Debug("Beacon1");
        
    }

    char output[20];
    dtoa(sDemoData.sState.dDistanceA, output, 4);
    vLcdWriteTextRightJustified(output, 3, 127);
    dtoa(sDemoData.sState.dDistanceB, output, 4);
    vLcdWriteTextRightJustified(output, 4, 127);
    dtoa(sDemoData.sState.dDistanceC, output, 4);
    vLcdWriteTextRightJustified(output, 5, 127);
    dtoa(sDemoData.sState.dXpos, output, 4);
    vLcdWriteTextRightJustified(output, 6, 127);
    dtoa(sDemoData.sState.dYpos, output, 4);
    vLcdWriteTextRightJustified(output, 7, 127);
    vLcdRefreshAll();
}


/****************************************************************************
 *
 * NAME: vStringCopy
 *
 * DESCRIPTION:
 * Simple string copy as standard libraries not available.
 *
 * PARAMETERS:      Name    RW  Usage
 *                  pcFrom  R   Pointer to string to copy
 *                  pcTo    W   Pointer to store for new string
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vStringCopy(char *pcFrom, char *pcTo)
{
    while (*pcFrom != '\0')
    {
        *pcTo = *pcFrom;
        pcTo++;
        pcFrom++;
    }
    *pcTo = '\0';
}

/****************************************************************************
 *
 * NAME: vValToDec
 *
 * DESCRIPTION:
 * Converts an 8-bit value to a string of the textual decimal representation.
 * Adds a text string after the text.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pcOutString     R   Location for new string
 *                  u8Value         R   Value to convert
 *                  pcLabel         R   Label to append to string
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vValToDec(char *pcOutString, uint8 u8Value, char *pcLabel)
{
    static const uint8 au8Digits[3] =
        {
            100, 10, 1
        };
    uint8 u8Digit;
    uint8 u8DigitIndex;
    uint8 u8Count;
    bool_t boPreviousDigitPrinted = FALSE;

    for (u8DigitIndex = 0; u8DigitIndex < 3; u8DigitIndex++)
    {
        u8Count = 0;
        u8Digit = au8Digits[u8DigitIndex];
        while (u8Value >= u8Digit)
        {
            u8Value -= u8Digit;
            u8Count++;
        }

        if ((u8Count != 0) || (boPreviousDigitPrinted == TRUE)
                || (u8DigitIndex == 2))
        {
            *pcOutString = '0' + u8Count;
            boPreviousDigitPrinted = TRUE;
            pcOutString++;
        }
    }

    vStringCopy(pcLabel, pcOutString);
}

/****************************************************************************
 *
 * NAME: button_adjustChannel
 *
 * DESCRIPTION:
 * Increment a variable: If the variable is the maximum in the normal range,
 * sets it to a value that signifies 'off'. If the value is already 'off',
 * sets it to 0 (assumed to be the minimum within the normal range). This
 * function is used to set alarm levels.
 *
 * Decrement a variable: If the variable is 0 (assumed to be the minimum
 * within the normal range), sets it to a value that signifies 'off'. If the
 * value is already 'off', sets it to the maximum value in the normal range.
 * This function is used to set alarm levels.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pu8Value        R   Pointer to variable to adjust
 *                  u8MaxValue      R   Maximum value in normal range
 *                  u8OffValue      R   Value that signifies 'off'
 *                  bUpNotDown      R   TRUE to increment, FALSE to decrement
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void button_adjustChannel(uint8 *pu8Value, uint8 u8MaxValue, uint8 u8OffValue,
                          bool_t bUpNotDown)
{
    if (bUpNotDown)
    {
        if (*pu8Value == u8MaxValue)
        {
            *pu8Value = u8OffValue;
        }
        else
        {
            *pu8Value = *pu8Value + 1;
        }
    }
    else
    {
        if (*pu8Value == u8OffValue)
        {
            *pu8Value = u8MaxValue;
        }
        else
        {
            if (*pu8Value == 0)
            {
                *pu8Value = u8OffValue;
            }
            else
            {
                *pu8Value = *pu8Value - 1;
            }
        }
    }
}

/****************************************************************************
 *
 * NAME: button_ProcessSetChannelKeyPress
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
PRIVATE void button_ProcessSetChannelKeyPress(uint8 u8KeyMap)
{
    switch (u8KeyMap)
    {
    case E_KEY_0:
        /* Further setup button: go to setup screen */
        sDemoData.sSystem.eState = E_STATE_STATUS_SCREEN;
        sHomeData.eAppState = E_STATE_STARTUP;
        lcd_BuildStatusScreen();
        break;

    case E_KEY_1:
        /* Plus button: increment value */
    case E_KEY_2:
        /* Minus button: decrement value */

        button_adjustChannel(&sDemoData.sSystem.u8Channel, CHANNEL_MAX, CHANNEL_MIN, u8KeyMap == E_KEY_1);
        lcd_UpdateSetChannelScreen();
        break;

    case E_KEY_3:
        /* Done button: start beaconing and go to network screen */
        // vStartBeacon();
        sDemoData.sSystem.eState = E_STATE_STATUS_SCREEN;
        sHomeData.eAppState = E_STATE_STARTUP;
        lcd_BuildStatusScreen();
        break;

    default:
        break;
    }
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
    if ((sDemoData.sSystem.eState != E_STATE_SET_CHANNEL)
            && (sDemoData.sSystem.eState != E_STATE_SCANNING))
    {
        u8TimeBlock++;
        if (u8TimeBlock >= MAX_BLOCKS)
        {
            u8TimeBlock = 0;
        }
    }
    vPrintf("vProcessCurrentTimeBlock: %i\n", u8TimeBlock);        

    return u8TimeBlock;
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
    #ifdef DEBUG_APPSTATE
        vUtils_Debug("Timer Set");
    #endif
    /* Set timer for next block */
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, sDemoData.sSystem.u32CalibratedTimeout);
}

/*****************************************************************************
 * interrupt_RegisterBeacon
 * 
 *****************************************************************************/
PRIVATE void interrupt_RegisterBeacon(uint64 beaconAddress)
{
    int i;
    teBeaconAssignment eBeaconRole = E_BEACON_NOT_ASSIGNED;
    bool_t beacon0Assigned = FALSE;
    bool_t beacon1Assigned = FALSE;
    for (i=0; i<sDemoData.sBeaconState.u8ConnectedBeacons; i++)
    {
        switch(sDemoData.sBeaconState.asBeacons[i].eBeaconRole)
        {
            case E_BEACON_0:
                beacon0Assigned = TRUE;
                vUtils_Debug("Beacon 0 Already Assigned");
                break;
            case E_BEACON_1:
                beacon1Assigned = TRUE;
                vUtils_Debug("Beacon 1 Already Assigned");
                break;
            case E_BEACON_NOT_ASSIGNED:
                vUtils_Debug("Beacon Slot Not Assigneed");
                break;
            default:
                vUtils_Debug("Unknown Beacon Role");
                break;
        }
        if (sDemoData.sBeaconState.asBeacons[i].u64BeaconAddress == beaconAddress)
        {
            vUtils_DisplayMsg("Beacon Address already registered", (uint32)beaconAddress);
            eBeaconRole = sDemoData.sBeaconState.asBeacons[i].eBeaconRole;
            break;
        }
    }

    if (eBeaconRole == E_BEACON_NOT_ASSIGNED)
    {
        if (!beacon0Assigned)
        {
            eBeaconRole = E_BEACON_0;
            vUtils_Debug("Assigning Node Beacon Role: 0");
            vLedControl(1, FALSE);
        }
        else if (!beacon1Assigned)
        {
            eBeaconRole = E_BEACON_1;
            vUtils_Debug("Assigning Node Beacon Role: 1");
            vLedControl(2, FALSE);
        }
        else
        {
            vUtils_Debug("All Nodes already allocated.");
        }

        for (i = 0; i<MAX_BEACONS; i++) 
        {
            if (sDemoData.sBeaconState.asBeacons[i].eBeaconRole == E_BEACON_NOT_ASSIGNED)
            {
                sDemoData.sBeaconState.asBeacons[i].u64BeaconAddress = beaconAddress;
                sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.uAddr.sExt.u32L = beaconAddress & 0x00000000ffffffff;
                sDemoData.sBeaconState.asBeacons[i].sAddrMACBeaconAddress.uAddr.sExt.u32H = beaconAddress >> 32;
                sDemoData.sBeaconState.asBeacons[i].eBeaconRole = eBeaconRole;
                break;
            }
        }
        sDemoData.sBeaconState.u8ConnectedBeacons += 1;
    }
}

PRIVATE void dataTx_AssignBeaconRole(void)
{
    vUtils_Debug("dataTx_AssignBeaconRole");
    int i;
    for (i=0; i<sDemoData.sBeaconState.u8ConnectedBeacons; i++)
    {
        vUtils_Debug("Transmit Beacon");
        if (sDemoData.sBeaconState.asBeacons[i].eBeaconRole != E_BEACON_NOT_ASSIGNED)
        {
            uint8 au8Payload[8];
            au8Payload[0] = BEACON_ASSIGNMENT;
            au8Payload[1] = sDemoData.sBeaconState.asBeacons[i].eBeaconRole;
            eJenie_SendData(sDemoData.sBeaconState.asBeacons[i].u64BeaconAddress,au8Payload,2,0);
        }
    }
}

PRIVATE void task_StartTOF(tsAppApiTof_Data *pTofData, int iBeacon)
{
    switch(sDemoData.sBeaconState.asBeacons[iBeacon].eBeaconRole)
    {
        case E_BEACON_0:
        case E_BEACON_1:
            if (bTofInProgress == FALSE)
            {
                vUtils_Debug("Starting TOF Measurement.");
                // Set eTofStatus to invalid value. 
                // Will be updated in ToF callback 
                eTofStatus = -1;
                bTofInProgress = bAppApiGetTof(pTofData, &(sDemoData.sBeaconState.asBeacons[iBeacon].sAddrMACBeaconAddress), MAX_READINGS, API_TOF_FORWARDS, interrupt_TofCallback);
            }
            else
            {
                vUtils_Debug("Skipping TOF Measurement.");
            }
            break;
        default:
            vUtils_Debug("Skipping TOF Measurement.");
            break;
    }
}

PRIVATE void interrupt_TofCallback(eTofReturn eStatus)
{
    vUtils_Debug("\ninterrupt_TofCallback\n");
	eTofStatus = eStatus;
    switch(eStatus)
    {
        case MAC_TOF_STATUS_SUCCESS:
            vPrintf("eTofStatus: MAC_TOF_STATUS_SUCCESS  Reading completed successfully\n");
            break;
        case MAC_TOF_STATUS_RTE:
            vPrintf("eTofStatus: MAC_TOF_STATUS_RTE  Remote time value invalid\n");
            break;
        case MAC_TOF_STATUS_LTE:
            vPrintf("eTofStatus: MAC_TOF_STATUS_LTE  Local time value invalid\n");
            break;
        case MAC_TOF_STATUS_NOACK:
            vPrintf("eTofStatus: MAC_TOF_STATUS_NOACK  Failed to receive acknowledgement\n");
            break;
        case MAC_TOF_STATUS_DATAERROR:
            vPrintf("eTofStatus: MAC_TOF_STATUS_DATAERROR  Failed to receive data from remote node\n");
            break;
        default:
            vPrintf("eTofStatus: %i  Invalid Status\n", eStatus);
            break;
    }
    vSetTimer();

}

PRIVATE void task_CalculateTOFDistance(tsAppApiTof_Data *asTofData, double *distance)
{
	int n;
	int32 s32Mean, s32StanDev;
    double dTofDistance = 0;
	uint32 u32RssiDistance;
	double dStd, dMean;
	uint8  u8NumErrors, u8NumSuccessfullTofs=0;
    
    /* Has callback indicated completion of burst */
    /* Check for return code to have been set in callback */
    if (eTofStatus != -1)
    {
        if (eTofStatus == TOF_SUCCESS)
        {
            u8NumSuccessfullTofs++;

            double dAcc = 0.0;
            u32RssiDistance = 0;

            u8NumErrors = 0;

            vPrintf("\n\n| #  \x1BH| ToF (ps) \x1BH| Lcl RSSI \x1BH| Lcl SQI \x1BH| Rmt RSSI \x1BH| Rmt SQI \x1BH| Timestamp \x1BH| Status \x1BH|");
            vPrintf("\n--------------------------------------------------------------------------------");

            for(n = 0; n < MAX_READINGS; n++)
            {
                vPrintf("\n|%d",n);

                /* Only include successful readings */
                if (asTofData[n].u8Status == MAC_TOF_STATUS_SUCCESS)
                {
                    dAcc += asTofData[n].s32Tof;
                    u32RssiDistance += au32RSSIdistance[asTofData[n].s8LocalRSSI];
                    u32RssiDistance += au32RSSIdistance[asTofData[n].s8RemoteRSSI];

                    vPrintf("\t|%i\t|%d\t|%d\t|%d\t|%d\t|%d\t|%d\t|",
                            asTofData[n].s32Tof,
                            asTofData[n].s8LocalRSSI,
                            asTofData[n].u8LocalSQI,
                            asTofData[n].s8RemoteRSSI,
                            asTofData[n].u8RemoteSQI,
                            asTofData[n].u32Timestamp,
                            asTofData[n].u8Status);
                }
                else
                {
                    u8NumErrors++;

                    vPrintf("\t|-\t|-\t|-\t|-\t|-\t|-\t|%d\t|",
                            asTofData[n].u8Status);
                }
            }

            /* Calculate statistics */
            if(u8NumErrors != MAX_READINGS)
            {
                dMean = dAcc / (MAX_READINGS - u8NumErrors);

                /* Calculate standard deviation = sqrt((1/N)*(sigma(xi-xmean)2) */
                dStd = 0.0;

                /* Accumulate sum of squared deviances */
                for(n = 0; n < MAX_READINGS; n++)
                {
                    if(asTofData[n].u8Status == MAC_TOF_STATUS_SUCCESS)
                    {
                        dStd += ((double)asTofData[n].s32Tof - dMean) * ((double)asTofData[n].s32Tof - dMean);
                    }
                }

                /* std = sqrt(mean of sum of squared deviances) */
                dStd /= (MAX_READINGS - u8NumErrors);
                dStd = sqrt(dStd);


                s32StanDev = (int32)dStd;
                s32Mean    = (int32)dMean;

                /* Calculate distances */
                dTofDistance  = dMean * 0.0003;
                u32RssiDistance /= (MAX_READINGS - u8NumErrors) * 2;
            }
            else
            {
                s32StanDev      = 0;
                s32Mean         = 0;
                dTofDistance    = 0;
                u32RssiDistance = 0;
            }

            vPrintf("\n\nStandDev (ToF): %ips, Mean (ToF): %ips, Errors: %d",
                    s32StanDev,
                    s32Mean,
                    u8NumErrors);

            vPrintf("\nDistance (ToF): %dm, Distance (RSSI): %dcm",
                    dTofDistance,
                    u32RssiDistance);
        }
        else
        {
            vPrintf("\nToF failed with error %d", eTofStatus);
        }

        *distance = dTofDistance;
        /* Reset flags for next ToF burst */
        eTofStatus = -1;
        bTofInProgress = FALSE;
    }

    /* Clear flag to allow next measurement */
    bTofInProgress = FALSE;
    eTofStatus = -1;
}

PRIVATE void task_CalculateXYPos(void)
{
    vUtils_Debug("task_CalculateXYPos");
    if (sDemoData.sBeaconState.u8ConnectedBeacons >= 2)
    {
        double a = sDemoData.sState.dDistanceA;
        double b = sDemoData.sState.dDistanceB;
        double c = sDemoData.sState.dDistanceC;
        double s = (a + b + c) / 2.0;
        double y = 2.0 * sqrt(s * (s-a) * (s-b) * (s-c)) / c;
        double x = sqrt(pow(a, 2) - pow(y, 2));
        sDemoData.sState.dYpos = y;
        sDemoData.sState.dXpos = x;
    }
    vUtils_Debug("Calculation Complete");
}



// reverses a string 'str' of length 'len'
// retrieved from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 
 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
 // retrieved from https://www.geeksforgeeks.org/convert-floating-point-number-string/
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}
 
// Converts a floating point number to string.
// retrieved and modified from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
PRIVATE void dtoa(double n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    double fpart = n - (double)ipart;
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

PRIVATE void vPutChar(unsigned char c) {
	while ((u8AHI_UartReadLineStatus(UART) & E_AHI_UART_LS_THRE) == 0);
	vAHI_UartWriteData(UART, c);
    while ((u8AHI_UartReadLineStatus(UART) & (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT)) != (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT));
}
