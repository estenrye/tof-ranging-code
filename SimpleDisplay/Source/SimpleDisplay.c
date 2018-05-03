
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Block (time slice) values */
#define BLOCK_TIME_IN_32K_PERIODS   1600
#define BLOCK_MIN_RX                2
#define BLOCK_UPDATE                (BLOCK_MIN_RX + DEMO_ENDPOINTS)
#define BLOCK_START_TEMP            13
#define BLOCK_READ_TEMP             15
#define BLOCK_START_HUMIDITY        16
#define BLOCK_READ_HUMIDITY         18
#define BLOCK_READ_LIGHT            19
#define MAX_BLOCKS                  20

/* Control screen and alarm values */
#define CONTROL_LIST_LEN            4
#define TEMP_HIGH_MAX               100
#define LIGHT_HIGH_MAX              6

/* Setup screen values */
#define SETUP_LIST_LEN              2

#define FRAMES_MISSED_INDICATION    10


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

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <string.h>
#include "LcdDriver.h"
#include "AlsDriver.h"
#include "HtsDriver.h"
#include "HomeSensorConfig.h"
#include "JennicLogo.h"
#include "Button.h"
#include "LedControl.h"
#include "Jenie.h"
#include "Utils.h"

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* Holds all stored data for a particular sensor for a node */
typedef struct
{
    uint8 u8NowValue;
    uint8 u8HighAlarm;
    uint8 u8LowAlarm;
    uint8 au8GraphData[DEMO_HISTORY_LEN];
}
tsNodeElementData;

/* Holds all stored data for a node */
typedef struct
{
    bool_t boDeviceOn;
    tsNodeElementData asNodeElementData[DEMO_SENSOR_LIST_LEN];
    uint8 u8PrevRxCount;
    uint8 u8FramesMissed;
    uint8 u8SwitchOn;
    uint8 u8Rssi;
}
tsNodeData;

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
    E_STATE_NETWORK,
    E_STATE_NODE,
    E_STATE_NODE_CONTROL,
    E_STATE_SET_CHANNEL,
    E_STATE_SETUP_SCREEN,
    E_STATE_SCANNING
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
        tsNodeData   asNodeData[DEMO_ENDPOINTS];
        tsAssocNodes asAssocNodes[DEMO_ENDPOINTS];
        bool_t       bLocalNode;
        uint8        u8AssociatedNodes;
    }
    sNode;

    struct
    {
        uint16 u16Hi;
        uint16 u16Lo;
    }
    sLightSensor;

    struct
    {
        teSensor eCurrentSensor;
        uint8    u8CurrentNode;
        uint8    u8GraphPos;
        uint8    u8ControlSelection;
        uint8    u8SetupSelection;
        bool_t   bShowFourNodes;
    }
    sGui;

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
tsDemoData;

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
    E_STATE_WAITING
}teAppState;

/* Temperature/Humidity Sensor - reading state definitions */
typedef enum
{
    E_STATE_READ_TEMP_START,
    E_STATE_READ_TEMP_RUNNING,
    E_STATE_READ_TEMP_COMPLETE,
    E_STATE_READ_TEMP_READY
}teStateReadTemperature;

typedef enum
{
    E_STATE_READ_HUMID_START,
    E_STATE_READ_HUMID_RUNNING,
    E_STATE_READ_HUMID_COMPLETE,
    E_STATE_READ_HUMID_READY
}teStateReadHumid;


/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

PRIVATE tsHomeData sHomeData;

PRIVATE tsDemoData sDemoData;

PRIVATE teStateReadTemperature  eStateReadTemperature;

PRIVATE teStateReadHumid    eStateReadHumid;

/* Row and column positions of info fields on LCD */
static const uint8 au8NodeLcdRow[DEMO_ENDPOINTS] =
    {
        0,  0, 3,  3
    };
static const uint8 au8NodeLcdCol[DEMO_ENDPOINTS] =
    {
        0, 64, 0, 64
    };

static const char *apcNodeNameList[DEMO_ENDPOINTS] =
    {
        "Hall", "Bedroom", "Lounge", "Bathroom"
    };

PRIVATE bool_t bKeyDebounce = FALSE;

/* Routing table storage */
PRIVATE tsJenieRoutingTable asRoutingTable[100];

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE bool_t bProcessKeys(uint8 *pu8Keys);
PRIVATE void vInitCoord(void);
PRIVATE void vInitSystem(void);
PRIVATE void vProcessCurrentTimeBlock(uint8 u8TimeBlock);
PRIVATE void vProcessUpdateBlock(void);

PRIVATE void vBuildSetChannelScreen(void);
PRIVATE void vUpdateSetChannelScreen(void);
PRIVATE void vBuildNetworkScreen(teSensor eSensor);
PRIVATE void vUpdateNetworkScreen(teSensor eSensor);
PRIVATE void vLcdUpdateElement(tsNodeData *psNodeData, teSensor eSensor,
                               uint8 u8Row, uint8 u8Col, bool_t bShowLinkStatus);
PRIVATE void vDrawGraph(uint8 *pu8GraphData, uint8 u8StartCol,
                        uint8 u8StartRow);
PRIVATE void vStringCopy(char *pcFrom,char *pcTo);
PRIVATE void vValToDec(char *pcOutString, uint8 u8Value, char *pcLabel);
PRIVATE void button_adjustChannel(uint8 *pu8Value, uint8 u8MaxValue, uint8 u8OffValue, bool_t bUpNotDown);
PRIVATE uint8 u8UpdateTimeBlock(uint8 u8TimeBlock);
PRIVATE void vProcessSetChannelKeyPress(uint8 u8KeyMap);

PRIVATE void vSetTimer(void);


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
    vUtils_Debug("vJenie_CbConfigureNetwork");
    sDemoData.sSystem.u8Channel = CHANNEL_MID;

    vBuildSetChannelScreen();
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
    vUtils_Debug("exiting vJenie_CbConfigureNetwork");
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
    vUtils_Debug("vJenie_CbInit");

    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);

    if (bWarmStart==FALSE)
    {

        sHomeData.bStackReady=FALSE;
        sHomeData.eAppState = E_STATE_STARTUP;
        vUtils_Debug("E_STATE_STARTUP");
        vInitSystem();

        vInitCoord();

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
            vUtils_Debug("E_STATE_STARTUP");
            if (!(bJenie_GetPermitJoin() ))
            {
                eJenie_SetPermitJoin(TRUE);
            }

            if (sDemoData.sSystem.eState == E_STATE_SETUP_SCREEN)
            {
            }
            else
            {
                vBuildNetworkScreen(sDemoData.sGui.eCurrentSensor);
            }

            if (sDemoData.sNode.bLocalNode)
            {
                vLedControl(0,FALSE);
            }

            sHomeData.eAppState = E_STATE_RUNNING;
            break;

        case E_STATE_RUNNING:

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
            if (bTimer0Fired)
            {
                bTimer0Fired = FALSE;
                sHomeData.eAppState = E_STATE_RUNNING;
            }
            break;

        default:
            vUtils_Debug("default");
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

    /* Set up hardware and splash screen */
    vALSreset();
    vHTSreset();

    /* Start ambient light sensor now: it automatically keeps re-sampling after this */
    vALSstartReadChannel(0);

    eStateReadTemperature = E_STATE_READ_TEMP_START;
    eStateReadHumid       = E_STATE_READ_HUMID_START;

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
    tsNodeData        *psNodeData;
    tsNodeElementData *psNodeElementData;
    uint8             *pu8GraphData;
    int                i, j, k;

    /* Initialise node data */
    for (i = 0; i < DEMO_ENDPOINTS; i++)
    {
        psNodeData = &sDemoData.sNode.asNodeData[i];
        psNodeData->boDeviceOn = FALSE;
        psNodeData->u8FramesMissed = 0;
        /* Set label */
        for (j = 0; j < DEMO_SENSOR_LIST_LEN; j++)
        {
            psNodeElementData = &psNodeData->asNodeElementData[j];

            /* Clear alarms and values */
            psNodeElementData->u8NowValue = 0;
            psNodeElementData->u8HighAlarm = 0;
            psNodeElementData->u8LowAlarm = 255;

            /* Clear history list */
            pu8GraphData = psNodeElementData->au8GraphData;
            for (k = 0; k < DEMO_HISTORY_LEN; k++)
            {
                *pu8GraphData = 0;
                pu8GraphData++;
            }
        }
    }
    sDemoData.sNode.bLocalNode = TRUE; /* Default to local node in use */
    sDemoData.sNode.u8AssociatedNodes = 1; /* As local node is in use, 1 node is 'associated' */

    /* Initialise GUI state */
    sDemoData.sGui.eCurrentSensor = E_SENSOR_TEMP;
    sDemoData.sGui.u8CurrentNode = 0;
    sDemoData.sGui.u8GraphPos = 0;
    sDemoData.sGui.u8ControlSelection = 0;
    sDemoData.sGui.u8SetupSelection = 0;
    sDemoData.sGui.bShowFourNodes = TRUE;

    /* Get software version numbers */
    sDemoData.sSystem.u32AppApiVersion = u32Jenie_GetVersion(E_JENIE_COMPONENT_MAC);
    sDemoData.sSystem.u32JenieVersion = u32Jenie_GetVersion(E_JENIE_COMPONENT_JENIE);

    /* Set light sensor values to 'wrong' ends of range, so the first time
       a value is read they will get updated */
    sDemoData.sLightSensor.u16Hi = 0;
    sDemoData.sLightSensor.u16Lo = 65535;
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
    uint8 u8LocalSensor = 0;

    /* Process current block scheduled activity */
    switch (u8TimeBlock)
    {
    case BLOCK_UPDATE:
        /* Time to update the display */
        vProcessUpdateBlock();
        break;

    case BLOCK_START_TEMP:
        /* Time to start read of the temperature sensor. We read sensors
           even if we don't use the data, as the coordinator is assumed to
           be a device that doesn't have to be particularly economical on
           power */
        vHTSstartReadTemp();
        break;

    case BLOCK_READ_TEMP:
        /* Time to read the temperature sensor */
        if ((u32AHI_DioReadInput() & HTS_DATA_DIO_BIT_MASK) == 0)
        {
            u8LocalSensor = (uint8)u16HTSreadTempResult();

            if (u8LocalSensor > 52)
            {
                u8LocalSensor = 52;
            }

            if (sDemoData.sNode.bLocalNode)
            {
                sDemoData.sNode.asNodeData[0].asNodeElementData[E_SENSOR_TEMP].u8NowValue = u8LocalSensor;
                sDemoData.sNode.asNodeData[0].u8FramesMissed = 0;
            }

        }
        break;

    }
}


/****************************************************************************
 *
 * NAME: vProcessUpdateBlock
 *
 * DESCRIPTION:
 * Called once per second to update the scrolling graphs and, if showing a
 * screen with graphs on, updating the LCD.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vProcessUpdateBlock(void)
{
    tsNodeData *psNodeData;
    tsNodeElementData *psNodeElementData;
    uint8 *pu8GraphData;
    uint8 u8PrevPoint;
    uint8 u8Node;
    uint8 u8Sensor;
    uint8 u8Value;

    /* Update graphs */
    for (u8Node = 0; u8Node < DEMO_ENDPOINTS; u8Node++)
    {
        psNodeData = &sDemoData.sNode.asNodeData[u8Node];
        if (psNodeData->u8FramesMissed)
        {
            /* Missed data, so copy previous value forward */
            u8PrevPoint = (sDemoData.sGui.u8GraphPos - 1) & (DEMO_HISTORY_LEN - 1);
            for (u8Sensor = 0; u8Sensor < DEMO_SENSOR_LIST_LEN; u8Sensor++)
            {
                pu8GraphData = psNodeData->asNodeElementData[u8Sensor].au8GraphData;
                pu8GraphData[sDemoData.sGui.u8GraphPos] = pu8GraphData[u8PrevPoint];
            }
        }
        else
        {
            /* Data must be scaled for graph (0-13)
               Temp range is 0-52
               Humidity range is 0-104
               Light range is 0-6
            */
            for (u8Sensor = 0; u8Sensor < DEMO_SENSOR_LIST_LEN; u8Sensor++)
            {
                psNodeElementData = &psNodeData->asNodeElementData[u8Sensor];
                u8Value = psNodeElementData->u8NowValue;
                switch (u8Sensor)
                {
                case E_SENSOR_TEMP:
                    u8Value = u8Value >> 2;
                    break;

                case E_SENSOR_HTS:
                    u8Value = u8Value >> 3;
                    break;

                case E_SENSOR_ALS:
                    u8Value = u8Value * 2;
                    break;
                }
                if (u8Value > 13)
                {
                    u8Value = 13;
                }
                psNodeElementData->au8GraphData[sDemoData.sGui.u8GraphPos] = u8Value;
            }
        }

        /* For next time, assume failed until proven otherwise */
        if (psNodeData->u8FramesMissed < FRAMES_MISSED_INDICATION)
        {
            psNodeData->u8FramesMissed++;
        }
    }

    /* Increment graph position */
    sDemoData.sGui.u8GraphPos = (sDemoData.sGui.u8GraphPos + 1) & (DEMO_HISTORY_LEN - 1);

    /* Update display */
    switch (sDemoData.sSystem.eState)
    {
    case E_STATE_NETWORK:
        vUpdateNetworkScreen(sDemoData.sGui.eCurrentSensor);
        break;

    default:
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
                vProcessSetChannelKeyPress(u8KeysDown);
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
 * NAME: vBuildSetChannelScreen
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
PRIVATE void vBuildSetChannelScreen(void)
{
    vLcdClear();

    vLcdWriteBitmap((tsBitmap *)&sJennicLogo, 0, 1);

    vLcdWriteText("Ch", 7, 0);
    vLcdWriteText("\\", 7, 47);
    vLcdWriteText("]", 7, 74);
    vLcdWriteText("Done", 7, 103);

    /* Update to display the data */
    vUpdateSetChannelScreen();
}

/****************************************************************************
 *
 * NAME: vUpdateSetChannelScreen
 *
 * DESCRIPTION:
 * Updates the Set Channel screen, when it first appears or when the user
 * changes the channel number.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vUpdateSetChannelScreen(void)
{
    char acString[5];

    vValToDec(acString, sDemoData.sSystem.u8Channel, "  ");
    vLcdWriteText(acString, 7, 16);

    vLcdRefreshAll();
}

/****************************************************************************
 *
 * NAME: vBuildNetworkScreen
 *
 * DESCRIPTION:
 * Creates the Network screen. Depending on how the GUI has been configured
 * it may want to display up to 3 or up to 4 nodes simultaneuously. Also, it
 * only shows nodes that have successfully associated. To achieve this, it
 * makes use of an array of the four display positions on the screen, and
 * loops through this to position each node in the correct position.
 *
 * This function only draws the text required, then uses the related update
 * function to display the actual data and to refresh the LCD.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  eSensor         R   Sensor to display
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vBuildNetworkScreen(teSensor eSensor)
{
    static const char *apcSensorLabel[DEMO_SENSOR_LIST_LEN] =
        {
            "Temp", "Humidity", "Light"
        };
    static const uint8 au8LabelPos[DEMO_SENSOR_LIST_LEN] =
        {
            29, 58, 102
        };
    uint8 u8Node;
    int iPos;

    vLcdClear();
    /* If showing four nodes, they appear on screen as follows:
         Node 0    Node 1
         Node 2    Node 3

       If showing only three nodes, they appear as follows:
         Info      Node 0
         Node 1    Node 2
    */
    if (sDemoData.sGui.bShowFourNodes)
    {
        iPos = 0;
    }
    else
    {
        iPos = 1;
        vLcdWriteText("Network", 0, 0);
        vLcdWriteText("overview", 1, 0);
    }

    /* Show labels */
    if (sDemoData.sNode.u8AssociatedNodes == 0)
    {
        vLcdWriteText("No nodes associated", 3, 0);
    }
    else
    {
        u8Node = 0;
        while ((u8Node < sDemoData.sNode.u8AssociatedNodes) && (iPos < 4))
        {
            vLcdWriteText((char *)apcNodeNameList[u8Node], au8NodeLcdRow[iPos], au8NodeLcdCol[iPos]);
            u8Node++;
            iPos++;
        }
    }

    /* Hot buttons at bottom of screen */
    vLcdWriteText("Node", 7, 0);
    for (iPos = 0; iPos < DEMO_SENSOR_LIST_LEN; iPos++)
    {
        vLcdWriteText((char *)apcSensorLabel[iPos], 7, au8LabelPos[iPos]);
    }
    vLcdWriteInvertedText((char *)apcSensorLabel[eSensor], 7, au8LabelPos[eSensor]);

    vUpdateNetworkScreen(eSensor);
}

/****************************************************************************
 *
 * NAME: vUpdateNetworkScreen
 *
 * DESCRIPTION:
 * Draws the graphs and values for the Network screen. See the description
 * for vBuildNetworkScreen for an explanation of the positioning of elements
 * on the display.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  eSensor         R   Sensor to display
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vUpdateNetworkScreen(teSensor eSensor)
{
    uint8 u8Node;
    uint8 u8Row;
    uint8 u8Col;
    int iPos;
    bool_t bShowLinkStatus;

    /* If showing four nodes, they appear on screen as follows:
         Node 0    Node 1
         Node 2    Node 3

       If showing only three nodes, they appear as follows:
         Info      Node 0
         Node 1    Node 2
    */
    if (sDemoData.sGui.bShowFourNodes)
    {
        iPos = 0;
    }
    else
    {
        iPos = 1;
    }

    u8Node = 0;
    bShowLinkStatus = sDemoData.sNode.bLocalNode ? FALSE : TRUE;

    while ((u8Node < sDemoData.sNode.u8AssociatedNodes) && (iPos < 4))
    {
        u8Row = au8NodeLcdRow[iPos] + 1;
        u8Col = au8NodeLcdCol[iPos];

        vLcdUpdateElement(&sDemoData.sNode.asNodeData[u8Node],
                          eSensor, u8Row, u8Col, bShowLinkStatus);

        u8Node++;
        iPos++;
        bShowLinkStatus = TRUE;
    }

    vLcdRefreshAll();
}

/****************************************************************************
 *
 * NAME: vLcdUpdateElement
 *
 * DESCRIPTION:
 * Draws the graph and text for a single sensor for a single node. The text
 * includes alarm indications if the sensor value exceeds user specified
 * limits.
 *
 * PARAMETERS:  Name                RW  Usage
 *              psNodeElementData   R   Pointer to data for node
 *              u8Row               R   Character row to display on
 *              u8Col               R   Pixel column to display on
 *              bShowLinkStatus           R   True to show RSSI data
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vLcdUpdateElement(tsNodeData *psNodeData, teSensor eSensor,
                               uint8 u8Row, uint8 u8Col, bool_t bShowLinkStatus)
{
    char acString[10];
    uint8 u8NowValue;
    tsNodeElementData *psNodeElementData = &psNodeData->asNodeElementData[eSensor];

    u8NowValue = psNodeElementData->u8NowValue;

    switch (eSensor)
    {
    case E_SENSOR_TEMP:
        vValToDec(acString, u8NowValue, "[C ");
        break;

    case E_SENSOR_HTS:
        vValToDec(acString, u8NowValue, "% ");
        break;

    case E_SENSOR_ALS:
        /* This is a light sensor so display symbol */
        acString[0] = '&' + u8NowValue;
        acString[1] = '\0';
        break;

    default:
        break;
    }

    vLcdWriteText(acString, u8Row, u8Col);

    /* Print alarm */
    vLcdWriteText("       ", (uint8)(u8Row + 1), u8Col);

    if ((u8NowValue >= psNodeElementData->u8HighAlarm)
            && (psNodeElementData->u8HighAlarm != 0))
    {
        vLcdWriteInvertedText("High", (uint8)(u8Row + 1), u8Col);
    }
    else
    {
        if ((u8NowValue <= psNodeElementData->u8LowAlarm)
                && (psNodeElementData->u8LowAlarm != 255))
        {
            vLcdWriteInvertedText("Low", (uint8)(u8Row + 1), u8Col);
        }
        else
        {

            if (bShowLinkStatus)
            {
                // vValToDec(acString, psNodeData->u8Rssi, "");
                // vLcdWriteText(acString, (uint8)(u8Row + 1), u8Col);
                vValToDec(acString, psNodeData->u8FramesMissed - 1, "");
                vLcdWriteText(acString, (uint8)(u8Row + 1), u8Col + 20);
            }
        }
    }

    /* Draw graph */
    vDrawGraph(psNodeElementData->au8GraphData, (uint8)(u8Col + 27), u8Row);
}


/****************************************************************************
 *
 * NAME: vDrawGraph
 *
 * DESCRIPTION:
 * Creates a bitmap from an array of values. Each value is represented by a
 * column on the graph, and a lookup table is used to translate each value
 * (assumed to be in the range 0 to 13) to the data required for the bitmap.
 * Finally, the bitmap is displayed via a board API.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  pu8GraphData    R   Array of 32 elements of graph data
 *                  u8StartCol      R   First column of bitmap
 *                  u8StartRow      R   Top character row of bitmap
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vDrawGraph(uint8 *pu8GraphData, uint8 u8StartCol,
                        uint8 u8StartRow)
{
    static const uint16 au16LineData[14] =
        {
            0x4000, 0x6000, 0x7000, 0x7800,
            0x7c00, 0x7e00, 0x7f00, 0x7f80,
            0x7fc0, 0x7fe0, 0x7ff0, 0x7ff8,
            0x7ffc, 0x7ffe
        };
    uint8 au8GraphBitmap[66];
    const tsBitmap sGraphBitmap =
        {
            au8GraphBitmap, 33, 2
        };
    int    i;
    uint16 u16LineData;
    uint8  u8DataPos = sDemoData.sGui.u8GraphPos;

    /* Draw y axis */
    au8GraphBitmap[0] = 0xfe;
    au8GraphBitmap[33] = 0x7f;

    /* Fill in data */
    for (i = 1; i <= DEMO_HISTORY_LEN; i += 1)
    {
        u16LineData = au16LineData[pu8GraphData[u8DataPos]];

        au8GraphBitmap[i] = (uint8)(u16LineData & 0xff);
        au8GraphBitmap[i + 33] = (uint8)(u16LineData >> 8);

        /* Increment data point */
        u8DataPos = (u8DataPos + 1) & (DEMO_HISTORY_LEN - 1);
    }

    /* Write bitmap to shadow memory */
    vLcdWriteBitmap((tsBitmap *)&sGraphBitmap, u8StartCol, u8StartRow);
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
 * NAME: vProcessSetChannelKeyPress
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
PRIVATE void vProcessSetChannelKeyPress(uint8 u8KeyMap)
{
    switch (u8KeyMap)
    {
    case E_KEY_0:
        /* Further setup button: go to setup screen */
        sDemoData.sSystem.eState = E_STATE_SETUP_SCREEN;
        vLcdWriteTextToClearLine("Initialising",7,0);
        vLcdRefreshArea(0,7,128,1);
        break;

    case E_KEY_1:
        /* Plus button: increment value */
    case E_KEY_2:
        /* Minus button: decrement value */

        button_adjustChannel(&sDemoData.sSystem.u8Channel, CHANNEL_MAX, CHANNEL_MIN, u8KeyMap == E_KEY_1);
        vUpdateSetChannelScreen();
        break;

    case E_KEY_3:
        /* Done button: start beaconing and go to network screen */
        // vStartBeacon();
        sDemoData.sSystem.eState = E_STATE_NETWORK;
        vLcdWriteTextToClearLine("Initialising",7,0);
        vLcdRefreshArea(0,7,128,1);
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
            && (sDemoData.sSystem.eState != E_STATE_SETUP_SCREEN)
            && (sDemoData.sSystem.eState != E_STATE_SCANNING))
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
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, sDemoData.sSystem.u32CalibratedTimeout);
}