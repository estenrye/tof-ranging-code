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

/* Button values */
typedef enum
{
    E_KEY_0 = BUTTON_0_MASK,
    E_KEY_1 = BUTTON_1_MASK,
    E_KEY_2 = BUTTON_2_MASK,
    E_KEY_3 = BUTTON_3_MASK,
    E_KEYS_0_AND_3 = (BUTTON_0_MASK | BUTTON_3_MASK)
} teKeyValues;

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
	/*
    uint8 u8Keys = 0;
    volatile uint32 wait;
	*/
    /* Starting LCD and buttons here so channel can be set */
	/*
    vButtonInitFfd();
    vLcdResetDefault();

    sDemoData.sSystem.u8Channel = CHANNEL_MID;

    vBuildSetChannelScreen();
    */
    /* Change to channel setting state */
	/*
    sDemoData.sSystem.eState = E_STATE_SET_CHANNEL;
	*/
    /* Loop while on set channel screen */
	/*
    while ((sDemoData.sSystem.eState == E_STATE_SET_CHANNEL))
    {
        (void)bProcessKeys(&u8Keys);
        for (wait = 0; wait < 100000; wait++);
        bKeyDebounce = FALSE;
    }
	*/
    /* Set PAN_ID and other network stuff or defaults will be used */
	/*
    gJenie_Channel = sDemoData.sSystem.u8Channel;
    gJenie_NetworkApplicationID=0xdeaddead;
    gJenie_PanID   = DEMO_PAN_ID;
	*/
    /* Configure stack with routing table data */
	/*
    gJenie_RoutingEnabled    = TRUE;
    gJenie_RoutingTableSize  = 100;
    gJenie_RoutingTableSpace = (void *)asRoutingTable;
    */
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
	/*
    vUtils_Init();

    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_1, TRUE);

    if (bWarmStart==FALSE)
    {

        sHomeData.bStackReady=FALSE;
        sHomeData.eAppState = E_STATE_STARTUP;

        vInitSystem();

        vInitCoord();

        vSetTimer();
        switch (eJenie_Start(E_JENIE_COORDINATOR))        // Start network as coordinator
        {
        case E_JENIE_SUCCESS:
            #ifdef HIGH_POWER
                // Set high power mode
                eJenie_RadioPower(18, TRUE);
            #endif
            break;

        case E_JENIE_ERR_UNKNOWN:
        case E_JENIE_ERR_INVLD_PARAM:
        case E_JENIE_ERR_STACK_RSRC:
        case E_JENIE_ERR_STACK_BUSY:

        default:
            // Do something on failure??
            break;
        }
    }
    */
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
	/*
    if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & (1 << E_AHI_SYSCTRL_WK0)))      // added for timer 0 interrupt
    {
        bTimer0Fired = TRUE;

    } else if ((u32DeviceId == E_AHI_DEVICE_SYSCTRL)
                && (u32ItemBitmap & E_AHI_SYSCTRL_WK1_MASK) )
    {
        bKeyDebounce = FALSE;
    }
    */
}

