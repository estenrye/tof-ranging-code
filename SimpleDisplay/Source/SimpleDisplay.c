
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
PUBLIC void app_WriteHelloWorld(void);

// Application Screens
PUBLIC void lcd_BuildChannelSelectionScreen(void);
PUBLIC void lcd_UpdateChannelSelectionScreen(void);

// Button Handlers
PRIVATE bool_t button_ProcessKeys(uint8 *pu8Keys);
PRIVATE void button_ProcessSetChannelKeyPressHandler(uint8 u8KeyMap);

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* System states with respect to screen display being shown */
typedef enum
{
    E_STATE_NETWORK,
    E_STATE_SET_CHANNEL,
    E_STATE_SETUP_SCREEN
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

typedef struct {
	struct
    {
        teState eState;
        uint8   u8Channel;
    } sSystem;
} tsApplicationState;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsApplicationState sAppState;
PRIVATE bool_t bKeyDebounce = FALSE;
/* Routing table storage */
PRIVATE tsJenieRoutingTable asRoutingTable[100];

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define ONE_MSEC_IN_32KHZ_CYCLES    32
#define BUTTON_DEBOUNCE             (500 * ONE_MSEC_IN_32KHZ_CYCLES)
/* PAN ID on which demo operates */
#define DEMO_PAN_ID                       0x0e1c
/* Channels available */
#define CHANNEL_MIN                       11
#define CHANNEL_MID                       18
#define CHANNEL_MAX                       26

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
}

PUBLIC void vJenie_CbInit(bool_t bWarmStart)
{
	/* to be implemented */
	app_WriteHelloWorld();
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


PUBLIC void app_WriteHelloWorld(void)
{
	/* Docs: JN-RM-2003 
	 * Page 56: void vLcdClear()
	 * Page 57: void vLcdWriteText(char *pcString, uint8 u8Row, uint8 u8Column)
	 * Page 58: void vLcdWriteTextRightJustified(char *pcString, uint8 u8Row, uint8 u8EndColumn)
	 * Page 66: void vLcdRefreshAll(void)
	 */
    vLcdClear();  
	vLcdWriteText("Esten Rye", 1, 0);
	vLcdWriteTextRightJustified("SEIS 740", 1, 127);
    vLcdWriteText("TOF Ranging Project", 2, 0);
    vLcdWriteText("Hello World", 6, 0);
	vLcdWriteText("I am here", 7, 0);
    vLcdRefreshAll();
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
	char displayOutput[5];
	vUtils_ValToDec(displayOutput, sAppState.sSystem.u8Channel);
	vLcdWriteText(displayOutput, 7, 16);
    vLcdRefreshAll();
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
        vLcdWriteTextToClearLine("Initialising",7,0);
        vLcdRefreshArea(0,7,128,1);
        break;

    default:
        break;
    }
}