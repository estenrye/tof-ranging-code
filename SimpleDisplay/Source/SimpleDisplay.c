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
	/* Starting LCD and buttons here so channel can be set */
	vButtonInitFfd();    // Docs: JN-RM-2003 Page 45
	vLcdResetDefault();  // Docs: JN-RM-2003 Page 53

	app_WriteHelloWorld();
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
