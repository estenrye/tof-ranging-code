/*
 * SimpleDisplay.c
 *
 *  Created on: Apr 28, 2018
 *      Author: esten
 */

#define MAX_BLOCKS                  20

/* All application data with scope within the entire file is kept here, */
typedef struct {
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



PRIVATE tsHomeData sHomeData;

PUBLIC void vJenie_CbMain(void)
{
    static uint8 u8TimeBlock = MAX_BLOCKS;
    uint8 u8Keys = 0;

    /* regular watch dog reset */
    #ifdef WATCHDOG_ENABLED
       vAHI_WatchdogRestart();
    #endif

    if (sHomeData.bStackReady)
    {
        switch (sHomeData.eAppState)
        {
        case E_STATE_STARTUP:
            #ifdef DEBUG
                vUtils_Debug("Startup");
            #endif
            if (!(bJenie_GetPermitJoin() ))
            {
                eJenie_SetPermitJoin(TRUE);
            }

            if (sDemoData.sSystem.eState == E_STATE_SETUP_SCREEN)
            {
                vBuildSetupScreen();
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
            break;
        }
    }

}
