/*****************************************************************************
 *
 * MODULE:              Demo board LED controls
 *
 * COMPONENT:           $RCSfile: MyLedControl.h,v $
 *
 * VERSION:             $Name: $
 *
 * REVISION:            $Revision: 1.3 $
 *
 * DATED:               $Date: 2008-07-09 10:40:13 $
 *
 * STATUS:              $State: Exp $
 *
 * AUTHOR:              CJG
 *
 * DESCRIPTION:
 * Macros to make it easier to drive LEDs on demo boards
 *
 * LAST MODIFIED BY:    $Author: moz $
 *                      $Modtime: $
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

#ifndef LED_CONTROL_INCLUDED
#define LED_CONTROL_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "AppHardwareApi.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define LED_CTRL_BASE_BIT 16        //14
#define LED_CTRL_MASK_RFD 0x03
#define LED_CTRL_MASK_FFD 0x0f

#define vLedInitRfd() \
            vAHI_DioSetDirection(0, (LED_CTRL_MASK_RFD << LED_CTRL_BASE_BIT))
#define vLedInitFfd() \
            vAHI_DioSetDirection(0, (LED_CTRL_MASK_FFD << LED_CTRL_BASE_BIT))
#define vLedControl(LED,ON) \
            vAHI_DioSetOutput((ON) ? 0 : (1 << (LED_CTRL_BASE_BIT + LED)), \
                              (ON) ? (1 << (LED_CTRL_BASE_BIT + LED)) : 0)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* LED_CONTROL_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

/*  R&DRelease_31May2007 Fri Jun 1 10:20:58 BST 2007 */
