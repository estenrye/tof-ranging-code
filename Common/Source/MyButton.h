/*****************************************************************************
 *
 * MODULE:              Test board button controls
 *
 * COMPONENT:           $RCSfile: MyButton.h,v $
 *
 * VERSION:             $Name:  $
 *
 * REVISION:            $Revision: 1.3 $
 *
 * DATED:               $Date: 2008-07-09 10:40:13 $
 *
 * STATUS:              $State: Exp $
 *
 * AUTHOR:              GP
 *
 * DESCRIPTION:
 * Macros to make it easier to read buttons on demo boards
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

#ifndef BUTTON_INCLUDED
#define BUTTON_INCLUDED

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
#define BUTTON_BASE_BIT     9
#define BUTTON_0_MASK       1
#define BUTTON_1_MASK       4
#define BUTTON_2_MASK       8
#define BUTTON_3_MASK       16


#define BUTTON_ALL_MASK_RFD (BUTTON_0_MASK | BUTTON_1_MASK)
#define vButtonInitRfd() \
            vAHI_DioSetDirection((BUTTON_ALL_MASK_RFD << BUTTON_BASE_BIT), 0)
#define u8ButtonReadRfd() \
            ((uint8)(((u32AHI_DioReadInput() >> BUTTON_BASE_BIT) \
                      & BUTTON_ALL_MASK_RFD) ^ BUTTON_ALL_MASK_RFD))

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

#endif  /* BUTTON_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

