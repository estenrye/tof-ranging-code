/****************************************************************************
 *
 * MODULE:             Serial I/O Config
 *
 * COMPONENT:          $RCSfile: siocfg.h,v $
 *
 * VERSION:            $Name: $
 *
 * REVISION:           $Revision: 1.3 $
 *
 * DATED:              $Date: 2008-07-09 10:40:13 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:
 *
 * DESCRIPTION
 * Serial I/O Config
 *
 * LAST MODIFIED BY:   $Author: moz $
 *                     $Modtime: $
 *
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

#ifndef  SIO_CONFIG
#define  SIO_CONFIG

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Define which of the two available hardware UARTs and what baud rate to use */
//#define UART                    E_AHI_UART_1
#define UART                    E_AHI_UART_0


#define UART_BAUD_RATE          E_AHI_UART_RATE_19200

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

#endif  /* SIO_CONFIG*/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/


