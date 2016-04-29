/**************************************************************************//**
 * @file
 * @brief Capacitive sense driver
 * @version 3.20.9
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/



#ifndef __CAPLESENSE_H_
#define __CAPLESENSE_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(TARGET_EFM32GG_STK3700)
#include "capsenseconfig_gg_stk.h"
#elif defined(TARGET_EFM32TG_STK3300)
#include "capsenseconfig_tg_stk.h"
#elif defined(TARGET_EFM32LG_STK3600)
#include "capsenseconfig_lg_stk.h"
#elif defined(TARGET_EFM32WG_STK3800)
#include "capsenseconfig_wg_stk.h"
#else
#error "Unknown target for EFM32 CapSenseSlider driver."
#endif

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup CapSense
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  CAPLESENSE_getSegmentChannel(uint8_t capSegment);
uint32_t CAPLESENSE_getVal(uint8_t channel);
uint32_t CAPLESENSE_getNormalizedVal(uint8_t channel);
int32_t CAPLESENSE_getSliderPosition(void);
void CAPLESENSE_Init(bool sleep);
void CAPLESENSE_setupLESENSE(bool sleep);
void CAPLESENSE_setupCallbacks(void (*scanCb)(void), void (*chCb)(void));
void CAPLESENSE_Sleep(void);

#ifdef __cplusplus
}
#endif

/** @} (end group CapSense) */
/** @} (end group Drivers) */

#endif /* __CAPSENSE_H_ */
