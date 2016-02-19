/************************************************************************************
 * arch/arm/src/stm32/stm32_comp.h
 *
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_COMP_H
#define __ARCH_ARM_SRC_STM32_STM32_COMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32_STM32L4X3)
#include "chip/stm32l4x3xx_comp.h"
#elif defined(CONFIG_STM32_STM32L4X6)
#include "chip/stm32l4x6xx_comp.h"
#endif

#ifndef __ASSEMBLY__
#include <stdbool.h>
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32L4X3)
/* Comparators **********************************************************************/
typedef enum {
    STM32_COMP1,
    STM32_COMP2,
    STM32_COMP_NUM  /* Number of comparators */
} stm32_comp_t;

/* Plus input ***********************************************************************/
typedef enum {
    STM32_COMP_INP_PIN_1,   /* COMP1: PC5, COMP2: PB4 */
    STM32_COMP_INP_PIN_2,   /* COMP1: PB2, COMP2: PB6 */
    STM32_COMP_INP_PIN_3,   /* COMP1: PA1, COMP2: PA3 */
} stm32_comp_inp_t;

/* Minus input ***********************************************************************/
typedef enum {
    STM32_COMP_INM_1_4_VREF,
    STM32_COMP_INM_1_2_VREF,
    STM32_COMP_INM_3_4_VREF,
    STM32_COMP_INM_VREF,
    STM32_COMP_INM_DAC_1,
    STM32_COMP_INM_DAC_2,
    STM32_COMP_INM_PIN_1,   /* COMP1: PB1, COMP2: PB3 */
    STM32_COMP_INM_PIN_2,   /* COMP1: PC4, COMP2: PB7 */
    STM32_COMP_INM_PIN_3,   /* COMP1: PA0, COMP2: PA2 */
    STM32_COMP_INM_PIN_4,   /* COMP1: PA4, COMP2: PA4 */
    STM32_COMP_INM_PIN_5,   /* COMP1: PA5, COMP2: PA5 */
} stm32_comp_inm_t;
#elif defined(CONFIG_STM32_STM32L4X6)
/* Comparators **********************************************************************/
typedef enum {
    STM32_COMP1,
    STM32_COMP2,
    STM32_COMP_NUM  /* Number of comparators */
} stm32_comp_t;

/* Plus input ***********************************************************************/
typedef enum {
    STM32_COMP_INP_PIN_1,   /* COMP1: PC5, COMP2: PB4 */
    STM32_COMP_INP_PIN_2,   /* COMP1: PB2, COMP2: PB6 */
} stm32_comp_inp_t;

/* Minus input ***********************************************************************/
typedef enum {
    STM32_COMP_INM_1_4_VREF,
    STM32_COMP_INM_1_2_VREF,
    STM32_COMP_INM_3_4_VREF,
    STM32_COMP_INM_VREF,
    STM32_COMP_INM_DAC_1,
    STM32_COMP_INM_DAC_2,
    STM32_COMP_INM_PIN_1,   /* COMP1: PB1, COMP2: PB3 */
    STM32_COMP_INM_PIN_2,   /* COMP1: PC4, COMP2: PB7 */
} stm32_comp_inm_t;
#else
/* Unsupported STM32 COMP */
typedef int stm32_comp_t;
typedef int stm32_comp_inp_t;
typedef int stm32_comp_inm_t;
#endif

/* Hysteresis ************************************************************************/
typedef enum {
    STM32_COMP_HYST_NONE,
    STM32_COMP_HYST_LOW,
    STM32_COMP_HYST_MEDIUM,
    STM32_COMP_HYST_HIGH,
} stm32_comp_hyst_t;

/* Power/Speed Modes ******************************************************************/
typedef enum {
    STM32_COMP_SPEED_HIGH,
    STM32_COMP_SPEED_MEDIUM,
    STM32_COMP_SPEED_LOW,
} stm32_comp_speed_t;

/* Comparator configuration ***********************************************************/
typedef struct stm32_comp_config
{
    stm32_comp_inp_t inp;       /* minus input */
    stm32_comp_inm_t inm;       /* plus input */
    stm32_comp_hyst_t hyst;     /* hysteresis */
    stm32_comp_speed_t speed;   /* speed */
    bool inverted;              /* invert output? */
} stm32_comp_config_s;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: stm32_compconfig
 *
 * Description:
 *   Configure comparator and I/Os used as comparators inputs
 *
 * Parameters:
 *  - cmp: comparator
 *  - cfg: configuration
 *
 * Returns:
 *  0 on success, error otherwise
 *
 ************************************************************************************/

EXTERN int stm32_compconfig(stm32_comp_t cmp, const stm32_comp_config_s* cfg);

/************************************************************************************
 * Name: stm32_compenable
 *
 * Description:
 *   Enable/disable comparator
 *
 * Parameters:
 *  - cmp: comparator
 *  - cfg: enable/disable flag
 *
 * Returns:
 *  0 on success, error otherwise
 *
 ************************************************************************************/

EXTERN int stm32_compenable(stm32_comp_t cmp, bool en);

/************************************************************************************
 * Name: stm32_compread
 *
 * Description:
 *   Read comparator output
 *
 * Parameters:
 *  - cmp: comparator
 *
 * Returns:
 *  true for high, false for low
 *
 ************************************************************************************/

EXTERN bool stm32_compread(stm32_comp_t cmp);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32_STM32_COMP_H */
