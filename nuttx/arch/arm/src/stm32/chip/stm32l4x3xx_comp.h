/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l4x3xx_comp.h
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_COMP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_COMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "stm32l4x3xx_memorymap.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define COMP1_CSR_OFFSET            0x0000  /* Comparator 1 control/status register */
#define COMP2_CSR_OFFSET            0x0004  /* Comparator 2 control/status register */

/* Register Addresses ***************************************************************/

#define COMP1_CSR                   (STM32_COMP_BASE + COMP1_CSR_OFFSET)
#define COMP2_CSR                   (STM32_COMP_BASE + COMP2_CSR_OFFSET)

/* Control/status register **********************************************************/

#define COMP_CSR_EN                 (1 << 0)    /* Bit 0: Comparator enable */
#define COMP_CSR_PWRMODE_SHIFT      (2)         /* Bits 2-3: Power modes */
#define COMP_CSR_PWRMODE_MASK       (3 << COMP_CSR_PWRMODE_SHIFT)
#  define COMP_CSR_PWRMODE_HSPD     (0 << COMP_CSR_PWRMODE_SHIFT) /* OO: High Speed */
#  define COMP_CSR_PWRMODE_MSPD     (1 << COMP_CSR_PWRMODE_SHIFT) /* 01 or 10: Medium speed */
#  define COMP_CSR_PWRMODE_LSPD     (3 << COMP_CSR_PWRMODE_SHIFT) /* 11: Low speed */
#define COMP_CSR_INMSEL_SHIFT       (4)         /* Bits 4-6: Input minus selection */
#define COMP_CSR_INMSEL_MASK        (7 << COMP_CSR_INMSEL_SHIFT)
#  define COMP_CSR_INMSEL_1_4_VREF  (0 << COMP_CSR_INMSEL_SHIFT) /* 000: 1/4 Vrefint */
#  define COMP_CSR_INMSEL_1_2_VREF  (1 << COMP_CSR_INMSEL_SHIFT) /* 001: 1/2 Vrefint */
#  define COMP_CSR_INMSEL_3_4_VREF  (2 << COMP_CSR_INMSEL_SHIFT) /* 010: 3/4 Vrefint */
#  define COMP_CSR_INMSEL_VREF      (3 << COMP_CSR_INMSEL_SHIFT) /* 011: Vrefint */
#  define COMP_CSR_INMSEL_DAC_1     (4 << COMP_CSR_INMSEL_SHIFT) /* 100: DAC Ch 1 */
#  define COMP_CSR_INMSEL_DAC_2     (5 << COMP_CSR_INMSEL_SHIFT) /* 101: DAC Ch 2 */
#  define COMP_CSR_INMSEL_PIN_1     (6 << COMP_CSR_INMSEL_SHIFT) /* 110: COMP1 is PB1, COMP2 is PB3 */
#  define COMP_CSR_INMSEL_INMESEL   (7 << COMP_CSR_INMSEL_SHIFT) /* 111: Selected by INMESEL */
#define COMP_CSR_INPSEL_SHIFT       (7)         /* Input plus selection */
#  define COMP_CSR_INPSEL_MASK      (3 << COMP_CSR_INPSEL_SHIFT)
#  define COMP_CSR_INPSEL_PIN_1     (0 << COMP_CSR_INPSEL_SHIFT) /* 00: COMP1 is PC5, COMP2 is PB4 */
#  define COMP_CSR_INPSEL_PIN_2     (1 << COMP_CSR_INPSEL_SHIFT) /* 01: COMP1 is PB2, COMP2 is PB6 */
#  define COMP_CSR_INPSEL_PIN_3     (2 << COMP_CSR_INPSEL_SHIFT) /* 10: COMP1 is PA1, COMP2 is PA3 */
#define COMP_CSR_POLARITY_INV       (1 << 15)   /* Bit 15: Invert output value */
#define COMP_CSR_HYST_SHIFT         (16)        /* Bits 16-17: Hysteresis selection */
#define COMP_CSR_HYST_MASK          (3 << COMP_CSR_HYST_SHIFT)
#  define COMP_CSR_HYST_NONE        (0 << COMP_CSR_HYST_SHIFT) /* 00: No hysteresis */
#  define COMP_CSR_HYST_LOW         (1 << COMP_CSR_HYST_SHIFT) /* 01: Low hysteresis */
#  define COMP_CSR_HYST_MED         (2 << COMP_CSR_HYST_SHIFT) /* 10: Medium hysteresis */
#  define COMP_CSR_HYST_HIGH        (3 << COMP_CSR_HYST_SHIFT) /* 11: High hysteresis */
#define COMP_CSR_BLANK_SHIFT        (18)        /* Bits 18-20: Blanking source selection */
#define COMP_CSR_BLANK_MASK         (7 << COMP_CSR_BLANK_SHIFT)
#  define COMPX_CSR_BLANK_NONE      (0 << COMP_CSR_BLANK_SHIFT) /* 000: No blanking */
#  define COMP1_CSR_BLANK_TIM1_OC5  (1 << COMP_CSR_BLANK_SHIFT) /* 001: COMP1 blanking source is TIM1 OC5 */
#  define COMP1_CSR_BLANK_TIM2_OC3  (2 << COMP_CSR_BLANK_SHIFT) /* 010: COMP1 blanking source is TIM2 OC3 */
#  define COMP2_CSR_BLANK_TIM15_OC1 (4 << COMP_CSR_BLANK_SHIFT) /* 001: COMP2 blanking source is TIM15 OC1 */
#define COMP_CSR_BRGEN              (1 << 22)   /* Bit 22: Scaler bridge enable */
#define COMP_CSR_SCALEN             (1 << 23)   /* Bit 23: Voltage scaler enable */
#define COMP_CSR_INMESEL_SHIFT      (25)        /* Bits 25-26: Input minus extended selection */
#define COMP_CSR_INMESEL_MASK       (3 << COMP_CSR_INMESEL_SHIFT)
#  define COMP_CSR_INMESEL_PIN_2    (0 << COMP_CSR_INMESEL_SHIFT) /* 00: COMP1 is PC4, COMP2 is PB7 */
#  define COMP_CSR_INMESEL_PIN_3    (1 << COMP_CSR_INMESEL_SHIFT) /* 01: COMP1 is PA0, COMP2 is PA2 */
#  define COMP_CSR_INMESEL_PIN_4    (2 << COMP_CSR_INMESEL_SHIFT) /* 10: PA4 */
#  define COMP_CSR_INMESEL_PIN_5    (3 << COMP_CSR_INMESEL_SHIFT) /* 11: PA5 */
#define COMP_CSR_VALUE              (1 << 30)   /* Bit 30: Output value */
#define COMP_CSR_LOCK               (1 << 31)   /* Bit 31: Register lock */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_COMP_H */
