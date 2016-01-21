/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l4x3xx_pwr.h
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_PWR_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_PWR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_PWR_CR1_OFFSET   0x0000  /* Power control register 1 */
#define STM32_PWR_CR2_OFFSET   0x0004  /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET   0x0008  /* Power control register 3 */
#define STM32_PWR_CR4_OFFSET   0x000c  /* Power control register 4 */
#define STM32_PWR_SR1_OFFSET   0x0010  /* Power status register 1 */
#define STM32_PWR_SR2_OFFSET   0x0014  /* Power status register 2 */
#define STM32_PWR_SCR_OFFSET   0x0018  /* Power status clear register */
#define STM32_PWR_PUCRA_OFFSET 0x0020  /* Power Port A pull-up control register */
#define STM32_PWR_PDCRA_OFFSET 0x0024  /* Power Port A pull-down control register */
#define STM32_PWR_PUCRB_OFFSET 0x0028  /* Power Port B pull-up control register */
#define STM32_PWR_PDCRB_OFFSET 0x002c  /* Power Port B pull-down control register */
#define STM32_PWR_PUCRC_OFFSET 0x0030  /* Power Port C pull-up control register */
#define STM32_PWR_PDCRC_OFFSET 0x0034  /* Power Port C pull-down control register */
#define STM32_PWR_PUCRD_OFFSET 0x0038  /* Power Port D pull-up control register */
#define STM32_PWR_PDCRD_OFFSET 0x003c  /* Power Port D pull-down control register */
#define STM32_PWR_PUCRE_OFFSET 0x0040  /* Power Port E pull-up control register */
#define STM32_PWR_PDCRE_OFFSET 0x0044  /* Power Port E pull-down control register */
#define STM32_PWR_PUCRF_OFFSET 0x0048  /* Power Port F pull-up control register */
#define STM32_PWR_PDCRF_OFFSET 0x004c  /* Power Port F pull-down control register */
#define STM32_PWR_PUCRG_OFFSET 0x0050  /* Power Port G pull-up control register */
#define STM32_PWR_PDCRG_OFFSET 0x0054  /* Power Port G pull-down control register */
#define STM32_PWR_PUCRH_OFFSET 0x0058  /* Power Port H pull-up control register */
#define STM32_PWR_PDCRH_OFFSET 0x005c  /* Power Port H pull-down control register */

/* Register Addresses ***************************************************************/

#define STM32_PWR_CR1          (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2          (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3          (STM32_PWR_BASE+STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4          (STM32_PWR_BASE+STM32_PWR_CR4_OFFSET)
#define STM32_PWR_SR1          (STM32_PWR_BASE+STM32_PWR_SR1_OFFSET)
#define STM32_PWR_SR2          (STM32_PWR_BASE+STM32_PWR_SR2_OFFSET)
#define STM32_PWR_SCR          (STM32_PWR_BASE+STM32_PWR_SCR_OFFSET)
#define STM32_PWR_PUCRA        (STM32_PWR_BASE+STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA        (STM32_PWR_BASE+STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB        (STM32_PWR_BASE+STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB        (STM32_PWR_BASE+STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC        (STM32_PWR_BASE+STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC        (STM32_PWR_BASE+STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD        (STM32_PWR_BASE+STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD        (STM32_PWR_BASE+STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE        (STM32_PWR_BASE+STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE        (STM32_PWR_BASE+STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF        (STM32_PWR_BASE+STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF        (STM32_PWR_BASE+STM32_PWR_PDCRF_OFFSET)
#define STM32_PWR_PUCRG        (STM32_PWR_BASE+STM32_PWR_PUCRG_OFFSET)
#define STM32_PWR_PDCRG        (STM32_PWR_BASE+STM32_PWR_PDCRG_OFFSET)
#define STM32_PWR_PUCRH        (STM32_PWR_BASE+STM32_PWR_PUCRH_OFFSET)
#define STM32_PWR_PDCRH        (STM32_PWR_BASE+STM32_PWR_PDCRH_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT      (0)       /* Bits 2-0: Low-power mode selection */
#define PWR_CR1_LPMS_MASK       (7 << PWR_CR1_LPMS_SHIFT)
#  define PWR_CR1_LPMS_STOP1MR  (0 << PWR_CR1_LPMS_SHIFT) /* 000: Stop 1 mode with main regulator (MR) */
#  define PWR_CR1_LPMS_STOP1LPR (1 << PWR_CR1_LPMS_SHIFT) /* 001: Stop 1 mode with low-power regulator (LPR) */
#  define PWR_CR1_LPMS_STOP2    (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STANDBY  (3 << PWR_CR1_LPMS_SHIFT) /* 010: Standby mode */
#define PWR_CR1_DBP             (1 << 8)  /* Bit 8: Disable Backup Domain write protection */
#define PWR_CR1_VOS_SHIFT       (9)       /* Bits 10-9: Voltage scaling range selection */
#define PWR_CR1_VOS_MASK        (3 << PWR_CR1_VOS_SHIFT)
#  define PWR_CR1_VOS_RANGE1    (1 << PWR_CR1_VOS_SHIFT) /* 001: Range 1 */
#  define PWR_CR1_VOS_RANGE2    (2 << PWR_CR1_VOS_SHIFT) /* 010: Range 2 */
#define PWR_CR1_LPR             (1 << 14)  /* Bit 14: Low-power run */

/* Power control register 2 */

#define PWR_CR2_PVDE            (1 << 0)  /* Bit 0: Power Voltage Detector Enable */
#define PWR_CR2_PLS_SHIFT       (1)       /* Bits 3-1: PVD Level Selection */
#define PWR_CR2_PLS_MASK        (7 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_2p0V          (0 << PWR_CR2_PLS_SHIFT) /* 000: 2.0V */
#  define PWR_CR2_2p2V          (1 << PWR_CR2_PLS_SHIFT) /* 001: 2.2V */
#  define PWR_CR2_2p4V          (2 << PWR_CR2_PLS_SHIFT) /* 010: 2.4V */
#  define PWR_CR2_2p5V          (3 << PWR_CR2_PLS_SHIFT) /* 011: 2.5V */
#  define PWR_CR2_2p6V          (4 << PWR_CR2_PLS_SHIFT) /* 100: 2.6V */
#  define PWR_CR2_2p8V          (5 << PWR_CR2_PLS_SHIFT) /* 101: 2.8V */
#  define PWR_CR2_2p9V          (6 << PWR_CR2_PLS_SHIFT) /* 110: 2.9V */
#  define PWR_CR2_PVD_IN        (7 << PWR_CR2_PLS_SHIFT) /* 111: External input analog voltage PVD_IN */
#define PWR_CR2_PVME1           (1 << 4)  /* Bit 4: Peripheral voltage monitoring 1 enable */
#define PWR_CR2_PVME3           (1 << 6)  /* Bit 6: Peripheral voltage monitoring 3 enable */
#define PWR_CR2_PVME4           (1 << 7)  /* Bit 7: Peripheral voltage monitoring 4 enable */
#define PWR_CR2_USV             (1 << 10) /* Bit 10: VDDUSB USB supply valid */

/* Power control register 3 */

#define PWR_CR3_EWUP1           (1 << 0)  /* Bit 0:  Enable WKUP1 pin */
#define PWR_CR3_EWUP2           (1 << 1)  /* Bit 1:  Enable WKUP2 pin */
#define PWR_CR3_EWUP3           (1 << 2)  /* Bit 2:  Enable WKUP3 pin */
#define PWR_CR3_EWUP4           (1 << 3)  /* Bit 3:  Enable WKUP4 pin */
#define PWR_CR3_EWUP5           (1 << 4)  /* Bit 4:  Enable WKUP5 pin */
#define PWR_CR3_RRS             (1 << 8)  /* Bit 8:  SRAM2 retention in Standby mode */
#define PWR_CR3_APC             (1 << 10) /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_EIWF            (1 << 15) /* Bit 15: Enable internal wakeup line */

/* Power status register 2 */

#define PWR_SR2_REGLPS          (1 << 8)  /* Bit 8:  Low-power regulator started */
#define PWR_SR2_REGLPF          (1 << 9)  /* Bit 9:  Low-power regulator flag */
#define PWR_SR2_VOSF            (1 << 10) /* Bit 10: Voltage scaling flag */
#define PWR_SR2_PVDO            (1 << 11) /* Bit 11: Power voltage detector output */
#define PWR_SR2_PVMO1           (1 << 12) /* Bit 12: Peripheral voltage monitoring output: VDDIO2 vs 1.2 V */
#define PWR_SR2_PVMO3           (1 << 14) /* Bit 14: Peripheral voltage monitoring output: VDDA vs 1.62 V */
#define PWR_SR2_PVMO4           (1 << 15) /* Bit 15: Peripheral voltage monitoring output: VDDA vs 2.2 V */

/* Power status clear register */

#define PWR_SCR_CWUF1           (1 << 0)  /* Bit 0:  Clear wakeup flag 1 */
#define PWR_SCR_CWUF2           (1 << 1)  /* Bit 1:  Clear wakeup flag 2 */
#define PWR_SCR_CWUF3           (1 << 2)  /* Bit 2:  Clear wakeup flag 3 */
#define PWR_SCR_CWUF4           (1 << 3)  /* Bit 3:  Clear wakeup flag 4 */
#define PWR_SCR_CWUF5           (1 << 4)  /* Bit 4:  Clear wakeup flag 5 */
#define PWR_SCR_CSBF            (1 << 8)  /* Bit 8:  Clear standby flag */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_PWR_H */
