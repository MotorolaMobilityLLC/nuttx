/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32l4x3xx_rcc.h
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_RCC_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_RCC_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define STM32_RCC_CR_OFFSET          0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET       0x0004  /* Internal clock sources calibration register */
#define STM32_RCC_CFGR_OFFSET        0x0008  /* Clock configuration register */
#define STM32_RCC_PLLCFG_OFFSET      0x000c  /* PLL configuration register */
#define STM32_RCC_PLLSAI1CFGR_OFFSET 0x0010  /* PLLSAI1 configuration register */
#define STM32_RCC_CIER_OFFSET        0x0018  /* Clock interrupt enable register */
#define STM32_RCC_CIFR_OFFSET        0x001c  /* Clock interrupt flag register */
#define STM32_RCC_CICR_OFFSET        0x0020  /* Clock interrupt clear register */
#define STM32_RCC_AHB1RSTR_OFFSET    0x0028  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET    0x002c  /* AHB2 peripheral reset register */
#define STM32_RCC_AHB3RSTR_OFFSET    0x0030  /* AHB3 peripheral reset register */
#define STM32_RCC_APB1RSTR1_OFFSET   0x0038  /* APB1 Peripheral reset register 1 */
#define STM32_RCC_APB1RSTR2_OFFSET   0x003c  /* APB1 Peripheral reset register 2 */
#define STM32_RCC_APB2RSTR_OFFSET    0x0040  /* APB2 Peripheral reset register */
#define STM32_RCC_AHB1ENR_OFFSET     0x0048  /* AHB1 Peripheral Clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET     0x004c  /* AHB2 Peripheral Clock enable register */
#define STM32_RCC_AHB3ENR_OFFSET     0x0050  /* AHB3 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR1_OFFSET    0x0058  /* APB1 Peripheral Clock enable register 1 */
#define STM32_RCC_APB1ENR2_OFFSET    0x005c  /* APB1 Peripheral Clock enable register 2 */
#define STM32_RCC_APB2ENR_OFFSET     0x0060  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_AHB1SMENR_OFFSET   0x0068  /* AHB1 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_AHB2SMENR_OFFSET   0x006c  /* AHB2 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_AHB3SMENR_OFFSET   0x0070  /* AHB3 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_APB1SMENR1_OFFSET  0x0078  /* APB1 peripheral clocks enable in Sleep and Stop modes register 1 */
#define STM32_RCC_APB1SMENR2_OFFSET  0x007c  /* APB1 peripheral clocks enable in Sleep and Stop modes register 2 */
#define STM32_RCC_APB2SMENR_OFFSET   0x0080  /* APB2 peripheral clocks enable in Sleep and Stop modes register */
#define STM32_RCC_CCIPR_OFFSET       0x0088  /* Peripherals independent clock configuration register */
#define STM32_RCC_BDCR_OFFSET        0x0090  /* Backup domain control register */
#define STM32_RCC_CSR_OFFSET         0x0094  /* Control/status register */
#define STM32_RCC_CRRCR_OFFSET       0x0098  /* Clock recovery RC register */

/* Register Addresses *******************************************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_PLLCFG            (STM32_RCC_BASE+STM32_RCC_PLLCFG_OFFSET)
#define STM32_RCC_PLLSAI1CFGR       (STM32_RCC_BASE+STM32_RCC_PLLSAI1CFGR_OFFSET)
#define STM32_RCC_CIER              (STM32_RCC_BASE+STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR              (STM32_RCC_BASE+STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR              (STM32_RCC_BASE+STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR          (STM32_RCC_BASE+STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR          (STM32_RCC_BASE+STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR          (STM32_RCC_BASE+STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_APB1RSTR1         (STM32_RCC_BASE+STM32_RCC_APB1RSTR1_OFFSET)
#define STM32_RCC_APB1RSTR2         (STM32_RCC_BASE+STM32_RCC_APB1RSTR2_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_AHB1ENR           (STM32_RCC_BASE+STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR           (STM32_RCC_BASE+STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR           (STM32_RCC_BASE+STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_APB1ENR1          (STM32_RCC_BASE+STM32_RCC_APB1ENR1_OFFSET)
#define STM32_RCC_APB1ENR2          (STM32_RCC_BASE+STM32_RCC_APB1ENR2_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_AHB1SMENR         (STM32_RCC_BASE+STM32_RCC_AHB1SMENR_OFFSET)
#define STM32_RCC_AHB2SMENR         (STM32_RCC_BASE+STM32_RCC_AHB2SMENR_OFFSET)
#define STM32_RCC_AHB3SMENR         (STM32_RCC_BASE+STM32_RCC_AHB3SMENR_OFFSET)
#define STM32_RCC_APB1SMENR1        (STM32_RCC_BASE+STM32_RCC_APB1SMENR1_OFFSET)
#define STM32_RCC_APB1SMENR2        (STM32_RCC_BASE+STM32_RCC_APB1SMENR2_OFFSET)
#define STM32_RCC_APB2SMENR         (STM32_RCC_BASE+STM32_RCC_APB2SMENR_OFFSET)
#define STM32_RCC_CCIPR             (STM32_RCC_BASE+STM32_RCC_CCIPR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#define STM32_RCC_CRRCR             (STM32_RCC_BASE+STM32_RCC_CRRCR_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* Clock control register */

#define RCC_CR_MSION                (1 << 0)  /* Bit 0: Multispeed internal clock enable */
#define RCC_CR_MSIRDY               (1 << 1)  /* Bit 1: Multispeed internal clock ready flag */
#define RCC_CR_MSIPLLEN             (1 << 2)  /* Bit 2: Multispeed internal clock PLL enable */
#define RCC_CR_MSIRGSEL             (1 << 3)  /* Bit 3: Multispeed internal clock range selection */
#define RCC_CR_MSIRANGE_SHIFT       (4)       /* Bits 7-4: Multispeed internal clock ranges */
#define RCC_CR_MSIRANGE_MASK        (0xf << RCC_CR_MSIRANGE_SHIFT)
#  define RCC_CR_MSIRANGE_0         (0 << RCC_CR_MSIRANGE_SHIFT)  /* 0000: Range 0 around 100 kHz */
#  define RCC_CR_MSIRANGE_1         (1 << RCC_CR_MSIRANGE_SHIFT)  /* 0001: Range 1 around 200 kHz */
#  define RCC_CR_MSIRANGE_2         (2 << RCC_CR_MSIRANGE_SHIFT)  /* 0010: Range 2 around 400 kHz */
#  define RCC_CR_MSIRANGE_3         (3 << RCC_CR_MSIRANGE_SHIFT)  /* 0011: Range 3 around 800 kHz */
#  define RCC_CR_MSIRANGE_4         (4 << RCC_CR_MSIRANGE_SHIFT)  /* 0100: Range 4 around 1 MHz */
#  define RCC_CR_MSIRANGE_5         (5 << RCC_CR_MSIRANGE_SHIFT)  /* 0101: Range 5 around 2 MHz */
#  define RCC_CR_MSIRANGE_6         (6 << RCC_CR_MSIRANGE_SHIFT)  /* 0110: Range 6 around 4 MHz (reset value) */
#  define RCC_CR_MSIRANGE_7         (7 << RCC_CR_MSIRANGE_SHIFT)  /* 0111: Range 7 around 8 MHz */
#  define RCC_CR_MSIRANGE_8         (8 << RCC_CR_MSIRANGE_SHIFT)  /* 1000: Range 8 around 16 MHz */
#  define RCC_CR_MSIRANGE_9         (9 << RCC_CR_MSIRANGE_SHIFT)  /* 1001: Range 9 around 24 MHz */
#  define RCC_CR_MSIRANGE_10        (10 << RCC_CR_MSIRANGE_SHIFT) /* 1010: Range 10 around 32 MHz */
#  define RCC_CR_MSIRANGE_11        (11 << RCC_CR_MSIRANGE_SHIFT) /* 1011: Range 11 around 48 MHz */
#define RCC_CR_HSION                (1 << 8)  /* Bit 8: Internal High Speed clock enable */
#define RCC_CR_HSIKERON             (1 << 9)  /* Bit 9: Internal High Speed clock always enable for peripheral kernels */
#define RCC_CR_HSIRDY               (1 << 10) /* Bit 10: Internal High Speed clock ready flag */
#define RCC_CR_HSIASFS              (1 << 11) /* Bit 11: Internal High Speed clock automatic start from Stop */
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
#define RCC_CR_PLLSAI1ON            (1 << 26) /* Bit 26: PLLSAI1 enable */
#define RCC_CR_PLLSAI1RDY           (1 << 27) /* Bit 27: PLLSAI1 clock ready flag */

/* Internal clock sources calibration register */

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)       /* Bits 0-1: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_MSI           (0 << RCC_CFGR_SW_SHIFT) /* 00: MSI selected as system clock */
#  define RCC_CFGR_SW_HSI           (1 << RCC_CFGR_SW_SHIFT) /* 01: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (2 << RCC_CFGR_SW_SHIFT) /* 10: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (3 << RCC_CFGR_SW_SHIFT) /* 11: PLL selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (2)       /* Bits 2-3: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_MSI          (0 << RCC_CFGR_SWS_SHIFT) /* 00: MSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSI          (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (2 << RCC_CFGR_SWS_SHIFT) /* 10: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (3 << RCC_CFGR_SWS_SHIFT) /* 11: PLL used as system clock */
#define RCC_CFGR_HPRE_SHIFT         (4)       /* Bits 4-7: AHB prescaler */
#define RCC_CFGR_HPRE_MASK          (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK      (0 << RCC_CFGR_HPRE_SHIFT) /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2    (8 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4    (9 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8    (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16   (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64   (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128  (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256  (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512  (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */
#define RCC_CFGR_PPRE1_SHIFT        (8)      /* Bits 8-10: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_PPRE2_SHIFT        (11)      /* Bits 11-13: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK         (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK       (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2     (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4     (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8     (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16    (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_STOPWUCK           (1 << 15) /* Bit 15: Wakeup from Stop and CSS backup clock selection */
#define RCC_CFGR_MCOSEL_SHIFT       (24)      /* Bits 24-26: Microcontroller clock output */
#define RCC_CFGR_MCOSEL_MASK        (3 << RCC_CFGR_MCOSEL_SHIFT)
#  define RCC_CFGR_MCOSEL_NONE      (0 << RCC_CFGR_MCOSEL_SHIFT) /* 000: MCO output disabled */
#  define RCC_CFGR_MCOSEL_SYSCLK    (1 << RCC_CFGR_MCOSEL_SHIFT) /* 001: System clock (SYSCLK) selected */
#  define RCC_CFGR_MCOSEL_MSI       (2 << RCC_CFGR_MCOSEL_SHIFT) /* 010: MSI clock selected */
#  define RCC_CFGR_MCOSEL_HSI       (3 << RCC_CFGR_MCOSEL_SHIFT) /* 011: HSI clock selected */
#  define RCC_CFGR_MCOSEL_HSE       (4 << RCC_CFGR_MCOSEL_SHIFT) /* 100: HSE clock selected */
#  define RCC_CFGR_MCOSEL_PLL       (5 << RCC_CFGR_MCOSEL_SHIFT) /* 101: Main PLL clock selected */
#  define RCC_CFGR_MCOSEL_LSI       (6 << RCC_CFGR_MCOSEL_SHIFT) /* 110: LSI clock selected */
#  define RCC_CFGR_MCOSEL_LSE       (7 << RCC_CFGR_MCOSEL_SHIFT) /* 111: LSE clock divided by 2 selected */
#define RCC_CFGR_MCOPRE_SHIFT       (28)      /* Bits 28-30: MCO prescaler */
#define RCC_CFGR_MCOPRE_MASK        (7 << RCC_CFGR_MCOPRE_SHIFT)
#  define RCC_CFGR_MCOPRE_DIV1      (0 << RCC_CFGR_MCOPRE_SHIFT) /* 000: division by 1 */
#  define RCC_CFGR_MCOPRE_DIV2      (1 << RCC_CFGR_MCOPRE_SHIFT) /* 001: division by 2 */
#  define RCC_CFGR_MCOPRE_DIV4      (2 << RCC_CFGR_MCOPRE_SHIFT) /* 010: division by 4 */
#  define RCC_CFGR_MCOPRE_DIV8      (3 << RCC_CFGR_MCOPRE_SHIFT) /* 011: division by 8 */
#  define RCC_CFGR_MCOPRE_DIV16     (4 << RCC_CFGR_MCOPRE_SHIFT) /* 100: division by 16 */

/* PLL configuration register */

#define RCC_PLLCFG_PLLSRC_SHIFT     (0)       /* Bits 0-1: Main PLL, PLLSAI1 and PLLSAI2 entry clock source */
#define RCC_PLLCFG_PLLSRC_MASK      (3 << RCC_PLLCFG_PLLSRC_SHIFT)
#  define RCC_PLLCFG_PLLSRC_NONE    (0 << RCC_PLLCFG_PLLSRC_SHIFT) /* 00: No clock sent */
#  define RCC_PLLCFG_PLLSRC_MSI     (1 << RCC_PLLCFG_PLLSRC_SHIFT) /* 01: MSI clock selected */
#  define RCC_PLLCFG_PLLSRC_HSI     (2 << RCC_PLLCFG_PLLSRC_SHIFT) /* 10: HSI clock selected */
#  define RCC_PLLCFG_PLLSRC_HSE     (3 << RCC_PLLCFG_PLLSRC_SHIFT) /* 11: HSE clock selected */
#define RCC_PLLCFG_PLLM_SHIFT       (4)       /* Bits 4-6: Division factor for main PLL and audio PLL input clocks */
#define RCC_PLLCFG_PLLM_MASK        (3 << RCC_PLLCFG_PLLM_SHIFT)
#  define RCC_PLLCFG_PLLM(n)        (((n)-1) << RCC_PLLCFG_PLLM_SHIFT) /* n = 1-8 */
#define RCC_PLLCFG_PLLN_SHIFT       (8)       /* Bits 8-14: Main PLL VCO multiplier */
#define RCC_PLLCFG_PLLN_MASK        (0x7f << RCC_PLLCFG_PLLN_SHIFT)
#  define RCC_PLLCFG_PLLN(n)        ((n) << RCC_PLLCFG_PLLN_SHIFT) /* n = 8..86 */
#define RCC_PLLCFG_PLLPEN           (1 << 16) /* Bit 16: Main PLLSAI3CLK output enable */
#define RCC_PLLCFG_PLLP             (1 << 17)
#  define RCC_PLLCFG_PLLP_7         (0)
#  define RCC_PLLCFG_PLLP_17        RCC_PLLCFG_PLLP
#define RCC_PLLCFG_PLLQEN           (1 << 20) /* Bit 20: Main PLL PLLUSB1CLK output enable */
#define RCC_PLLCFG_PLLQ_SHIFT       (21)      /* Bits 21-22: Main PLL divider for PLLUSB1CLK */
#define RCC_PLLCFG_PLLQ_MASK        (3 << RCC_PLLCFG_PLLQ_SHIFT)
#  define RCC_PLLCFG_PLLQ(n)        ((((n)>>1)-1) << RCC_PLLCFG_PLLQ_SHIFT) /* n=2,4,6,8 */
#define RCC_PLLCFG_PLLREN           (1 << 24) /* Bit 24: Main PLL PLLCLK output enable */
#define RCC_PLLCFG_PLLR_SHIFT       (25)      /* Bits 25-26: Main PLL divider for PLLCLK */
#define RCC_PLLCFG_PLLR_MASK        (3 << RCC_PLLCFG_PLLR_SHIFT)
#  define RCC_PLLCFG_PLLR(n)        ((((n)>>1)-1) << RCC_PLLCFG_PLLR_SHIFT) /* n=2,4,6,8 */
#define RCC_PLLCFG_PLLPDIV_SHIFT    (27)      /* Bits 27-31: Main PLL divider for PLLSAI2CLK */
#define RCC_PLLCFG_PLLPDIV_MASK     (0x1f << RCC_PLLCFG_PLLPDIV_SHIFT)

#define RCC_PLLCFG_RESET            (0x00001000) /* PLLCFG reset value */

/* PLLSAI1 configuration register */

/* PLLSAI2 configuration register */

/* Clock interrupt enable register */

#define RCC_CIER_LSIRDYIE           (1 << 0)  /* Bit 0: LSI Ready Interrupt Enable */
#define RCC_CIER_LSERDYIE           (1 << 1)  /* Bit 1: LSE Ready Interrupt Enable */
#define RCC_CIER_MSIRDYIE           (1 << 2)  /* Bit 2: MSI Ready Interrupt Enable */
#define RCC_CIER_HSIRDYIE           (1 << 3)  /* Bit 3: HSI Ready Interrupt Enable */
#define RCC_CIER_HSERDYIE           (1 << 4)  /* Bit 4: HSE Ready Interrupt Enable */
#define RCC_CIER_PLLRDYIE           (1 << 5)  /* Bit 5: PLL Ready Interrupt Enable */
#define RCC_CIER_PLLSAI1RDYIE       (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt enable */
#define RCC_CIER_LSECSSIE           (1 << 9)  /* Bit 9: LSE clock security system Interrupt enable */
#define RCC_CIER_HSI48RDYIE         (1 << 10) /* Bit 10: HSI48 ready interrupt enable */

/* Clock interrupt flag register */

#define RCC_CIFR_LSIRDYF            (1 << 0)  /* Bit 0: LSI Ready Interrupt flag */
#define RCC_CIFR_LSERDYF            (1 << 1)  /* Bit 1: LSE Ready Interrupt flag */
#define RCC_CIFR_MSIRDYF            (1 << 2)  /* Bit 2: MSI Ready Interrupt flag */
#define RCC_CIFR_HSIRDYF            (1 << 3)  /* Bit 3: HSI Ready Interrupt flag */
#define RCC_CIFR_HSERDYF            (1 << 4)  /* Bit 4: HSE Ready Interrupt flag */
#define RCC_CIFR_PLLRDYF            (1 << 5)  /* Bit 5: PLL Ready Interrupt flag */
#define RCC_CIFR_PLLSAI1RDYF        (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt flag */
#define RCC_CIFR_CSSF               (1 << 8)  /* Bit 8: Clock Security System Interrupt flag */
#define RCC_CIFR_LSECSSF            (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt flag */
#define RCC_CIFR_HSI48RDYF          (1 << 10) /* Bit 10: HSI48 ready interrupt flag */

/* Clock interrupt clear register */

#define RCC_CICR_LSIRDYC            (1 << 0)  /* Bit 0: LSI Ready Interrupt clear */
#define RCC_CICR_LSERDYC            (1 << 1)  /* Bit 1: LSE Ready Interrupt clear */
#define RCC_CICR_MSIRDYC            (1 << 2)  /* Bit 2: MSI Ready Interrupt clear */
#define RCC_CICR_HSIRDYC            (1 << 3)  /* Bit 3: HSI Ready Interrupt clear */
#define RCC_CICR_HSERDYC            (1 << 4)  /* Bit 4: HSE Ready Interrupt clear */
#define RCC_CICR_PLLRDYC            (1 << 5)  /* Bit 5: PLL Ready Interrupt clear */
#define RCC_CICR_PLLSAI1RDYC        (1 << 6)  /* Bit 6: PLLSAI1 Ready Interrupt clear */
#define RCC_CICR_CSSC               (1 << 8)  /* Bit 8: Clock Security System Interrupt clear */
#define RCC_CICR_LSECSSC            (1 << 9)  /* Bit 9: LSE Clock Security System Interrupt clear */
#define RCC_CICR_HSI48RDYC          (1 << 10) /* Bit 10: HSI48 oscillator ready interrupt clear */

/* AHB1 peripheral reset register */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_GPIOARST      (1 << 0)  /* Bit 0:  IO port A reset */
#define RCC_AHB2RSTR_GPIOBRST      (1 << 1)  /* Bit 1:  IO port B reset */
#define RCC_AHB2RSTR_GPIOCRST      (1 << 2)  /* Bit 2:  IO port C reset */
#define RCC_AHB2RSTR_GPIODRST      (1 << 3)  /* Bit 3:  IO port D reset */
#define RCC_AHB2RSTR_GPIOERST      (1 << 4)  /* Bit 4:  IO port E reset */
#define RCC_AHB2RSTR_GPIOHRST      (1 << 7)  /* Bit 7:  IO port H reset */
#define RCC_AHB2RSTR_ADCRST        (1 << 13) /* Bit 13: ADC reset */
#define RCC_AHB2RSTR_AESRST        (1 << 16) /* Bit 16: AES hardare accelerator reset */
#define RCC_AHB2RSTR_RNGRST        (1 << 18) /* Bit 18: Random number generator reset */

/* AHB3 peripheral reset register */

/* APB1 Peripheral reset register 1 */

#define RCC_APB1RSTR1_TIM2RST       (1 << 0)  /* Bit 0:  TIM2 reset */
#define RCC_APB1RSTR1_TIM6RST       (1 << 4)  /* Bit 4:  TIM6 reset */
#define RCC_APB1RSTR1_TIM7RST       (1 << 5)  /* Bit 5:  TIM7 reset */
#define RCC_APB1RSTR1_LCDRST        (1 << 9)  /* Bit 9:  LCD interface reset */
#define RCC_APB1RSTR1_SPI2RST       (1 << 14) /* Bit 14: SPI 2 reset */
#define RCC_APB1RSTR1_SPI3RST       (1 << 15) /* Bit 15: SPI 3 reset */
#define RCC_APB1RSTR1_USART2RST     (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR1_USART3RST     (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR1_I2C1RST       (1 << 21) /* Bit 21: I2C1 reset */
#define RCC_APB1RSTR1_I2C2RST       (1 << 22) /* Bit 22: I2C2 reset */
#define RCC_APB1RSTR1_I2C3RST       (1 << 23) /* Bit 23: I2C3 reset */
#define RCC_APB1RSTR1_CRSRST        (1 << 24) /* Bit 24: CRS reset */
#define RCC_APB1RSTR1_CAN1RST       (1 << 25) /* Bit 25: CAN1 reset */
#define RCC_APB1RSTR1_USBFSRST      (1 << 26) /* Bit 26: USB FS reset */
#define RCC_APB1RSTR1_PWRRST        (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR1_DAC1RST       (1 << 29) /* Bit 29: DAC1 reset */
#define RCC_APB1RSTR1_OPAMPRST      (1 << 30) /* Bit 30: OPAMP reset */
#define RCC_APB1RSTR1_LPTIM1RST     (1 << 31) /* Bit 31: LPTIM1 reset */

/* APB1 Peripheral reset register 2 */

#define RCC_APB1RSTR2_LPTIM2RST     (1 << 18) /* Bit 18: LPTIM2 reset */

/* APB2 Peripheral reset register */
#define RCC_APB2RSTR_SPI1RST       (1 << 12) /* Bit 12: SPI 1 reset */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN          (1 << 0)  /* Bit 0: DMA1 clock enable */
#define RCC_AHB1ENR_DMA2EN          (1 << 1)  /* Bit 1: DMA2 clock enable */
#define RCC_AHB1ENR_FLASHEN         (1 << 8)  /* Bit 8: Flash memory interface clock enable */
#define RCC_AHB1ENR_CRCEN           (1 << 12) /* Bit 12: CRC clock enable */
#define RCC_AHB1ENR_TSCEN           (1 << 16) /* Bit 16: Touch Sensing Controller clock enable */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOEN(n)       (1 << (n))
#define RCC_AHB2ENR_GPIOAEN         (1 << 0)  /* Bit 0: IO port A clock enable */
#define RCC_AHB2ENR_GPIOBEN         (1 << 1)  /* Bit 1: IO port B clock enable */
#define RCC_AHB2ENR_GPIOCEN         (1 << 2)  /* Bit 2: IO port C clock enable */
#define RCC_AHB2ENR_GPIODEN         (1 << 3)  /* Bit 3: IO port D clock enable */
#define RCC_AHB2ENR_GPIOEEN         (1 << 4)  /* Bit 4: IO port E clock enable */
#define RCC_AHB2ENR_GPIOHEN         (1 << 7)  /* Bit 7: IO port H clock enable */
#define RCC_AHB2ENR_ADCEN           (1 << 13) /* Bit 13: ADC clock enable */
#define RCC_AHB2ENR_AESEN           (1 << 16) /* Bit 16: AES accelerator clock enable */
#define RCC_AHB2ENR_RNGEN           (1 << 18) /* Bit 18: Random number generator clock enable */

/* AHB3 Peripheral Clock enable register */

/* APB1 Peripheral Clock enable register 1 */

#define RCC_APB1ENR1_TIM2EN         (1 << 0)  /* Bit 0:  TIM2 enable */
#define RCC_APB1ENR1_TIM6EN         (1 << 4)  /* Bit 4:  TIM6 enable */
#define RCC_APB1ENR1_TIM7EN         (1 << 5)  /* Bit 5:  TIM7 enable */
#define RCC_APB1ENR1_LCDEN          (1 << 9)  /* Bit 9:  LCD interface enable */
#define RCC_APB1ENR1_RTCAPBEN       (1 << 10) /* Bit 10: RTC APB enable */
#define RCC_APB1ENR1_WWDGEN         (1 << 11) /* Bit 11: Window watchdog enable */
#define RCC_APB1ENR1_SPI2EN         (1 << 14) /* Bit 14: SPI 2 enable */
#define RCC_APB1ENR1_SPI3EN         (1 << 15) /* Bit 15: SPI 3 enable */
#define RCC_APB1ENR1_USART2EN       (1 << 17) /* Bit 17: USART 2 enable */
#define RCC_APB1ENR1_USART3EN       (1 << 18) /* Bit 18: USART 3 enable */
#define RCC_APB1ENR1_I2C1EN         (1 << 21) /* Bit 21: I2C1 enable */
#define RCC_APB1ENR1_I2C2EN         (1 << 22) /* Bit 22: I2C2 enable */
#define RCC_APB1ENR1_I2C3EN         (1 << 23) /* Bit 23: I2C3 enable */
#define RCC_APB1ENR1_CRSEN          (1 << 24) /* Bit 24: CRS enable */
#define RCC_APB1ENR1_CAN1EN         (1 << 25) /* Bit 25: CAN1 enable */
#define RCC_APB1ENR1_USBFSEN        (1 << 26) /* Bit 26: USB FS enable */
#define RCC_APB1ENR1_PWREN          (1 << 28) /* Bit 28: Power interface enable */
#define RCC_APB1ENR1_DAC1EN         (1 << 29) /* Bit 29: DAC1 enable */
#define RCC_APB1ENR1_OPAMPEN        (1 << 30) /* Bit 30: OPAMP enable */
#define RCC_APB1ENR1_LPTIM1EN       (1 << 31) /* Bit 31: LPTIM1 enable */

/* APB1 Peripheral Clock enable register 2 */

#define RCC_APB1ENR2_LPTIM2EN       (1 << 18) /* Bit 18: LPTIM2 enable */

/* APB2 Peripheral Clock enable register (incomplete) */

#define RCC_APB2ENR_SYSCFGEN        (1 << 0)  /* Bit 0:  System configuration controller clock enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */

/* AHB1 Peripheral Clocks Enable in Sleep and Stop modes register */

/* AHB2 Peripheral Clocks Enable in Sleep and Stop modes register */

/* AHB3 Peripheral Clocks Enable in Sleep and Stop modes register */

/* APB1 Peripheral Clocks Enable in Sleep and Stop modes register 1 */

/* APB1 Peripheral Clocks Enable in Sleep and Stop modes register 2 */

/* APB2 Peripheral Clocks Enable in Sleep and Stop modes register */

/* Peripherals independent clock configuration register (incomplete) */

#define RCC_CCIPR_USART1SEL_SHIFT    (0)       /* Bits 0-1: USART1 clock source selection */
#define RCC_CCIPR_USART1SEL_MASK     (3 << RCC_CCIPR_USART1SEL_SHIFT)
#  define RCC_CCIPR_USART1SEL_PCLK   (0 << RCC_CCIPR_USART1SEL_SHIFT) /* 00: PCLK selected as USART1 clock */
#  define RCC_CCIPR_USART1SEL_SYSCLK (1 << RCC_CCIPR_USART1SEL_SHIFT) /* 01: SYSCLK selected as USART1 clock */
#  define RCC_CCIPR_USART1SEL_HSI    (2 << RCC_CCIPR_USART1SEL_SHIFT) /* 10: HSI clock selected as USART1 clock */
#  define RCC_CCIPR_USART1SEL_LSE    (3 << RCC_CCIPR_USART1SEL_SHIFT) /* 11: LSE clock selected as USART1 clock */
#define RCC_CCIPR_USART2SEL_SHIFT    (2)       /* Bits 2-3: USART2 clock source selection */
#define RCC_CCIPR_USART2SEL_MASK     (3 << RCC_CCIPR_USART2SEL_SHIFT)
#  define RCC_CCIPR_USART2SEL_PCLK   (0 << RCC_CCIPR_USART2SEL_SHIFT) /* 00: PCLK selected as USART2 clock */
#  define RCC_CCIPR_USART2SEL_SYSCLK (1 << RCC_CCIPR_USART2SEL_SHIFT) /* 01: SYSCLK selected as USART2 clock */
#  define RCC_CCIPR_USART2SEL_HSI    (2 << RCC_CCIPR_USART2SEL_SHIFT) /* 10: HSI clock selected as USART2 clock */
#  define RCC_CCIPR_USART2SEL_LSE    (3 << RCC_CCIPR_USART2SEL_SHIFT) /* 11: LSE clock selected as USART2 clock */
#define RCC_CCIPR_USART3SEL_SHIFT    (4)       /* Bits 4-5: USART3 clock source selection */
#define RCC_CCIPR_USART3SEL_MASK     (3 << RCC_CCIPR_USART3SEL_SHIFT)
#  define RCC_CCIPR_USART3SEL_PCLK   (0 << RCC_CCIPR_USART3SEL_SHIFT) /* 00: PCLK selected as USART3 clock */
#  define RCC_CCIPR_USART3SEL_SYSCLK (1 << RCC_CCIPR_USART3SEL_SHIFT) /* 01: SYSCLK selected as USART3 clock */
#  define RCC_CCIPR_USART3SEL_HSI    (2 << RCC_CCIPR_USART3SEL_SHIFT) /* 10: HSI clock selected as USART3 clock */
#  define RCC_CCIPR_USART3SEL_LSE    (3 << RCC_CCIPR_USART3SEL_SHIFT) /* 11: LSE clock selected as USART3 clock */
#define RCC_CCIPR_I2C1SEL_SHIFT      (12)      /* Bits 12-13: I2C1 clock source selection */
#define RCC_CCIPR_I2C1SEL_MASK       (3 << RCC_CCIPR_I2C1SEL_SHIFT)
#  define RCC_CCIPR_I2C1SEL_PCLK     (0 << RCC_CCIPR_I2C1SEL_SHIFT) /* 00: PCLK selected as I2C1 clock */
#  define RCC_CCIPR_I2C1SEL_SYSCLK   (1 << RCC_CCIPR_I2C1SEL_SHIFT) /* 01: SYSCLK selected as I2C1 clock */
#  define RCC_CCIPR_I2C1SEL_HSI      (2 << RCC_CCIPR_I2C1SEL_SHIFT) /* 10: HSI clock selected as I2C1 clock */
#define RCC_CCIPR_I2C2SEL_SHIFT      (14)      /* Bits 14-15: I2C2 clock source selection */
#define RCC_CCIPR_I2C2SEL_MASK       (3 << RCC_CCIPR_I2C2SEL_SHIFT)
#  define RCC_CCIPR_I2C2SEL_PCLK     (0 << RCC_CCIPR_I2C2SEL_SHIFT) /* 00: PCLK selected as I2C2 clock */
#  define RCC_CCIPR_I2C2SEL_SYSCLK   (1 << RCC_CCIPR_I2C2SEL_SHIFT) /* 01: SYSCLK selected as I2C2 clock */
#  define RCC_CCIPR_I2C2SEL_HSI      (2 << RCC_CCIPR_I2C2SEL_SHIFT) /* 10: HSI clock selected as I2C2 clock */
#define RCC_CCIPR_I2C3SEL_SHIFT      (16)      /* Bits 16-17: I2C3 clock source selection */
#define RCC_CCIPR_I2C3SEL_MASK       (3 << RCC_CCIPR_I2C3SEL_SHIFT)
#  define RCC_CCIPR_I2C3SEL_PCLK     (0 << RCC_CCIPR_I2C3SEL_SHIFT) /* 00: PCLK selected as I2C3 clock */
#  define RCC_CCIPR_I2C3SEL_SYSCLK   (1 << RCC_CCIPR_I2C3SEL_SHIFT) /* 01: SYSCLK selected as I2C3 clock */
#  define RCC_CCIPR_I2C3SEL_HSI      (2 << RCC_CCIPR_I2C3SEL_SHIFT) /* 10: HSI clock selected as I2C3 clock */
#define RCC_CCIPR_LPTIM1SEL_SHIFT    (18)      /* Bits 18-19: LPTIM1 clock source selection */
#define RCC_CCIPR_LPTIM1SEL_MASK     (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM1SEL_PCLK   (0 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 00: PCLK selected as LPTIM1 clock */
#  define RCC_CCIPR_LPTIM1SEL_LSI    (1 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 01: LSI selected as LPTIM1 clock */
#  define RCC_CCIPR_LPTIM1SEL_HSI    (2 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 10: HSI clock selected as LPTIM1 clock */
#  define RCC_CCIPR_LPTIM1SEL_LSE    (3 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 11: LSE clock selected as LPTIM1 clock */
#define RCC_CCIPR_LPTIM2SEL_SHIFT    (20)      /* Bits 20-21: LPTIM2 clock source selection */
#define RCC_CCIPR_LPTIM2SEL_MASK     (3 << RCC_CCIPR_LPTIM1SEL_SHIFT)
#  define RCC_CCIPR_LPTIM2SEL_PCLK   (0 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 00: PCLK selected as LPTIM2 clock */
#  define RCC_CCIPR_LPTIM2SEL_LSI    (1 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 01: LSI selected as LPTIM2 clock */
#  define RCC_CCIPR_LPTIM2SEL_HSI    (2 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 10: HSI clock selected as LPTIM2 clock */
#  define RCC_CCIPR_LPTIM2SEL_LSE    (3 << RCC_CCIPR_LPTIM1SEL_SHIFT) /* 11: LSE clock selected as LPTIM2 clock */
#define RCC_CCIPR_ADCSEL_SHIFT       (28)       /* Bits 28-29: ADCs clock source selection */
#define RCC_CCIPR_ADCSEL_MASK        (3 << RCC_CCIPR_ADCSEL_SHIFT)
#  define RCC_CCIPR_ADCSEL_NONE      (0 << RCC_CCIPR_ADCSEL_SHIFT) /* 00: No clock selected */
#  define RCC_CCIPR_ADCSEL_PLLSAI1   (1 << RCC_CCIPR_ADCSEL_SHIFT) /* 01: PLLSAI1 clock selected as ADCs clock */
#  define RCC_CCIPR_ADCSEL_PLLSAI2   (2 << RCC_CCIPR_ADCSEL_SHIFT) /* 10: PLLSAI2 clock selected as ADCs clock */
#  define RCC_CCIPR_ADCSEL_SYSCLK    (3 << RCC_CCIPR_ADCSEL_SHIFT) /* 11: System clock selected as ADCs clock */

/* Backup domain control register (incomplete) */

#define RCC_BDCR_LSEON              (1 << 0)  /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY             (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP             (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_RTCSEL_SHIFT       (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK        (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK     (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE       (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI       (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE       (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 32 used as RTC clock */
#define RCC_BDCR_RTCEN              (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST              (1 << 16) /* Bit 16: Backup domain software reset */

/* Control/status register (incomplete) */

#define RCC_CSR_LSION               (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY              (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF                (1 << 23) /* Bit 23: Remove reset flag */
#define RCC_CSR_FWRSTF              (1 << 24) /* Bit 24: Firewall reset flag */
#define RCC_CSR_OBLRSTF             (1 << 25) /* Bit 25: Option byte loader reset flag */
#define RCC_CSR_PINRSTF             (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_BORRSTF             (1 << 27) /* Bit 27: Brown Out Reset flag */
#define RCC_CSR_SFTRSTF             (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF            (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF            (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF            (1 << 31) /* Bit 31: Low-Power reset flag */

#define RCC_CSR_RSTF_MASK           (0xff000000)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L4X3XX_RCC_H */
