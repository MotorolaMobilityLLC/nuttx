/************************************************************************************
 * configs/stm32l4discovery/include/board.h
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_STM32L4DISCOVERY_INCLUDE_BOARD_H
#define __CONFIG_STM32L4DISCOVERY_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The STM32L4 Discovery board features a single 8MHz crystal.  Space is provided
 * for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 *   System Clock source           : PLL (MSI)
 *   SYSCLK(Hz)                    : 80000000     Selected by System Clock Mux
 *   HCLK(Hz)                      : 80000000     (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 1            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 1            (STM32_RCC_CFGR_PPRE2)
 *   MSI Frequency(Hz)             : 8000000      (nominal)
 *   PLLM                          : 1            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 20           (STM32_PLLCFG_PLLN)
 *   PLLP                          : 7            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 4            (STM32_PLLCFG_PLLQ)
 *   PLLR                          : 2            (STM32_PLLCFG_PLLR)
 *   Flash Latency(WS)             : 4
 *   Prefetch Buffer               : ON
 *   Instruction cache             : ON
 *   Data cache                    : ON
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

#define STM32_BOARD_USEHSI      1

#ifdef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
#  define STM32_SYSCLK_FREQUENCY  STM32_HSI_FREQUENCY
#else
/* Prescaler common to all PLL inputs */
#  define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock */
#  define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(40)
#  define STM32_PLLCFG_PLLP       0
#  define STM32_PLLCFG_PLLQ       0
#  define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)
#  undef  STM32_PLLCFG_PLLP_EN
#  undef  STM32_PLLCFG_PLLQ_EN
#  define STM32_PLLCFG_PLLR_EN

/* 'PLLSAI1' is used to generate the 48 MHz clock */
#  define STM32_BOARD_USE_PLLSAI1 1
#  define STM32_PLLSAI1CFG_PLLN   RCC_PLLSAI1CFG_PLLN(24)
#  define STM32_PLLSAI1CFG_PLLP   0
#  define STM32_PLLSAI1CFG_PLLQ   RCC_PLLSAI1CFG_PLLQ(2)
#  define STM32_PLLSAI1CFG_PLLR   0
#  undef  STM32_PLLSAI1CFG_PLLP_EN
#  define STM32_PLLSAI1CFG_PLLQ_EN
#  undef  STM32_PLLSAI1CFG_PLLR_EN

/* 'PLLSAI2' is used to generate the 24.571 MHz clock */
#  define STM32_BOARD_USE_PLLSAI2 1
#  define STM32_PLLSAI2CFG_PLLN   RCC_PLLSAI2CFG_PLLN(43)
#  define STM32_PLLSAI2CFG_PLLP   RCC_PLLSAI2CFG_PLLP_7
#  define STM32_PLLSAI2CFG_PLLQ   0
#  define STM32_PLLSAI2CFG_PLLR   0
#  define STM32_PLLSAI2CFG_PLLP_EN
#  undef  STM32_PLLSAI2CFG_PLLQ_EN
#  undef  STM32_PLLSAI2CFG_PLLR_EN

/* Use the MSI; freq = 4 MHz; autotrim from LSE */
#  define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_MSI

#  define STM32_BOARD_USELSE      1
#  define STM32_BOARD_USEMSI      1
#  define STM32_BOARD_MSIRANGE    RCC_CR_MSIRANGE_6  /* 4 MHz */

/* Select the clock sources */

#  define STM32_BOARD_CLK48_SEL   RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32_BOARD_SAI1_SEL    RCC_CCIPR_SAI1SEL_PLLSAI2

#  define STM32_SYSCLK_FREQUENCY  80000000ul
#endif

/* AHB clock (HCLK) is SYSCLK */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK     /* PCLK1 = HCLK / 1 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will equal to PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB2 will be equal to PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define STM32_TIM18_FREQUENCY   (STM32_PCLK2_FREQUENCY)
#define STM32_TIM27_FREQUENCY   (STM32_PCLK1_FREQUENCY)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * stm32l4discovery.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED1 */
#define LED_IRQSENABLED   2  /* LED1 */
#define LED_STACKCREATED  3  /* LED1 */
#define LED_INIRQ         4  /* LED1 + LED2 */
#define LED_SIGNAL        5  /* LED1 + LED2 */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED2 */

/* USART2 */

#define STM32_USART2_FREQUENCY  STM32_HSI_FREQUENCY

#define GPIO_USART2_RX   GPIO_USART2_RX_2    /* PD6 */
#define GPIO_USART2_TX   GPIO_USART2_TX_2    /* PD5 */

/* I2C */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_1

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1

#define GPIO_I2C3_SCL    GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA    GPIO_I2C3_SDA_1

/* SPI */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_3
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_3
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_3

#define SPI1_MODE_TYPE   SPI_MODE_TYPE_SLAVE

#define stm32_spi1select NULL
#define stm32_spi1status NULL

#define DMACHAN_SPI1_RX  DMACHAN_SPI1_RX_1
#define DMACHAN_SPI1_TX  DMACHAN_SPI1_TX_1

/* SAI */

#define STM32_SAI1_FREQUENCY  24576000ul

#define GPIO_SAI1_FS_A   GPIO_SAI1_FS_A_2
#define GPIO_SAI1_SCK_A  GPIO_SAI1_SCK_A_2
#define GPIO_SAI1_SD_A   GPIO_SAI1_SD_A_3
#define GPIO_SAI1_MCLK_A GPIO_SAI1_MCLK_A_2

#define DMACHAN_SAI1_A   DMACHAN_SAI1_A_1

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

/************************************************************************************
 * Name:  stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void stm32_ledinit(void);
void stm32_setled(int led, bool ledon);
void stm32_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_STM32L4DISCOVERY_INCLUDE_BOARD_H */
