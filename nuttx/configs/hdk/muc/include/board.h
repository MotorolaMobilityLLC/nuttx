/************************************************************************************
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

/* Copied from configs/nucleo-f401re/include/board.h */
#ifndef __CONFIG_HDK_MUC_INCLUDE_BOARD_H
#define __CONFIG_HDK_MUC_INCLUDE_BOARD_H

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

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - 32.768 kHz
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_LSE_FREQUENCY     32768

#define STM32_BOARD_USEHSI      1
#define STM32_BOARD_USELSEBYP   1

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

/* Use the MSI; freq = 4 MHz; autotrim from LSE */
#  define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_MSI

#  define STM32_BOARD_USELSE      1
#  define STM32_BOARD_USEMSI      1
#  define STM32_BOARD_MSIRANGE    RCC_CR_MSIRANGE_6  /* 4 MHz */

#  define STM32_SYSCLK_FREQUENCY  80000000ul
#endif

/* AHB clock (HCLK) is SYSCLK */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be equal to PCLK1 */

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

/* The board only has one button */

#define BUTTON_POWER       0
#define NUM_BUTTONS        1
#define BUTTON_POWER_BIT   (1 << BUTTON_POWER)

/* USART */
#define STM32_USART2_FREQUENCY  STM32_HSI_FREQUENCY
#define STM32_USART3_FREQUENCY  STM32_HSI_FREQUENCY

#define GPIO_USART1_CTS  GPIO_USART1_CTS_3   /* PG11 */
#define GPIO_USART1_RTS  GPIO_USART1_RTS_2   /* PB3  */
#define GPIO_USART1_RX   GPIO_USART1_RX_2    /* PB7  */
#define GPIO_USART1_TX   GPIO_USART1_TX_2    /* PB6  */

#define GPIO_USART2_CTS  GPIO_USART2_CTS_1   /* PA0  */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_1   /* PA1  */
#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3  */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2  */

#define GPIO_USART3_RX   GPIO_USART3_RX_2    /* PC11 */
#define GPIO_USART3_TX   GPIO_USART3_TX_2    /* PC10 */

/* I2C */

#define STM32_I2C2_FREQUENCY    STM32_HSI_FREQUENCY
#define STM32_I2C3_FREQUENCY    STM32_HSI_FREQUENCY

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1     /* PB10 */
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1     /* PB11 */

#define GPIO_I2C3_SCL    GPIO_I2C3_SCL_1     /* PC0  */
#define GPIO_I2C3_SDA    GPIO_I2C3_SDA_1     /* PC1  */

/* SPI */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1
#define GPIO_SPI1_NSS    GPIO_SPI1_NSS_2
#define SPI1_MODE_TYPE   SPI_MODE_TYPE_MASTER
#define SPI1_CRC16_EN    0
#define DMACHAN_SPI1_RX  DMACHAN_SPI1_RX_1
#define DMACHAN_SPI1_TX  DMACHAN_SPI1_TX_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_1
#define GPIO_SPI2_NSS    GPIO_SPI2_NSS_1
#define SPI2_MODE_TYPE   SPI_MODE_TYPE_SLAVE
#define SPI2_CRC16_EN    1
#define stm32_spi2select NULL
#define stm32_spi2status NULL

/* LPTIM1 */

#define STM32_LPTIM1_FREQUENCY    STM32_LSI_FREQUENCY

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HDK_MUC_INCLUDE_BOARD_H */
