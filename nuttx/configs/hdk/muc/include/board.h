/************************************************************************************
 * configs/hdk/muc/include/board.h
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
/* HDK will run off the 16MHz HSI clock.
 *
 *   System Clock source           : HSI
 *   SYSCLK(Hz)                    : 16000000     Selected by System Clock Mux
 *   HCLK(Hz)                      : 16000000     (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 1            (STM32_RCC_CFGR_PPRE2)
 *   HSI Frequency(Hz)             : 16000000     (nominal)
 *   PLL                           : OFF
 *   Flash Latency(WS)             : 0
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Disabled (PLL off)
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - not installed
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_SYSCLK_FREQUENCY  16000000ul

/* AHB clock (HCLK) is SYSCLK (16MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (8MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (16MHz) */

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
#define STM32_TIM27_FREQUENCY   (2*STM32_PCLK1_FREQUENCY)

/* The board only has one button */

#define BUTTON_POWER       0
#define NUM_BUTTONS        1
#define BUTTON_POWER_BIT   (1 << BUTTON_POWER)

/* USART */

#define GPIO_USART1_RX   GPIO_USART1_RX_1    /* PA10 */
#define GPIO_USART1_TX   GPIO_USART1_TX_1    /* PA9  */

#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3  */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2  */

/* I2C */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_1

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1

#define GPIO_I2C3_SCL    GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA    GPIO_I2C3_SDA_1

/* SPI */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1
#define SPI1_MODE_TYPE   SPI_MODE_TYPE_MASTER
#define stm32_spi1select NULL
#define stm32_spi1status NULL
#define DMACHAN_SPI1_RX  DMACHAN_SPI1_RX_1
#define DMACHAN_SPI1_TX  DMACHAN_SPI1_TX_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_1
#define SPI2_MODE_TYPE   SPI_MODE_TYPE_SLAVE
#define stm32_spi2select NULL
#define stm32_spi2status NULL

/* Mods ****************************************************************************/
#define GPIO_MODS_CORE_INT        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

#define GPIO_APBE_SPIBOOT_N       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

#define GPIO_APBE_BOOTRET         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN12)

#define GPIO_APBE_PWR_EN          (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)

#define GPIO_APBE_RST_N           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                                   GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN1)

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
