/************************************************************************************
 * arch/arm/include/stm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_STM32_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip and provide alternate function pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a special
 * alternate function (such as USART, CAN, USB, SDIO, etc.).  That particular
 * pin-mapping will depend on the package and STM32 family.  If you are incorporating
 * a new STM32 chip into NuttX, you will need to add the pin-mapping to a header file
 * and to include that header file below. The chip-specific pin-mapping is defined in
 * the chip datasheet.
 */

/* STM32 L4 Family ************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L431KB) || defined(CONFIG_ARCH_CHIP_STM32L431KC)
#  define CONFIG_STM32_STM32L4X3         1   /* STM32L4X3 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    3   /* Two 16-bit general timers TIM15,16 with DMA
                                              * One 32-bit general timers TIM2 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     2   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   2   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x19 */
#  define STM32_NUSBOTG                  0   /* USB OTG FS (only) */
#  define STM32_NGPIO                    26  /* GPIOA-C, plus few in GPIOD,E, H. GPIOF & GPIOG not available */
#  define STM32_NADC                     1   /* ADC1-1, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                3   /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L433CB) || defined(CONFIG_ARCH_CHIP_STM32L433CC) || defined(CONFIG_ARCH_CHIP_STM32L431CC) || defined(CONFIG_ARCH_CHIP_STM32L431CB)
#  define CONFIG_STM32_STM32L4X3         1   /* STM32L4X3 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    3   /* Two 16-bit general timers TIM15,16 with DMA
                                              * One 32-bit general timers TIM2 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x19 */
#  define STM32_NUSBOTG                  0   /* USB OTG FS (only) */
#  define STM32_NGPIO                    38  /* GPIOA-C, plus few in GPIOD,E, H. GPIOF & GPIOG not available */
#  define STM32_NADC                     1   /* ADC1-1, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                6   /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L433RB) || defined(CONFIG_ARCH_CHIP_STM32L433RC)
#  define CONFIG_STM32_STM32L4X3         1   /* STM32L4X3 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    3   /* Two 16-bit general timers TIM15,16 with DMA
                                              * One 32-bit general timers TIM2 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x19 */
#  define STM32_NUSBOTG                  0   /* USB OTG FS (only) */
#  define STM32_NGPIO                    52  /* GPIOA-C, plus few in GPIOD,E, H. GPIOF & GPIOG not available */
#  define STM32_NADC                     1   /* ADC1-1, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                12  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L433VC)
#  define CONFIG_STM32_STM32L4X3         1   /* STM32L4X3 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    3   /* Two 16-bit general timers TIM15,16 with DMA
                                              * One 32-bit general timers TIM2 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x19 */
#  define STM32_NUSBOTG                  0   /* USB OTG FS (only) */
#  define STM32_NGPIO                    83  /* GPIOA-C, plus few in GPIOD,E, H. GPIOF & GPIOG not available */
#  define STM32_NADC                     1   /* ADC1-1, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                21  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476RC) || defined(CONFIG_ARCH_CHIP_STM32L476RE) || defined(CONFIG_ARCH_CHIP_STM32L476RG)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    51  /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                12  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476JE) || defined(CONFIG_ARCH_CHIP_STM32L476JG)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x28*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    57  /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                12  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476MG) || defined(CONFIG_ARCH_CHIP_STM32L476ME)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x30*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    65  /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                12  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476VC) || defined(CONFIG_ARCH_CHIP_STM32L476VE) || defined(CONFIG_ARCH_CHIP_STM32L476VG)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x30*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    82  /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                21  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476QE) || defined(CONFIG_ARCH_CHIP_STM32L476QG)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x30*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    109 /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                24  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32L476ZE) || defined(CONFIG_ARCH_CHIP_STM32L476ZG)
#  define CONFIG_STM32_STM32L4X6         1   /* STM32L4X6 family */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1,8*/
#  define STM32_NGTIM                    7   /* 16-bit general timers TIM3,4,15,16,17 with DMA
                                              * 32-bit general timers TIM2,5 with DMA */
#  define STM32_NGTIMNDMA                0   /* All general timers have DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 without DMA */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* Has Serial Audio Interface (SAI) instead */
#  define STM32_NUSART                   5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     1   /* bxCAN */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     1   /* LCD 4x32, 8x30*/
#  define STM32_NUSBOTG                  1   /* USB OTG FS (only) */
#  define STM32_NGPIO                    114 /* GPIOA-C, plus few in GPIOD,G, H */
#  define STM32_NADC                     3   /* ADC1-3, 16-channels */
#  define STM32_NDAC                     2   /* DAC1-2, 1 channel */
                                             /* (2) Comparators */
#  define STM32_NCAPSENSE                24  /* Capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */
#else
#  error "Unsupported STM32 chip"
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the irqsave() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32_CHIP_H */

