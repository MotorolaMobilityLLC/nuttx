/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l4x6xx_memorymap.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32L4X6XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32L4X6XX_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32L4X6XX Address Blocks *******************************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM1_BASE     0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FMC_BASE12     0x60000000     /* 0x60000000-0x7fffffff: 512Mb FMC bank1&2 block */
#  define STM32_FMC_BANK1    0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRAM/SRAM */
#define STM32_FMC_BASE34     0x80000000     /* 0x80000000-0x8fffffff: 512Mb FMC bank3&4 block */
#  define STM32_FMC_BANK3    0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define STM32_FMC_BASE       0xa0000000     /* 0xa0000000-0xa0000fff: FMC register block */
#define STM32_QUADSPI_BASE   0xa0001000     /* 0xa0001000-0xbfffffff: QUADSPI register block */
                                            /* 0xc0000000-0xdfffffff: 512Mb (not used) */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses **************************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
                                            /* 0x00100000-0x07ffffff: Reserved */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
                                            /* 0x08100000-0x0fffffff: Reserved */
#define STM32_SRAM2_BASE     0x10000000     /* 0x10000000-0x1000ffff: 64Kb SRAM */
                                            /* 0x10010000-0x1ffeffff: Reserved */
#define STM32_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff6fff: System memory */
#define STM32_OTP_BASE       0x1fff7000     /* 0x1fff7000-0x1fff73ff: OTP area */
                                            /* 0x1fff7400-0x1fff758f: Reserved */
#define STM32_UID_BASE       0x1fff7590     /* 0x1fff7590-0x1fff759b: UID */
                                            /* 0x1fff759c-0x1fff77ff: Reserved */
#define STM32_OPTION_BASE    0x1fff7800     /* 0x1fff7800-0x1fff780f: Option bytes */
                                            /* 0x1fff7810-0x1fff7fff: Reserved */
#define STM32_SYSMEM2_BASE   0x1fff8000     /* 0x1fff8000-0x1fffefff: System memory */
                                            /* 0x1ffff000-0x1ffff7ff: Reserved */
#define STM32_OPTION2_BASE   0x1ffff800     /* 0x1ffff800-0x1ffff80f: Option bytes */
                                            /* 0x1ffff810-0x1fffffff: Reserved */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x400097ff: APB1 */
                                            /* 0x40009800-0x4000ffff: Reserved */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x400063ff: APB2 */
                                            /* 0x40016400-0x4001ffff: Reserved */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x400243ff: APB1 */
                                            /* 0x40024400-0x47ffffff: Reserved */
#define STM32_AHB2_BASE      0x48000000     /* 0x48000000-0x50060bff: AHB2 */
                                            /* 0x50060c00-0x5fffffff: Reserved */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 timer */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 timer */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 timer */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 timer */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 timer */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 timer */
#define STM32_LCD_BASE       0x40002400     /* 0x40002400-0x400027ff: LCD */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC & BKP registers */
#define STM32_BKP_BASE       0x40002850
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: Window watchdog (WWDG) */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: Independent watchdog (IWDG) */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2/I2S2 */
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3/I2S3 */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005Bff: I2C2 */
#define STM32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: bxCAN1 */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: Power control PWR */
#define STM32_DAC1_BASE      0x40007400     /* 0x40007400-0x400077ff: DAC1 */
#define STM32_OPAMP_BASE     0x40007800     /* 0x40007800-0x40007bff: OPAMP */
#define STM32_LPTIM1_BASE    0x40007c00     /* 0x40007c00-0x40007fff: LPTIM1 */
#define STM32_LPUART1_BASE   0x40008000     /* 0x40008000-0x400083ff: LPUART1 */
#define STM32_SWPMI1_BASE    0x40008800     /* 0x40008800-0x40008bff: SWPMI1 */
#define STM32_LPTIM2_BASE    0x40009400     /* 0x40009400-0x400097ff: LPTIM2 */

/* APB2 Base Addresses **************************************************************/

#define STM32_SYSCFG_BASE    0x40010000     /* 0x40010000-0x4001002f SYSCFG */
#define STM32_VREF_BASE      0x40010030     /* 0x40010030-0x400101ff VREF */
#define STM32_COMP_BASE      0x40010200     /* 0x40010200-0x400103ff COMP */
#define STM32_EXTI_BASE      0x40010400     /* 0x40010400-0x400107ff EXTI */
#define STM32_FIREWALL_BASE  0x40011c00     /* 0x40011c00-0x40011fff FIREWALL */
#define STM32_SDMMC_BASE     0x40012800     /* 0x40012800-0x40012bff SDMMC */
#define STM32_TIM1_BASE      0x40012c00     /* 0x40012c00-0x40012fff TIM1 */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff SPI1 */
#define STM32_TIM8_BASE      0x40013400     /* 0x40013400-0x400137ff TIM8 */
#define STM32_USART1_BASE    0x40013800     /* 0x40013800-0x40013bff USART1 */
#define STM32_TIM15_BASE     0x40014000     /* 0x40014000-0x400143ff TIM15 */
#define STM32_TIM16_BASE     0x40014400     /* 0x40014400-0x400147ff TIM16 */
#define STM32_TIM17_BASE     0x40014800     /* 0x40014800-0x40014bff TIM17 */
#define STM32_SAI1_BASE      0x40015400     /* 0x40015400-0x400157ff SAI1 */
#  define STM32_SAI1_A_BASE  0x40015404     /*                       Block A */
#  define STM32_SAI1_B_BASE  0x40015424     /*                       Block B */
#define STM32_SAI2_BASE      0x40015800     /* 0x40015800-0x40015bff SAI2 */
#  define STM32_SAI2_A_BASE  0x40015804     /*                       Block A */
#  define STM32_SAI2_B_BASE  0x40015824     /*                       Block B */
#define STM32_DFSDM_BASE     0x40016000     /* 0x40016000-0x400163ff DFSDM */

/* AHB1 Base Addresses **************************************************************/

#define STM32_DMA1_BASE      0x40020000     /* 0x40020000-0x400203ff: DMA1  */
#define STM32_DMA2_BASE      0x40020400     /* 0x40020400-0x400207ff: DMA2  */
#define STM32_RCC_BASE       0x40021000     /* 0x40021000-0x400213ff: Reset and Clock control RCC */
#define STM32_FLASHIF_BASE   0x40022000     /* 0x40022000-0x400223ff: Flash memory interface */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define STM32_TSC_BASE       0x40024000     /* 0x40024000-0x400243ff: TSC */

/* AHB2 Base Addresses **************************************************************/

#define STM32_GPIOA_BASE     0x48000000     /* 0x48000000-0x480003ff: GPIO Port A */
#define STM32_GPIOB_BASE     0x48000400     /* 0x48000400-0x480007ff: GPIO Port B */
#define STM32_GPIOC_BASE     0x48000800     /* 0x48000800-0x48000bff: GPIO Port C */
#define STM32_GPIOD_BASE     0X48000c00     /* 0x48000c00-0x48000fff: GPIO Port D */
#define STM32_GPIOE_BASE     0x48001000     /* 0x48001000-0x480013ff: GPIO Port E */
#define STM32_GPIOF_BASE     0x48001400     /* 0x48001400-0x480017ff: GPIO Port F */
#define STM32_GPIOG_BASE     0x48001800     /* 0x48001800-0x48001bff: GPIO Port G */
#define STM32_GPIOH_BASE     0x48001c00     /* 0x48001c00-0x48001fff: GPIO Port H */
#define STM32_OTGFS_BASE     0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
#define STM32_ADC_BASE       0x50040000     /* 0x50040000-0x500403ff: ADC */
#  define STM32_ADC1_BASE    0x50040000     /*                        ADC1 */
#  define STM32_ADC2_BASE    0x50040100     /*                        ADC2 */
#  define STM32_ADC3_BASE    0x50040200     /*                        ADC3 */
#  define STM32_ADCCMN_BASE  0x50040300     /*                        Common */
#define STM32_AES_BASE       0x50060000     /* 0x50060000-0x500603ff: AES */
#define STM32_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* Cortex-M4 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M4 registers in this
 * address range
 */

#define STM32_SCS_BASE      0xe000e000
#define STM32_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32L4X6XX_MEMORYMAP_H */
