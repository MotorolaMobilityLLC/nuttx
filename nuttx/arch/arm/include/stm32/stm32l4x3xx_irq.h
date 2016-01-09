/****************************************************************************************************
 * arch/arm/include/stm32/stm32l4x3xx_irq.h
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32L4X3XX_IRQ_H
#define __ARCH_ARM_INCLUDE_STM32L4X3XX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in nuttx/arch/arm/include/stm32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

#define STM32_IRQ_WWDG        (STM32_IRQ_INTERRUPTS+0)  /* 0:  Window Watchdog interrupt */
#define STM32_IRQ_PVD         (STM32_IRQ_INTERRUPTS+1)  /* 1:  PVD through EXTI Line detection interrupt */
#define STM32_IRQ_TAMPER      (STM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper and time stamp interrupts */
#define STM32_IRQ_TIMESTAMP   (STM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper and time stamp interrupts */
#define STM32_IRQ_RTC_WKUP    (STM32_IRQ_INTERRUPTS+3)  /* 3:  RTC global interrupt */
#define STM32_IRQ_FLASH       (STM32_IRQ_INTERRUPTS+4)  /* 4:  Flash global interrupt */
#define STM32_IRQ_RCC         (STM32_IRQ_INTERRUPTS+5)  /* 5:  RCC global interrupt */
#define STM32_IRQ_EXTI0       (STM32_IRQ_INTERRUPTS+6)  /* 6:  EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1       (STM32_IRQ_INTERRUPTS+7)  /* 7:  EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2       (STM32_IRQ_INTERRUPTS+8)  /* 8:  EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3       (STM32_IRQ_INTERRUPTS+9)  /* 9:  EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4       (STM32_IRQ_INTERRUPTS+10) /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1CH1     (STM32_IRQ_INTERRUPTS+11) /* 11: DMA1 Channel 1 global interrupt */
#define STM32_IRQ_DMA1CH2     (STM32_IRQ_INTERRUPTS+12) /* 12: DMA1 Channel 2 global interrupt */
#define STM32_IRQ_DMA1CH3     (STM32_IRQ_INTERRUPTS+13) /* 13: DMA1 Channel 3 global interrupt */
#define STM32_IRQ_DMA1CH4     (STM32_IRQ_INTERRUPTS+14) /* 14: DMA1 Channel 4 global interrupt */
#define STM32_IRQ_DMA1CH5     (STM32_IRQ_INTERRUPTS+15) /* 15: DMA1 Channel 5 global interrupt */
#define STM32_IRQ_DMA1CH6     (STM32_IRQ_INTERRUPTS+16) /* 16: DMA1 Channel 6 global interrupt */
#define STM32_IRQ_DMA1CH7     (STM32_IRQ_INTERRUPTS+17) /* 17: DMA1 Channel 7 global interrupt */
#define STM32_IRQ_ADC1        (STM32_IRQ_INTERRUPTS+18) /* 18: ADC1 global interrupt */
#define STM32_IRQ_CAN1TX      (STM32_IRQ_INTERRUPTS+19) /* 19: CAN1 TX interrupts */
#define STM32_IRQ_CAN1RX0     (STM32_IRQ_INTERRUPTS+20) /* 20: CAN1 RX0 interrupts */
#define STM32_IRQ_CAN1RX1     (STM32_IRQ_INTERRUPTS+21) /* 21: CAN1 RX1 interrupt */
#define STM32_IRQ_CAN1SCE     (STM32_IRQ_INTERRUPTS+22) /* 22: CAN1 SCE interrupt */
#define STM32_IRQ_EXTI95      (STM32_IRQ_INTERRUPTS+23) /* 23: EXTI Line[9:5] interrupts */
#define STM32_IRQ_TIM1BRK     (STM32_IRQ_INTERRUPTS+24) /* 24: TIM1 Break interrupt */
#define STM32_IRQ_TIM15       (STM32_IRQ_INTERRUPTS+24) /* 24: TIM15 global interrupt */
#define STM32_IRQ_TIM1UP      (STM32_IRQ_INTERRUPTS+25) /* 25: TIM1 Update interrupt */
#define STM32_IRQ_TIM16       (STM32_IRQ_INTERRUPTS+25) /* 25: TIM16 global interrupt */
#define STM32_IRQ_TIM1TRGCOM  (STM32_IRQ_INTERRUPTS+26) /* 26: TIM1 Trigger and Commutation interrupts */
#define STM32_IRQ_TIM17       (STM32_IRQ_INTERRUPTS+26) /* 26: TIM17 global interrupt */
#define STM32_IRQ_TIM1CC      (STM32_IRQ_INTERRUPTS+27) /* 27: TIM1 Capture Compare interrupt */
#define STM32_IRQ_TIM2        (STM32_IRQ_INTERRUPTS+28) /* 28: TIM2 global interrupt */
#define STM32_IRQ_RESERVED29  (STM32_IRQ_INTERRUPTS+29) /* 29: Reserved */
#define STM32_IRQ_RESERVED30  (STM32_IRQ_INTERRUPTS+30) /* 30: Reserved */
#define STM32_IRQ_I2C1EV      (STM32_IRQ_INTERRUPTS+31) /* 31: I2C1 event interrupt */
#define STM32_IRQ_I2C1ER      (STM32_IRQ_INTERRUPTS+32) /* 32: I2C1 error interrupt */
#define STM32_IRQ_I2C2EV      (STM32_IRQ_INTERRUPTS+33) /* 33: I2C2 event interrupt */
#define STM32_IRQ_I2C2ER      (STM32_IRQ_INTERRUPTS+34) /* 34: I2C2 error interrupt */
#define STM32_IRQ_SPI1        (STM32_IRQ_INTERRUPTS+35) /* 35: SPI1 global interrupt */
#define STM32_IRQ_SPI2        (STM32_IRQ_INTERRUPTS+36) /* 36: SPI2 global interrupt */
#define STM32_IRQ_USART1      (STM32_IRQ_INTERRUPTS+37) /* 37: USART1 global interrupt */
#define STM32_IRQ_USART2      (STM32_IRQ_INTERRUPTS+38) /* 38: USART2 global interrupt */
#define STM32_IRQ_USART3      (STM32_IRQ_INTERRUPTS+39) /* 39: USART3 global interrupt */
#define STM32_IRQ_EXTI1510    (STM32_IRQ_INTERRUPTS+40) /* 40: EXTI Line[15:10] interrupts */
#define STM32_IRQ_RTCALRM     (STM32_IRQ_INTERRUPTS+41) /* 41: RTC alarm through EXTI line interrupt */
#define STM32_IRQ_RESERVED42  (STM32_IRQ_INTERRUPTS+42) /* 42: Reserved */
#define STM32_IRQ_RESERVED43  (STM32_IRQ_INTERRUPTS+43) /* 43: Reserved */
#define STM32_IRQ_RESERVED44  (STM32_IRQ_INTERRUPTS+44) /* 44: Reserved */
#define STM32_IRQ_RESERVED45  (STM32_IRQ_INTERRUPTS+45) /* 45: Reserved */
#define STM32_IRQ_RESERVED46  (STM32_IRQ_INTERRUPTS+46) /* 46: Reserved */
#define STM32_IRQ_RESERVED47  (STM32_IRQ_INTERRUPTS+47) /* 47: Reserved */
#define STM32_IRQ_RESERVED48  (STM32_IRQ_INTERRUPTS+48) /* 48: Reserved */
#define STM32_IRQ_SDMMC       (STM32_IRQ_INTERRUPTS+49) /* 49: SDMMC global interrupt */
#define STM32_IRQ_RESERVED50  (STM32_IRQ_INTERRUPTS+50) /* 50: Reserved */
#define STM32_IRQ_SPI3        (STM32_IRQ_INTERRUPTS+51) /* 51: SPI3 global interrupt */
#define STM32_IRQ_RESERVED52  (STM32_IRQ_INTERRUPTS+52) /* 52: Reserved */
#define STM32_IRQ_RESERVED53  (STM32_IRQ_INTERRUPTS+53) /* 53: Reserved */
#define STM32_IRQ_TIM6        (STM32_IRQ_INTERRUPTS+54) /* 54: TIM6 global interrupt */
#define STM32_IRQ_DAC         (STM32_IRQ_INTERRUPTS+54) /* 54: DAC1 and DAC2 underrun error interrupts */
#define STM32_IRQ_TIM7        (STM32_IRQ_INTERRUPTS+55) /* 55: TIM7 global interrupt */
#define STM32_IRQ_DMA2CH1     (STM32_IRQ_INTERRUPTS+56) /* 56: DMA2 Channel 1 global interrupt */
#define STM32_IRQ_DMA2CH2     (STM32_IRQ_INTERRUPTS+57) /* 57: DMA2 Channel 2 global interrupt */
#define STM32_IRQ_DMA2CH3     (STM32_IRQ_INTERRUPTS+58) /* 58: DMA2 Channel 3 global interrupt */
#define STM32_IRQ_DMA2CH4     (STM32_IRQ_INTERRUPTS+59) /* 59: DMA2 Channel 4 global interrupt */
#define STM32_IRQ_DMA2CH5     (STM32_IRQ_INTERRUPTS+60) /* 60: DMA2 Channel 5 global interrupt */
#define STM32_IRQ_RESERVED61  (STM32_IRQ_INTERRUPTS+61) /* 61: Reserved */
#define STM32_IRQ_RESERVED62  (STM32_IRQ_INTERRUPTS+62) /* 62: Reserved */
#define STM32_IRQ_RESERVED63  (STM32_IRQ_INTERRUPTS+63) /* 63: Reserved */
#define STM32_IRQ_COMP        (STM32_IRQ_INTERRUPTS+64) /* 64: COMP1/COMP2 through EXTI lines 21/22 interrupts */
#define STM32_IRQ_LPTIM1      (STM32_IRQ_INTERRUPTS+65) /* 65: LPTIM1 global interrupt */
#define STM32_IRQ_LPTIM2      (STM32_IRQ_INTERRUPTS+66) /* 66: LPTIM2 global interrupt */
#define STM32_IRQ_USBWKUP     (STM32_IRQ_INTERRUPTS+67) /* 67: USB event interrupt through EXTI */
#define STM32_IRQ_DMA2CH6     (STM32_IRQ_INTERRUPTS+68) /* 68: DMA2 Channel 6 global interrupt */
#define STM32_IRQ_DMA2CH7     (STM32_IRQ_INTERRUPTS+69) /* 69: DMA2 Channel 7 global interrupt */
#define STM32_IRQ_LPUART1     (STM32_IRQ_INTERRUPTS+70) /* 70: LPUART1 global interrupt */
#define STM32_IRQ_QUADSPI     (STM32_IRQ_INTERRUPTS+71) /* 71: QUADSPI global interrupt */
#define STM32_IRQ_I2C3EV      (STM32_IRQ_INTERRUPTS+72) /* 72: I2C3 event interrupt */
#define STM32_IRQ_I2C3ER      (STM32_IRQ_INTERRUPTS+73) /* 73: I2C3 error interrupt */
#define STM32_IRQ_SAI1        (STM32_IRQ_INTERRUPTS+74) /* 74: Serial Audio Interface 1 global interrupt */
#define STM32_IRQ_RESERVED75  (STM32_IRQ_INTERRUPTS+75) /* 75: Reserved */
#define STM32_IRQ_SWPMI1      (STM32_IRQ_INTERRUPTS+76) /* 76: SWPMI1 global interrupt */
#define STM32_IRQ_TSC         (STM32_IRQ_INTERRUPTS+77) /* 77: TSC global interrupt */
#define STM32_IRQ_LCD         (STM32_IRQ_INTERRUPTS+78) /* 78: LCD global interrupt */
#define STM32_IRQ_AES         (STM32_IRQ_INTERRUPTS+79) /* 79: AES global interrupt */
#define STM32_IRQ_RNG         (STM32_IRQ_INTERRUPTS+80) /* 80: Random Number Generator global interrupt */
#define STM32_IRQ_FPU         (STM32_IRQ_INTERRUPTS+81) /* 81: FPU global interrupt */
#define STM32_IRQ_CRS         (STM32_IRQ_INTERRUPTS+82) /* 82: CRS interrupt */

#define NR_VECTORS            (STM32_IRQ_INTERRUPTS+83)
#define NR_IRQS               (STM32_IRQ_INTERRUPTS+83)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32L4X3XX_IRQ_H */

