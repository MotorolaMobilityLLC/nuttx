/****************************************************************************
 * arch/arm/src/stm32/stm32l4x3xx_rcc.c
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2011-2012, 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_pwr.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* Same for MSI */

#define MSIRDY_TIMEOUT HSERDY_TIMEOUT

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the Multispeed Internal clock (MSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_MSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset other clock enable bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSION|RCC_CR_HSEON|RCC_CR_CSSON|RCC_CR_PLLON|RCC_CR_PLLSAI1ON);
  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFG_RESET, STM32_RCC_PLLCFG);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIER);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enable the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);

#ifdef CONFIG_STM32_DMA1
  /* DMA 1 clock enable */
  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32_DMA2
  /* DMA 2 clock enable */
  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32_CRC
  /* CRC clock enable */
  regval |= RCC_AHB1ENR_CRCEN;
#endif

  /* TODO: Add TSCEN */

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enable the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

  /* Enable GPIOA-H (Keep GPIOF-G disable. not supported on STM32L 4X3 ) */
#if STM32_NGPIO > 0
  regval |= RCC_AHB2ENR_GPIOAEN;
#endif
#if STM32_NGPIO > 16
  regval |= RCC_AHB2ENR_GPIOBEN;
#endif
#if STM32_NGPIO > 32
  regval |= RCC_AHB2ENR_GPIOCEN;
#endif
#if STM32_NGPIO > 48
  regval |= RCC_AHB2ENR_GPIODEN;
#endif
#if STM32_NGPIO > 64
  regval |= RCC_AHB2ENR_GPIOEEN;
#endif
#if STM32_NGPIO > 112
  regval |= RCC_AHB2ENR_GPIOHEN;
#endif

#if defined(CONFIG_STM32_ADC1)
  /* ADC clock enable */
  regval |= RCC_AHB2ENR_ADCEN;
#endif

  /* TODO: Add AESEN */

#ifdef CONFIG_STM32_RNG
  /* Random number generator clock enable */
  regval |= RCC_AHB2ENR_RNGEN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  /* TODO */
}

/****************************************************************************
 * Name: rcc_enableapb1_1
 *
 * Description:
 *   Enable selected APB1 (register 1) peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1_1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR1 register to enable the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1ENR1);

#ifdef CONFIG_STM32_TIM2
  /* TIM2 clock enable */
  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#ifdef CONFIG_STM32_TIM6
  /* TIM6 clock enable */
  regval |= RCC_APB1ENR1_TIM6EN;
#endif

#ifdef CONFIG_STM32_TIM7
  /* TIM7 clock enable */
  regval |= RCC_APB1ENR1_TIM7EN;
#endif

  /* TODO Add LCDEN */

#ifdef CONFIG_STM32_WWDG
  /* Window watchdog clock enable */
  regval |= RCC_APB1ENR1_WWDGEN;
#endif

#ifdef CONFIG_STM32_SPI2
  /* SPI2 clock enable */
  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#ifdef CONFIG_STM32_SPI3
  /* SPI3 clock enable */
  regval |= RCC_APB1ENR1_SPI3EN;
#endif

#ifdef CONFIG_STM32_USART2
  /* USART 2 clock enable */
  regval |= RCC_APB1ENR1_USART2EN;
#endif

#ifdef CONFIG_STM32_USART3
  /* USART3 clock enable */
  regval |= RCC_APB1ENR1_USART3EN;
#endif

#ifdef CONFIG_STM32_I2C1
  /* I2C1 clock enable */
  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#ifdef CONFIG_STM32_I2C2
  /* I2C2 clock enable */
  regval |= RCC_APB1ENR1_I2C2EN;
#endif

#ifdef CONFIG_STM32_I2C3
  /* I2C3 clock enable */
  regval |= RCC_APB1ENR1_I2C3EN;
#endif

#ifdef CONFIG_STM32_CAN1
  /* CAN 1 clock enable */
  regval |= RCC_APB1ENR1_CAN1EN;
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator for maximum performance.
   */
  regval |= RCC_APB1ENR1_PWREN;

#ifdef CONFIG_STM32_DAC1
  /* DAC1 interface clock enable */
  regval |= RCC_APB1ENR_DAC1EN;
#endif

  /* TODO Add OPAMP & LPTIM1 */

  putreg32(regval, STM32_RCC_APB1ENR1);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1_2
 *
 * Description:
 *   Enable selected APB1 (register 2) peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1_2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR2 register to enable the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1ENR2);

  /* TODO Add LPUART1EN, SWPMI1EN and LPTIM2EN */

  putreg32(regval, STM32_RCC_APB1ENR2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enable the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#if defined (CONFIG_STM32_SYSCFG) || defined (CONFIG_STM32_COMP)
  /* System configuration controller, comparators, and voltage reference buffer
   * clock enable
   */
  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

  /* TODO Add FWEN and SDMMCEN */

#ifdef CONFIG_STM32_TIM1
  /* TIM1 clock enable */
  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32_SPI1
  /* SPI1 clock enable */
  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32_TIM8
  /* TIM8 clock enable */
  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32_USART1
  /* USART1 clock enable */
  regval |= RCC_APB2ENR_USART1EN;
#endif

  /* TODO Add TIM15EN, TIM16EN, TIM17EN, SAI1EN, SAI2EN, and DFSDMEN */

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableccip
 *
 * Description:
 *   Set peripherals independent clock configuration.
 *
 ****************************************************************************/

static inline void rcc_enableccip(void)
{
  uint32_t regval;

  /* Certain peripherals have no clock selected even when their enable bit is
   * set above. Set some defaults in the CCIPR register so those peripherals
   * will at least have a clock.
   */

  regval = getreg32(STM32_RCC_CCIPR);

#if defined(CONFIG_STM32_OTGFS) || defined(CONFIG_STM32_RNG)
  /* TODO: What is a good default 48 MHz clock source? */
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || defined(CONFIG_STM32_ADC3)
  /* Select SYSCLK as ADC clock source */
  regval |= RCC_CCIPR_ADCSEL_SYSCLK;
#endif

#ifdef STM32_BOARD_USEHSI
#if defined(CONFIG_STM32_USART1) && (STM32_USART1_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set USART1 to use HSI clock */
  regval |= RCC_CCIPR_USART1SEL_HSI;
#endif

#if defined(CONFIG_STM32_USART2) && (STM32_USART2_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set USART2 to use HSI clock */
  regval |= RCC_CCIPR_USART2SEL_HSI;
#endif

#if defined(CONFIG_STM32_USART3) && (STM32_USART3_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set USART3 to use HSI clock */
  regval |= RCC_CCIPR_USART3SEL_HSI;
#endif

#if defined(CONFIG_STM32_I2C1) && (STM32_I2C1_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set I2C1 to use HSI clock */
  regval |= RCC_CCIPR_I2C1SEL_HSI;
#endif

#if defined(CONFIG_STM32_I2C2) && (STM32_I2C2_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set I2C2 to use HSI clock */
  regval |= RCC_CCIPR_I2C2SEL_HSI;
#endif

#if defined(CONFIG_STM32_I2C3) && (STM32_I2C3_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set I2C3 to use HSI clock */
  regval |= RCC_CCIPR_I2C3SEL_HSI;
#endif

#endif /* STM32_BOARD_USEHSI */

#if defined(CONFIG_STM32_LPTIM1)
# if (STM32_LPTIM1_FREQUENCY == STM32_LSI_FREQUENCY)
  /* Set LPTIM1 to use LSI clock */
  regval |= RCC_CCIPR_LPTIM1SEL_LSI;
# elif (STM32_LPTIM1_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set LPTIM1 to use HSI clock */
  regval |= RCC_CCIPR_LPTIM1SEL_HSI;
# elif (STM32_LPTIM1_FREQUENCY == STM32_LSE_FREQUENCY)
  /* Set LPTIM1 to use LSE clock */
  regval |= RCC_CCIPR_LPTIM1SEL_LSE;
# endif
#endif

#if defined(CONFIG_STM32_LPTIM2)
# if (STM32_LPTIM2_FREQUENCY == STM32_LSI_FREQUENCY)
  /* Set LPTIM2 to use LSI clock */
  regval |= RCC_CCIPR_LPTIM2SEL_LSI;
# elif (STM32_LPTIM2_FREQUENCY == STM32_HSI_FREQUENCY)
  /* Set LPTIM2 to use HSI clock */
  regval |= RCC_CCIPR_LPTIM2SEL_HSI;
# elif (STM32_LPTIM2_FREQUENCY == STM32_LSE_FREQUENCY)
  /* Set LPTIM2 to use LSE clock */
  regval |= RCC_CCIPR_LPTIM2SEL_LSE;
# endif
#endif

  putreg32(regval, STM32_RCC_CCIPR);
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
  {
    /* Check if the HSIRDY flag is the set in the CR */

    if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
      {
        /* If so, then break-out with timeout > 0 */

        break;
      }
  }
#endif

#ifdef STM32_BOARD_USEHSE
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
  {
    /* Check if the HSERDY flag is the set in the CR */

    if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
      {
        /* If so, then break-out with timeout > 0 */

        break;
      }
  }
#endif

#ifdef STM32_BOARD_USEMSI
  /* Set MSI clock to 8 MHz */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_MSIRANGE_7; /* 8 MHz */
  regval |= RCC_CR_MSIRGSEL;   /* Select new MSIRANGE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the MSI is ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
  {
    /* Check if the MSIRDY flag is the set in the CR */

    if ((getreg32(STM32_RCC_CR) & RCC_CR_MSIRDY) != 0)
      {
        /* If so, then break-out with timeout > 0 */

        break;
      }
  }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 80 MHz.
       */

      regval  = getreg32(STM32_RCC_APB1ENR1);
      regval |= RCC_APB1ENR1_PWREN;
      putreg32(regval, STM32_RCC_APB1ENR1);

      regval  = getreg32(STM32_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
      regval |= PWR_CR1_VOS_RANGE1;
      putreg32(regval, STM32_PWR_CR1);

      /* Set the HCLK source/divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32_RCC_CFGR_HPRE;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32_RCC_CFGR_PPRE2;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK1 divider */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32_RCC_CFGR_PPRE1;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PLL dividers and multiplers to configure the main PLL */

      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
                STM32_PLLCFG_PLLSRC | STM32_PLLCFG_PLLQ | STM32_PLLCFG_PLLR |
                RCC_PLLCFG_PLLREN);
      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

      /* Enable FLASH prefetch, instruction cache, data cache, and 4 wait states */

#ifdef CONFIG_STM32_FLASH_PREFETCH
      regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
      regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
        {
        }
    }
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableccip();
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1_1();
  rcc_enableapb1_2();
  rcc_enableapb2();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
