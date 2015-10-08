/****************************************************************************
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

#include <nuttx/config.h>
#include <arch/board/board.h>

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * That is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */
#define HSIRDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

void stm32_board_clockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

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

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select HSI oscillator as wakeup from stop clock */

      regval  = getreg32(STM32_RCC_CFGR);
      regval |= RCC_CFGR_STOPWUCK;
      putreg32(regval, STM32_RCC_CFGR);

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

      /* Normally would configure the main PLL here, but since it is currently
       * not used, leave it off to save power.
       */

      /* Enable FLASH prefetch, instruction cache, data cache, and 2 wait states */

#ifdef CONFIG_STM32_FLASH_PREFETCH
      regval = (FLASH_ACR_LATENCY_2 | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
      regval = (FLASH_ACR_LATENCY_2 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
      putreg32(regval, STM32_FLASH_ACR);

      /* Select HSI clock as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_HSI;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the HSI source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI)
        {
        }

      /* Select regulator voltage Range 2 (for frequencies up to 26 MHz) */

      regval  = getreg32(STM32_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
      regval |= PWR_CR1_VOS_RANGE2;
      putreg32(regval, STM32_PWR_CR1);
    }
}
