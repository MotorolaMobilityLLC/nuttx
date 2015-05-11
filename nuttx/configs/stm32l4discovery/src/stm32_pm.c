/****************************************************************************
 * configs/stm32l4discovery/src/stm32_pm.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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

#include <nuttx/config.h>

#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "stm32_waste.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

#define OPT_KEY1        0x08192A3B
#define OPT_KEY2        0x4C5D6E7F

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_STM32_IWDG
static void stm32_wait_if_busy(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }
}

static void stm32_options_unlock(void)
{
  stm32_wait_if_busy();

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Flash unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_OPTLOCK)
    {
      /* Options unlock sequence */

      putreg32(OPT_KEY1, STM32_FLASH_OPTKEYR);
      putreg32(OPT_KEY2, STM32_FLASH_OPTKEYR);
    }
}

static void stm32_options_start(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OPTSTRT);
  stm32_wait_if_busy();

  /* When the BSY bit is cleared, the options are saved to flash, but not yet
   * applied to the system. Set OBL_LAUNCH to generate a reset and reload
   * of the option bytes.
   */
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OBL_LAUNCH);
}

static void stm32_options_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_OPTLOCK | FLASH_CR_LOCK);
}

static void stm32_freeze_iwdg_stop(void)
{
  uint32_t regval;

  regval = getreg32(STM32_FLASH_OPTR);
  if (regval & (FLASH_OPTR_IWDG_STOP | FLASH_OPTR_IWDG_STDBY))
    {
      stm32_options_unlock();

      regval &= ~(FLASH_OPTR_IWDG_STOP | FLASH_OPTR_IWDG_STDBY);
      putreg32(regval, STM32_FLASH_OPTR);

      stm32_options_start();
      stm32_options_lock();
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pminitialize
 *
 * Description:
 *   This function is called by MCU-specific logic at power-on reset in
 *   order to provide one-time initialization the power management subystem.
 *   This function must be called *very* early in the initialization sequence
 *   *before* any other device drivers are initialized (since they may
 *   attempt to register with the power management subsystem).
 *
 * Input parameters:
 *   None.
 *
 * Returned value:
 *    None.
 *
 ****************************************************************************/

void up_pminitialize(void)
{
  uint32_t regval;

  /* Enable USART2 in Stop Mode */

  regval  = getreg32(STM32_USART2_CR1);
  regval |= USART_CR1_UESM;
  putreg32(regval, STM32_USART2_CR1);

#ifdef CONFIG_STM32_IWDG
  /* IWDG must be frozen in Stop Mode */
  stm32_freeze_iwdg_stop();
#endif

  /* Then initialize the NuttX power management subsystem proper */

  pm_initialize();
}

#endif /* CONFIG_PM */
