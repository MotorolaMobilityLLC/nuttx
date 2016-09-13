/************************************************************************************
 * arch/arm/src/stm32/stm32_pwr.h
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32_STM32_PWR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Include the correct PWR register definitions for this STM32 family */

#if defined(CONFIG_STM32_STM32L4X3)
#  include "chip/stm32l4x3xx_pwr.h"
#elif defined(CONFIG_STM32_STM32L4X6)
#  include "chip/stm32l4x6xx_pwr.h"
#else
#  include "chip/stm32_pwr.h"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwr_enablebkp
 *
 * Description:
 *   Enables access to the backup domain (RTC registers, RTC backup data registers
 *   and backup SRAM).
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ************************************************************************************/

void stm32_pwr_enablebkp(void);

/************************************************************************************
 * Name: stm32_pwr_setvos
 *
 * Description:
 *   Set voltage scaling for EneryLite devices.
 *
 * Input Parameters:
 *   vos - Properly aligned voltage scaling select bits for the PWR_CR register.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   At present, this function is called only from initialization logic.  If used
 *   for any other purpose that protection to assure that its operation is atomic
 *   will be required.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_ENERGYLITE
void stm32_pwr_setvos(uint16_t vos);
#endif

/************************************************************************************
 * Name: stm32_pwr_enableusv
 *
 * Description:
 *   Enables or disables the USB Supply Valid monitoring.  Setting this bit is
 *   mandatory to use the USB OTG FS peripheral.
 *
 * Input Parameters:
 *   set - True: Vddusb is valid; False: Vddusb is not present. Logical and electrical
 *         isolation is applied to ignore this supply.
 *
 * Returned Value:
 *   True: The bit was previously set.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_STM32L4X6
bool stm32_pwr_enableusv(bool set);
#else
#  define stm32_pwr_enableusv(set)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_PWR_H */
