/****************************************************************************
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
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

#include <debug.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>

#include <arch/board/board.h>

static uint32_t g_resetflags;

int up_resetflags(uint32_t *flags)
{
  uint32_t regval;

  /* If the reset flags have previously been read, return the earlier result. */

  if (g_resetflags)
    {
      *flags = g_resetflags;
      return OK;
    }

  regval = getreg32(STM32_RCC_CSR);

  /*
   * Reset flags stay set until cleared by software, so clear the reset flags.
   * This is done so that when another reset occurs, there won't be old reset
   * flags still set from a previous boot.
   */

  regval |= RCC_CSR_RMVF;
  putreg32(regval, STM32_RCC_CSR);

  /* Only return the reset flags to the caller */

  regval &= RCC_CSR_RSTF_MASK;
  *flags = regval;

  /*
   * Because the reset flags are now clear, save off the original register value
   * so this function can be called again and get the same result.
   */

  g_resetflags = regval;

  return OK;
}
