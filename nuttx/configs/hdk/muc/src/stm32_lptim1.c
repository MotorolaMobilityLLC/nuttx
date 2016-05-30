/****************************************************************************
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
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

#include <errno.h>
#include <unistd.h>

#include "stm32_lptim.h"

struct stm32_lptim_dev_s *g_lptim1;

int stm32_lptim1_on(void)
{
  int ret;

  if (g_lptim1)
    {
      return -EBUSY;
    }

  g_lptim1 = stm32_lptim_init(1 /* LPTIM1 */);
  if (!g_lptim1)
    {
      return -ENODEV;
    }


  ret = STM32_LPTIM_SETMODE(g_lptim1, STM32_LPTIM_MODE_CONTINUOUS);
  if (ret)
    {
      return ret;
    }

  ret = STM32_LPTIM_SETCHANNEL(g_lptim1, STM32_LPTIM_CH_CH1, 1 /* enable */);

  return ret;
}

int stm32_lptim1_off(void)
{
  int ret;

  if (!g_lptim1)
    {
      return -EINVAL;
    }

  ret = stm32_lptim_deinit(g_lptim1);
  g_lptim1 = NULL;

  return ret;
}