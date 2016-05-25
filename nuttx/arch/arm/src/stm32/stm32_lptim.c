/************************************************************************************
 * arm/arm/src/stm32/stm32_lptim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
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
/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * * may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_lptim.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Configuration ********************************************************************/

/* This module then only compiles if there are enabled timers that are not intended for
 * some other purpose.
 */

#if defined(CONFIG_STM32_LPTIM1) || defined(CONFIG_STM32_LPTIM2)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* TIM Device Structure */

struct stm32_lptim_priv_s
{
  struct stm32_lptim_ops_s *ops;
  stm32_lptim_mode_t        mode;
  uint32_t                  base;   /* LPTIMn base address */
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int stm32_lptim_enable(FAR struct stm32_lptim_dev_s *dev)
{
  ASSERT(dev);

  switch (((struct stm32_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32_LPTIM1
      case STM32_LPTIM1_BASE:
        modifyreg32(STM32_RCC_APB1ENR1, 0, RCC_APB1ENR1_LPTIM1EN);
        break;
#endif
#if CONFIG_STM32_LPTIM2
      case STM32_LPTIM2_BASE:
        modifyreg32(STM32_RCC_APB1ENR2, 0, RCC_APB1ENR2_LPTIM2EN);
        break;
#endif

      default:
        return ERROR;
    }

  return OK;
}

static int stm32_lptim_disable(FAR struct stm32_lptim_dev_s *dev)
{
  ASSERT(dev);

  switch (((struct stm32_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32_LPTIM1
      case STM32_LPTIM1_BASE:
        modifyreg32(STM32_RCC_APB1ENR1, RCC_APB1ENR1_LPTIM1EN, 0);
        break;
#endif
#if CONFIG_STM32_LPTIM2
      case STM32_LPTIM2_BASE:
        modifyreg32(STM32_RCC_APB1ENR2, RCC_APB1ENR2_LPTIM2EN, 0);
        break;
#endif

      default:
        return ERROR;
    }

  return OK;
}

static int stm32_lptim_reset(FAR struct stm32_lptim_dev_s *dev)
{
  ASSERT(dev);

  switch (((struct stm32_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32_LPTIM1
      case STM32_LPTIM1_BASE:
        modifyreg32(STM32_RCC_APB1RSTR1, 0, RCC_APB1RSTR1_LPTIM1RST);
        modifyreg32(STM32_RCC_APB1RSTR1, RCC_APB1RSTR1_LPTIM1RST, 0);
        break;
#endif
#if CONFIG_STM32_LPTIM2
      case STM32_LPTIM2_BASE:
        modifyreg32(STM32_RCC_APB1RSTR2, 0, RCC_APB1RSTR2_LPTIM2RST);
        modifyreg32(STM32_RCC_APB1RSTR2, RCC_APB1RSTR2_LPTIM2RST, 0);
        break;
#endif
    }

  return OK;
}

static int stm32_lptim_get_gpioconfig(FAR struct stm32_lptim_dev_s *dev,
                                      stm32_lptim_channel_t channel,
                                      uint32_t *cfg)
{
  ASSERT(dev);
  ASSERT(cfg);

  channel &= STM32_LPTIM_CH_MASK;

  switch (((struct stm32_lptim_priv_s *)dev)->base)
    {
#if CONFIG_STM32_LPTIM1
      case STM32_LPTIM1_BASE:
        switch (channel)
          {
# if defined(GPIO_LPTIM1_OUT_1)
            case 1:
              *cfg = GPIO_LPTIM1_OUT_1;
              break;
# endif
# if defined(GPIO_LPTIM1_OUT_2)
            case 2:
              *cfg = GPIO_LPTIM1_OUT_2;
              break;
# endif
# if defined(GPIO_LPTIM1_OUT_3)
            case 3:
              *cfg = GPIO_LPTIM1_OUT_3;
              break;
# endif
            default:
              return ERROR;
          }
        break;
#endif /* CONFIG_STM32_LPTIM1 */

#if CONFIG_STM32_LPTIM2
      case STM32_LPTIM2_BASE:
        switch (channel)
          {
# if defined(GPIO_LPTIM2_OUT_1)
            case 1:
              *cfg = GPIO_LPTIM2_OUT_1;
              break;
# endif
# if defined(GPIO_LPTIM2_OUT_2)
            case 2:
              *cfg = GPIO_LPTIM2_OUT_2;
              break;
# endif
# if defined(GPIO_LPTIM2_OUT_3)
            case 3:
              *cfg = GPIO_LPTIM2_OUT_3;
              break;
# endif
            default:
              return ERROR;
          }
        break;
#endif /* CONFIG_STM32_LPTIM2 */

      default:
        return ERROR;
    }

  return OK;
}

/************************************************************************************
 * General Functions
 ************************************************************************************/

static int stm32_lptim_setmode(FAR struct stm32_lptim_dev_s *dev, stm32_lptim_mode_t mode)
{
  const uint32_t addr = ((struct stm32_lptim_priv_s *)dev)->base +
                        STM32_LPTIM_CR_OFFSET;

  ASSERT(dev);

  /* Mode */
  switch (mode & STM32_LPTIM_MODE_MASK)
    {
      case STM32_LPTIM_MODE_DISABLED:
          modifyreg32(addr, LPTIM_CR_ENABLE, 0);
          break;
      case STM32_LPTIM_MODE_SINGLE:
          modifyreg32(addr, 0, LPTIM_CR_ENABLE);
          modifyreg32(addr, 0, LPTIM_CR_SNGSTRT);
          break;
      case STM32_LPTIM_MODE_CONTINUOUS:
          modifyreg32(addr, 0, LPTIM_CR_ENABLE);
          modifyreg32(addr, 0, LPTIM_CR_CNTSTRT);
          break;
      default:
          return ERROR;
    }

  /* Save mode */

  ((struct stm32_lptim_priv_s *)dev)->mode = mode;

  return OK;
}

static int stm32_lptim_setchannel(FAR struct stm32_lptim_dev_s *dev,
                                  stm32_lptim_channel_t channel, int enable)
{
  int ret = OK;
  uint32_t cfg = 0;

  ASSERT(dev);

  /* Configure GPIOs */
  ret = stm32_lptim_get_gpioconfig(dev, channel, &cfg);
  if (!ret)
    {
      if (enable)
        {
          stm32_configgpio(cfg);
        }
      else
        {
          stm32_unconfiggpio(cfg);
        }
    }

  return ret;
}

/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct stm32_lptim_ops_s stm32_lptim_ops =
{
  .setmode        = &stm32_lptim_setmode,
  .setchannel     = &stm32_lptim_setchannel,
};

#if CONFIG_STM32_LPTIM1
struct stm32_lptim_priv_s stm32_lptim1_priv =
{
  .ops        = &stm32_lptim_ops,
  .mode       = STM32_LPTIM_MODE_UNUSED,
  .base       = STM32_LPTIM1_BASE,
};
#endif

#if CONFIG_STM32_LPTIM2
struct stm32_lptim_priv_s stm32_lptim2_priv =
{
  .ops        = &stm32_lptim_ops,
  .mode       = STM32_LPTIM_MODE_UNUSED,
  .base       = STM32_LPTIM2_BASE,
};
#endif

static struct stm32_lptim_dev_s *stm32_lptim_getstruct(int timer)
{
  switch (timer)
    {
#if CONFIG_STM32_LPTIM1
      case 1:
        return (struct stm32_lptim_dev_s *)&stm32_lptim1_priv;
#endif
#if CONFIG_STM32_LPTIM2
      case 2:
        return (struct stm32_lptim_dev_s *)&stm32_lptim2_priv;
#endif
      default:
        return NULL;
    }
}

/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct stm32_lptim_dev_s *stm32_lptim_init(int timer)
{
  struct stm32_lptim_dev_s *dev = NULL;

  /* Get structure and enable power */

  dev = stm32_lptim_getstruct(timer);
  if (!dev)
    {
      return NULL;
    }

  /* Is device already allocated */

  if (((struct stm32_lptim_priv_s *)dev)->mode != STM32_LPTIM_MODE_UNUSED)
    {
      return NULL;
    }

  /* Enable power */

  stm32_lptim_enable(dev);

  /* Reset timer */

  stm32_lptim_reset(dev);

  /* Mark it as used */

  ((struct stm32_lptim_priv_s *)dev)->mode = STM32_LPTIM_MODE_DISABLED;

  return dev;
}

int stm32_lptim_deinit(FAR struct stm32_lptim_dev_s * dev)
{
  ASSERT(dev);

  /* Disable power */

  stm32_lptim_disable(dev);

  /* Mark it as free */

  ((struct stm32_lptim_priv_s *)dev)->mode = STM32_LPTIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32_LPTIM1 || ... || LPTIM2) */