/************************************************************************************
 * configs/stm32l4discovery/src/stm32_boot.c
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32l4discovery.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* An analog pin is not supported so makes a good N/A value */
#define GB_GPIO_UNDEFINED GPIO_ANALOG

struct gb_gpio_s {
  uint32_t cfgset[2];
};

static struct gb_gpio_s gb_gpios[] = {
  { .cfgset = { GPIO_SLICE_APBE_WAKE_IN,    GPIO_SLICE_APBE_WAKE_OUT    }},
  { .cfgset = { GPIO_SLICE_APBE_BOOTRET_IN, GPIO_SLICE_APBE_BOOTRET_OUT }},
  { .cfgset = { GPIO_SLICE_APBE_PWR_EN_IN,  GPIO_SLICE_APBE_PWR_EN_OUT  }},
  { .cfgset = { GPIO_SLICE_APBE_RST_N_IN ,  GPIO_SLICE_APBE_RST_N_OUT   }},

};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
uint8_t slice_gb_gpios_count(void)
{
  return (uint8_t)(sizeof(gb_gpios) / sizeof(struct gb_gpio_s));
}

uint32_t slice_gb_gpios_get_cfgset(uint8_t which)
{
  return gb_gpios[which].cfgset[0];
}

/* if allowed, swap the primary and secondary configurations */
bool slice_gb_gpios_swap(uint8_t which, uint32_t *new)
{
  uint32_t cs = gb_gpios[which].cfgset[0];
  uint32_t cs_sv = gb_gpios[which].cfgset[1];

  if (cs_sv == GB_GPIO_UNDEFINED)
    return false;

  *new = cs_sv;
  gb_gpios[which].cfgset[0] = cs_sv;
  gb_gpios[which].cfgset[1] = cs;

  return true;
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_led_initialize();
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
  /* Perform board-specific initialization */

  (void)stm32_bringup();
}
#endif
