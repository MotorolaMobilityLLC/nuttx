/*
 * Copyright (C) 2015-2016 Motorola Mobility, LLC.
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
 * contributors may be used to endorse or promote products derived from this
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

#include <debug.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>

#include <arch/board/mods.h>
#include <arch/irq.h>

#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/notifier.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

/*
 * Priority to report to PM framework when reporting activity. Report high
 * activity since user-initiated.
 */
#define PM_MODS_ACTIVITY 10

/*
 * The base must be gone for 50 milliseconds before we can declare mod is
 * detached.
 */
#define BASE_GONE_TICKS  MSEC2TICK(50)

/*
 * The chip select line must be asserted for 1 second before we can declare mod
 * is attached to a dead/off base.
 */
#define CS_ASSERT_TICKS  MSEC2TICK(1000)

/* If base attach GPIO has not been selected by the board, use SL_BPLUS_EN */
#ifndef GPIO_MODS_BASE_ATTACH
#  define GPIO_MODS_BASE_ATTACH  GPIO_MODS_SL_BPLUS_EN
#endif

static NOTIFIER_HEAD(g_notify_head);
static enum base_attached_e g_base_state = BASE_INVALID;
static struct work_s g_attach_work;

#ifdef GPIO_MODS_SPI_CS_N
static xcpt_t g_cs_isr;
#endif

static enum base_attached_e read_base_state(void)
{
  enum base_attached_e base_state;

  base_state = !!gpio_get_value(GPIO_MODS_BASE_ATTACH);

#ifdef GPIO_MODS_SPI_CS_N
  if (!base_state)
      base_state = (!gpio_get_value(GPIO_MODS_SPI_CS_N) & 0x01) << 1;
#endif

  return base_state;
}

static void base_attach_worker(FAR void *arg)
{
  enum base_attached_e base_state;

  base_state = read_base_state();
  if (g_base_state != base_state)
    {
      dbg("base_state=%d\n", base_state);

      /* If moving from the attached state to a not attached state, send the
       * disconnected message to all greybus drivers.
       */
      if (g_base_state == BASE_ATTACHED)
          gb_notify_all(GB_EVT_DISCONNECTED);

      g_base_state = base_state;

      notifier_call_chain(&g_notify_head, &base_state);
    }
}

static int bplus_isr(int irq, void *context)
{
  uint8_t base;

  pm_activity(PM_MODS_ACTIVITY);
  base = gpio_get_value(GPIO_MODS_BASE_ATTACH);

  llvdbg("base=%d\n", base);

  if (!work_available(&g_attach_work))
      work_cancel(LPWORK, &g_attach_work);

  return work_queue(LPWORK, &g_attach_work, base_attach_worker, NULL,
                    base ? 0 : BASE_GONE_TICKS);
}

#ifdef GPIO_MODS_SPI_CS_N
static int cs_isr(int irq, void *context)
{
  uint8_t cs_val;

  /* Call into SPI chip select ISR (if available) */
  if (g_cs_isr)
      g_cs_isr(irq, context);

  /* If the base is attached, ignore the chip select line */
  if (g_base_state == BASE_ATTACHED)
      return OK;

  pm_activity(PM_MODS_ACTIVITY);
  cs_val = gpio_get_value(GPIO_MODS_SPI_CS_N);

  if (!work_available(&g_attach_work))
      work_cancel(LPWORK, &g_attach_work);

  /*
   * If chip select is asserted (active low), then wait for CS_ASSERT_TICKS
   * before running the worker. Else, run worker immediately to update the
   * state.
   */
  return work_queue(LPWORK, &g_attach_work, base_attach_worker, NULL,
                    cs_val ? 0 : CS_ASSERT_TICKS);
}
#endif

int mods_attach_register(notifier_fn_t callback, void *arg)
{
  int ret;

  ret = notifier_register(&g_notify_head, callback, arg);
  if (ret)
      return ret;

  /* Immediately call back with current state */
  callback(arg, &g_base_state);

  return OK;
}

int mods_attach_init(void)
{
  gpio_irqattach(GPIO_MODS_BASE_ATTACH, bplus_isr);
  set_gpio_triggering(GPIO_MODS_BASE_ATTACH, IRQ_TYPE_EDGE_BOTH);

#ifdef GPIO_MODS_SPI_CS_N
  /*
   * The chip select is already used by the SPI driver, so the previously
   * registered ISR must be saved to be called from within this driver's
   * chip select ISR.
   */
  gpio_irqattach_old(GPIO_MODS_SPI_CS_N, cs_isr, &g_cs_isr);
  set_gpio_triggering(GPIO_MODS_SPI_CS_N, IRQ_TYPE_EDGE_BOTH);
#endif

  /* Get the initial state of base */
  base_attach_worker(NULL);

  return OK;
}
