/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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

#include <nuttx/gpio.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/list.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

/*
 * Priority to report to PM framework when reporting activity. Report high
 * activity since user-initiated.
 */
#define PM_MODS_ACTIVITY 10

struct notify_node_s
{
  mods_attach_t callback;
  void *arg;
  struct list_head list;
};

struct mods_attach_data_s
{
  struct list_head notify_list;
  struct work_s attach_work;
  enum base_attached_e base_state;
};

static struct mods_attach_data_s *mods_attach_data;

static enum base_attached_e read_base_state(void)
{
  enum base_attached_e base_state;

  base_state  = (gpio_get_value(GPIO_MODS_SL_BPLUS_EN)   & 0x01) << 0;
#ifdef GPIO_MODS_SL_FORCEFLASH
  base_state |= (gpio_get_value(GPIO_MODS_SL_FORCEFLASH) & 0x01) << 1;
#endif

  return base_state;
}

static void base_attach_worker(FAR void *arg)
{
  struct mods_attach_data_s *ad = (struct mods_attach_data_s *)arg;
  enum base_attached_e base_state;
  struct list_head *iter;
  struct list_head *iter_next;
  struct notify_node_s *node;

  base_state = read_base_state();
  if (ad->base_state != base_state)
    {
      dbg("base_state=%d\n", base_state);
      ad->base_state = base_state;

      list_foreach_safe(&ad->notify_list, iter, iter_next)
        {
          node = list_entry(iter, struct notify_node_s, list);
          node->callback(node->arg, base_state);
        }
    }
}

static int base_attach_isr(int irq, void *context)
{
  pm_activity(PM_MODS_ACTIVITY);

  if (work_available(&mods_attach_data->attach_work))
    {
      work_queue(HPWORK, &mods_attach_data->attach_work,
                 base_attach_worker, mods_attach_data, 0);
    }

  return OK;
}

int mods_attach_register(mods_attach_t callback, void *arg)
{
  irqstate_t flags;
  struct notify_node_s *node;

  if (!callback)
      return -EINVAL;

  if (!mods_attach_data)
      return -EAGAIN;

  node = malloc(sizeof(struct notify_node_s));
  if (!node)
      return -ENOMEM;

  node->callback = callback;
  node->arg = arg;

  flags = irqsave();
  list_add(&mods_attach_data->notify_list, &node->list);
  irqrestore(flags);

  /* Immediately call back with current state */
  callback(arg, mods_attach_data->base_state);

  return 0;
}

int mods_attach_init(void)
{
  mods_attach_data = zalloc(sizeof(struct mods_attach_data_s));
  if (!mods_attach_data)
      return -ENOMEM;

  list_init(&mods_attach_data->notify_list);

  gpio_irqattach(GPIO_MODS_SL_BPLUS_EN, base_attach_isr);
  set_gpio_triggering(GPIO_MODS_SL_BPLUS_EN, IRQ_TYPE_EDGE_BOTH);

#ifdef GPIO_MODS_SL_FORCEFLASH
  gpio_irqattach(GPIO_MODS_SL_FORCEFLASH, base_attach_isr);
  set_gpio_triggering(GPIO_MODS_SL_FORCEFLASH, IRQ_TYPE_EDGE_BOTH);
#endif

  /* Get the initial state of base */
  mods_attach_data->base_state = read_base_state();

  return 0;
}
