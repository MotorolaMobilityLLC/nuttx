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
#include <pthread.h>
#include <semaphore.h>
#include <stddef.h>
#include <stdlib.h>

#include <arch/board/slice.h>
#include <arch/irq.h>

#include <nuttx/gpio.h>
#include <nuttx/greybus/slice.h>
#include <nuttx/list.h>
#include <nuttx/power/pm.h>

/*
 * Priority to report to PM framework when reporting activity. Report high
 * activity since user-initiated.
 */
#define PM_SLICE_ACTIVITY 10

struct notify_node_s
{
  slice_attach_t callback;
  void *arg;
  struct list_head list;
};

struct slice_attach_data_s
{
  pthread_t attach_thread;
  struct list_head notify_list;
  sem_t attach_lock;
  enum base_attached_e base_state;
};

static struct slice_attach_data_s *slice_attach_data;

static void *base_attach_worker(void *v)
{
  struct slice_attach_data_s *ad = (struct slice_attach_data_s *)v;
  enum base_attached_e base_state;
  struct list_head *iter;
  struct list_head *iter_next;
  struct notify_node_s *node;

  while (1)
    {
      pm_activity(PM_SLICE_ACTIVITY);
      usleep(100000);

      base_state  = (gpio_get_value(GPIO_SLICE_SL_BPLUS_EN)   & 0x01) << 0;
      base_state |= (gpio_get_value(GPIO_SLICE_SL_FORCEFLASH) & 0x01) << 1;

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

      sem_wait(&ad->attach_lock);
    }

  return NULL;
}

static int base_attach_isr(int irq, void *context)
{
  sem_post(&slice_attach_data->attach_lock);
  return OK;
}

int slice_attach_register(slice_attach_t callback, void *arg)
{
  irqstate_t flags;
  struct notify_node_s *node;

  if (!callback)
      return -EINVAL;

  if (!slice_attach_data)
      return -EAGAIN;

  node = malloc(sizeof(struct notify_node_s));
  if (!node)
      return -ENOMEM;

  node->callback = callback;
  node->arg = arg;

  flags = irqsave();
  list_add(&slice_attach_data->notify_list, &node->list);
  irqrestore(flags);

  /* Immediately call back with current state */
  callback(arg, slice_attach_data->base_state);

  return 0;
}

int slice_attach_init(void)
{
  int ret;

  slice_attach_data = zalloc(sizeof(struct slice_attach_data_s));
  if (!slice_attach_data)
      return -ENOMEM;

  list_init(&slice_attach_data->notify_list);
  sem_init(&slice_attach_data->attach_lock, 0, 0);

  ret = pthread_create(&slice_attach_data->attach_thread, NULL,
                       base_attach_worker, slice_attach_data);
  if (ret)
    {
      free(slice_attach_data);
      return ret;
    }

  gpio_irqattach(GPIO_SLICE_SL_BPLUS_EN, base_attach_isr);
  set_gpio_triggering(GPIO_SLICE_SL_BPLUS_EN, IRQ_TYPE_EDGE_BOTH);

  gpio_irqattach(GPIO_SLICE_SL_FORCEFLASH, base_attach_isr);
  set_gpio_triggering(GPIO_SLICE_SL_FORCEFLASH, IRQ_TYPE_EDGE_BOTH);

  return 0;
}
