/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/list.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/stm32/slice.h>

#include <apps/greybus-utils/manifest.h>
#include <apps/greybus-utils/svc.h>
#include <apps/greybus-utils/utils.h>

#include "bus.h"
#include "bus-i2c.h"
#include "cmd_main.h"

static struct slice_bus_data bus_data;

void bus_interrupt(struct slice_bus_data *slf, uint8_t int_mask, bool assert)
{
  if (assert)
    slf->reg_int |= int_mask;
  else
    slf->reg_int &= ~(int_mask);

  logd("set = %d\n", slf->reg_int > 0);
  slice_host_int_set(slf->reg_int > 0);
}

void bus_greybus_from_base(struct slice_bus_data *slf, size_t len)
{
  struct slice_unipro_msg *umsg;

  umsg = (struct slice_unipro_msg *) slf->reg_unipro_rx;
  slf->reg_unipro_rx_cport = umsg->ap_cport;
  greybus_rx_handler(umsg->slice_cport, umsg->data, len - 2);
}

int bus_greybus_to_base(unsigned int cportid, const void *buf, size_t len)
{
  struct cport_msg *cmsg;

  logd("slice_cport=%d, ap_cport=%d, len=%d\n",
       cportid, bus_data.reg_unipro_rx_cport, len);

  if (bus_data.reg_unipro_tx)
    {
      // TODO: Check if a FIFO is needed, or if only one message is received at a time.
      logd("Dropping message\n");
      return -ENOMEM;
    }

  cmsg = malloc(len + 1);
  if (!cmsg)
      return -ENOMEM;

  cmsg->cport = bus_data.reg_unipro_rx_cport;
  memcpy(cmsg->data, buf, len);

  gb_dump(cmsg->data, len);

  bus_data.reg_unipro_tx_size = len + 1;
  bus_data.reg_unipro_tx = (uint8_t *)cmsg;
  bus_interrupt(&bus_data, SLICE_REG_INT_UNIPRO, true);

  return 0;
}

int bus_svc_to_base(void *buf, size_t length)
{
  struct slice_svc_msg *m;

  bool was_empty = list_is_empty(&bus_data.reg_svc_tx_fifo);
  logd("length=%d, fifo_empty=%d\n", length, was_empty);

  m = malloc(sizeof(struct slice_svc_msg));
  if (m)
    {
      m->buf = malloc(length);
      if (m->buf)
        {
          memcpy(m->buf, buf, length);
          m->size = length;
          list_add(&bus_data.reg_svc_tx_fifo, &m->list);

          bus_interrupt(&bus_data, SLICE_REG_INT_SVC, true);
          return 0;
        }
      else
        {
          free(m);
        }
    }

  return -1;
}

int bus_init(void)
{
  list_init(&bus_data.reg_svc_tx_fifo);

  return bus_i2c_init(&bus_data);
}

