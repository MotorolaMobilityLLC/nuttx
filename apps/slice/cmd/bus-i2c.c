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
#include <nuttx/i2c.h>
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
#include "cmd_main.h"

struct slice_bus_i2c_data
{
    FAR struct i2c_dev_s *i2c;
    struct slice_bus_data *bus;

    int reg;
    int reg_idx;
    bool reg_write;
};

static struct slice_bus_i2c_data bus_i2c_data;

/* called when master is writing to slave */
static int bus_i2c_read_cb(void *v)
{
  struct slice_bus_i2c_data *slf = (struct slice_bus_i2c_data *)v;
  uint8_t val;

  I2C_SLAVE_READ(slf->i2c, &val, sizeof(val));

  switch (slf->reg)
    {
      case SLICE_REG_INVALID:
        // Ignore additional written bytes since register invalid
        break;
      case SLICE_REG_NOT_SET:
        if (val < SLICE_REG__NUM)
          {
            slf->reg = val;
          }
        else
          {
            slf->reg = SLICE_REG_INVALID;
          }
        break;
      case SLICE_REG_INT:
      case SLICE_REG_VERSION:
        // Writes to these registers are not allowed
        break;
      case SLICE_REG_SVC:
        if (slf->reg_idx < SLICE_REG_SVC_RX_SZ)
          {
            slf->bus->reg_svc_rx[slf->reg_idx++] = val;
          }
        slf->reg_write = true;
        break;
      case SLICE_REG_UNIPRO:
        if (slf->reg_idx < SLICE_REG_UNIPRO_SZ)
          {
            slf->bus->reg_unipro_rx[slf->reg_idx++] = val;
          }
        slf->reg_write = true;
        break;
    }

  return 0;
}

/* called when master is reading from slave */
static int bus_i2c_write_cb(void *v)
{
  struct slice_bus_i2c_data *slf = (struct slice_bus_i2c_data *)v;
  struct slice_svc_msg *m;
  uint8_t val = 0;

  if (slf->reg == SLICE_REG_INT)
    {
      val = slf->bus->reg_int;
    }
  else if ((slf->reg == SLICE_REG_SVC) && !list_is_empty(&slf->bus->reg_svc_tx_fifo))
    {
      m = list_entry(slf->bus->reg_svc_tx_fifo.next, struct slice_svc_msg, list);
      val = m->buf[slf->reg_idx];
      slf->reg_idx = (slf->reg_idx + 1) % m->size;
      if (slf->reg_idx == 0)
        {
          /* host has read entire SVC message */
          list_del(slf->bus->reg_svc_tx_fifo.next);
          bus_interrupt(slf->bus, SLICE_REG_INT_SVC,
                        !list_is_empty(&slf->bus->reg_svc_tx_fifo));
          free(m->buf);
          free(m);
        }
    }
  else if ((slf->reg == SLICE_REG_UNIPRO) && slf->bus->reg_unipro_tx)
    {
      val = slf->bus->reg_unipro_tx[slf->reg_idx];
      slf->reg_idx = (slf->reg_idx + 1) % slf->bus->reg_unipro_tx_size;
      if (slf->reg_idx == 0)
        {
          /* host has read entire Unipro message */
          bus_interrupt(slf->bus, SLICE_REG_INT_UNIPRO, false);
          slf->bus->reg_unipro_tx_size = 0;
          free(slf->bus->reg_unipro_tx);
          slf->bus->reg_unipro_tx = NULL;
        }
    }
  else if (slf->reg == SLICE_REG_VERSION)
    {
      val = FIRMWARE_VERSION_NUM;
    }

  I2C_SLAVE_WRITE(slf->i2c, &val, sizeof(val));
  return 0;
}

/* called to end transaction */
static int bus_i2c_stop_cb(void *v)
{
  struct slice_bus_i2c_data *slf = (struct slice_bus_i2c_data *)v;

  if (slf->reg_write)
    {
      switch (slf->reg)
        {
          case SLICE_REG_SVC:
            svc_handle(slf->bus->reg_svc_rx, slf->reg_idx);
            break;
          case SLICE_REG_UNIPRO:
            bus_greybus_from_base(slf->bus, slf->reg_idx);
            break;
        }
    }

  slf->reg = SLICE_REG_NOT_SET;
  slf->reg_idx = 0;
  slf->reg_write = false;

  return 0;
}

static struct i2c_cb_ops_s cb_ops =
{
  .read = bus_i2c_read_cb,
  .write = bus_i2c_write_cb,
  .stop = bus_i2c_stop_cb,
};

int bus_i2c_init(struct slice_bus_data *bus)
{
  FAR struct i2c_dev_s *dev1;

  dev1 = up_i2cinitialize(CONFIG_SLICE_CMD_I2C_SLAVE_BUS);

  bus_i2c_data.i2c = dev1;
  bus_i2c_data.reg = SLICE_REG_NOT_SET;
  bus_i2c_data.bus = bus;

  if (I2C_SETOWNADDRESS(dev1, CONFIG_SLICE_CMD_I2C_SLAVE_ADDR, 7) == OK)
    {
      I2C_REGISTERCALLBACK(dev1, &cb_ops, &bus_i2c_data);
      logd("I2C slave setup complete!\n");
    }
  else
    {
      logd("I2C slave setup failed!\n");
    }

  return 0;
}

