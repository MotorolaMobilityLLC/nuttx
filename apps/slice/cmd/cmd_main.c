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

#define EXTRA_FMT "%s: "
#define EXTRA_ARG ,__FUNCTION__
#define logd(format, ...) \
  lowsyslog(EXTRA_FMT format EXTRA_ARG, ##__VA_ARGS__)

#define FIRMWARE_VERSION_NUM   0x01

#define SLICE_REG_INVALID        -2
#define SLICE_REG_NOT_SET        -1
#define SLICE_REG_INT             0     /* Interrupt register        */
#define SLICE_REG_SVC             1     /* SVC message register      */
#define SLICE_REG_UNIPRO          2     /* Unipro message register   */
#define SLICE_REG_VERSION         3     /* Firmware version register */
#define SLICE_REG__NUM            4     /* Add new registers above   */

#define SLICE_REG_INT_SVC      0x01
#define SLICE_REG_INT_UNIPRO   0x02

#define SLICE_REG_SVC_RX_SZ       8
#define SLICE_REG_UNIPRO_SZ      16

#define SLICE_MID           "MID-1"

#ifdef CONFIG_EXAMPLES_NSH
extern int nsh_main(int argc, char *argv[]);
#endif

struct slice_svc_msg
{
    uint8_t *buf;
    int size;
    struct list_head list;
};

struct slice_cmd_data
{
  FAR struct i2c_dev_s *i2c;

  pthread_t base_det_thread;
  sem_t base_det_lock;
  bool base_attached;

  int reg;
  int reg_idx;
  bool reg_write;

  uint8_t reg_int;

  struct list_head reg_svc_tx_fifo;
  uint8_t reg_svc_rx[SLICE_REG_SVC_RX_SZ];

  uint8_t reg_unipro_tx_size;
  uint8_t *reg_unipro_tx;
  uint8_t reg_unipro_rx[SLICE_REG_UNIPRO_SZ];
  uint8_t reg_unipro_rx_cport;
};

struct slice_unipro_msg {
    __u8    slice_cport;
    __u8    ap_cport;
    __u8    data[0];
};

static struct slice_cmd_data slice_cmd_self;

static void slice_cmd_interrupt(struct slice_cmd_data *slf, uint8_t int_mask, bool assert)
{
  if (assert)
    slf->reg_int |= int_mask;
  else
    slf->reg_int &= ~(int_mask);

  logd("set = %d\n", slf->reg_int > 0);
  slice_host_int_set(slf->reg_int > 0);
}

static void *slice_cmd_base_det_worker(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  bool base_present;

  while (1)
    {
      base_present = !slice_base_det_read(); // active low

      if (slf->base_attached != base_present)
        {
          logd("base_present=%d\n", base_present);
          slf->base_attached = base_present;

          if (base_present)
            {
              send_svc_handshake();
              send_svc_event(0, SLICE_MID, NULL);
              enable_cports();
            }
          else
            {
              // Ensure that base charging is disabled
              slice_vbus_en_sw(false);
            }
        }

      sem_wait(&slf->base_det_lock);
    }

  return NULL;
}

static int slice_cmd_base_det_isr(int irq, void *context)
{
  sem_post(&slice_cmd_self.base_det_lock);
  return OK;
}

static void slice_cmd_init(void)
{
  sem_init(&slice_cmd_self.base_det_lock, 0, 0);

  if (pthread_create(&slice_cmd_self.base_det_thread, NULL,
                     slice_cmd_base_det_worker, &slice_cmd_self))
    {
      logd("Failed to create pthread for interrupts!\n");
    }
  else if (slice_init(slice_cmd_base_det_isr) != OK)
    {
      logd("Failed to configure GPIOs!\n");
    }
}

static int slice_cmd_listen(unsigned int cport)
{
  logd("cport=%d\n", cport);
  return 0;
}

static int slice_cmd_recv_from_unipro(unsigned int cportid, const void *buf, size_t len)
{
  struct cport_msg *cmsg;

  logd("slice_cport=%d, ap_cport=%d, len=%d\n",
       cportid, slice_cmd_self.reg_unipro_rx_cport, len);

  if (slice_cmd_self.reg_unipro_tx)
    {
      // TODO: Check if a FIFO is needed, or if only one message is received at a time.
      logd("Dropping message\n");
      return -ENOMEM;
    }

  cmsg = malloc(len + 1);
  if (!cmsg)
      return -ENOMEM;

  cmsg->cport = slice_cmd_self.reg_unipro_rx_cport;
  memcpy(cmsg->data, buf, len);

  gb_dump(cmsg->data, len);

  slice_cmd_self.reg_unipro_tx_size = len + 1;
  slice_cmd_self.reg_unipro_tx = (uint8_t *)cmsg;
  slice_cmd_interrupt(&slice_cmd_self, SLICE_REG_INT_UNIPRO, true);

  return 0;
}

struct gb_transport_backend slice_cmd_unipro_backend = {
    .init = slice_cmd_init,
    .listen = slice_cmd_listen,
    .send = slice_cmd_recv_from_unipro,
};

static int slice_cmd_recv_from_svc(void *buf, size_t length)
{
  struct slice_svc_msg *m;

  bool was_empty = list_is_empty(&slice_cmd_self.reg_svc_tx_fifo);
  logd("length=%d, fifo_empty=%d\n", length, was_empty);

  m = malloc(sizeof(struct slice_svc_msg));
  if (m)
    {
      m->buf = malloc(length);
      if (m->buf)
        {
          memcpy(m->buf, buf, length);
          m->size = length;
          list_add(&slice_cmd_self.reg_svc_tx_fifo, &m->list);

          slice_cmd_interrupt(&slice_cmd_self, SLICE_REG_INT_SVC, true);
          return 0;
        }
      else
        {
          free(m);
        }
    }

  return -1;
}

/* called when master is writing to slave */
static int slice_cmd_read_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
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
            slf->reg_svc_rx[slf->reg_idx++] = val;
          }
        slf->reg_write = true;
        break;
      case SLICE_REG_UNIPRO:
        if (slf->reg_idx < SLICE_REG_UNIPRO_SZ)
          {
            slf->reg_unipro_rx[slf->reg_idx++] = val;
          }
        slf->reg_write = true;
        break;
    }

  return 0;
}

/* called when master is reading from slave */
static int slice_cmd_write_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  struct slice_svc_msg *m;
  uint8_t val = 0;

  if (slf->reg == SLICE_REG_INT)
    {
      val = slf->reg_int;
    }
  else if ((slf->reg == SLICE_REG_SVC) && !list_is_empty(&slf->reg_svc_tx_fifo))
    {
      m = list_entry(slf->reg_svc_tx_fifo.next, struct slice_svc_msg, list);
      val = m->buf[slf->reg_idx];
      slf->reg_idx = (slf->reg_idx + 1) % m->size;
      if (slf->reg_idx == 0)
        {
          /* host has read entire SVC message */
          list_del(slf->reg_svc_tx_fifo.next);
          slice_cmd_interrupt(slf, SLICE_REG_INT_SVC,
                              !list_is_empty(&slf->reg_svc_tx_fifo));
          free(m->buf);
          free(m);
        }
    }
  else if ((slf->reg == SLICE_REG_UNIPRO) && slf->reg_unipro_tx)
    {
      val = slf->reg_unipro_tx[slf->reg_idx];
      slf->reg_idx = (slf->reg_idx + 1) % slf->reg_unipro_tx_size;
      if (slf->reg_idx == 0)
        {
          /* host has read entire Unipro message */
          slice_cmd_interrupt(&slice_cmd_self, SLICE_REG_INT_UNIPRO, false);
          slf->reg_unipro_tx_size = 0;
          free(slf->reg_unipro_tx);
          slf->reg_unipro_tx = NULL;
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
static int slice_cmd_stop_cb(void *v)
{
  struct slice_cmd_data *slf = (struct slice_cmd_data *)v;
  struct slice_unipro_msg *umsg;

  if (slf->reg_write)
    {
      switch (slf->reg)
        {
          case SLICE_REG_SVC:
            svc_handle(slf->reg_svc_rx, slf->reg_idx);
            break;
          case SLICE_REG_UNIPRO:
            umsg = (struct slice_unipro_msg *) slf->reg_unipro_rx;
            slf->reg_unipro_rx_cport = umsg->ap_cport;
            greybus_rx_handler(umsg->slice_cport, umsg->data, slf->reg_idx - 2);
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
  .read = slice_cmd_read_cb,
  .write = slice_cmd_write_cb,
  .stop = slice_cmd_stop_cb,
};

int slice_cmd_main(int argc, char *argv[])
{
  FAR struct i2c_dev_s *dev1;

  dev1 = up_i2cinitialize(CONFIG_SLICE_CMD_I2C_SLAVE_BUS);

  slice_cmd_self.i2c = dev1;
  slice_cmd_self.reg = SLICE_REG_NOT_SET;
  list_init(&slice_cmd_self.reg_svc_tx_fifo);

  if (I2C_SETOWNADDRESS(dev1, CONFIG_SLICE_CMD_I2C_SLAVE_ADDR, 7) == OK)
    {
      I2C_REGISTERCALLBACK(dev1, &cb_ops, &slice_cmd_self);
      logd("I2C slave setup complete!\n");
    }
  else
    {
      logd("I2C slave setup failed!\n");
    }

  svc_register(slice_cmd_recv_from_svc);
  gb_init(&slice_cmd_unipro_backend);

#ifdef CONFIG_EXAMPLES_NSH
  logd("Calling NSH\n");
  return nsh_main(argc, argv);
#else
  return 0;
#endif
}
