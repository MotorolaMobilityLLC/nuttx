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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/i2c.h>
#include <nuttx/list.h>
#include <nuttx/util.h>

#include <arch/board/slice.h>

#include "datalink.h"

#define I2C_BUF_SIZE             32

#define HDR_BIT_VALID (0x01 << 7)
#define HDR_BIT_RESV  (0x03 << 5)
#define HDR_BIT_PKTS  (0x1F << 0)

static int i2c_write_cb(void *v);

struct slice_i2c_msg
{
    uint8_t checksum;
    uint8_t hdr_bits;
    uint8_t data[I2C_BUF_SIZE];
} __packed;

struct slice_i2c_data
{
  /* interfaces up in the stack layer */
  struct slice_dl_s dl;
  struct slice_dl_cb_s *cb;

  /* interfaces down in the stack */
  FAR struct i2c_dev_s *dev;

  /* rx */
  bool in_rx;
  uint32_t rx_ndx;
  uint8_t rx_buf[I2C_BUF_SIZE];

  uint8_t rx_payload[SLICE_DL_PAYLOAD_MAX_SZ];
  int rx_payload_ndx;

  /* tx */
  bool in_tx;
  struct list_head tx_fifo;

};

struct fifo_entry
{
    union {
        struct slice_i2c_msg *msg;
        uint8_t *payload;
    };
    int size;
    int ndx;    /* index into the payload */
    struct list_head list;
};

static inline struct fifo_entry *fifo_entry_alloc(void)
{
  struct fifo_entry *entry;

  entry = (struct fifo_entry *)zalloc(sizeof(struct fifo_entry));
  if (!entry)
      return NULL;

  entry->ndx = 0;
  entry->size = sizeof(struct slice_i2c_msg);

  entry->payload = (uint8_t *)zalloc(entry->size);
  if (!entry->payload)
    {
      free(entry);
      return NULL;
    }
  return entry;
}

static inline void fifo_entry_free(struct fifo_entry *entry)
{
  if (entry)
    {
      free(entry->payload);
      free(entry);
    }
}

static inline uint8_t calc_checksum(uint8_t *data, size_t len)
{
  uint8_t chksum = 0;
  int i;

  // Calculate the checksum
  for (i = 0; i < len; i++)
      chksum += data[i];
  return ~chksum + 1;
}

/* called from transport_send                        */
/* put the data on the tx_fifo and trigger the write */
static int i2c_send(FAR struct slice_dl_s *dev, const void *buf, size_t len)
{
  struct slice_i2c_data *slf = (struct slice_i2c_data *)dev;
  struct fifo_entry *entry;
  size_t remaining;
  irqstate_t irq_flags;

  uint8_t pkts_total; /* total number of pkts */
  uint8_t pkts;
  int i;
  uint8_t * dbuf = (uint8_t *)buf;

  if (len > SLICE_DL_PAYLOAD_MAX_SZ)
    {
      lowsyslog("request too big\n");
      return -E2BIG;
    }


  pkts_total = (len / I2C_BUF_SIZE) + ((len % I2C_BUF_SIZE) > 0);

  for (pkts = pkts_total, i = 0; pkts > 0; pkts--, i++)
  {
      remaining = len - (i * I2C_BUF_SIZE);

      entry = fifo_entry_alloc();
      if (!entry)
        return -ENOMEM;

      memcpy(&entry->msg->data[0], &dbuf[i * I2C_BUF_SIZE], MIN(remaining, I2C_BUF_SIZE));
      entry->msg->hdr_bits |= HDR_BIT_VALID;
      entry->msg->hdr_bits |= pkts;
      entry->msg->checksum = 0xff;

      irq_flags = irqsave();
      list_add(&slf->tx_fifo, &entry->list);
      irqrestore(irq_flags);
   }

  /* trigger write */
  slice_host_int_set(true);

  return 0;
}

/* mod (slave) ---> i2c ---> base (master) */
/* read one byte off fifo entry and write on i2c */
/* once the fifo entry has been sent, then remove it */
static int i2c_write_cb(void *v)
{
  struct slice_i2c_data *slf = (struct slice_i2c_data *)v;
  struct fifo_entry *entry;
  uint8_t val = 0;

  if (!list_is_empty(&slf->tx_fifo))
    {
      entry = list_entry(slf->tx_fifo.next, struct fifo_entry, list);

      if (entry->ndx < entry->size)
        {
          slf->in_tx = true;

          val = entry->payload[entry->ndx];
          entry->ndx++;
        }
        if ((entry->ndx == 1) && (list_count(&slf->tx_fifo) == 1))
        {
            slice_host_int_set(false);
        }
    }

  I2C_SLAVE_WRITE(slf->dev, &val, sizeof(val));
  return 0;
}

/* base (master) ---> i2c ---> mod (slave) */
/* here we just fill the buffer until stop_cb is called */
static int i2c_read_cb(void *v)
{
  struct slice_i2c_data *slf = (struct slice_i2c_data *)v;
  uint8_t val;

  slf->in_rx = true;

  I2C_SLAVE_READ(slf->dev, &val, sizeof(val));

  if (slf->rx_ndx < I2C_BUF_SIZE)
    {
      slf->rx_buf[slf->rx_ndx++] = val;
    }

  return 0;
}

/* called when when the i2c read transaction is complete */
static int i2c_stop_cb(void *v)
{
  struct slice_i2c_data *slf = (struct slice_i2c_data *)v;

  if (slf->in_rx)
    {
      struct slice_i2c_msg *msg = (struct slice_i2c_msg *)slf->rx_buf;

      slf->in_rx = false;
      slf->rx_ndx = 0;

      if (slf->rx_payload_ndx + I2C_BUF_SIZE > SLICE_DL_PAYLOAD_MAX_SZ)
        {
          slf->rx_payload_ndx = 0;
          return -EINVAL;
        }

      memcpy(&slf->rx_payload[slf->rx_payload_ndx], msg->data, sizeof(slf->rx_buf));
      slf->rx_payload_ndx += sizeof(slf->rx_buf);

      if ((msg->hdr_bits & HDR_BIT_PKTS) <= 1)
        {
            /* this is the last i2c_data packet in the message */
            /* we can send it up */
            /* TODO: validate checksum */
            slf->cb->recv(slf->rx_payload, slf->rx_payload_ndx);
            slf->rx_payload_ndx =  0;
        }
    }
  else if (slf->in_tx)
    {
        struct fifo_entry *entry = list_entry(slf->tx_fifo.next, struct fifo_entry, list);
        slf->in_tx = false;

        if (entry->ndx == entry->size)
          {
            int irq_flags = irqsave();
            list_del(slf->tx_fifo.next);
            fifo_entry_free(entry);
            irqrestore(irq_flags);
          }
        else
          {
            /* reset ndx and hope for the best */
            entry->ndx = 0;
          }
    }

  return 0;
}

static struct i2c_cb_ops_s i2c_cb_ops =
{
  .read = i2c_read_cb,
  .write = i2c_write_cb,
  .stop = i2c_stop_cb,
};

static struct slice_dl_ops_s slice_dl_ops =
{
  .send = i2c_send,
};

static struct slice_i2c_data i2c_data = 
{
  .dl  = { &slice_dl_ops },
};

FAR struct slice_dl_s *slice_dl_init(struct slice_dl_cb_s *cb)
{
  i2c_data.cb = cb;
  i2c_data.dev = up_i2cinitialize(CONFIG_GREYBUS_SLICE_PORT);

  list_init(&i2c_data.tx_fifo);
  i2c_data.in_rx = false;

  if (I2C_SETOWNADDRESS(i2c_data.dev, CONFIG_GREYBUS_SLICE_I2C_ADDR, 7) == OK)
    {
      I2C_REGISTERCALLBACK(i2c_data.dev, &i2c_cb_ops, &i2c_data);
      lowsyslog("I2C primary slave setup complete!\n");
    }
  else
    {
      lowsyslog("I2C primary slave setup failed!\n");
    }

  return (FAR struct slice_dl_s*)&i2c_data;
}
