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

#include <arch/board/slice.h>

#include <nuttx/greybus/slice.h>
#include <nuttx/greybus/types.h>
#include <nuttx/list.h>
#include <nuttx/spi/spi.h>
#include <nuttx/util.h>

#include "datalink.h"

/* Size of payload of individual SPI packet (in bytes) */
#define SLICE_SPI_MSG_PAYLOAD_SZ (32)

struct slice_spi_msg
{
  __u8    valid:1;
  __u8    more:1;
  __u8    reserved:6;
  __u8    data[SLICE_SPI_MSG_PAYLOAD_SZ];

  /* Temporary placeholder. Will be calculated and added automatically by HW */
  __le16  crc16;
};

struct slice_fifo_node
{
  void *packet;
  struct list_head list;
};

struct slice_spi_dl_s
{
  struct slice_dl_s dl;          /* Externally visible part of the data link interface */
  FAR struct spi_dev_s *spi;     /* SPI handle */

  struct slice_dl_cb_s *cb;      /* Callbacks to network layer */
  struct list_head tx_fifo;      /* List of messages to send */

  __u8 rx_buf[sizeof(struct slice_spi_msg)];
  __u8 *tx_buf;

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[SLICE_DL_PAYLOAD_MAX_SZ];
  int rcvd_payload_idx;
};

static void setup_exchange(FAR struct slice_spi_dl_s *priv)
{
  irqstate_t flags;
  struct list_head *head;
  struct slice_fifo_node *node;

  flags = irqsave();

  /* Check that active exchange can be interrupted */
  if (priv->tx_buf)
    {
      /* Already setup to transmit packet. Do nothing. */
      goto out;
    }

  /* TODO: Investigate if cancel must be called here */

  if (list_is_empty(&priv->tx_fifo))
    {
      /* Setup to only RX data */
      SPI_RECVBLOCK(priv->spi, priv->rx_buf, sizeof(struct slice_spi_msg));
    }
  else
    {
      head = priv->tx_fifo.next;
      list_del(priv->tx_fifo.next);

      node = list_entry(head, struct slice_fifo_node, list);
      priv->tx_buf = node->packet;
      free(node);

      SPI_EXCHANGE(priv->spi, priv->tx_buf, priv->rx_buf,
                   sizeof(struct slice_spi_msg));
    }

  /* Set the base interrupt line if data is available to be sent. */
  slice_host_int_set(priv->tx_buf != NULL);

out:
  irqrestore(flags);
}

static void attach_cb(FAR void *arg, bool attached)
{
  FAR struct slice_spi_dl_s *priv = (FAR struct slice_spi_dl_s *)arg;
  struct list_head *iter;
  struct list_head *iter_next;
  struct slice_fifo_node *node;

  dbg("attached=%d\n", attached);

  if (attached)
    {
      setup_exchange(priv);
    }
  else
    {
      /* TODO: Cancel SPI transaction */

      /* Cleanup any unsent messages */
      list_foreach_safe(&priv->tx_fifo, iter, iter_next)
        {
          node = list_entry(iter, struct slice_fifo_node, list);
          list_del(iter);
          free(node->packet);
          free(node);
        }
    }
}

/*
 * Called when transaction with base has completed. The CRC has been
 * successfully checked by the hardware.
 */
static int txn_finished_cb(void *v)
{
  FAR struct slice_spi_dl_s *priv = (FAR struct slice_spi_dl_s *)v;
  struct slice_spi_msg *m = (struct slice_spi_msg *)priv->rx_buf;

  if (priv->tx_buf)
    {
      free(priv->tx_buf);
      priv->tx_buf = NULL;
    }

  if (!m->valid)
    {
      /* Received a dummy packet - nothing to do! */
      goto done;
    }

  if (priv->rcvd_payload_idx >= SLICE_DL_PAYLOAD_MAX_SZ)
    {
      /* Too many packets received! */
      goto done;
    }

  memcpy(&priv->rcvd_payload[priv->rcvd_payload_idx], m->data,
         SLICE_SPI_MSG_PAYLOAD_SZ);
  priv->rcvd_payload_idx += SLICE_SPI_MSG_PAYLOAD_SZ;

  if (m->more)
    {
      /* Need additional packets */
      goto done;
    }

  priv->cb->recv(priv->rcvd_payload, priv->rcvd_payload_idx);
  memset(priv->rcvd_payload, 0, SLICE_SPI_MSG_PAYLOAD_SZ);
  priv->rcvd_payload_idx = 0;

done:
  setup_exchange(priv);
  return 0;
}

static const struct spi_cb_ops_s cb_ops =
{
  .read = txn_finished_cb,
  /* write and txn_end callbacks not needed */
};

/* Called by network layer when there is data to be sent to base */
static int queue_data(FAR struct slice_dl_s *dl, const void *buf,
                      size_t len)
{
  FAR struct slice_spi_dl_s *priv = (FAR struct slice_spi_dl_s *)dl;
  struct slice_spi_msg *m;
  struct slice_fifo_node *node;
  size_t remaining = len;
  irqstate_t flags;

  if (len > SLICE_DL_PAYLOAD_MAX_SZ)
      return -E2BIG;

  while (remaining > 0)
    {
      m = malloc(sizeof(struct slice_spi_msg));
      if (!m)
          return -ENOMEM;

      node = malloc(sizeof(struct slice_fifo_node));
      if (!node)
        {
          free(m);
          return -ENOMEM;
        }

      m->valid = 1;
      m->more = (remaining > SLICE_SPI_MSG_PAYLOAD_SZ);
      memcpy(m->data, buf, MIN(len, SLICE_SPI_MSG_PAYLOAD_SZ));
      m->crc16 = 0; /* TODO enable HW CRC */
      node->packet = m;

      flags = irqsave();
      list_add(&priv->tx_fifo, &node->list);
      irqrestore(flags);

      remaining -= MIN(len, SLICE_SPI_MSG_PAYLOAD_SZ);
    }

  setup_exchange(priv);
  return 0;
}

static struct slice_dl_ops_s slice_dl_ops =
{
  .send = queue_data,
};

static struct slice_spi_dl_s slice_spi_dl =
{
  .dl  = { &slice_dl_ops },
};

FAR struct slice_dl_s *slice_dl_init(struct slice_dl_cb_s *cb)
{
  FAR struct spi_dev_s *spi;

  if (!cb)
    return NULL;

  spi = up_spiinitialize(CONFIG_GREYBUS_SLICE_PORT);
  if (!spi)
    return NULL;

  SPI_SLAVE_REGISTERCALLBACK(spi, &cb_ops, &slice_spi_dl);

  slice_spi_dl.cb = cb;
  slice_spi_dl.spi = spi;
  list_init(&slice_spi_dl.tx_fifo);

  slice_attach_register(attach_cb, &slice_spi_dl);

  return (FAR struct slice_dl_s *)&slice_spi_dl;
}

