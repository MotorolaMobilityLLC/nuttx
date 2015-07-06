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

#include <arch/atomic.h>
#include <arch/board/slice.h>

#include <nuttx/gpio.h>
#include <nuttx/greybus/slice.h>
#include <nuttx/greybus/types.h>
#include <nuttx/power/pm.h>
#include <nuttx/ring_buf.h>
#include <nuttx/spi/spi.h>
#include <nuttx/util.h>

#include "datalink.h"

/* Size of payload of individual SPI packet (in bytes) */
#define SLICE_SPI_MSG_PAYLOAD_SZ (32)

#define HDR_BIT_VALID (0x01 << 7)
#define HDR_BIT_MORE  (0x01 << 6)
#define HDR_BIT_RSVD  (0x3F << 0)

/* Priority to report to PM framework when WAKE line asserted. */
#define PM_ACTIVITY_WAKE 10

struct slice_spi_msg
{
  __u8    hdr_bits;
  __u8    data[SLICE_SPI_MSG_PAYLOAD_SZ];

  /* Temporary placeholder. Will be calculated and added automatically by HW */
  __le16  crc16;
} __packed;

struct slice_spi_dl_s
{
  struct slice_dl_s dl;          /* Externally visible part of the data link interface */
  FAR struct spi_dev_s *spi;     /* SPI handle */

  struct slice_dl_cb_s *cb;      /* Callbacks to network layer */
  atomic_t wake;                 /* Flag to indicate wake line asserted */
  struct ring_buf *txp_rb;       /* Producer ring buffer for TX */
  struct ring_buf *txc_rb;       /* Consumer ring buffer for TX */

  __u8 rx_buf[sizeof(struct slice_spi_msg)];

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[SLICE_DL_PAYLOAD_MAX_SZ];
  int rcvd_payload_idx;

#ifdef CONFIG_PM
  enum pm_state_e pmstate;       /* Current power management state */
#endif
};

static void setup_exchange(FAR struct slice_spi_dl_s *priv)
{
  irqstate_t flags;
  struct ring_buf *rb;

  flags = irqsave();
  rb = priv->txc_rb;

  /* Verify not already setup to tranceive packet */
  if (!gpio_get_value(GPIO_SLICE_RDY_N))
    {
      dbg("Already setup to tranceive packet. Do nothing.\n");
      goto out;
    }

  /* Only setup exchange if base has asserted wake */
  if (!atomic_get(&priv->wake))
    {
      dbg("WAKE not asserted\n");
      goto out;
    }
  atomic_dec(&priv->wake);

  if (ring_buf_is_producers(rb))
    {
      dbg("RX only\n");

      /* Claim a ring buffer entry from the producer for the dummy packet */
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }
  else
    {
      dbg("RX and TX\n");
    }

  SPI_EXCHANGE(priv->spi, ring_buf_get_data(rb),
               priv->rx_buf, sizeof(struct slice_spi_msg));

  /* Deassert interrupt line (if appropriate) before asserting RDY */
  rb = ring_buf_get_next(rb);
  slice_host_int_set(ring_buf_is_consumers(rb));

  /* Signal to base that we're ready to tranceive */
  gpio_set_value(GPIO_SLICE_RDY_N, 0);

out:
  /* Set the base interrupt line if data is available to be sent. */
  slice_host_int_set(ring_buf_is_consumers(rb));

  irqrestore(flags);
}

static void cleanup_txc_rb_entry(FAR struct slice_spi_dl_s *priv)
{
  memset(ring_buf_get_data(priv->txc_rb), 0, sizeof(struct slice_spi_msg));
  ring_buf_reset(priv->txc_rb);
  ring_buf_pass(priv->txc_rb);
  priv->txc_rb = ring_buf_get_next(priv->txc_rb);
}

static void attach_cb(FAR void *arg, bool attached)
{
  FAR struct slice_spi_dl_s *priv = (FAR struct slice_spi_dl_s *)arg;

  if (!attached)
    {
      dbg("Cleaning up datalink\n");

      /* Reset GPIOs to initial state */
      slice_host_int_set(false);
      gpio_set_value(GPIO_SLICE_RDY_N, 1);

      /* Cancel SPI transaction */
      SPI_SLAVE_DMA_CANCEL(priv->spi);

      /* Cleanup any unsent messages */
      while (ring_buf_is_consumers(priv->txc_rb))
        {
          cleanup_txc_rb_entry(priv);
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

  dbg("Tranceive complete: hdr_bits=0x%02X\n", m->hdr_bits);

  /* Deassert ready line to base */
  gpio_set_value(GPIO_SLICE_RDY_N, 1);

  /* Cleanup TX consumer ring buffer entry */
  cleanup_txc_rb_entry(priv);

  if (!(m->hdr_bits & HDR_BIT_VALID))
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

  if (m->hdr_bits & HDR_BIT_MORE)
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
  int remaining = len;
  irqstate_t flags;
  __u8 *dbuf = (__u8 *)buf;
  int pl_size;

  dbg("len=%d\n", len);

  if (len > SLICE_DL_PAYLOAD_MAX_SZ)
      return -E2BIG;

  flags = irqsave();
  while (remaining > 0)
    {
      if (ring_buf_is_consumers(priv->txp_rb))
        {
          dbg("Ring buffer is full!\n");
          irqrestore(flags);
          return -ENOMEM;
        }

      m = ring_buf_get_data(priv->txp_rb);

      pl_size = MIN(remaining, SLICE_SPI_MSG_PAYLOAD_SZ);
      m->hdr_bits |= HDR_BIT_VALID;
      m->hdr_bits |= (remaining > SLICE_SPI_MSG_PAYLOAD_SZ) ? HDR_BIT_MORE : 0;
      memcpy(m->data, dbuf, pl_size);

      remaining -= pl_size;
      dbuf += pl_size;

      ring_buf_put(priv->txp_rb, sizeof(*m));
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }
  irqrestore(flags);

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

#ifdef CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
  /*
   * Do not allow standby when WAKE line or RDY line is asserted. Need to stay
   * awake to reply to commands.
   */
  if ((state >= PM_STANDBY) &&
      (!gpio_get_value(GPIO_SLICE_WAKE_N) || !gpio_get_value(GPIO_SLICE_RDY_N)))
      return -EIO;

  return OK;
}

static void pm_notify(struct pm_callback_s *cb, enum pm_state_e state)
{
  slice_spi_dl.pmstate = state;

  /* If state is returned to normal and base has requested wake, setup for
   * SPI exchange.
   */
  if ((PM_NORMAL == state) && atomic_get(&slice_spi_dl.wake))
    {
      setup_exchange(&slice_spi_dl);
    }
}

static struct pm_callback_s pm_callback =
{
  .prepare = pm_prepare,
  .notify = pm_notify,
};
#endif

static int wake_isr(int irq, void *context)
{
  dbg("Wake signal asserted by base\n");

  pm_activity(PM_ACTIVITY_WAKE);
  atomic_inc(&slice_spi_dl.wake);

#ifdef CONFIG_PM
  /* If running at full speed ("normal"), go ahead and setup SPI exchange.
   * else, wait until power management returns system state to normal.
   */
  if (PM_NORMAL == slice_spi_dl.pmstate)
#endif
    {
      setup_exchange(&slice_spi_dl);
    }

  return OK;
}

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
  slice_spi_dl.txp_rb = ring_buf_alloc_ring(
      (SLICE_DL_PAYLOAD_MAX_SZ / SLICE_SPI_MSG_PAYLOAD_SZ) + 1 /* entries */,
      0 /* headroom */, sizeof(struct slice_spi_msg) /* data len */,
      0 /* tailroom */, NULL /* alloc_callback */, NULL /* free_callback */,
      NULL /* arg */);
  slice_spi_dl.txc_rb = slice_spi_dl.txp_rb;
  atomic_init(&slice_spi_dl.wake, 0);

  /* RDY GPIO must be initialized before the WAKE interrupt */
  gpio_direction_out(GPIO_SLICE_RDY_N, 1);

  gpio_direction_in(GPIO_SLICE_WAKE_N);
  gpio_irqattach(GPIO_SLICE_WAKE_N, wake_isr);
  set_gpio_triggering(GPIO_SLICE_WAKE_N, IRQ_TYPE_EDGE_FALLING);

  slice_attach_register(attach_cb, &slice_spi_dl);

#ifdef CONFIG_PM
  if (pm_register(&pm_callback) != OK)
    {
      dbg("Failed register to power management!\n");
    }
#endif

  return (FAR struct slice_dl_s *)&slice_spi_dl;
}
