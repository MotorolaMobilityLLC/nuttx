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
#include <arch/board/mods.h>

#include <nuttx/gpio.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/greybus/types.h>
#include <nuttx/power/pm.h>
#include <nuttx/ring_buf.h>
#include <nuttx/spi/spi.h>
#include <nuttx/util.h>

#include "datalink.h"

/* Default payload size of a SPI packet (in bytes) */
#define DEFAULT_PAYLOAD_SZ (32)

/* SPI packet header bit definitions */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = valid packet, 0 = dummy packet */
#define HDR_BIT_RSVD   (0x03 << 5)  /* Reserved */
#define HDR_BIT_PKTS   (0x1F << 0)  /* How many additional packets to expect */

/* SPI packet CRC size (in bytes) */
#define CRC_SIZE       (2)

/* Priority to report to PM framework when WAKE line asserted. */
#define PM_ACTIVITY_WAKE 10

/* Macro to determine the payload size from the packet size */
#define PL_SIZE(pkt_size)  (pkt_size - sizeof(struct spi_msg_hdr) - CRC_SIZE)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + sizeof(struct spi_msg_hdr) + CRC_SIZE)

struct spi_msg_hdr
{
  __u8 bits;
} __packed;

struct mods_spi_dl_s
{
  struct mods_dl_s dl;           /* Externally visible part of the data link interface */
  FAR struct spi_dev_s *spi;     /* SPI handle */

  struct mods_dl_cb_s *cb;       /* Callbacks to network layer */
  enum base_attached_e bstate;   /* Base state (attached/detached) */
  atomic_t xfer;                 /* Flag to indicate transfer in progress */
  size_t pkt_size;               /* Size of packet (hdr + pl + CRC) in bytes */

  struct ring_buf *txp_rb;       /* Producer ring buffer for TX */
  struct ring_buf *txc_rb;       /* Consumer ring buffer for TX */

  __u8 *rx_buf;                  /* Buffer for received packets */

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[MODS_DL_PAYLOAD_MAX_SZ];
  int rcvd_payload_idx;
};

static int set_packet_size(FAR struct mods_spi_dl_s *priv, size_t pkt_size)
{
  struct ring_buf *tx_rb_new;
  __u8 *rx_buf_new;

  /* Only change packet size if TX buffer is empty */
  if (priv->txp_rb != priv->txc_rb)
      return -EBUSY;

  /* Allocate RX buffer */
  rx_buf_new = malloc(pkt_size);
  if (!rx_buf_new)
      return -ENOMEM;

  /* Allocate TX ring buffer */
  tx_rb_new = ring_buf_alloc_ring(
      (MODS_DL_PAYLOAD_MAX_SZ / PL_SIZE(pkt_size)) + 1 /* entries */,
      0 /* headroom */, pkt_size /* data len */,
      0 /* tailroom */, NULL /* alloc_callback */, NULL /* free_callback */,
      NULL /* arg */);
  if (!tx_rb_new)
    {
      free(rx_buf_new);
      return -ENOMEM;
    }

  /* Save new packet size */
  priv->pkt_size = pkt_size;

  /* Free any existing RX buffer (if any) */
  if (priv->rx_buf)
    free(priv->rx_buf);

  /* Save pointer to new RX buffer */
  priv->rx_buf = rx_buf_new;

  /* Free existing TX ring buffer (if any) */
  ring_buf_free_ring(priv->txp_rb, NULL /* free_callback */, NULL /* arg */);

  /* Save pointer to new TX ring buffer */
  priv->txp_rb = tx_rb_new;
  priv->txc_rb = tx_rb_new;

  dbg("%d bytes\n", priv->pkt_size);

  return OK;
}

static void setup_xfer(FAR struct mods_spi_dl_s *priv)
{
  irqstate_t flags;
  struct ring_buf *rb;

  flags = irqsave();
  rb = priv->txc_rb;

  /* Verify not already setup to tranceive packet */
  if (atomic_get(&priv->xfer))
    {
      vdbg("Already setup to tranceive packet. Do nothing.\n");
      goto already_setup;
    }

  /* Only setup exchange if base has asserted wake */
  if (gpio_get_value(GPIO_MODS_WAKE_N))
    {
      vdbg("WAKE not asserted\n");
      goto no_wake;
    }

  /* Set flag to indicate a transfer is setup */
  atomic_inc(&priv->xfer);

  if (ring_buf_is_producers(rb))
    {
      dbg("RX\n");

      /* Claim a ring buffer entry from the producer for the dummy packet */
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }
  else
    {
      dbg("RX/TX\n");
    }

  SPI_EXCHANGE(priv->spi, ring_buf_get_data(rb), priv->rx_buf, priv->pkt_size);

  /* Deassert interrupt line (if appropriate) before asserting RDY */
  rb = ring_buf_get_next(rb);
  mods_host_int_set(ring_buf_is_consumers(rb));

  /* Signal to base that we're ready to tranceive */
  mods_rfr_set(1);

no_wake:
  /* Set the base interrupt line if data is available to be sent. */
  mods_host_int_set(ring_buf_is_consumers(rb));

already_setup:
  irqrestore(flags);
}

static void cleanup_txc_rb_entry(FAR struct mods_spi_dl_s *priv)
{
  memset(ring_buf_get_data(priv->txc_rb), 0, priv->pkt_size);
  ring_buf_reset(priv->txc_rb);
  ring_buf_pass(priv->txc_rb);
  priv->txc_rb = ring_buf_get_next(priv->txc_rb);
}

static void attach_cb(FAR void *arg, enum base_attached_e state)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)arg;

  priv->bstate = state;

  if (state == BASE_DETACHED)
    {
      vdbg("Cleaning up datalink\n");

      /* Reset GPIOs to initial state */
      mods_host_int_set(false);
      mods_rfr_set(0);

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
 * Called when transaction with base has started.
 */
static void txn_start_cb(void *v)
{
  /* Deassert ready line to base */
  mods_rfr_set(0);
}

/*
 * Called when transaction with base has completed. The CRC has been
 * successfully checked by the hardware.
 */
static void txn_finished_cb(void *v)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)v;
  struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)priv->rx_buf;
  size_t pl_size = PL_SIZE(priv->pkt_size);

  vdbg("hdr.bits=0x%02X\n", hdr->bits);

  /* Cleanup TX consumer ring buffer entry */
  cleanup_txc_rb_entry(priv);

  /* Clear transfer setup flag */
  atomic_dec(&priv->xfer);

  if (!(hdr->bits & HDR_BIT_VALID))
    {
      /* Received a dummy packet - nothing to do! */
      goto done;
    }

  /* Check if un-packetizing is required */
  if (MODS_DL_PAYLOAD_MAX_SZ != pl_size)
    {
      if (priv->rcvd_payload_idx >= MODS_DL_PAYLOAD_MAX_SZ)
        {
          /* Too many packets received! */
          goto done;
        }

      memcpy(&priv->rcvd_payload[priv->rcvd_payload_idx],
             &priv->rx_buf[sizeof(struct spi_msg_hdr)], pl_size);
      priv->rcvd_payload_idx += pl_size;

      if (hdr->bits & HDR_BIT_PKTS)
        {
          /* Need additional packets */
          goto done;
        }

      priv->cb->recv(priv->rcvd_payload, priv->rcvd_payload_idx);
      priv->rcvd_payload_idx = 0;
    }
  else
    {
      /* Un-packetizing not required */
      priv->cb->recv(&priv->rx_buf[sizeof(struct spi_msg_hdr)], pl_size);
    }

done:
  setup_xfer(priv);
}

/*
 * Called when transaction with base has errored.
 */
static void txn_error_cb(void *v)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)v;

  dbg("Tranceive error\n");

  /* Deassert ready line to base */
  mods_rfr_set(0);

  /* Cleanup TX consumer ring buffer entry */
  cleanup_txc_rb_entry(priv);

  /* Ignore any received payload from previous packets */
  priv->rcvd_payload_idx = 0;

  /* Clear transfer setup flag */
  atomic_dec(&priv->xfer);

  setup_xfer(priv);
}

static const struct spi_cb_ops_s cb_ops =
{
  .txn_start = txn_start_cb,
  .txn_end = txn_finished_cb,
  .txn_err = txn_error_cb,
};

/* Called by network layer when there is data to be sent to base */
static int queue_data(FAR struct mods_dl_s *dl, const void *buf,
                      size_t len)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)dl;
  int remaining = len;
  irqstate_t flags;
  __u8 *dbuf = (__u8 *)buf;
  size_t pl_size = PL_SIZE(priv->pkt_size);
  int packets;

  /* Calculate how many packets are required to send whole payload */
  packets = (remaining + pl_size - 1) / pl_size;

  vdbg("len=%d, packets=%d, bstate=%d\n", len, packets, priv->bstate);

  if (priv->bstate != BASE_ATTACHED)
      return -ENODEV;

  if (len > MODS_DL_PAYLOAD_MAX_SZ)
      return -E2BIG;

  flags = irqsave();
  while ((remaining > 0) && (packets > 0))
    {
      struct spi_msg_hdr *hdr;
      __u8 *payload;
      int this_pl;

      if (ring_buf_is_consumers(priv->txp_rb))
        {
          dbg("Ring buffer is full!\n");
          irqrestore(flags);
          return -ENOMEM;
        }

      hdr = ring_buf_get_data(priv->txp_rb);
      payload = ((__u8 *)ring_buf_get_data(priv->txp_rb)) + sizeof(*hdr);

      /* Determine the payload size of this packet */
      this_pl = MIN(remaining, pl_size);

      /* Populate the SPI message */
      hdr->bits  = HDR_BIT_VALID;
      hdr->bits |= (--packets & HDR_BIT_PKTS);
      memcpy(payload, dbuf, this_pl);

      remaining -= this_pl;
      dbuf += this_pl;

      ring_buf_put(priv->txp_rb, priv->pkt_size);
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }
  irqrestore(flags);

  setup_xfer(priv);
  return 0;
}

static struct mods_dl_ops_s mods_dl_ops =
{
  .send = queue_data,
};

static struct mods_spi_dl_s mods_spi_dl =
{
  .dl  = { &mods_dl_ops },
};

#ifdef CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
  /*
   * Do not allow IDLE when WAKE line or RDY line is asserted. Need to stay
   * awake to reply to commands.
   */
  if ((state >= PM_IDLE) &&
      (!gpio_get_value(GPIO_MODS_WAKE_N) || mods_rfr_get()))
      return -EIO;

  return OK;
}

static struct pm_callback_s pm_callback =
{
  .prepare = pm_prepare,
};
#endif

static int wake_isr(int irq, void *context)
{
  vdbg("Asserted\n");

  pm_activity(PM_ACTIVITY_WAKE);
  setup_xfer(&mods_spi_dl);

  return OK;
}

FAR struct mods_dl_s *mods_dl_init(struct mods_dl_cb_s *cb)
{
  FAR struct spi_dev_s *spi;

  if (!cb)
    return NULL;

  spi = up_spiinitialize(CONFIG_GREYBUS_MODS_PORT);
  if (!spi)
    return NULL;

  SPI_SLAVE_REGISTERCALLBACK(spi, &cb_ops, &mods_spi_dl);

  mods_spi_dl.cb = cb;
  mods_spi_dl.spi = spi;
  atomic_init(&mods_spi_dl.xfer, 0);

  if (set_packet_size(&mods_spi_dl, PKT_SIZE(DEFAULT_PAYLOAD_SZ)) != OK)
    return NULL;

  /* RDY GPIO must be initialized before the WAKE interrupt */
  mods_rfr_init();

  gpio_direction_in(GPIO_MODS_WAKE_N);
  gpio_irqattach(GPIO_MODS_WAKE_N, wake_isr);
  set_gpio_triggering(GPIO_MODS_WAKE_N, IRQ_TYPE_EDGE_FALLING);

  mods_attach_register(attach_cb, &mods_spi_dl);

#ifdef CONFIG_PM
  if (pm_register(&pm_callback) != OK)
    {
      dbg("Failed register to power management!\n");
    }
#endif

  return (FAR struct mods_dl_s *)&mods_spi_dl;
}
