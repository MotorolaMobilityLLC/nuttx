/*
 * Copyright (C) 2015-2016 Motorola Mobility, LLC.
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
#include <semaphore.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <arch/atomic.h>
#include <arch/board/mods.h>
#include <arch/byteorder.h>

#include <nuttx/gpio.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/greybus/types.h>
#include <nuttx/power/pm.h>
#include <nuttx/ring_buf.h>
#include <nuttx/spi/spi.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include "datalink.h"

/* Default payload size of a SPI packet (in bytes) */
#define DEFAULT_PAYLOAD_SZ (32)

/*
 * Maximum number of entries allowed in the ring buffer. This limit exists
 * to keep RAM usage under control.
 */
#define MAX_NUM_RB_ENTRIES (160)

/* SPI packet header bit definitions */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = valid packet, 0 = dummy packet */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* SPI packet CRC size (in bytes) */
#define CRC_SIZE       (2)

/* Priority to report to PM framework when WAKE line asserted. */
#define PM_ACTIVITY_WAKE 10

/* Macro to determine the payload size from the packet size */
#define PL_SIZE(pkt_size)  (pkt_size - sizeof(struct spi_msg_hdr) - CRC_SIZE)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + sizeof(struct spi_msg_hdr) + CRC_SIZE)

/* Macro for checking if the provided number is a power of two. */
#define IS_PWR_OF_TWO(x) (!(x & (x - 1)) && x)

/*
 * Possible data link layer messages. Responses IDs should be the request ID
 * with the MSB set.
 */
enum dl_msg_id
{
  DL_MSG_ID_BUS_CFG_REQ         = 0x00,
  DL_MSG_ID_BUS_CFG_RESP        = 0x80,
};

struct spi_msg_hdr
{
  __u8 bits;
  __u8 rsvd;
} __packed;

struct mods_spi_dl_s
{
  struct mods_dl_s dl;           /* Externally visible part of the data link interface */
  FAR struct spi_dev_s *spi;     /* SPI handle */

  struct mods_dl_cb_s *cb;       /* Callbacks to network layer */
  enum base_attached_e bstate;   /* Base state (attached/detached) */
  atomic_t xfer;                 /* Flag to indicate transfer in progress */
  size_t pkt_size;               /* Size of packet (hdr + pl + CRC) in bytes */
  size_t new_pkt_size;           /* Packet size to use next time TX queue empty */

  FAR struct work_s wake_work;
  FAR struct work_s tend_work;
  FAR struct work_s terr_work;
  sem_t sem;

  struct ring_buf *txp_rb;       /* Producer ring buffer for TX */
  struct ring_buf *txc_rb;       /* Consumer ring buffer for TX */

  __u8 *rx_buf;                  /* Buffer for received packets */

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[MODS_DL_PAYLOAD_MAX_SZ];
  int rcvd_payload_idx;

  uint32_t rfr_cnt;              /* Counter of RFR still asserted errors seen */
};

struct spi_dl_msg_bus_config_req
{
  __le16 max_pl_size;            /* Maximum payload size supported by base */
} __packed;

struct spi_dl_msg_bus_config_resp
{
  __le32 max_speed;              /* Maximum bus speed supported by the mod */
  __le16 pl_size;                /* Payload size that mod has selected to use */
} __packed;

struct spi_dl_msg
{
  __u8 id;                       /* enum dl_msg_id */
  union
    {
      struct spi_dl_msg_bus_config_req     bus_req;
      struct spi_dl_msg_bus_config_resp    bus_resp;
    };
} __packed;

static int queue_data(FAR struct mods_spi_dl_s *priv, __u8 msg_type,
                      const void *buf, size_t len);

static inline void dl_work_queue(FAR struct work_s *work, worker_t worker,
                                 FAR struct mods_spi_dl_s *priv)
{
  if (work_available(work))
      work_queue(HPWORK, work, worker, priv, 0);
}

static int alloc_callback(struct ring_buf *rb, void *arg)
{
  int *header = ring_buf_get_buf(rb);
  int *rb_num = arg;

  *header = (*rb_num)++;

  return OK;
}

static void set_pkt_size(FAR struct mods_spi_dl_s *priv, size_t pkt_size)
{
  int rb_num;
  unsigned int rb_entries;

  /* Immediately return if packet size is not changing */
  if (pkt_size == priv->pkt_size)
      return;

  /* Free any existing RX buffer (if any) */
  if (priv->rx_buf)
    free(priv->rx_buf);

  /* Free existing TX ring buffer (if any) */
  ring_buf_free_ring(priv->txp_rb, NULL /* free_callback */, NULL /* arg */);

  /* Calculate the number of ring buffer entries are needed */
  rb_entries  = (MODS_DL_PAYLOAD_MAX_SZ / PL_SIZE(pkt_size));
  rb_entries *= unipro_cport_count();
  rb_entries  = MIN(rb_entries, MAX_NUM_RB_ENTRIES);

  /* Allocate RX buffer */
  priv->rx_buf = malloc(pkt_size);
  ASSERT(priv->rx_buf);

  /* Allocate TX ring buffer */
  rb_num = 0;
  priv->txp_rb = ring_buf_alloc_ring(rb_entries /* entries */,
      sizeof(rb_num) /* headroom */, pkt_size /* data len */,
      0 /* tailroom */, alloc_callback /* alloc_callback */,
      NULL /* free_callback */, &rb_num /* arg */);
  ASSERT(priv->txp_rb);
  priv->txc_rb = priv->txp_rb;

  /* Save new packet size */
  priv->pkt_size = pkt_size;

  dbg("%d bytes, %d entries\n", priv->pkt_size, rb_entries);
}

/* Caller must hold semaphore before calling this function! */
static int dl_recv(FAR struct mods_dl_s *dl, FAR const void *buf, size_t len)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)dl;
  struct spi_dl_msg *req = (struct spi_dl_msg *)buf;
  struct spi_dl_msg resp;
  uint16_t pl_size;

  if (sizeof(*req) > len)
    {
      dbg("Dropping short message\n");
      return -EINVAL;
    }

  /* Only BUS_CFG_REQ is supported */
  if (req->id != DL_MSG_ID_BUS_CFG_REQ)
    {
      dbg("Unknown ID (%d)!\n", req->id);
      return -EINVAL;
    }

  resp.id = DL_MSG_ID_BUS_CFG_RESP;
  resp.bus_resp.max_speed = cpu_to_le32(CONFIG_GREYBUS_MODS_MAX_BUS_SPEED);

  pl_size = MIN(CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE,
                le16_to_cpu(req->bus_req.max_pl_size));
  /*
   * Verify new packet size is valid. The payload must be no smaller than
   * the default packet size and must be a power of two.
   */
  if (IS_PWR_OF_TWO(pl_size) && (pl_size >= DEFAULT_PAYLOAD_SZ))
    {
      /*
       * The new packet size cannot be used immediately since the reply
       * must use the current packet size. Save off for now to be set later.
       */
      priv->new_pkt_size = PKT_SIZE(pl_size);
    }
  else
    {
      /*
       * The new payload size is invalid so set returned payload size to
       * zero so the base does not change the payload size.
       */
      pl_size = 0;
    }
  resp.bus_resp.pl_size = cpu_to_le16(pl_size);

  return queue_data(priv, MSG_TYPE_DL, &resp, sizeof(resp));
}

/* Caller must hold semaphore before calling this function! */
static void xfer(FAR struct mods_spi_dl_s *priv)
{
  struct ring_buf *rb;

  rb = priv->txc_rb;

  /* Verify not already setup to tranceive packet */
  if (atomic_get(&priv->xfer))
    {
      vdbg("Already setup to tranceive packet\n");
      return;
    }

  /* Only setup exchange if base has asserted wake or we have something to
   * transmit. */
  if (ring_buf_is_producers(rb) && gpio_get_value(GPIO_MODS_WAKE_N))
    {
      vdbg("WAKE not asserted\n");
      return;
    }

  /* Set flag to indicate a transfer is setup */
  atomic_inc(&priv->xfer);

  if (ring_buf_is_producers(rb))
    {
      vdbg("%d RX\n", *((int *)ring_buf_get_buf(rb)));

      /* Claim a ring buffer entry from the producer for the dummy packet */
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }
  else
    {
      vdbg("%d RX/TX\n", *((int *)ring_buf_get_buf(rb)));
    }

  SPI_EXCHANGE(priv->spi, ring_buf_get_data(rb), priv->rx_buf, priv->pkt_size);

  /* Signal to base that we're ready to tranceive */
  mods_rfr_set(1);

  /* Set the base interrupt line if data is available to be sent. */
  mods_host_int_set(ring_buf_is_consumers(rb));
}

static void cleanup_txc_rb_entry(FAR struct mods_spi_dl_s *priv)
{
  vdbg("%d\n", *((int *)ring_buf_get_buf(priv->txc_rb)));

  memset(ring_buf_get_data(priv->txc_rb), 0, priv->pkt_size);
  ring_buf_reset(priv->txc_rb);
  ring_buf_pass(priv->txc_rb);
  priv->txc_rb = ring_buf_get_next(priv->txc_rb);
}

static void attach_cb(FAR void *arg, enum base_attached_e state)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)arg;
  irqstate_t flags;

  flags = irqsave();

  if (state != BASE_ATTACHED)
    {
      vdbg("Cleaning up datalink\n");

      /* Reset GPIOs to initial state */
      mods_host_int_set(false);
      mods_rfr_set(0);

      if (atomic_get(&priv->xfer))
        {
          /* Cancel SPI transaction */
          SPI_SLAVE_DMA_CANCEL(priv->spi);

          /* Clear transfer setup flag */
          atomic_dec(&priv->xfer);
        }

      /* Ensure all work has stopped */
      work_cancel(HPWORK, &priv->wake_work);
      work_cancel(HPWORK, &priv->tend_work);
      work_cancel(HPWORK, &priv->terr_work);

      /* Cleanup any unsent messages */
      while (ring_buf_is_consumers(priv->txc_rb))
        {
          cleanup_txc_rb_entry(priv);
        }

      /* Return packet size back to default */
      set_pkt_size(priv, PKT_SIZE(DEFAULT_PAYLOAD_SZ));

      /* Reset counters */
      priv->rfr_cnt = 0;
    }

  priv->bstate = state;

  irqrestore(flags);
}

/*
 * Called when transaction with base has started.
 */
static void txn_start_cb(void *v)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)v;
  struct ring_buf *rb;

  /* Deassert ready line to base */
  mods_rfr_set(0);

  /* Deassert interrupt line if no more packets to be sent */
  rb = ring_buf_get_next(priv->txc_rb);
  mods_host_int_set(ring_buf_is_consumers(rb));
}

static void txn_finished_worker(FAR void *arg)
{
  FAR struct mods_spi_dl_s *priv = arg;
  struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)priv->rx_buf;
  size_t pl_size = PL_SIZE(priv->pkt_size);
  buf_t recv = priv->cb->recv;
  int ret;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  vdbg("hdr.bits=0x%02X\n", hdr->bits);

  /* Handle race condition where txn_start_cb does not get invoked
   * before this txn_finished_cb, thereby leaving RFR and INT in
   * active states.
   */
  if (mods_rfr_get())
    {
      txn_start_cb(priv);
      dbg("RFR still asserted: cnt=%d\n", ++(priv->rfr_cnt));
    }

  /* Cleanup TX consumer ring buffer entry */
  cleanup_txc_rb_entry(priv);

  /* Clear transfer setup flag */
  atomic_dec(&priv->xfer);

  if (!(hdr->bits & HDR_BIT_VALID))
    {
      /* Received a dummy packet - no processing to do! */

      /* Only change packet size if TX buffer is empty */
      if ((priv->new_pkt_size > 0) && (priv->txp_rb == priv->txc_rb))
        {
          set_pkt_size(priv, priv->new_pkt_size);
          priv->new_pkt_size = 0;
        }

      goto done;
    }

  if ((hdr->bits & HDR_BIT_TYPE) == MSG_TYPE_DL)
      recv = dl_recv;

  /* Check if un-packetizing is required */
  if (MODS_DL_PAYLOAD_MAX_SZ != pl_size)
    {
      if (priv->rcvd_payload_idx >= MODS_DL_PAYLOAD_MAX_SZ)
        {
          /* Too many packets received! */
          priv->rcvd_payload_idx = 0;
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

      recv(&priv->dl, priv->rcvd_payload, priv->rcvd_payload_idx);
      priv->rcvd_payload_idx = 0;
    }
  else
    {
      /* Un-packetizing not required */
      recv(&priv->dl, &priv->rx_buf[sizeof(struct spi_msg_hdr)], pl_size);
    }

done:
  xfer(priv);

  sem_post(&priv->sem);
}

/*
 * Called when transaction with base has completed. The CRC has been
 * successfully checked by the hardware.
 */
static void txn_finished_cb(void *v)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)v;

  dl_work_queue(&priv->tend_work, txn_finished_worker, priv);
}

static void txn_error_worker(FAR void *arg)
{
  FAR struct mods_spi_dl_s *priv = arg;
  int ret;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  /* Cleanup TX consumer ring buffer entry */
  cleanup_txc_rb_entry(priv);

  /* Ignore any received payload from previous packets */
  priv->rcvd_payload_idx = 0;

  /* Clear transfer setup flag */
  atomic_dec(&priv->xfer);

  xfer(priv);

  sem_post(&priv->sem);
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

  dl_work_queue(&priv->terr_work, txn_error_worker, priv);
}

static const struct spi_cb_ops_s cb_ops =
{
  .txn_start = txn_start_cb,
  .txn_end = txn_finished_cb,
  .txn_err = txn_error_cb,
};

/* Caller must hold semaphore before calling this function! */
static int queue_data(FAR struct mods_spi_dl_s *priv, __u8 msg_type,
                      const void *buf, size_t len)
{
  int remaining = len;
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

  while ((remaining > 0) && (packets > 0))
    {
      struct spi_msg_hdr *hdr;
      __u8 *payload;
      int this_pl;

      if (ring_buf_is_consumers(priv->txp_rb))
        {
          dbg("Ring buffer is full!\n");
          return -ENOMEM;
        }

      hdr = ring_buf_get_data(priv->txp_rb);
      payload = ((__u8 *)ring_buf_get_data(priv->txp_rb)) + sizeof(*hdr);

      /* Determine the payload size of this packet */
      this_pl = MIN(remaining, pl_size);

      /* Populate the SPI message */
      hdr->bits  = HDR_BIT_VALID;
      hdr->bits |= (msg_type & HDR_BIT_TYPE);
      hdr->bits |= (--packets & HDR_BIT_PKTS);
      memcpy(payload, dbuf, this_pl);

      remaining -= this_pl;
      dbuf += this_pl;

      ring_buf_put(priv->txp_rb, priv->pkt_size);
      ring_buf_pass(priv->txp_rb);
      priv->txp_rb = ring_buf_get_next(priv->txp_rb);
    }

  return OK;
}

/* Called by network layer when there is data to be sent to base */
static int queue_data_nw(FAR struct mods_dl_s *dl, const void *buf, size_t len)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)dl;
  int ret;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  ret = queue_data(priv, MSG_TYPE_NW, buf, len);
  if (ret)
      goto err;

  xfer(priv);

err:
  sem_post(&priv->sem);

  return ret;
}

static struct mods_dl_ops_s mods_dl_ops =
{
  .send = queue_data_nw,
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

static void wake_worker(FAR void *arg)
{
  FAR struct mods_spi_dl_s *priv = arg;
  int ret;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  xfer(priv);
  sem_post(&priv->sem);
}

static int wake_isr(int irq, void *context)
{
  /* Any wake interrupts when not attached are spurious */
  if (mods_spi_dl.bstate != BASE_ATTACHED)
    {
      llvdbg("ignored\n");
      return OK;
    }

  llvdbg("asserted\n");

  pm_activity(PM_ACTIVITY_WAKE);
  dl_work_queue(&mods_spi_dl.wake_work, wake_worker, &mods_spi_dl);

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
  sem_init(&mods_spi_dl.sem, 0, 0);

  set_pkt_size(&mods_spi_dl, PKT_SIZE(DEFAULT_PAYLOAD_SZ));

  /* RDY GPIO must be initialized before the WAKE interrupt */
  mods_rfr_init();

  gpio_direction_in(GPIO_MODS_WAKE_N);
  gpio_irqattach(GPIO_MODS_WAKE_N, wake_isr);
  set_gpio_triggering(GPIO_MODS_WAKE_N, IRQ_TYPE_EDGE_FALLING);

#ifdef GPIO_MODS_CC_EN
  gpio_direction_out(GPIO_MODS_CC_EN, 1);
#endif

  mods_attach_register(attach_cb, &mods_spi_dl);

#ifdef CONFIG_PM
  if (pm_register(&pm_callback) != OK)
    {
      dbg("Failed register to power management!\n");
    }
#endif

  sem_post(&mods_spi_dl.sem);

  return (FAR struct mods_dl_s *)&mods_spi_dl;
}
