/*
 * Copyright (C) 2015-2016 Motorola Mobility, LLC.
 * Copyright (c) 2014-2015 Google Inc.
 * Copyright (C) 2009, 2011-2013 Gregory Nutt.
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
#include <queue.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <arch/board/mods.h>
#include <arch/byteorder.h>

#include <nuttx/config.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/greybus/types.h>
#include <nuttx/hires_tmr.h>
#include <nuttx/kthread.h>
#include <nuttx/power/pm.h>
#include <nuttx/ring_buf.h>
#include <nuttx/spi/spi.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/util.h>
#include <nuttx/wqueue.h>

#include "datalink.h"

#if CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE > MODS_DL_PAYLOAD_MAX_SZ
#  error "CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE > MODS_DL_PAYLOAD_MAX_SZ"
#endif

/*
 * Before root version 3, the MuC was responsible for deasserting the RFR and
 * INT signals manually. The signals must be deasserted ASAP when the transfer
 * starts to avoid the base seeing them still asserted after the transfer and
 * taking additional action.
 *
 * To avoid this race, root version 3 automatically deasserts those signals
 * until the MuC deasserts them later.
 */
#if CONFIG_GREYBUS_MODS_HW_ROOT_VERSION < 3
#  define SIGNAL_SOFTWARE_DEASSERT
#endif

/* Protocol version supported by this driver */
#define PROTO_VER           (2)

/*
 * The bare minimum protocol version required by the driver. To remain backwards
 * compatible with every base ever shipped, this value should never be changed
 * and new features must only be enabled once determined that the base supports
 * them.
 */
#define MIN_PROTO_VER       (2)

/* Default payload size of a SPI packet (in bytes) */
#define DEFAULT_PAYLOAD_SZ (32)

/*
 * Maximum number of entries allowed in the ring buffer. This limit exists
 * to keep RAM usage under control.
 */
#define MAX_NUM_RB_ENTRIES (160)

/* SPI packet header bit definitions */
#define HDR_BIT_DUMMY  (0x01 << 9)  /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 << 8)  /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* Possible values for bus config features */
#define DL_BIT_ACK     (1 << 0)     /* Flag to indicate ACKing is supported */

/* SPI packet CRC size (in bytes) */
#define CRC_SIZE       (2)

/* Priority to report to PM framework when WAKE line asserted. */
#define PM_ACTIVITY_WAKE 10

/* The number of times to try sending a datagram before giving up */
#define NUM_TRIES      (3)

/* Maximum time to wait for an ACK from the base */
#define ACK_TIMEOUT_US 100000       /* 100 milliseconds */

/* Size of the header in bytes */
#define HDR_SIZE       sizeof(struct spi_msg_hdr)

/* Macro to determine the payload size from the packet size */
#define PL_SIZE(pkt_size)  (pkt_size - HDR_SIZE - CRC_SIZE)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + HDR_SIZE + CRC_SIZE)

/* Macro for checking if the provided number is a power of two. */
#define IS_PWR_OF_TWO(x) (!(x & (x - 1)) && x)

enum ack
{
  ACK_NEEDED,
  ACK_NOT_NEEDED,
  ACK_ERROR,
};

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
  __le16 bitmask;                /* See HDR_BIT_* defines for values */
} __packed;

struct spi_work_s
{
  struct dq_entry_s dq;          /* Implements a doubly linked list */
  worker_t worker;               /* Work callback */
  FAR void *arg;                 /* Callback argument */
};

struct mods_spi_dl_s
{
  struct mods_dl_s dl;           /* Externally visible part of the data link interface */
  FAR struct spi_dev_s *spi;     /* SPI handle */

  struct mods_dl_cb_s *cb;       /* Callbacks to network layer */
  enum base_attached_e bstate;   /* Base state (attached/detached) */
  bool xfer_setup;               /* Flag to indicate transfer in progress */
  size_t pkt_size;               /* Size of packet (hdr + pl + CRC) in bytes */
  size_t new_pkt_size;           /* Packet size to use next time TX queue empty */
  __u8 proto_ver;                /* Protocol version supported by base */

  pid_t pid;                     /* The task ID of the worker thread */
  struct dq_queue_s q;           /* The queue of pending work */
  struct spi_work_s wake_work;
  struct spi_work_s tend_work;
  struct spi_work_s terr_work;
  sem_t sem;

#ifdef CONFIG_GREYBUS_MODS_ACK
  bool ack_supported;            /* Base supports ACK'ing on success */
  gpio_cfg_t tack_cfg;           /* Saved config for the ACK transmit line */
  gpio_cfg_t rack_cfg;           /* Saved config for the ACK receive line */
  uint8_t tx_tries_remaining;    /* Send attempts remaining before giving up */
#endif

  struct ring_buf *txp_rb;       /* Producer TX ring buffer */
  struct ring_buf *txc_rb;       /* Consumer TX ring buffer */

  __u8 *rx_buf;                  /* Buffer for received packets */

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[MODS_DL_PAYLOAD_MAX_SZ];
  uint32_t rcvd_payload_idx;
  uint8_t pkts_remaining;        /* Number of packets needed to complete msg */
};

struct spi_dl_msg_bus_config_req
{
  __le16 max_pl_size;            /* Maximum payload size supported by base */
  __u8   features;               /* See DL_BIT_* defines for values */
  __u8   version;                /* SPI msg format version of base */
} __packed;

struct spi_dl_msg_bus_config_resp
{
  __le32 max_speed;              /* Maximum bus speed supported by the mod */
  __le16 pl_size;                /* Payload size that mod has selected to use */
  __u8   features;               /* See DL_BIT_* defines for values */
  __u8   version;                /* SPI msg format version supported by mod */
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

static void dl_work_queue(FAR struct mods_spi_dl_s *priv,
                          struct spi_work_s *work, worker_t worker)
{
  irqstate_t flags;

  if (work_available(work))
    {
      work->worker = worker;
      work->arg = priv;

      flags = irqsave();

      dq_addlast((FAR dq_entry_t *)work, &priv->q);
      kill(priv->pid, SIGWORK);  /* Wake up the worker thread */

      irqrestore(flags);
    }
}

/* Interrupts must be disabled when this function is called */
static void dl_work_cancel(FAR struct mods_spi_dl_s *priv,
                           struct spi_work_s *work)
{
  if (work->worker != NULL)
    {
      /* Remove the entry from the work queue and make sure that it is
       * marked as available (i.e., the worker field is nullified).
       */

      dq_rem((FAR dq_entry_t *)work, &priv->q);
      work->worker = NULL;
    }
}

static inline bool txc_rb_is_valid(FAR struct mods_spi_dl_s *priv)
{
  struct spi_msg_hdr *hdr = ring_buf_get_data(priv->txc_rb);

  return (le16_to_cpu(hdr->bitmask) & HDR_BIT_VALID) != 0;
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

  /*
   * Immediately return if packet size is not changing and ring buffer is
   * in a good state.
   */
  if ((pkt_size == priv->pkt_size) && (priv->txp_rb == priv->txc_rb))
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
  struct spi_dl_msg req;
  struct spi_dl_msg resp;
  uint16_t pl_size;

  /*
   * To support bases running firmware with fewer values in the
   * request, copy the message into a zero'd local struct.
   */
  memset(&req, 0, sizeof(req));
  memcpy(&req, buf, MIN(len, sizeof(req)));

  /* Only BUS_CFG_REQ is supported */
  if (req.id != DL_MSG_ID_BUS_CFG_REQ)
    {
      dbg("Unknown ID (%d)!\n", req.id);
      return -EINVAL;
    }

  if (req.bus_req.version < MIN_PROTO_VER)
    {
      dbg("Unsupported protocol version (%d)\n", req.bus_req.version);
      return -EPROTONOSUPPORT;
    }

  priv->proto_ver = req.bus_req.version;

  resp.id = DL_MSG_ID_BUS_CFG_RESP;
  resp.bus_resp.max_speed = cpu_to_le32(CONFIG_GREYBUS_MODS_MAX_BUS_SPEED);

  pl_size = MIN(CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE,
                le16_to_cpu(req.bus_req.max_pl_size));
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

  resp.bus_resp.version = PROTO_VER;
  resp.bus_resp.features = 0;

#ifdef CONFIG_GREYBUS_MODS_ACK
  if (req.bus_req.features & DL_BIT_ACK)
    {
      priv->ack_supported = true;
      resp.bus_resp.features |= DL_BIT_ACK;
    }

  vdbg("ack_supported = %d\n", priv->ack_supported);
#endif

  return queue_data(priv, MSG_TYPE_DL, &resp, sizeof(resp));
}

/* Caller must hold semaphore before calling this function! */
static inline void set_txp_hdr(FAR struct mods_spi_dl_s *priv, uint16_t bits)
{
  struct spi_msg_hdr *hdr = ring_buf_get_data(priv->txp_rb);
  hdr->bitmask = cpu_to_le16(bits);
}

/* Caller must hold semaphore before calling this function! */
static inline void next_txp(FAR struct mods_spi_dl_s *priv)
{
  ring_buf_pass(priv->txp_rb);
  priv->txp_rb = ring_buf_get_next(priv->txp_rb);
}

/* Caller must hold semaphore before calling this function! */
static inline void setup_for_dummy_tx(FAR struct mods_spi_dl_s *priv)
{
  set_txp_hdr(priv, HDR_BIT_DUMMY);
  next_txp(priv);
}

static inline void deassert_rfr_int(void)
{
  mods_rfr_set(0);
  mods_host_int_set(false);
}

/* Caller must hold semaphore before calling this function! */
static void xfer(FAR struct mods_spi_dl_s *priv)
{
  struct ring_buf *rb;
  bool set_int = false;

  rb = priv->txc_rb;

  /* Verify not already setup to transceive packet */
  if (priv->xfer_setup)
    {
      vdbg("Already setup to transceive packet\n");
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
  priv->xfer_setup = true;

  if (ring_buf_is_producers(rb))
    {
      vdbg("%d RX\n", *((int *)ring_buf_get_buf(rb)));
      setup_for_dummy_tx(priv);
    }
  else
    {
      vdbg("%d RX/TX\n", *((int *)ring_buf_get_buf(rb)));
      set_int = true;
    }

#ifdef CONFIG_GREYBUS_MODS_ACK
  if (priv->ack_supported)
    {
      /* Restore the GPIOs back to SPI configuration */
      gpio_cfg_restore(GPIO_MODS_SPI_TACK, priv->tack_cfg);
      gpio_cfg_restore(GPIO_MODS_SPI_RACK, priv->rack_cfg);
      priv->tack_cfg = NULL;
      priv->rack_cfg = NULL;
    }
#endif

  SPI_EXCHANGE(priv->spi, ring_buf_get_data(rb), priv->rx_buf, priv->pkt_size);

  /* Signal to base that we're ready to transceive */
  mods_rfr_set(1);

  /* Set the base interrupt line if data is available to be sent. */
  mods_host_int_set(set_int);
}

static void reset_txc_rb_entry(FAR struct mods_spi_dl_s *priv)
{
  if ((priv->txp_rb == priv->txc_rb) && ring_buf_is_producers(priv->txp_rb))
    {
      vdbg("skip\n");
      return;
    }

  vdbg("%d\n", *((int *)ring_buf_get_buf(priv->txc_rb)));

  memset(ring_buf_get_data(priv->txc_rb), 0, priv->pkt_size);
  ring_buf_reset(priv->txc_rb);
  ring_buf_pass(priv->txc_rb);
  priv->txc_rb = ring_buf_get_next(priv->txc_rb);
}

static int attach_cb(FAR void *arg, const void *data)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)arg;
  enum base_attached_e state = *((enum base_attached_e *)data);
  irqstate_t flags;
  int ret;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  flags = irqsave();

  if (state != BASE_ATTACHED)
    {
      vdbg("Cleaning up datalink\n");

      /* Reset GPIOs to initial state */
      deassert_rfr_int();

#ifdef CONFIG_GREYBUS_MODS_ACK
      if (priv->ack_supported)
        {
          /* Restore the GPIOs back to SPI configuration */
          gpio_cfg_restore(GPIO_MODS_SPI_TACK, priv->tack_cfg);
          gpio_cfg_restore(GPIO_MODS_SPI_RACK, priv->rack_cfg);
          priv->tack_cfg = NULL;
          priv->rack_cfg = NULL;

          priv->ack_supported = false;
        }
      priv->tx_tries_remaining = NUM_TRIES;
#endif

      if (priv->xfer_setup)
        {
          /* Cancel SPI transaction */
          SPI_SLAVE_DMA_CANCEL(priv->spi);

          /* Clear transfer setup flag */
          priv->xfer_setup = false;
        }

      /* Ensure all work has stopped */
      dl_work_cancel(priv, &priv->wake_work);
      dl_work_cancel(priv, &priv->tend_work);
      dl_work_cancel(priv, &priv->terr_work);

      /* Cleanup any unsent messages */
      while (ring_buf_is_consumers(priv->txc_rb))
        {
          reset_txc_rb_entry(priv);
        }

      /* Return packet size back to default */
      set_pkt_size(priv, PKT_SIZE(DEFAULT_PAYLOAD_SZ));

      /* Assume next base only supports the minimum protocol version */
      priv->proto_ver = MIN_PROTO_VER;
    }

  priv->bstate = state;

  irqrestore(flags);
  sem_post(&priv->sem);

  return OK;
}

#ifdef SIGNAL_SOFTWARE_DEASSERT
/*
 * Called when transaction with base has started.
 */
static void txn_start_cb(void *v)
{
  deassert_rfr_int();
}
#endif

static bool ack_handler(FAR struct mods_spi_dl_s *priv, enum ack ack_req)
{
#ifdef CONFIG_GREYBUS_MODS_ACK
  uint32_t start;
  uint8_t ack;
  uint8_t wake_n;

  if (!priv->ack_supported) return true;

  start = hrt_getusec();

  /* Send ACK to base (if requested) */
  priv->tack_cfg = gpio_cfg_save(GPIO_MODS_SPI_TACK);
  gpio_direction_out(GPIO_MODS_SPI_TACK, (ack_req == ACK_NEEDED) ? 1 : 0);

  /* Setup to receive ACK from base */
  priv->rack_cfg = gpio_cfg_save(GPIO_MODS_SPI_RACK);
  gpio_direction_in(GPIO_MODS_SPI_RACK);

  /* If a packet was sent to the base, look for ACK */
  if (txc_rb_is_valid(priv))
    {
      while (!(ack = gpio_get_value(GPIO_MODS_SPI_RACK)) &&
             (wake_n = gpio_get_value(GPIO_MODS_WAKE_N)) &&
             ((hrt_getusec() - start) < ACK_TIMEOUT_US));
      if (!ack && wake_n)
        {
          if (ack_req == ACK_ERROR)
            {
              /*
               * Since both TX and RX failed, need to spin longer to ensure
               * base does not see false ACK.
               */
              while ((wake_n = gpio_get_value(GPIO_MODS_WAKE_N)) &&
                     ((hrt_getusec() - start) < (2 * ACK_TIMEOUT_US)));
            }

          if (--priv->tx_tries_remaining > 0)
            {
              dbg("Retry: No ACK received\n");
              return false;
            }
          else
            {
              dbg("Abort: No ACK received\n");
              priv->tx_tries_remaining = NUM_TRIES;
              return true;
            }
        }
      else if (priv->tx_tries_remaining != NUM_TRIES)
        {
          dbg("Retry successful\n");
          priv->tx_tries_remaining = NUM_TRIES;
        }
    }

  if (ack_req == ACK_ERROR)
    {
      /* Must block long enough to ensure base does not see false ACK. */
      while ((wake_n = gpio_get_value(GPIO_MODS_WAKE_N)) &&
             ((hrt_getusec() - start) < (2 * ACK_TIMEOUT_US)));

      /* If here, there either was no TX or the TX was successful and ACK'd,
       * so it is okay to return true.
       */
    }
#endif

  return true;
}

static void txn_finished_worker(FAR void *arg)
{
  FAR struct mods_spi_dl_s *priv = arg;
  struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)priv->rx_buf;
  uint16_t bitmask = le16_to_cpu(hdr->bitmask);
  size_t pl_size = PL_SIZE(priv->pkt_size);
  buf_t recv = priv->cb->recv;
  int ret;
  enum ack ack_req;

  do
    {
      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);

  vdbg("bitmask=0x%04X\n", bitmask);

  deassert_rfr_int();

  switch (bitmask & (HDR_BIT_VALID | HDR_BIT_DUMMY))
    {
      case HDR_BIT_VALID:
        ack_req = ACK_NEEDED;
        break;

      case HDR_BIT_DUMMY:
        ack_req = ACK_NOT_NEEDED;
        break;

      /* If valid and dummy are equal (both 0 or both 1), the packet is
       * garbage.
       */
      default:
        dbg("garbage packet\n");
        ack_req = ACK_ERROR;
        break;
    }

  if (ack_handler(priv, ack_req))
    {
      /* Reset TX consumer ring buffer entry */
      reset_txc_rb_entry(priv);
    }

  /* Clear transfer setup flag */
  priv->xfer_setup = false;

  if (ack_req != ACK_NEEDED)
    {
      /* Received a dummy or garbage packet - no processing to do! */

      /* Only change packet size if TX buffer is empty */
      if ((priv->new_pkt_size > 0) && (priv->txp_rb == priv->txc_rb))
        {
          set_pkt_size(priv, priv->new_pkt_size);
          priv->new_pkt_size = 0;
        }

      goto done;
    }

  if ((bitmask & HDR_BIT_TYPE) == MSG_TYPE_DL)
      recv = dl_recv;

#if CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE == MODS_DL_PAYLOAD_MAX_SZ
  /* Check if un-packetizing is not required */
  if (MODS_DL_PAYLOAD_MAX_SZ == pl_size)
    {
      if (bitmask & HDR_BIT_PKT1)
          recv(&priv->dl, &priv->rx_buf[HDR_SIZE], pl_size);
      else
          dbg("1st pkt bit not set\n");
      goto done;
    }
#endif

  if (bitmask & HDR_BIT_PKT1)
    {
      /* Check if data exists from earlier packets */
      if (priv->rcvd_payload_idx)
        {
          dbg("1st pkt recv'd before prev msg complete\n");
          priv->rcvd_payload_idx = 0;
        }

      priv->pkts_remaining = bitmask & HDR_BIT_PKTS;
    }
  else /* not first packet of message */
    {
      /* Check for data from earlier packets */
      if (!priv->rcvd_payload_idx)
        {
          dbg("Ignore non-first packet\n");
          goto done;
        }

      if ((bitmask & HDR_BIT_PKTS) != --priv->pkts_remaining)
        {
          dbg("Packets remaining out of sync\n");

          /* Drop the entire message */
          priv->rcvd_payload_idx = 0;
          priv->pkts_remaining = 0;
          goto done;
        }
    }

  if (priv->rcvd_payload_idx >= MODS_DL_PAYLOAD_MAX_SZ)
    {
      dbg("Too many packets received\n");

      /* Drop the entire message */
      priv->rcvd_payload_idx = 0;
      priv->pkts_remaining = 0;
      goto done;
    }

  memcpy(&priv->rcvd_payload[priv->rcvd_payload_idx],
         &priv->rx_buf[HDR_SIZE], pl_size);
  priv->rcvd_payload_idx += pl_size;

  if (bitmask & HDR_BIT_PKTS)
    {
      /* Need additional packets */
      goto done;
    }

  recv(&priv->dl, priv->rcvd_payload, priv->rcvd_payload_idx);
  priv->rcvd_payload_idx = 0;

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

  dl_work_queue(priv, &priv->tend_work, txn_finished_worker);
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

  /*
   * To reduce log spam, only log here if SPI debugging is disabled. When
   * enabled, the lower level SPI driver logs more details of the error.
   */
#ifndef CONFIG_DEBUG_SPI
  dbg("Transceive error\n");
#endif

  deassert_rfr_int();

  if (ack_handler(priv, ACK_ERROR))
    {
      /* Reset TX consumer ring buffer entry */
      reset_txc_rb_entry(priv);
    }

  /* Ignore any received payload from previous packets */
  priv->rcvd_payload_idx = 0;
  priv->pkts_remaining = 0;

  /* Clear transfer setup flag */
  priv->xfer_setup = false;

  xfer(priv);

  sem_post(&priv->sem);
}

/*
 * Called when transaction with base has errored.
 */
static void txn_error_cb(void *v)
{
  FAR struct mods_spi_dl_s *priv = (FAR struct mods_spi_dl_s *)v;

#ifdef SIGNAL_SOFTWARE_DEASSERT
  deassert_rfr_int();
#endif

  dl_work_queue(priv, &priv->terr_work, txn_error_worker);
}

static const struct spi_cb_ops_s cb_ops =
{
#ifdef SIGNAL_SOFTWARE_DEASSERT
  .txn_start = txn_start_cb,
#endif
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
      uint16_t bitmask;
      __u8 *payload;
      int this_pl;

      if (ring_buf_is_consumers(priv->txp_rb))
        {
          dbg("Ring buffer is full!\n");
          return -ENOMEM;
        }

      payload = ((__u8 *)ring_buf_get_data(priv->txp_rb)) + HDR_SIZE;

      /* Determine the payload size of this packet */
      this_pl = MIN(remaining, pl_size);

      /* Setup bitmask for packet header */
      bitmask  = HDR_BIT_VALID;
      bitmask |= (msg_type & HDR_BIT_TYPE);
      bitmask |= (--packets & HDR_BIT_PKTS);
      if (remaining == len)
          bitmask |= HDR_BIT_PKT1;

      /* Populate the SPI message */
      set_txp_hdr(priv, bitmask);
      memcpy(payload, dbuf, this_pl);

      remaining -= this_pl;
      dbuf += this_pl;

      ring_buf_put(priv->txp_rb, priv->pkt_size);
      next_txp(priv);
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
  dl_work_queue(&mods_spi_dl, &mods_spi_dl.wake_work, wake_worker);

  return OK;
}

static int work_process_thread(int argc, char *argv[])
{
  FAR struct mods_spi_dl_s *priv = &mods_spi_dl;
  volatile struct spi_work_s *work;
  worker_t worker;
  irqstate_t flags;
  FAR void *arg;

  /* Loop forever */
  for (;;)
    {
      flags = irqsave();

      work = (struct spi_work_s *)priv->q.head;
      while (work)
        {
          /* Remove the work from the list */
          (void)dq_rem((struct dq_entry_s *)work, &priv->q);

          /* Extract the work description from the entry (in case the work
           * instance by the re-used after it has been de-queued).
           */
          worker = work->worker;

          /* Check for a race condition where the work may be nullified
           * before it is removed from the queue.
           */
          if (worker != NULL)
            {
              /* Extract the work argument (before re-enabling interrupts) */
              arg = work->arg;

              /* Mark the work as no longer being queued */
              work->worker = NULL;

              /* Re-enable interrupts while the work is being performed. */
              irqrestore(flags);

              worker(arg);

              /* Now, unfortunately, since we re-enabled interrupts we don't
               * know the state of the work list and we will have to start
               * back at the head of the list.
               */
              flags = irqsave();
              work  = (struct spi_work_s *)priv->q.head;
            }
          else
            {
              /* Cancelled... Just move to the next work in the list with
               * interrupts still disabled.
               */
              work = (struct spi_work_s *)work->dq.flink;
            }
        }

      irqrestore(flags);

      /* Wait awhile to check the work list.  We will wait here until either
       * the time elapses or until we are awakened by a signal.
       */

      usleep(CONFIG_SCHED_WORKPERIOD);
    }

  return OK;
}

FAR struct mods_dl_s *mods_dl_init(struct mods_dl_cb_s *cb)
{
  FAR struct spi_dev_s *spi;

  DEBUGASSERT(cb);

  spi = up_spiinitialize(CONFIG_GREYBUS_MODS_PORT);
  DEBUGASSERT(spi);

  SPI_SLAVE_REGISTERCALLBACK(spi, &cb_ops, &mods_spi_dl);

  mods_spi_dl.cb = cb;
  mods_spi_dl.spi = spi;
  sem_init(&mods_spi_dl.sem, 0, 0);

  mods_spi_dl.pid = kernel_thread("dl-spi", CONFIG_SCHED_WORKPRIORITY - 1,
                                  CONFIG_SCHED_WORKSTACKSIZE,
                                  work_process_thread, NULL);
  DEBUGASSERT(mods_spi_dl.pid > 0);

  set_pkt_size(&mods_spi_dl, PKT_SIZE(DEFAULT_PAYLOAD_SZ));

  /* RDY GPIO must be initialized before the WAKE interrupt */
  mods_rfr_init();

  gpio_direction_in(GPIO_MODS_WAKE_N);
  gpio_irqattach(GPIO_MODS_WAKE_N, wake_isr);
  set_gpio_triggering(GPIO_MODS_WAKE_N, IRQ_TYPE_EDGE_FALLING);

#ifdef GPIO_MODS_CC_EN
  gpio_direction_out(GPIO_MODS_CC_EN, 1);
#endif

  sem_post(&mods_spi_dl.sem);
  mods_attach_register(attach_cb, &mods_spi_dl);

#ifdef CONFIG_PM
  if (pm_register(&pm_callback) != OK)
    {
      dbg("Failed register to power management!\n");
    }
#endif

  return (FAR struct mods_dl_s *)&mods_spi_dl;
}
