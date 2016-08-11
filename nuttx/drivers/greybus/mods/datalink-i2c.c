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

#include <crc16_poly8005.h>
#include <debug.h>
#include <errno.h>
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
#include <nuttx/i2c.h>
#include <nuttx/power/pm.h>
#include <nuttx/ring_buf.h>
#include <nuttx/util.h>

#include "datalink.h"

#if CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE > MODS_DL_PAYLOAD_MAX_SZ
#  error "CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE > MODS_DL_PAYLOAD_MAX_SZ"
#endif

/* Protocol version supported by this driver */
#define PROTO_VER           (1)

/*
 * The bare minimum protocol version required by the driver. To remain backwards
 * compatible with every base ever shipped, this value should never be changed
 * and new features must only be enabled once determined that the base supports
 * them.
 */
#define MIN_PROTO_VER       (1)

/* Default payload size of a I2C packet (in bytes) */
#define DEFAULT_PAYLOAD_SZ (32)

/* Initial number of entries for the ring buffer (will grow as needed). */
#define INITIAL_RB_ENTRIES (10)

/* I2C packet header bit definitions */
#define HDR_BIT_ACK    (0x01 << 10) /* 1 = Base has ACK'd last packet sent */
#define HDR_BIT_DUMMY  (0x01 <<  9) /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 <<  8) /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 <<  7) /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 <<  6) /* I2C message type */
#define HDR_BIT_PKTS   (0x3F <<  0) /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* Priority to report to PM framework when WAKE line asserted. */
#define PM_ACTIVITY_WAKE 10

/* The number of times to try sending a datagram before giving up */
#define NUM_TRIES      (3)

/*
 * The number of error prints before sleeping the prints. A success will reset
 * the count.
 */
#define ERR_CNT_ZZZ    (5)

/* Size of the header in bytes */
#define HDR_SIZE       sizeof(struct i2c_msg_hdr)

/* Size of the CRC in bytes */
#define CRC_SIZE       (2)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + HDR_SIZE + CRC_SIZE)

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

/* All the possible states in the state machine */
enum dl_state_e
{
  DL_STATE_IDLE,                 /* 0 */
  DL_STATE_RX,                   /* 1 */
  DL_STATE_TX,                   /* 2 */
  DL_STATE_ACK_RX,               /* 3 */
  DL_STATE_ACK_TX,               /* 4 */
  DL_STATE_DUMMY,                /* 5 */

  /* Add new states above here */
  DL_STATE__NUM_STATES,
};

struct i2c_msg_hdr
{
  __le16 bitmask;                /* See HDR_BIT_* defines for values */
} __packed;

struct mods_i2c_dl_s
{
  struct mods_dl_s dl;           /* Externally visible part of the data link interface */
  FAR struct i2c_dev_s *i2c;     /* I2C handle */

  struct mods_dl_cb_s *cb;       /* Callbacks to network layer */
  enum base_attached_e bstate;   /* Base state (attached/detached) */
  size_t pl_size;                /* Size of packet payload in bytes */
  size_t new_pl_size;            /* Payload size to use next time TX queue empty */
  __u8 proto_ver;                /* Protocol version supported by base */

  enum dl_state_e txn_state;     /* Current transaction state */
  uint8_t tx_tries_remaining;    /* Send attempts remaining before giving up */

  struct ring_buf *txp_rb;       /* Producer TX ring buffer */
  struct ring_buf *txc_rb;       /* Consumer TX ring buffer */

  __u8 *rx_buf;                  /* Buffer for received packets */
  uint32_t stop_err_cnt;         /* Count of stop errors seen since last success */
  uint32_t crc_err_cnt;          /* Count of CRC errors seen since last success */

  /*
   * Buffer to hold incoming payload (which could be spread across
   * multiple packets)
   */
  __u8 rcvd_payload[MODS_DL_PAYLOAD_MAX_SZ];
  uint32_t rcvd_payload_idx;
  uint8_t pkts_remaining;        /* Number of packets needed to complete msg */
};

struct i2c_dl_msg_bus_config_req
{
  __le16 max_pl_size;            /* Maximum payload size supported by base */
  __u8   features;               /* See DL_BIT_* defines for values */
  __u8   version;                /* I2C msg format version of base */
} __packed;

struct i2c_dl_msg_bus_config_resp
{
  __le32 max_speed;              /* Maximum bus speed supported by the mod */
  __le16 pl_size;                /* Payload size that mod has selected to use */
  __u8   features;               /* See DL_BIT_* defines for values */
  __u8   version;                /* I2C msg format version supported by mod */
} __packed;

struct i2c_dl_msg
{
  __u8 id;                       /* enum dl_msg_id */
  union
    {
      struct i2c_dl_msg_bus_config_req     bus_req;
      struct i2c_dl_msg_bus_config_resp    bus_resp;
    };
} __packed;

struct state_funcs
{
  int (*start_tx)(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer);
  int (*start_rx)(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer);

  int (*stop_success)(FAR struct mods_i2c_dl_s *priv);
  int (*stop_error)(FAR struct mods_i2c_dl_s *priv);
};

static void set_pl_size(FAR struct mods_i2c_dl_s *priv, size_t pl_size)
{
  /*
   * Immediately return if payload size is not changing and ring buffer is
   * in a good state.
   */
  if ((pl_size == priv->pl_size) && (priv->txp_rb == priv->txc_rb) &&
      ring_buf_is_producers(priv->txp_rb))
      return;

  /* Free any existing RX buffer (if any) */
  if (priv->rx_buf)
      free(priv->rx_buf);

  /* Free existing TX ring buffer (if any) */
  ring_buf_free_ring(priv->txp_rb, NULL /* free_callback */, NULL /* arg */);

  /* Allocate RX buffer */
  priv->rx_buf = malloc(PKT_SIZE(pl_size));
  ASSERT(priv->rx_buf);

  /* Allocate TX ring buffer */
  priv->txp_rb = ring_buf_alloc_ring(INITIAL_RB_ENTRIES,
      HDR_SIZE /* headroom */, pl_size /* data_len */,
      CRC_SIZE /* tailroom */, NULL /* alloc_callback */,
      NULL /* free_callback */, NULL /* arg */);
  ASSERT(priv->txp_rb);
  priv->txc_rb = priv->txp_rb;

  /* Save new packet size */
  priv->pl_size = pl_size;

  lldbg("%d bytes\n", priv->pl_size);
}

static struct ring_buf *find_prev_rb_entry(struct ring_buf *next_rb)
{
  struct ring_buf *rb;

  rb = next_rb;
  while(rb->next != next_rb)
      rb = rb->next;

  return rb;
}

static int add_rb_entry(FAR struct mods_i2c_dl_s *priv)
{
  struct ring_buf *rb;
  struct ring_buf *prev_rb;

  rb = ring_buf_alloc(HDR_SIZE /* headroom */, priv->pl_size /* data_len */,
                      CRC_SIZE /* tailroom */);
  if (!rb)
      return -ENOMEM;

  prev_rb = find_prev_rb_entry(priv->txp_rb);

  /* Insert new ring buffer entry */
  rb->next = priv->txp_rb;
  prev_rb->next = rb;

  /* Set producer pointer to newly added entry */
  priv->txp_rb = rb;

  llvdbg("RB entry added\n");

  return OK;
}

static inline void set_txp_hdr(FAR struct mods_i2c_dl_s *priv, uint16_t bits)
{
  struct i2c_msg_hdr *hdr = ring_buf_get_buf(priv->txp_rb);
  hdr->bitmask = cpu_to_le16(bits);
}

static inline void next_txp(FAR struct mods_i2c_dl_s *priv)
{
  ring_buf_pass(priv->txp_rb);
  priv->txp_rb = ring_buf_get_next(priv->txp_rb);
}

static inline void setup_for_dummy_tx(FAR struct mods_i2c_dl_s *priv)
{
  set_txp_hdr(priv, HDR_BIT_DUMMY);
  next_txp(priv);
}

static inline void set_ack_txc_hdr(FAR struct mods_i2c_dl_s *priv)
{
  struct i2c_msg_hdr *hdr = ring_buf_get_buf(priv->txc_rb);
  hdr->bitmask = cpu_to_le16(le16_to_cpu(hdr->bitmask) | HDR_BIT_ACK);
}

static inline void set_int_if_needed(FAR struct mods_i2c_dl_s *priv)
{
  /* Set the base interrupt line if data is available to be sent. */
  mods_host_int_set(ring_buf_is_consumers(priv->txc_rb));
}

static void set_crc(FAR struct mods_i2c_dl_s *priv)
{
  uint16_t crc = crc16_poly8005(ring_buf_get_buf(priv->txc_rb),
                                priv->pl_size + HDR_SIZE, CRC_INIT_VAL);
  *((uint16_t *)priv->txc_rb->tailroom) = cpu_to_le16(crc);
}

static void reset_txc_rb_entry(FAR struct mods_i2c_dl_s *priv)
{
  if ((priv->txp_rb == priv->txc_rb) && ring_buf_is_producers(priv->txp_rb))
    {
      lldbg("skip\n");
      return;
    }

  memset(ring_buf_get_buf(priv->txc_rb), 0, PKT_SIZE(priv->pl_size));
  ring_buf_set_owner(priv->txc_rb, RING_BUF_OWNER_PRODUCER);
  priv->txc_rb = ring_buf_get_next(priv->txc_rb);
}

#ifdef CONFIG_DEBUG_VERBOSE
static const char* state_name(enum dl_state_e state)
{
  switch (state)
    {
      case DL_STATE_IDLE:   return "IDLE";
      case DL_STATE_RX:     return "RX";
      case DL_STATE_TX:     return "TX";
      case DL_STATE_ACK_RX: return "ACK_RX";
      case DL_STATE_ACK_TX: return "ACK_TX";
      case DL_STATE_DUMMY:  return "DUMMY";
      default:              return "???";
    }
}
#endif

static void set_state(FAR struct mods_i2c_dl_s *priv, enum dl_state_e state)
{
  if (priv->txn_state != state)
    {
      llvdbg("%s -> %s\n", state_name(priv->txn_state), state_name(state));
      priv->txn_state = state;
    }
}

static int attach_cb(FAR void *arg, const void *data)
{
  FAR struct mods_i2c_dl_s *priv = (FAR struct mods_i2c_dl_s *)arg;
  enum base_attached_e state = *((enum base_attached_e *)data);
  irqstate_t flags;

  flags = irqsave();

  if (state != BASE_ATTACHED)
    {
      vdbg("Cleaning up datalink\n");

      /* Reset GPIOs to initial state */
      mods_rfr_set(0);
      mods_host_int_set(false);
#ifdef GPIO_MODS_I2C_PU_EN
      gpio_direction_in(GPIO_MODS_I2C_PU_EN);
#endif

      /* Cancel I2C transaction */
      I2C_CANCEL(priv->i2c);

      /* Cleanup any unsent messages */
      while (ring_buf_is_consumers(priv->txc_rb))
        {
          reset_txc_rb_entry(priv);
        }

      /* Return packet size back to default */
      set_pl_size(priv, DEFAULT_PAYLOAD_SZ);

      /* Assume next base only supports the minimum protocol version */
      priv->proto_ver = MIN_PROTO_VER;

      /* Reset state machine */
      set_state(priv, DL_STATE_IDLE);
      priv->tx_tries_remaining = NUM_TRIES;
      priv->rcvd_payload_idx = 0;
      priv->pkts_remaining = 0;
      priv->stop_err_cnt = 0;
      priv->crc_err_cnt = 0;
    }
#ifdef GPIO_MODS_I2C_PU_EN
  else
    {
      gpio_direction_out(GPIO_MODS_I2C_PU_EN, 1);
    }
#endif

  priv->bstate = state;

  irqrestore(flags);

  return OK;
}

static int queue_data(FAR struct mods_i2c_dl_s *priv, __u8 msg_type,
                      const void *buf, size_t len)
{
  int remaining = len;
  __u8 *dbuf = (__u8 *)buf;
  int packets;

  /* Calculate how many packets are required to send whole payload */
  packets = (remaining + priv->pl_size - 1) / priv->pl_size;

  llvdbg("len=%d, packets=%d, bstate=%d\n", len, packets, priv->bstate);

  if (priv->bstate != BASE_ATTACHED)
      return -ENODEV;

  if (len > MODS_DL_PAYLOAD_MAX_SZ)
      return -E2BIG;

  while ((remaining > 0) && (packets > 0))
    {
      uint16_t bitmask;
      int this_pl;

      /* Check if the ring buffer is full */
      if (ring_buf_is_consumers(priv->txp_rb))
        {
          /* Try to grow the ring buffer */
          ASSERT(add_rb_entry(priv) == OK);
        }

      /* Determine the payload size of this packet */
      this_pl = MIN(remaining, priv->pl_size);

      /* Setup bitmask for packet header */
      bitmask  = HDR_BIT_VALID;
      bitmask |= (msg_type & HDR_BIT_TYPE);
      bitmask |= (--packets & HDR_BIT_PKTS);
      if (remaining == len)
          bitmask |= HDR_BIT_PKT1;

      /* Populate the I2C message */
      set_txp_hdr(priv, bitmask);
      memcpy(ring_buf_get_data(priv->txp_rb), dbuf, this_pl);

      remaining -= this_pl;
      dbuf += this_pl;

      next_txp(priv);
    }

  set_int_if_needed(priv);

  return OK;
}

static int dl_recv(FAR struct mods_dl_s *dl, FAR const void *buf, size_t len)
{
  FAR struct mods_i2c_dl_s *priv = (FAR struct mods_i2c_dl_s *)dl;
  struct i2c_dl_msg req;
  struct i2c_dl_msg resp;
  uint16_t pl_size;
  int ret;

  /*
   * To support bases running firmware with fewer values in the
   * request, copy the message into a zero'd local struct.
   */
  memset(&req, 0, sizeof(req));
  memcpy(&req, buf, MIN(len, sizeof(req)));

  /* Only BUS_CFG_REQ is supported */
  if (req.id != DL_MSG_ID_BUS_CFG_REQ)
    {
      lldbg("Unknown ID (%d)!\n", req.id);
      return -EINVAL;
    }

  if (req.bus_req.version < MIN_PROTO_VER)
    {
      lldbg("Unsupported protocol version (%d)\n", req.bus_req.version);
      return -EPROTONOSUPPORT;
    }

  priv->proto_ver = req.bus_req.version;

  resp.id = DL_MSG_ID_BUS_CFG_RESP;
  resp.bus_resp.max_speed = cpu_to_le32(CONFIG_GREYBUS_MODS_MAX_BUS_SPEED);

  pl_size = MIN(CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE,
                le16_to_cpu(req.bus_req.max_pl_size));
  /*
   * Verify new payload size is valid. The payload must be no smaller than
   * the default payload size and must be a power of two.
   */
  if (IS_PWR_OF_TWO(pl_size) && (pl_size >= DEFAULT_PAYLOAD_SZ))
    {
      /*
       * The new payload size cannot be used immediately since the reply
       * must use the current payload size. Save off for now to be set later.
       */
      priv->new_pl_size = pl_size;
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

  ret = queue_data(priv, MSG_TYPE_DL, &resp, sizeof(resp));
  if (ret)
    {
      /* Abort packet size change due to the send error */
      priv->new_pl_size = 0;
    }

  return ret;
}

static int idle_start_tx(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  if (ring_buf_is_consumers(priv->txc_rb))
    {
      mods_host_int_set(false);
      set_state(priv, DL_STATE_TX);
    }
  else /* Nothing to send! */
    {
      setup_for_dummy_tx(priv);
      set_state(priv, DL_STATE_DUMMY);
    }

  *buffer = ring_buf_get_buf(priv->txc_rb);
  set_crc(priv);

  return OK;
}

static int idle_start_rx(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  *buffer = priv->rx_buf;
  set_state(priv, DL_STATE_RX);

  return OK;
}

static int idle_stop(FAR struct mods_i2c_dl_s *priv)
{
  /* This should never happen */
  lldbg("???\n");

  return OK;
}

static int no_start(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  /* This is unexpected, so return to IDLE and start state machine again */
  lldbg("???\n");
  set_state(priv, DL_STATE_IDLE);

  return -EAGAIN;
}

static int rx_stop_error(FAR struct mods_i2c_dl_s *priv)
{
  /* Ignore any received payload from previous packets */
  priv->rcvd_payload_idx = 0;
  priv->pkts_remaining = 0;

  set_state(priv, DL_STATE_IDLE);

  return OK;
}

static int rx_stop_success(FAR struct mods_i2c_dl_s *priv)
{
  /* First verify valid CRC */
  uint16_t calc_crc = crc16_poly8005(priv->rx_buf, priv->pl_size + HDR_SIZE,
                                     CRC_INIT_VAL);
  uint16_t rcvd_crc = *((uint16_t *)&(priv->rx_buf[priv->pl_size + HDR_SIZE]));
  rcvd_crc = le16_to_cpu(rcvd_crc);
  if (calc_crc != rcvd_crc)
    {
      if (priv->crc_err_cnt++ < ERR_CNT_ZZZ)
        {
          lldbg("CRC mismatch: 0x%04x != 0x%04x%s\n", calc_crc, rcvd_crc,
                priv->crc_err_cnt >= ERR_CNT_ZZZ ? " (zzz)" : "");
        }
      return rx_stop_error(priv);
    }
  priv->crc_err_cnt = 0;

  struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->rx_buf;
  uint16_t bitmask = le16_to_cpu(hdr->bitmask);
  enum dl_state_e next_state = DL_STATE_ACK_TX;

  llvdbg("bitmask=0x%04X\n", bitmask);

  if (!(bitmask & HDR_BIT_VALID))
    {
      /* Received a dummy or garbage packet - no processing to do! */

      if (!(bitmask & HDR_BIT_DUMMY))
          lldbg("garbage packet\n");

      /* Change packet size if needed */
      if (priv->new_pl_size > 0)
        {
          DEBUGASSERT(priv->txp_rb == priv->txc_rb);
          set_pl_size(priv, priv->new_pl_size);
          priv->new_pl_size = 0;
        }

      next_state = DL_STATE_IDLE;
      goto done;
    }

  buf_t recv = priv->cb->recv;

  if ((bitmask & HDR_BIT_TYPE) == MSG_TYPE_DL)
      recv = dl_recv;

#if CONFIG_GREYBUS_MODS_DESIRED_PKT_SIZE == MODS_DL_PAYLOAD_MAX_SZ
  /* Check if un-packetizing is not required */
  if (MODS_DL_PAYLOAD_MAX_SZ == priv->pl_size)
    {
      if (bitmask & HDR_BIT_PKT1)
          recv(&priv->dl, &priv->rx_buf[HDR_SIZE], priv->pl_size);
      else
          lldbg("1st pkt bit not set\n");
      goto done;
    }
#endif

  if (bitmask & HDR_BIT_PKT1)
    {
      /* Check if data exists from earlier packets */
      if (priv->rcvd_payload_idx)
        {
          lldbg("1st pkt recv'd before prev msg complete\n");
          priv->rcvd_payload_idx = 0;
        }

      priv->pkts_remaining = bitmask & HDR_BIT_PKTS;
    }
  else /* not first packet of message */
    {
      /* Check for data from earlier packets */
      if (!priv->rcvd_payload_idx)
        {
          lldbg("Ignore non-first packet\n");
          goto done;
        }

      if ((bitmask & HDR_BIT_PKTS) != --priv->pkts_remaining)
        {
          lldbg("Packets remaining out of sync\n");

          /* Drop the entire message */
          priv->rcvd_payload_idx = 0;
          priv->pkts_remaining = 0;
          goto done;
        }
    }

  if (priv->rcvd_payload_idx >= MODS_DL_PAYLOAD_MAX_SZ)
    {
      lldbg("Too many packets received\n");

      /* Drop the entire message */
      priv->rcvd_payload_idx = 0;
      priv->pkts_remaining = 0;
      goto done;
    }

  memcpy(&priv->rcvd_payload[priv->rcvd_payload_idx],
         &priv->rx_buf[HDR_SIZE], priv->pl_size);
  priv->rcvd_payload_idx += priv->pl_size;

  if (bitmask & HDR_BIT_PKTS)
    {
      /* Need additional packets */
      goto done;
    }

  recv(&priv->dl, priv->rcvd_payload, priv->rcvd_payload_idx);
  priv->rcvd_payload_idx = 0;

done:
  set_int_if_needed(priv);
  set_state(priv, next_state);

  return OK;
}

static int tx_stop_success(FAR struct mods_i2c_dl_s *priv)
{
  set_state(priv, DL_STATE_ACK_RX);

  return OK;
}

static int tx_stop_error(FAR struct mods_i2c_dl_s *priv)
{
  if (--priv->tx_tries_remaining <= 0)
    {
      lldbg("abort\n");

      reset_txc_rb_entry(priv);
      priv->tx_tries_remaining = NUM_TRIES;
    }
  else
      lldbg("retry\n");

  set_state(priv, DL_STATE_IDLE);
  set_int_if_needed(priv);

  return OK;
}

static int ack_rx_start_tx(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  /* This is unexpected, so return to IDLE and start state machine again */
  set_state(priv, DL_STATE_IDLE);

  /* Assume the base was happy with last transmission */
  reset_txc_rb_entry(priv);
  priv->tx_tries_remaining = NUM_TRIES;

  set_int_if_needed(priv);

  return -EAGAIN;
}

static int ack_rx_start_rx(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  *buffer = priv->rx_buf;

  return OK;
}

static int ack_rx_stop_success(FAR struct mods_i2c_dl_s *priv)
{
  struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->rx_buf;
  uint16_t bitmask = le16_to_cpu(hdr->bitmask);

  if ((bitmask & HDR_BIT_ACK) || --priv->tx_tries_remaining <= 0)
    {
      if (priv->tx_tries_remaining <= 0)
          lldbg("abort\n");

      reset_txc_rb_entry(priv);
      priv->tx_tries_remaining = NUM_TRIES;
    }
  else
      lldbg("retry\n");

  /* It is possible the base has sent a valid packet that needs parsing */
  set_state(priv, DL_STATE_RX);
  return -EAGAIN;
}

static int ack_tx_start_tx(FAR struct mods_i2c_dl_s *priv, uint8_t **buffer)
{
  if (ring_buf_is_consumers(priv->txc_rb))
    {
      mods_host_int_set(false);
    }
  else /* Nothing to send! */
    {
      setup_for_dummy_tx(priv);
    }

  set_ack_txc_hdr(priv);

  *buffer = ring_buf_get_buf(priv->txc_rb);
  set_crc(priv);

  return OK;
}

static int ack_tx_stop_success(FAR struct mods_i2c_dl_s *priv)
{
  struct i2c_msg_hdr *hdr = ring_buf_get_buf(priv->txc_rb);
  uint16_t bitmask = le16_to_cpu(hdr->bitmask);

  /* It is possible we sent a valid packet with ACK */
  if (bitmask & HDR_BIT_VALID)
    {
      set_state(priv, DL_STATE_TX);
      return -EAGAIN;
    }

  reset_txc_rb_entry(priv);
  set_state(priv, DL_STATE_IDLE);

  return OK;
}

static int ack_tx_stop_error(FAR struct mods_i2c_dl_s *priv)
{
  struct i2c_msg_hdr *hdr = ring_buf_get_buf(priv->txc_rb);
  uint16_t bitmask = le16_to_cpu(hdr->bitmask);

  if (!(bitmask & HDR_BIT_VALID) || --priv->tx_tries_remaining <= 0)
    {
      if (priv->tx_tries_remaining <= 0)
          lldbg("abort\n");

      reset_txc_rb_entry(priv);
      priv->tx_tries_remaining = NUM_TRIES;
    }
  else
      lldbg("retry\n");

  set_state(priv, DL_STATE_IDLE);
  set_int_if_needed(priv);

  return OK;
}

static int dummy_stop(FAR struct mods_i2c_dl_s *priv)
{
  reset_txc_rb_entry(priv);
  set_state(priv, DL_STATE_IDLE);

  return OK;
}

static const struct state_funcs state_funcs_tbl[DL_STATE__NUM_STATES] =
{
  /* DL_STATE_IDLE */
  { idle_start_tx,   idle_start_rx,   idle_stop,           idle_stop         },

  /* DL_STATE_RX */
  { no_start,        no_start,        rx_stop_success,     rx_stop_error     },

  /* DL_STATE_TX */
  { no_start,        no_start,        tx_stop_success,     tx_stop_error     },

  /* DL_STATE_ACK_RX */
  { ack_rx_start_tx, ack_rx_start_rx, ack_rx_stop_success, tx_stop_error     },

  /* DL_STATE_ACK_TX */
  { ack_tx_start_tx, no_start,        ack_tx_stop_success, ack_tx_stop_error },

  /* DL_STATE_DUMMY */
  { no_start,        no_start,        dummy_stop,          dummy_stop        },
};

/*
 * Called when transaction with base has started.
 */
static int txn_start_cb(void *v, uint8_t dir, uint8_t **buffer, int *buflen)
{
  FAR struct mods_i2c_dl_s *priv = (FAR struct mods_i2c_dl_s *)v;
  irqstate_t flags;
  int ret;

  flags = irqsave();

  do
    {
      /* Master read is slave write */
      if (dir == I2C_READBIT)
        {
          ret = state_funcs_tbl[priv->txn_state].start_tx(priv, buffer);
        }
      else
        {
          ret = state_funcs_tbl[priv->txn_state].start_rx(priv, buffer);
        }
    }
  while (ret == -EAGAIN);

  *buflen = PKT_SIZE(priv->pl_size);

  irqrestore(flags);

  return ret;
}

/*
 * Called when transaction with base has completed.
 */
static void txn_stop_cb(void *v, int status, int xfered)
{
  FAR struct mods_i2c_dl_s *priv = (FAR struct mods_i2c_dl_s *)v;
  irqstate_t flags;
  int ret;

  flags = irqsave();

  do
    {
      if ((status == OK) && (xfered >= PKT_SIZE(priv->pl_size)))
        {
          ret = state_funcs_tbl[priv->txn_state].stop_success(priv);
          priv->stop_err_cnt = 0;
        }
      else
        {
          if (priv->stop_err_cnt++ < ERR_CNT_ZZZ)
            {
              lldbg("status=%d, xfered=%d%s\n", status, xfered,
                    priv->stop_err_cnt >= ERR_CNT_ZZZ ? " (zzz)" : "");
            }
          ret = state_funcs_tbl[priv->txn_state].stop_error(priv);
        }
    }
  while (ret == -EAGAIN);

  irqrestore(flags);
}

static const struct i2c_cb_ops_s cb_ops =
{
  .start = txn_start_cb,
  .stop = txn_stop_cb,
};

/* Called by network layer when there is data to be sent to base */
static int queue_data_nw(FAR struct mods_dl_s *dl, const void *buf, size_t len)
{
  FAR struct mods_i2c_dl_s *priv = (FAR struct mods_i2c_dl_s *)dl;
  int ret;
  irqstate_t flags;

  flags = irqsave();
  ret = queue_data(priv, MSG_TYPE_NW, buf, len);
  irqrestore(flags);

  return ret;
}

static struct mods_dl_ops_s mods_dl_ops =
{
  .send = queue_data_nw,
};

static struct mods_i2c_dl_s mods_i2c_dl =
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
  /* Any wake interrupts when not attached are spurious */
  if (mods_i2c_dl.bstate != BASE_ATTACHED)
    {
      llvdbg("ignored\n");
      return OK;
    }

  uint8_t wake_n = gpio_get_value(GPIO_MODS_WAKE_N);
  llvdbg("wake_n = %d\n", wake_n);

  if (!wake_n)
      pm_activity(PM_ACTIVITY_WAKE);

  mods_rfr_set(!wake_n);

  return OK;
}

FAR struct mods_dl_s *mods_dl_init(struct mods_dl_cb_s *cb)
{
  FAR struct i2c_dev_s *i2c;

  DEBUGASSERT(cb);

  i2c = up_i2cinitialize(CONFIG_GREYBUS_MODS_PORT);
  DEBUGASSERT(i2c);

  I2C_SETFREQUENCY(i2c, CONFIG_GREYBUS_MODS_MAX_BUS_SPEED);
  I2C_SETOWNADDRESS(i2c, CONFIG_GREYBUS_MODS_I2C_ADDR, 7);
  I2C_REGISTERCALLBACK(i2c, &cb_ops, &mods_i2c_dl);

  mods_i2c_dl.cb = cb;
  mods_i2c_dl.i2c = i2c;

  set_pl_size(&mods_i2c_dl, DEFAULT_PAYLOAD_SZ);

  /* RDY GPIO must be initialized before the WAKE interrupt */
  mods_rfr_init();

  gpio_direction_in(GPIO_MODS_WAKE_N);
  gpio_irqattach(GPIO_MODS_WAKE_N, wake_isr);
  set_gpio_triggering(GPIO_MODS_WAKE_N, IRQ_TYPE_EDGE_BOTH);

  mods_attach_register(attach_cb, &mods_i2c_dl);

#ifdef CONFIG_PM
  if (pm_register(&pm_callback) != OK)
    {
      dbg("Failed register to power management!\n");
    }
#endif

  return (FAR struct mods_dl_s *)&mods_i2c_dl;
}
