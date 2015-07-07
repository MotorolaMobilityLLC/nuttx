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

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/types.h>
#include <nuttx/util.h>

#include "datalink.h"

#define SLICE_NUM_CPORTS         (32)

struct slice_msg
{
  __le16  size;
  __u8    dest_cport;
  __u8    src_cport;
  __u8    gb_msg[0];
};

/* Maps Slice cport numbers to base cport numbers */
static __u8 to_base_cport[SLICE_NUM_CPORTS];

/* Handle to Slice data link layer */
static struct slice_dl_s *dl;

/* Preallocated buffer for network messages */
static __u8 network_buffer[SLICE_DL_PAYLOAD_MAX_SZ];

static int network_recv(const void *buf, size_t len)
{
  struct slice_msg *m = (struct slice_msg *)buf;

  if (m->size >= len)
    {
      /* Received an invalid message */
      return -EINVAL;
    }

  if (m->dest_cport < SLICE_NUM_CPORTS)
    {
      /* Save base cport so response can be sent back correctly */
      to_base_cport[m->dest_cport] = m->src_cport;

      greybus_rx_handler(m->dest_cport, m->gb_msg, m->size);
    }

  return 0;
}

struct slice_dl_cb_s slice_dl_cb =
{
  .recv = network_recv,
};

static void network_init(void)
{
  dl = slice_dl_init(&slice_dl_cb);
}

static int network_send(unsigned int cportid, const void *buf, size_t len)
{
  struct slice_msg *m = (struct slice_msg *)network_buffer;

  if (len > (SLICE_DL_PAYLOAD_MAX_SZ - sizeof(struct slice_msg)))
      return -ENOMEM;

  m->size = len;
  m->dest_cport = to_base_cport[cportid];
  m->src_cport = cportid;
  memcpy(m->gb_msg, buf, len);

  return SLICE_DL_SEND(dl, m, len + sizeof(struct slice_msg));
}

static int network_listen(unsigned int cport)
{
  /* Nothing to do */
  return 0;
}

static int network_stop_listening(unsigned int cport)
{
  /* Nothing to do */
  return 0;
}

struct gb_transport_backend slice_network =
{
  .init = network_init,
  .send = network_send,
  .listen = network_listen,
  .stop_listening = network_stop_listening,
};

int slice_network_init(void)
{
  return gb_init(&slice_network);
}
