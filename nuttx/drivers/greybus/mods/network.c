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

#include <arch/byteorder.h>

#include "datalink.h"

struct mods_msg_hdr
{
  __le16               cport;
} __packed;

struct mods_msg
{
  struct mods_msg_hdr  hdr;
  __u8                 gb_msg[0];
} __packed;


/* Handle to Mods data link layer */
static struct mods_dl_s *dl;

static int network_recv(FAR struct mods_dl_s *dev, const void *buf, size_t len)
{
  struct mods_msg *m = (struct mods_msg *)buf;

  return greybus_rx_handler(le16_to_cpu(m->hdr.cport), m->gb_msg,
                            (len - sizeof(m->hdr)));
}

struct mods_dl_cb_s mods_dl_cb =
{
  .recv = network_recv,
};

static void network_init(void)
{
  dl = mods_dl_init(&mods_dl_cb);
}

static int network_send(unsigned int cport, const void *buf, size_t len)
{
  struct mods_msg *m =
    (struct mods_msg *)((char *)buf - sizeof(struct mods_msg_hdr));

  m->hdr.cport = cpu_to_le16(cport);

  return MODS_DL_SEND(dl, m, len + sizeof(struct mods_msg_hdr));
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

const static struct gb_transport_backend mods_network =
{
  .headroom = (sizeof(struct mods_msg_hdr) + 3) & ~0x0003,
  .init = network_init,
  .send = network_send,
  .listen = network_listen,
  .stop_listening = network_stop_listening,
  .alloc_buf = zalloc,
  .free_buf = free,
};

int mods_network_init(void)
{
  return gb_init((struct gb_transport_backend *)&mods_network);
}
