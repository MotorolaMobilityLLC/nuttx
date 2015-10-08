/*
 * Copyright (c) 2015 Google Inc.
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <errno.h>
#include <pthread.h>
#include <string.h>

#include <nuttx/util.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/list.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/device_dma.h>

#include "debug.h"
#include "up_arch.h"
#include "tsb_unipro.h"

#if CONFIG_ARCH_UNIPROTX_DMA_NUM_CHANNELS <= 0
#   error DMA UniPro TX must have at least one channel
#endif

#define UNIPRO_DMA_CHANNEL_COUNT CONFIG_ARCH_UNIPROTX_DMA_NUM_CHANNELS
#define containerof(x, s, f) ((void*) ((char*)(x) - offsetof(s, f)))

struct dma_channel {
    unsigned id;
    struct list_head list;
};

struct unipro_xfer_descriptor {
    struct cport *cport;
    const void *data;
    size_t len;

    void *priv;
    unipro_send_completion_t callback;

    size_t remaining_xfer_len;
    device_dma_transfer_arg dma_arg;
    struct dma_channel *channel;

    struct list_head list;
};

struct unipro_xfer_descriptor_sync {
    sem_t lock;
    int retval;
};

static struct {
    pthread_t thread;
    sem_t tx_fifo_lock;
} worker;

static struct {
    struct device *dev;
    struct dma_channel channel[UNIPRO_DMA_CHANNEL_COUNT];
    struct list_head free_channel_list;
    sem_t dma_channel_lock;
    int max_channel;
} unipro_dma;

static int unipro_continue_xfer(struct unipro_xfer_descriptor *desc)
{
    size_t xfer_len;

    desc->remaining_xfer_len -= desc->dma_arg.unipro_tx_arg.next_transfer_len;

    xfer_len = unipro_get_tx_free_buffer_space(desc->cport);
    if (!xfer_len)
        return -ENOSPC;

    xfer_len = MIN(desc->remaining_xfer_len, xfer_len);
    desc->dma_arg.unipro_tx_arg.next_transfer_len = xfer_len;

    if (xfer_len == desc->remaining_xfer_len)
        desc->dma_arg.unipro_tx_arg.eom_addr = CPORT_EOM_BIT(desc->cport);

    return 0;
}

static void release_dma_channel(struct dma_channel *channel)
{
    irqstate_t flags;

    flags = irqsave();
    list_add(&unipro_dma.free_channel_list, &channel->list);
    irqrestore(flags);

    sem_post(&unipro_dma.dma_channel_lock);
}

static void unipro_xfer_pause(struct unipro_xfer_descriptor *desc)
{
    release_dma_channel(desc->channel);
    desc->channel = NULL;
}

static void unipro_xfer_dequeue_descriptor(struct unipro_xfer_descriptor *desc)
{
    irqstate_t flags;

    flags = irqsave();
    list_del(&desc->list);
    irqrestore(flags);

    release_dma_channel(desc->channel);
    free(desc);
}

static enum device_dma_cmd unipro_dma_callback(unsigned int channel,
                                               enum device_dma_event event,
                                               device_dma_transfer_arg *arg)
{
    struct unipro_xfer_descriptor *desc =
        containerof(arg, struct unipro_xfer_descriptor, dma_arg);
    int retval;


    switch (event) {
    case DEVICE_DMA_EVENT_BLOCKED:
        retval = unipro_continue_xfer(desc);
        if (retval) {
            unipro_xfer_pause(desc);
            return DEVICE_DMA_CMD_STOP;
        }
        return DEVICE_DMA_CMD_CONTINUE;

    case DEVICE_DMA_EVENT_COMPLETED:
        if (desc->callback)
            desc->callback(0, desc->data, desc->priv);

        unipro_xfer_dequeue_descriptor(desc);
        return DEVICE_DMA_CMD_STOP;

    default:
        if (desc->callback)
            desc->callback(-EBADE, desc->data, desc->priv);

        unipro_xfer_dequeue_descriptor(desc);
        return DEVICE_DMA_CMD_STOP;
    }
}

static struct dma_channel *pick_free_dma_channel(void)
{
    struct dma_channel *channel;
    irqstate_t flags;

    sem_wait(&unipro_dma.dma_channel_lock);

    channel = containerof(unipro_dma.free_channel_list.next, struct dma_channel,
                          list);
    flags = irqsave();
    list_del(&channel->list);
    irqrestore(flags);

    return channel;
}

static struct unipro_xfer_descriptor *pick_tx_descriptor(unsigned int cport_count)
{
    struct unipro_xfer_descriptor *desc;
    int i;

    for (i = 0; i < cport_count; i++) {
        struct cport *cport = cport_handle(i);
        if (!cport)
            continue;

        if (list_is_empty(&cport->tx_fifo))
            continue;

        desc = containerof(cport->tx_fifo.next, struct unipro_xfer_descriptor,
                           list);
        if (desc->channel)
            continue;

        if (!unipro_get_tx_free_buffer_space(desc->cport))
            continue;

        return desc;
    }

    return NULL;
}

static int unipro_dma_xfer(struct unipro_xfer_descriptor *desc,
                           struct dma_channel *channel)
{
    int retval;
    size_t xfer_len;
    void *cport_buf;
    void *xfer_buf;

    xfer_len = unipro_get_tx_free_buffer_space(desc->cport);
    if (!xfer_len)
        return -ENOSPC;

    xfer_len = MIN(desc->remaining_xfer_len, xfer_len);

    desc->channel = channel;
    desc->dma_arg.unipro_tx_arg.next_transfer_len = xfer_len;
    if (xfer_len == desc->remaining_xfer_len)
        desc->dma_arg.unipro_tx_arg.eom_addr = CPORT_EOM_BIT(desc->cport);

    DBG_UNIPRO("xfer: chan=%u, len=%zu\n", channel->id, xfer_len);

    cport_buf = desc->cport->tx_buf;
    xfer_buf = (void*) desc->data;

    /* resuming a paused xfer */
    if (desc->remaining_xfer_len != desc->len) {
        cport_buf = (char*) cport_buf + 1; /* skip the start byte */

        /* move buffer offset to the beginning of the remaning bytes to xfer */
        xfer_buf = (char*) xfer_buf + (desc->len - desc->remaining_xfer_len);
    }

    retval = device_dma_transfer(unipro_dma.dev, channel->id, xfer_buf,
                                 cport_buf, xfer_len, &desc->dma_arg);
    if (retval) {
        lowsyslog("unipro: failed to start DMA transfer: %d\n", retval);
        return retval;
    }

    return 0;
}

static void *unipro_tx_worker(void *data)
{
    struct dma_channel *channel;
    struct unipro_xfer_descriptor *desc;
    unsigned int cport_count = unipro_cport_count();

    while (1) {
        /* Block until a buffer is pending on any CPort */
        sem_wait(&worker.tx_fifo_lock);

        channel = pick_free_dma_channel();
        do {
            desc = pick_tx_descriptor(cport_count);
        } while (!desc);

        unipro_dma_xfer(desc, channel);
    }

    return NULL;
}

void unipro_reset_notify(unsigned int cportid)
{
}

int unipro_send_async(unsigned int cportid, const void *buf, size_t len,
                      unipro_send_completion_t callback, void *priv)
{
    struct cport *cport;
    struct unipro_xfer_descriptor *desc;
    irqstate_t flags;

    cport = cport_handle(cportid);
    if (!cport) {
        lowsyslog("unipro: invalid cport id: %u, dropping message...\n",
                  cportid);
        return -EINVAL;
    }

    if (cport->pending_reset) {
        return -EPIPE;
    }

    desc = zalloc(sizeof(*desc));
    if (!desc)
        return -ENOMEM;

    desc->data = buf;
    desc->len = desc->remaining_xfer_len = len;
    desc->callback = callback;
    desc->priv = priv;
    desc->cport = cport;

    list_init(&desc->list);

    flags = irqsave();
    list_add(&cport->tx_fifo, &desc->list);
    irqrestore(flags);

    sem_post(&worker.tx_fifo_lock);

    return 0;
}

static int unipro_send_cb(int status, const void *buf, void *priv)
{
    struct unipro_xfer_descriptor_sync *desc = priv;
    if (!desc)
        return -EINVAL;

    desc->retval = status;
    sem_post(&desc->lock);

    return 0;
}

int unipro_send(unsigned int cportid, const void *buf, size_t len)
{
    int retval;
    struct unipro_xfer_descriptor_sync desc;

    sem_init(&desc.lock, 0, 0);

    retval = unipro_send_async(cportid, buf, len, unipro_send_cb, &desc);
    if (retval) {
        goto out;
    }

    sem_wait(&desc.lock);
    retval = desc.retval;

out:
    sem_destroy(&desc.lock);

    return retval;
}

int unipro_tx_init(void)
{
    int i;
    int retval;

    sem_init(&worker.tx_fifo_lock, 0, 0);
    sem_init(&unipro_dma.dma_channel_lock, 0, 0);

    unipro_dma.dev = device_open(DEVICE_TYPE_DMA_HW, 0);
    if (!unipro_dma.dev) {
        lldbg("Failed to open DMA driver.\n");
        return -ENODEV;
    }

    unipro_dma.max_channel = -1;
    list_init(&unipro_dma.free_channel_list);

    for (i = 0; i < ARRAY_SIZE(unipro_dma.channel); i++) {
        device_dma_allocate_channel(unipro_dma.dev,
                                    DEVICE_DMA_UNIPRO_TX_CHANNEL,
                                    &unipro_dma.channel[i].id);
        if (unipro_dma.channel[i].id == DEVICE_DMA_INVALID_CHANNEL) {
            lowsyslog("unipro: couldn't allocate all %u requested channel(s)\n",
                      ARRAY_SIZE(unipro_dma.channel));
            break;
        }

        unipro_dma.max_channel++;
        sem_post(&unipro_dma.dma_channel_lock);
        list_add(&unipro_dma.free_channel_list, &unipro_dma.channel[i].list);
        device_dma_register_callback(unipro_dma.dev, unipro_dma.channel[i].id,
                                     unipro_dma_callback);
    }

    if (unipro_dma.max_channel < 0) {
        lowsyslog("unipro: couldn't allocate a single DMA channel\n");
        retval = -ENODEV;
        goto error_no_channel;
    }

    lowsyslog("unipro: %d DMA channel(s) allocated\n",
              unipro_dma.max_channel + 1);

    retval = pthread_create(&worker.thread, NULL, unipro_tx_worker, NULL);
    if (retval) {
        lldbg("Failed to create worker thread: %s.\n", strerror(errno));
        goto error_worker_create;
    }

    return 0;

error_worker_create:
    for (i = 0; i <= unipro_dma.max_channel; i++) {
        device_dma_release_channel(unipro_dma.dev, &unipro_dma.channel[i].id);
    }

    unipro_dma.max_channel = -1;

error_no_channel:
    device_close(unipro_dma.dev);
    unipro_dma.dev = NULL;

    return retval;
}
