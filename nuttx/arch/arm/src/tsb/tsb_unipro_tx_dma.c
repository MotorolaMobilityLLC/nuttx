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

struct unipro_xfer_descriptor {
    struct cport *cport;
    const void *data;
    size_t len;

    void *priv;
    unipro_send_completion_t callback;

    size_t data_offset;
    void *channel;

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
    void *channel[UNIPRO_DMA_CHANNEL_COUNT];
    struct list_head free_channel_list;
    sem_t dma_channel_lock;
    int max_channel;
} unipro_dma;

static void *pick_dma_channel(struct cport *cport)
{
    void *chan = unipro_dma.channel[cport->cportid % unipro_dma.max_channel];

    return chan;
}

static void unipro_dequeue_tx_desc(struct unipro_xfer_descriptor *desc, int status)
{
    irqstate_t flags;

    DEBUGASSERT(desc);

    flags = irqsave();
    list_del(&desc->list);
    irqrestore(flags);

    if (desc->callback) {
        desc->callback(status, desc->data, desc->priv);
    }

    free(desc);
}

static void unipro_flush_cport(struct cport *cport)
{
    struct unipro_xfer_descriptor *desc;

    if (list_is_empty(&cport->tx_fifo)) {
        goto reset;
    }

    desc = containerof(cport->tx_fifo.next, struct unipro_xfer_descriptor, list);

    while (!list_is_empty(&cport->tx_fifo)) {
        desc = containerof(cport->tx_fifo.next, struct unipro_xfer_descriptor, list);
        unipro_dequeue_tx_desc(desc, -ECONNRESET);
    }

reset:
    _unipro_reset_cport(cport->cportid);
    cport->pending_reset = false;
    if (cport->reset_completion_cb) {
        cport->reset_completion_cb(cport->cportid,
                                   cport->reset_completion_cb_priv);
    }
    cport->reset_completion_cb = cport->reset_completion_cb_priv = NULL;
}

static struct unipro_xfer_descriptor *pick_tx_descriptor(unsigned int cportid)
{
    struct unipro_xfer_descriptor *desc;
    unsigned int cport_count = unipro_cport_count();
    int i;

    for (i = 0; i < cport_count; i++, cportid++) {
        struct cport *cport;

        cportid = cportid % cport_count;
        cport = cport_handle(cportid);
        if (!cport)
            continue;

        if (list_is_empty(&cport->tx_fifo)) {
            if (cport->pending_reset) {
                unipro_flush_cport(cport);
            }

            continue;
        }

        if (cport->pending_reset) {
            unipro_flush_cport(cport);
        }

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

static inline void unipro_dma_tx_set_eom_flag(struct cport *cport)
{
    putreg8(1, CPORT_EOM_BIT(cport));
}

static void unipro_xfer_dequeue_descriptor(struct unipro_xfer_descriptor *desc)
{
    irqstate_t flags;

    flags = irqsave();
    list_del(&desc->list);
    irqrestore(flags);

    free(desc);
}

static int unipro_dma_tx_callback(struct device *dev, void *chan,
        struct device_dma_op *op, unsigned int event, void *arg)
{
    struct unipro_xfer_descriptor *desc = arg;

    if (event & DEVICE_DMA_CALLBACK_EVENT_COMPLETE) {
        if (desc->data_offset >= desc->len) {
            unipro_dma_tx_set_eom_flag(desc->cport);

            list_del(&desc->list);
            device_dma_op_free(unipro_dma.dev, op);

            if (desc->callback != NULL) {
                desc->callback(0, desc->data, desc->priv);
            }
            unipro_xfer_dequeue_descriptor(desc);
        } else {
            desc->channel = NULL;

            sem_post(&worker.tx_fifo_lock);
        }
    }

    return OK;
}

static int unipro_dma_xfer(struct unipro_xfer_descriptor *desc, void *channel)
{
    int retval;
    size_t xfer_len;
    void *cport_buf;
    void *xfer_buf;
    struct device_dma_op *dma_op = NULL;

    xfer_len = unipro_get_tx_free_buffer_space(desc->cport);
    if (!xfer_len)
        return -ENOSPC;

    xfer_len = MIN(desc->len - desc->data_offset, xfer_len);

    desc->channel = channel;
    retval = device_dma_op_alloc(unipro_dma.dev, 1, 0, &dma_op);
    if (retval != OK) {
        lowsyslog("unipro: failed allocate a DMA op, retval = %d.\n", retval);
        return retval;
    }

    dma_op->callback = (void *) unipro_dma_tx_callback;
    dma_op->callback_arg = desc;
    dma_op->callback_events = DEVICE_DMA_CALLBACK_EVENT_COMPLETE;
    dma_op->sg_count = 1;
    dma_op->sg[0].len = xfer_len;

    DBG_UNIPRO("xfer: chan=%u, len=%zu\n", channel->id, xfer_len);

    cport_buf = desc->cport->tx_buf;
    xfer_buf = (void*) desc->data;

    /* resuming a paused xfer */
    if (desc->data_offset != 0) {
        cport_buf = (char*) cport_buf + sizeof(uint32_t); /* skip the first DWORD */

        /* move buffer offset to the beginning of the remaning bytes to xfer */
        xfer_buf = (char*) xfer_buf + desc->data_offset;
    }

    dma_op->sg[0].src_addr = (off_t) xfer_buf;
    dma_op->sg[0].dst_addr = (off_t) cport_buf;

    desc->data_offset += xfer_len;

    retval = device_dma_enqueue(unipro_dma.dev, channel, dma_op);
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
    unsigned int next_cport;

    while (1) {
        /* Block until a buffer is pending on any CPort */
        sem_wait(&worker.tx_fifo_lock);

        next_cport = 0;
        while ((desc = pick_tx_descriptor(next_cport)) != NULL) {
            next_cport = desc->cport->cportid + 1;
            channel = pick_dma_channel(desc->cport);

            unipro_dma_xfer(desc, channel);
        }
    }

    return NULL;
}

void unipro_reset_notify(unsigned int cportid)
{
    /*
     * if the tx worker is blocked on the semaphore, post something on it
     * in order to unlock it and have the reset happen right away.
     */
    sem_post(&worker.tx_fifo_lock);
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
    desc->len = len;
    desc->data_offset = 0;
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
    int avail_chan = 0;

    sem_init(&worker.tx_fifo_lock, 0, 0);
    sem_init(&unipro_dma.dma_channel_lock, 0, 0);

    unipro_dma.dev = device_open(DEVICE_TYPE_DMA_HW, 0);
    if (!unipro_dma.dev) {
        lldbg("Failed to open DMA driver.\n");
        return -ENODEV;
    }

    unipro_dma.max_channel = 0;
    list_init(&unipro_dma.free_channel_list);
    avail_chan = device_dma_chan_free_count(unipro_dma.dev);

    if (avail_chan > ARRAY_SIZE(unipro_dma.channel)) {
        avail_chan = ARRAY_SIZE(unipro_dma.channel);
    }

    for (i = 0; i < avail_chan; i++) {
        struct device_dma_params chan_params = {
                .src_dev = DEVICE_DMA_DEV_MEM,
                .src_devid = 0,
                .src_inc_options = DEVICE_DMA_INC_AUTO,
                .dst_dev = DEVICE_DMA_DEV_MEM,
                .dst_devid = 0,
                .dst_inc_options = DEVICE_DMA_INC_AUTO,
                .transfer_size = DEVICE_DMA_TRANSFER_SIZE_64,
                .burst_len = DEVICE_DMA_BURST_LEN_16,
                .swap = DEVICE_DMA_SWAP_SIZE_NONE,
        };

        device_dma_chan_alloc(unipro_dma.dev, &chan_params,
                &unipro_dma.channel[i]);

        if (unipro_dma.channel[i] == NULL) {
            lowsyslog("unipro: couldn't allocate all %u requested channel(s)\n",
                    ARRAY_SIZE(unipro_dma.channel));
            break;
        }

        unipro_dma.max_channel++;
    }

    if (unipro_dma.max_channel <= 0) {
        lowsyslog("unipro: couldn't allocate a single DMA channel\n");
        retval = -ENODEV;
        goto error_no_channel;
    }

    lowsyslog("unipro: %d DMA channel(s) allocated\n", unipro_dma.max_channel);

    retval = pthread_create(&worker.thread, NULL, unipro_tx_worker, NULL);
    if (retval) {
        lldbg("Failed to create worker thread: %s.\n", strerror(errno));
        goto error_worker_create;
    }

    return 0;

error_worker_create:

    for (i = 0; i < unipro_dma.max_channel; i++) {
        device_dma_chan_free(unipro_dma.dev, &unipro_dma.channel[i]);
    }

    unipro_dma.max_channel = 0;

error_no_channel:
    device_close(unipro_dma.dev);
    unipro_dma.dev = NULL;

    return retval;
}
