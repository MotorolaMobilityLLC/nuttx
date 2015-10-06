/**
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
 * @author Mark Greer
 * @brief Pseudo DMA driver that uses memcpy instead of real DMA
 */

#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_dma.h>

#include "debug.h"
#include "up_arch.h"

#include "tsb_scm.h"
#include "tsb_dma.h"

static int tsb_dma_register_callback(struct device *dev, unsigned int chan,
        device_dma_callback callback)
{
    struct tsb_dma_info *info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    if (chan >= info->max_number_of_channels) {
        return -EINVAL;
    }

    sem_wait(&info->dma_channel[chan].lock);

    info->dma_channel[chan].callback = callback;

    sem_post(&info->dma_channel[chan].lock);

    return 0;
}

enum device_dma_cmd tsb_dma_transfer_done_callback(struct device *dev, unsigned int chan, enum device_dma_event event,
        device_dma_transfer_arg *arg)
{
    struct tsb_dma_info *info = device_get_private (dev);
    enum device_dma_cmd ret;

    if (info->dma_channel[chan].callback == NULL) {
        sem_post(&info->dma_channel[chan].tx_sem);
        ret = DEVICE_DMA_CMD_STOP;
    } else {
        ret= info->dma_channel[chan].callback(chan, event, arg);
    }

    return ret;
}

static int tsb_dma_transfer(struct device *dev, unsigned int chan, void *src,
        void *dst, size_t len, device_dma_transfer_arg *arg)
{
    struct tsb_dma_info *info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    if ((chan >= info->max_number_of_channels) || !src || !dst || !len) {
        return -EINVAL;
    }

    sem_wait(&info->dma_channel[chan].lock);

    /* Call the dma transfer function associated with GDMAC channel */
    if (info->dma_channel[chan].do_dma_transfer != NULL) {
        info->dma_channel[chan].do_dma_transfer(&info->dma_channel[chan], src,
                dst, len, arg);

        if (info->dma_channel[chan].callback == NULL) {
            sem_wait (&info->dma_channel[chan].tx_sem);
        }
    }

    sem_post(&info->dma_channel[chan].lock);

    return 0;
}

/* TODO!! add support for other transfer types */
static int tsb_dma_allocate_channel(struct device *dev,
        enum device_dma_channel_type type, unsigned int *chan)
{
    struct tsb_dma_info *info = device_get_private(dev);
    int ret = -ENOMEM;
    int index;
    irqstate_t flags;

    if (!info) {
        return -EIO;
    }

    if (chan == NULL) {
        return -EINVAL;
    }

    flags = irqsave();

    *chan = DEVICE_DMA_INVALID_CHANNEL;
    for (index = 0; index < info->max_number_of_channels; index++) {
        if (info->dma_channel[index].channel_info == NULL) {
            switch (type) {
            case DEVICE_DMA_UNIPRO_TX_CHANNEL:
                info->dma_channel[index].channel_id = index;
                ret = tsb_dma_allocal_unipro_tx_channel(
                        &info->dma_channel[index]);
                if (ret == 0) {
                    *chan = index;
                    ret = 0;
                }
                break;
            case DEVICE_DMA_AUDIO_INPUT_CHANNEL:
            case DEVICE_DMA_AUDIO_OUTPUT_CHANNEL:
            case DEVICE_DMA_MEMORY_TO_MEMORY_CHANNEL:
            case DEVICE_DMA_MEMORY_TO_PERIPHERAL_CHANNEL:
            case DEVICE_DMA_PERIPHERAL_TO_MEMORY_CHANNEL:
                ret = -EINVAL;
                break;
            default:
                ret = -EINVAL;
                break;
            }
            break;
        }
    }

    irqrestore(flags);

    if (ret == 0) {
        sem_init(&info->dma_channel[index].tx_sem, 0, 0);
    }

    return ret;
}

static int tsb_dma_release_channel(struct device *dev, unsigned int *chan)
{
    struct tsb_dma_info *info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    if ((chan == NULL) || (*chan >= info->max_number_of_channels)) {
        return -EINVAL;
    }

    if (info->dma_channel[*chan].release_channel != NULL) {
        info->dma_channel[*chan].release_channel(&info->dma_channel[*chan]);
    }
    info->dma_channel[*chan].channel_info = NULL;
    *chan = DEVICE_DMA_INVALID_CHANNEL;

    return 0;
}

static int tsb_dma_open(struct device *dev)
{
    struct tsb_dma_info *info;
    unsigned int chan = tsb_dma_max_number_of_channels();
    int ret, rc = 0;

    info = zalloc(sizeof(*info) + sizeof(struct tsb_dma_channel) * chan);
    if (!info) {
        return -ENOMEM;
    }

    info->max_number_of_channels = chan;
    for (chan = 0; chan < info->max_number_of_channels; chan++) {
        ret = sem_init(&info->dma_channel[chan].lock, 0, 1);
        if (ret != OK) {
            rc = -errno;
            break;
        }
    }

    if (rc) {
        for (; chan; chan--) {
            sem_destroy(&info->dma_channel[chan - 1].lock);
        }

        free(info);
        return rc;
    }

    device_set_private(dev, info);

    tsb_dma_init_controller(dev);

    return 0;
}

static void tsb_dma_close(struct device *dev)
{
    struct tsb_dma_info *info = device_get_private(dev);
    unsigned int chan;

    if (!info) {
        return;
    }

    for (chan = 0; chan < info->max_number_of_channels; chan++) {
        sem_destroy(&info->dma_channel[chan].lock);
    }

    tsb_dma_deinit_controller(dev);

    device_set_private(dev, NULL);

    free(info);
}

static struct device_dma_type_ops tsb_dma_type_ops = {
        .register_callback = tsb_dma_register_callback,
        .allocate_channel = tsb_dma_allocate_channel,
        .release_channel = tsb_dma_release_channel,
        .transfer = tsb_dma_transfer
};

static struct device_driver_ops tsb_dma_driver_ops = {
        .open = tsb_dma_open,
        .close = tsb_dma_close,
        .type_ops = &tsb_dma_type_ops
};

struct device_driver tsb_dma_driver = {
        .type = DEVICE_TYPE_DMA_HW,
        .name = "tsb_dma",
        .desc = "TSB DMA Driver",
        .ops = &tsb_dma_driver_ops
};
