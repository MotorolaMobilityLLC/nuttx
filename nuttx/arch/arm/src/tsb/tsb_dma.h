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

#ifndef __TSB_DMA_GDMAC_H
#define __TSB_DMA_GDMAC_H

#include <nuttx/device_dma.h>

struct tsb_dma_channel;
struct tsb_dma_channel_info;

/* Transfer function for GDMAC channel. */
typedef int (*dma_do_trandsfer)(struct tsb_dma_channel *channel, void *src,
        void *dst, size_t len, device_dma_transfer_arg *arg);

/* Transfer Done function for GDMAC channel */
typedef void (*dma_release_channel)(struct tsb_dma_channel *channel_info);

/* structure for GDMAC channel information. */
struct tsb_dma_channel {
    unsigned int channel_id;
    dma_do_trandsfer do_dma_transfer;
    dma_release_channel release_channel;

    sem_t lock;

    /* For async transfer. */
    sem_t tx_sem;
    struct device *dev;
    device_dma_callback callback;

    struct tsb_dma_channel_info *channel_info;
};

/* structure for GDMAC device driver's private info. */
struct tsb_dma_info {
    unsigned int max_number_of_channels;
    struct tsb_dma_channel dma_channel[0];
};

extern int tsb_dma_max_number_of_channels(void);
extern void tsb_dma_init_controller(struct device *);
extern void tsb_dma_deinit_controller(struct device *);
extern int tsb_dma_allocal_unipro_tx_channel(struct tsb_dma_channel *channel);

extern enum device_dma_cmd tsb_dma_transfer_done_callback(struct device *dev,
        unsigned int chan, enum device_dma_event event,
        device_dma_transfer_arg *arg);

#endif /* __TSB_DMA_GDMAC_H */
