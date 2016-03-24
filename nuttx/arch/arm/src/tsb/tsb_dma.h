/**
 * Copyright (c) 2016 Motorola Mobility, LLC
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

#include <nuttx/list.h>
#include <nuttx/device_dma.h>
#include <pthread.h>

#define TSB_DMA_SG_MAX				2

#define TSB_DMA_REG_BASE            0x40014000
#define TSB_DMA_REG_SAR0            0x00000400
#define TSB_DMA_REG_DAR0            0x00000404

struct gdmac_chan;

/* structure for GDMAC channel information. */
struct tsb_dma_chan {
    unsigned int chan_id;
    pthread_mutex_t chan_mutex;
    struct list_head queue;
    struct device_dma_params chan_params;
};

/*
 * Get the values of the source and destionation pointers for a channel.  The
 * values may or may not be valid based on the channel status.
 */
static inline void tsb_dma_get_chan_addr(void *chan, void **src, void **dst)
{
    struct tsb_dma_chan *tsb_chan = (struct tsb_dma_chan *)chan;

    if (tsb_chan != NULL)
    {
        *src = (void *)getreg32(TSB_DMA_REG_BASE+TSB_DMA_REG_SAR0+(tsb_chan->chan_id*0x20));
        *dst = (void *)getreg32(TSB_DMA_REG_BASE+TSB_DMA_REG_DAR0+(tsb_chan->chan_id*0x20));
    }
    else
    {
        *src = NULL;
        *dst = NULL;
    }
}

extern int gdmac_max_number_of_channels(void);
extern int gdmac_init_controller(struct device *);
extern void gdmac_deinit_controller(struct device *);
extern int gdmac_get_caps(struct device *dev, struct device_dma_caps *caps);
extern int gdmac_chan_alloc(struct device *dev,
        struct device_dma_params *params, struct tsb_dma_chan **tsb_chan);
extern int gdmac_chan_free(struct device *dev, struct tsb_dma_chan *tsb_chan);
extern int gdmac_start_op(struct device *dev, struct tsb_dma_chan *tsb_chan,
        struct device_dma_op *op, enum device_dma_error *error);
extern int gdmac_chan_check_op_params(struct device *dev,
        struct tsb_dma_chan *tsb_chan, struct device_dma_op *op);

extern int tsb_dma_callback(struct device *dev, struct tsb_dma_chan *tsb_chan,
        int event);
extern void gdmac_abort_chan(struct device *dev, struct tsb_dma_chan *tsb_chan);

#endif /* __TSB_DMA_GDMAC_H */
