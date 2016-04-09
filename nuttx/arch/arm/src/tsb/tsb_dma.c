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

#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_dma.h>
#include <nuttx/list.h>

#include "debug.h"
#include "up_arch.h"

#include "tsb_scm.h"
#include "tsb_dma.h"

enum tsb_dma_op_state {
    TSB_DMA_OP_STATE_IDLE,
    TSB_DMA_OP_STATE_QUEUED,
    TSB_DMA_OP_STATE_STARTING,
    TSB_DMA_OP_STATE_RUNNING,
    TSB_DMA_OP_STATE_COMPLETING,
    TSB_DMA_OP_STATE_COMPLETED,
    TSB_DMA_OP_STATE_ERROR,
    TSB_DMA_OP_STATE_UNDEFINED
};

struct tsb_dma_op {
    struct list_head list_node;
    int chan_id;
    enum tsb_dma_op_state state;
    enum device_dma_error error;
    unsigned int events;
    struct device_dma_op op;
};

/* structure for GDMAC device driver's private info. */
struct tsb_dma_info {
    pthread_mutex_t lock;
    unsigned int max_chan;
    unsigned int avail_chan;

    struct list_head completed_queue;
    sem_t op_completed_sem;
    pthread_t irq_thread;
    bool driver_closed;

    struct tsb_dma_chan *chans[0];
};

static int tsb_dma_restart_chan(struct device *dev,
        struct tsb_dma_chan *dma_chan)
{
    struct list_head *next_op = NULL;
    irqstate_t flags;
    int retval = OK;

    pthread_mutex_lock(&dma_chan->chan_mutex);

    flags = irqsave();

    list_foreach(&dma_chan->queue, next_op)
    {
        struct tsb_dma_op
        *dma_op = list_entry(next_op, struct tsb_dma_op, list_node);

        if (dma_op->state == TSB_DMA_OP_STATE_QUEUED) {
            dma_op->state = TSB_DMA_OP_STATE_STARTING;

            irqrestore(flags);

            retval = gdmac_start_op(dev, dma_chan, &dma_op->op, &dma_op->error);

            flags = irqsave();

            if (retval == OK) {
                /* there is a chance the op already completed when we get to
                 * here. If that is the case we don't need to set the op state
                 * to running because it might already set to completed state.
                 */
                if (dma_op->state == TSB_DMA_OP_STATE_STARTING) {
                    dma_op->state = TSB_DMA_OP_STATE_RUNNING;
                }
            } else {
                lldbg("failed to start op.\n");
                dma_op->error = DEVICE_DMA_ERROR_DMA_FAILED;
                dma_op->state = TSB_DMA_OP_STATE_ERROR;
            }
        }
        break;
    }

    irqrestore(flags);

    pthread_mutex_unlock(&dma_chan->chan_mutex);

    return retval;
}

static void *tsb_dma_process_completed_op(void *arg)
{
    struct device *dev = arg;
    struct tsb_dma_info *dma_info = device_get_private(dev);

    while (dma_info->driver_closed == 0) {
        struct list_head *node, *next_node;

        sem_wait(&dma_info->op_completed_sem);

        list_foreach_safe(&dma_info->completed_queue, node, next_node) {
            struct tsb_dma_op* dma_op;
            int chan_id;

            list_del(node);
            dma_op = list_entry(node, struct tsb_dma_op, list_node);
            chan_id = dma_op->chan_id;

            if (dma_op->op.callback == NULL) {
                lldbg("Invalid callback\n");
            } else {
                uint32_t callback_events = dma_op->op.callback_events;

                /*
                 * The dma_op->state '=' below is required for operation.  It was
                 * not cleaned up, to keep the code common with ara.
                 */
                if ((callback_events & DEVICE_DMA_CALLBACK_EVENT_COMPLETE) &&
                    (dma_op->state = TSB_DMA_OP_STATE_COMPLETED)) {
                    dma_op->op.callback(dev, &dma_info->chans[chan_id],
                            (void *) &dma_op->op,
                            DEVICE_DMA_CALLBACK_EVENT_COMPLETE,
                            dma_op->op.callback_arg);
                }

                if ((callback_events & DEVICE_DMA_CALLBACK_EVENT_ERROR) &&
                    (dma_op->state = TSB_DMA_OP_STATE_ERROR)) {
                    dma_op->op.callback(dev, &dma_info->chans[chan_id],
                            (void *) &dma_op->op,
                            DEVICE_DMA_CALLBACK_EVENT_ERROR,
                            dma_op->op.callback_arg);
                }
            }

            tsb_dma_restart_chan(dev, dma_info->chans[chan_id]);
        }
    }

    return NULL;
}

static int tsb_dma_open(struct device *dev)
{
    struct tsb_dma_info *info;
    unsigned int chan;
    int retval = 0;

    retval = gdmac_init_controller(dev);
    if (retval != OK) {
        return retval;
    }

    chan = gdmac_max_number_of_channels();

    info = zalloc(
            sizeof(struct tsb_dma_info) + sizeof(struct tsb_dma_chan *) * chan);
    if (!info) {
        return -ENOMEM;
    }

    info->driver_closed = false;

    pthread_mutex_init(&info->lock, NULL);
    sem_init(&info->op_completed_sem, 0, 0);

    info->max_chan = chan;
    info->avail_chan = chan;

    list_init(&info->completed_queue);

    device_set_private(dev, info);

    retval = pthread_create(&info->irq_thread, NULL,
            tsb_dma_process_completed_op, dev);
    if (retval) {
        gdmac_deinit_controller(dev);

        device_set_private(dev, NULL);
        free(info);

        retval = -retval;
    } else {
        pthread_mutex_unlock(&info->lock);
    }

    return retval;
}

static void tsb_dma_close(struct device *dev)
{
    struct tsb_dma_info *info = device_get_private(dev);
    unsigned int chan;

    if (!info) {
        return;
    }

    info->driver_closed = true;
    sem_post(&info->op_completed_sem);

    pthread_join(info->irq_thread, NULL);

    for (chan = 0; chan < info->max_chan; chan++) {
        if (info->chans[chan] == NULL) {
            device_dma_chan_free(dev, info->chans[chan]);
        }
    }

    gdmac_deinit_controller(dev);

    pthread_mutex_destroy(&info->lock);

    device_set_private(dev, NULL);

    free(info);
}

static int tsb_dma_get_caps(struct device *dev, struct device_dma_caps *caps)
{
    if ((caps == NULL) || (dev == NULL)) {
        return -EINVAL;
    }

    return gdmac_get_caps(dev, caps);
}

static int tsb_dma_chan_free_count(struct device *dev)
{
    struct tsb_dma_info *info;

    if (dev == NULL) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (!info) {
        return -EIO;
    }

    return info->avail_chan;
}

static int tsb_dma_check_chan_params(struct device_dma_params *params)
{
    if ((params->src_inc_options != DEVICE_DMA_INC_AUTO) &&
        (params->src_inc_options != DEVICE_DMA_INC_NOAUTO)) {
        return DEVICE_DMA_ERROR_BAD_FLAG;
    }

    if ((params->dst_inc_options != DEVICE_DMA_INC_AUTO) &&
        (params->dst_inc_options != DEVICE_DMA_INC_NOAUTO)) {
        return DEVICE_DMA_ERROR_BAD_FLAG;
    }

    if ((params->transfer_size > DEVICE_DMA_TRANSFER_SIZE_64) ||
        ((params->transfer_size & (params->transfer_size - 1)) != 0)) {
        return DEVICE_DMA_ERROR_BAD_TX_SIZE;
    }

    if ((params->burst_len > DEVICE_DMA_BURST_LEN_16) ||
        ((params->burst_len & (params->burst_len - 1)) != 0)) {
        return DEVICE_DMA_ERROR_BAD_BURST_LEN;
    }

    if ((params->swap > DEVICE_DMA_SWAP_SIZE_128) ||
        ((params->swap & (params->swap - 1)) != 0)) {
        return DEVICE_DMA_ERROR_BAD_SWAP;
    }

    return DEVICE_DMA_ERROR_NONE;
}

static int tsb_dma_chan_alloc(struct device *dev,
        struct device_dma_params *params, void **chan)
{
    struct tsb_dma_info *info;
    int retval = -ENOMEM;
    int index;

    if ((dev == NULL) || (params == NULL) || (chan == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    if (info->avail_chan == 0) {
        return -ENOMEM;
    }

    retval = tsb_dma_check_chan_params(params);
    if (retval != DEVICE_DMA_ERROR_NONE) {
        return retval;
    }

    pthread_mutex_lock(&info->lock);

    for (index = 0; index < info->max_chan; index++) {
        if (info->chans[index] == NULL) {
            struct tsb_dma_chan *dma_chan;

            retval = gdmac_chan_alloc(dev, params, &dma_chan);

            if (retval == OK) {
                dma_chan->chan_id = index;
                pthread_mutex_init(&dma_chan->chan_mutex, NULL);

                list_init(&dma_chan->queue);

                /* Save a copy of channel parameters so we can check the op
                 * parameters against its.
                 */
                memcpy(&dma_chan->chan_params, params, sizeof(*params));

                info->chans[index] = dma_chan;

                *chan = dma_chan;
                info->avail_chan--;
            }
            break;
        }
    }

    pthread_mutex_unlock(&info->lock);

    return retval;
}

static int tsb_dma_chan_free(struct device *dev, void *chan)
{
    struct tsb_dma_info *info;
    struct tsb_dma_chan *dma_chan = chan;
    int retval = -ENOMEM;

    if ((dev == NULL) || (chan == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    if (info->chans[dma_chan->chan_id] == dma_chan) {
        irqstate_t flags;
        struct list_head *node, *next_node;

        pthread_mutex_lock(&info->lock);

        flags = irqsave();

        list_foreach_safe(&dma_chan->queue, node, next_node) {
            struct tsb_dma_op *dma_op;

            dma_op = list_entry(node, struct tsb_dma_op, list_node);

            if (dma_op->state == TSB_DMA_OP_STATE_QUEUED) {
                struct device_dma_op *dev_op = &dma_op->op;
                list_del(node);

                dma_op->state = DEVICE_DMA_CALLBACK_EVENT_DEQUEUED;
                if ((dev_op->callback != NULL) &&
                    (dev_op->callback_events &
                            DEVICE_DMA_CALLBACK_EVENT_DEQUEUED)) {
                    dev_op->callback(dev, chan, dev_op,
                            DEVICE_DMA_CALLBACK_EVENT_DEQUEUED,
                            dev_op->callback_arg);
                }
            }
        }

        irqrestore(flags);

        if (list_is_empty(&dma_chan->queue) != true) {
            retval = -EIO;
        } else {
            pthread_mutex_destroy(&dma_chan->chan_mutex);

            info->chans[dma_chan->chan_id] = NULL;
            info->avail_chan++;
            retval = gdmac_chan_free(dev, dma_chan);
        }

        pthread_mutex_unlock(&info->lock);
    }

    return retval;
}

static int tsb_dma_op_alloc(struct device *dev, unsigned int sg_count,
        unsigned int extra, struct device_dma_op **op)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;

    if ((dev == NULL) || (op == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    dma_op = zalloc(
            sizeof(struct tsb_dma_op) + sizeof(struct device_dma_sg) * sg_count
                    + extra);
    if (dma_op == NULL) {
        return -ENOMEM;
    } else {
        list_init(&dma_op->list_node);
        dma_op->state = TSB_DMA_OP_STATE_IDLE;
        dma_op->error = DEVICE_DMA_ERROR_NONE;

        *op = &dma_op->op;
    }

    return OK;
}

static int tsb_dma_op_free(struct device *dev, struct device_dma_op *op)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;

    if ((dev == NULL) || (op == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    dma_op = containerof(op, struct tsb_dma_op, op);
    if ((list_is_empty(&dma_op->list_node) == 0) &&
        ((dma_op->state != TSB_DMA_OP_STATE_IDLE) ||
         (dma_op->state != TSB_DMA_OP_STATE_COMPLETED))) {
        return -EINVAL;
    } else {
        free(dma_op);
    }

    return OK;
}

static int tsb_dma_op_is_complete(struct device *dev, struct device_dma_op *op)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;

    if ((dev == NULL) || (op == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    dma_op = containerof(op, struct tsb_dma_op, op);

    return (dma_op->state == TSB_DMA_OP_STATE_COMPLETED) ? 1 : 0;
}

static int tsb_dma_op_get_error(struct device *dev, struct device_dma_op *op,
        enum device_dma_error *error)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;

    if ((op == NULL) || (error == NULL)) {
        return -EINVAL;
    }

    if (dev == NULL) {
        *error = DEVICE_DMA_ERROR_BAD_DEV;
        return OK;
    }

    dma_op = containerof(op, struct tsb_dma_op, op);

    info = device_get_private(dev);
    if (!info) {
        *error = DEVICE_DMA_ERROR_BAD_DEV;
        return OK;
    }

    *error = dma_op->error;

    return OK;
}

static int tsb_dma_enqueue(struct device *dev, void *chan,
        struct device_dma_op *op)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;
    struct tsb_dma_chan *dma_chan = (struct tsb_dma_chan *) chan;
    irqstate_t flags;
    int retval = -EIO;

    if ((chan == NULL) || (op == NULL)) {
        return -EINVAL;
    }

    dma_op = containerof(op, struct tsb_dma_op, op);

    if (dev == NULL) {
        dma_op->error = DEVICE_DMA_ERROR_BAD_DEV;
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        dma_op->error = DEVICE_DMA_ERROR_BAD_DEV;
        return -EIO;
    }

    /* Check the op if there is any alignment issue */
    retval = gdmac_chan_check_op_params(dev, dma_chan, op);
    if (retval != DEVICE_DMA_ERROR_NONE) {
        lldbg("Failed in parameters check.\n");
        dma_op->error = retval;

        return -EINVAL;
    }

    retval = OK;

    flags = irqsave();

    /* Add the op to the queue on the channel */
    /* TODO: Update to add a requeue function or a realloc function to reset the state instead of
     * checking for TSB_DMA_OP_STATE_COMPLETED.  Thoughts on the best way to handle this are
     * welcome
     */
    if ((dma_op->state == TSB_DMA_OP_STATE_IDLE) || (dma_op->state == TSB_DMA_OP_STATE_COMPLETED)) {
        list_add(&dma_chan->queue, &dma_op->list_node);
        dma_op->state = TSB_DMA_OP_STATE_QUEUED;
        dma_op->chan_id = dma_chan->chan_id;
        retval = OK;
    }

    irqrestore(flags);

    tsb_dma_restart_chan(dev, dma_chan);

    return retval;
}

static int tsb_dma_dequeue(struct device *dev, void *chan,
        struct device_dma_op *op)
{
    struct tsb_dma_info *info;
    struct tsb_dma_op *dma_op;
    irqstate_t flags;
    int retval = -EIO;

    if ((dev == NULL) || (op == NULL)) {
        return -EINVAL;
    }

    info = device_get_private(dev);
    if (!info) {
        return -EIO;
    }

    dma_op = containerof(op, struct tsb_dma_op, op);

    flags = irqsave();

    /* Make sure if the op is in the queued state before we dequeue it from
     * from the queue. If the op is already started, we can't stop the DMAC
     * and delete it.
     */
    if (dma_op->state == TSB_DMA_OP_STATE_QUEUED) {
        struct device_dma_op *dev_op = &dma_op->op;

        list_del(&dma_op->list_node);

        dma_op->state = DEVICE_DMA_CALLBACK_EVENT_DEQUEUED;
        if ((dev_op->callback != NULL) &&
            (dev_op->callback_events &
                    DEVICE_DMA_CALLBACK_EVENT_DEQUEUED)) {
            dev_op->callback(dev, chan, dev_op,
                    DEVICE_DMA_CALLBACK_EVENT_DEQUEUED,
                    dev_op->callback_arg);
        }
        retval = OK;
    }

    irqrestore(flags);

    return retval;
}

int tsb_dma_callback(struct device *dev, struct tsb_dma_chan *dma_chan,
        int event)
{
    struct tsb_dma_info *info = device_get_private(dev);
    struct list_head *next_op;

    /* This routine runs in the interrupt context, so there is no other
     * thread would manipulate the the op other than the user callback
     * routine which wouldn't happen until the op is removed from the
     * completed queue.
     */
    list_foreach(&dma_chan->queue, next_op)
    {
        struct tsb_dma_op
        *dma_op = list_entry(next_op, struct tsb_dma_op, list_node);

        /* Make sure the op is in either starting or running state. */
        if ((dma_op->state == TSB_DMA_OP_STATE_STARTING)
                || (dma_op->state == TSB_DMA_OP_STATE_RUNNING)) {
            dma_op->state = TSB_DMA_OP_STATE_COMPLETING;

            list_del(next_op);
            list_add(&info->completed_queue, next_op);

            sem_post(&info->op_completed_sem);
        }
        break;
    }

    return OK;
}

static struct device_dma_type_ops tsb_dma_type_ops = {
        .get_caps = tsb_dma_get_caps,
        .chan_free_count = tsb_dma_chan_free_count,
        .chan_alloc = tsb_dma_chan_alloc,
        .chan_free = tsb_dma_chan_free,
        .op_alloc = tsb_dma_op_alloc,
        .op_free = tsb_dma_op_free,
        .op_is_complete = tsb_dma_op_is_complete,
        .op_get_error = tsb_dma_op_get_error,
        .enqueue = tsb_dma_enqueue,
        .dequeue = tsb_dma_dequeue
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
