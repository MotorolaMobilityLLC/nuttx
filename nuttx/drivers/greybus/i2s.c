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
 * @brief Greybus I2S Protocol Driver
 */
/*
 * The overall data flow is from a Greybus transmitter module to one ore more
 * Greybus receiver modules.  However, a Greybus "transmitter" will be the
 * receiver on its local i2s connection.  Similarly, a Greybus receiver will
 * be the transmitter on its local i2s connection.  This can get confusing in
 * the code.  To help with this, all of the routines defined in this file refer
 * to the Greybus point of view except for the routines with 'll' in the name
 * which means its from the low-level/local i2s point of view.  So routine
 * names, struct members, etc.  that have "transmit" or "tx" in their name are
 * related to receiving data from the local i2s connection and transmitting
 * over the Greybus/UniPro.
 */

/* TODO:
 *
 * Supports only one Receiver CPort per I2S Bundle because all received
 * audio data goes to the one physical low-level controller.  So if there
 * is a mixer with two I2S connections, say, then use two separate I2S Bundles.
 *
 * For now, a FAST_HANDLER is not used for handling Greybus SEND_DATA
 * requests because the Greybus code on the AP assumes that every Greybus
 * message gets a response.  When the FAST_HANDLER is enabled,
 * gb_i2s_receiver_send_data_req_handler() can no longer call sem_wait().
 */
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>
#include <nuttx/util.h>
#include <nuttx/wdog.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>

#include <arch/byteorder.h>
#include <arch/tsb/unipro.h>

#include "i2s-gb.h"

#define GB_I2S_VERSION_MAJOR            0
#define GB_I2S_VERSION_MINOR            1

#define GB_I2S_CPORT_INVALID            (CPORT_MAX + 1)

#define GB_I2S_BUNDLE_0_ID              0
#define GB_I2S_BUNDLE_0_DEV_ID          0

#define GB_I2S_SAMPLES_PER_MSG_DEFAULT  1

#define GB_I2S_TX_SEND_DOWNSTREAM       8
#define GB_I2S_TX_TIMER_FUDGE_NS        (5 * 1000)

#define GB_I2S_TX_RING_BUF_PAD          8
#define GB_I2S_RX_RING_BUF_PAD          8

#define GB_I2S_CONFIG_MAX               32

#define GB_I2S_FLAG_CONFIGURED          BIT(0)
#define GB_I2S_FLAG_RX_PREPARED         BIT(1)
#define GB_I2S_FLAG_RX_STARTED          BIT(2)
#define GB_I2S_FLAG_TX_PREPARED         BIT(3)
#define GB_I2S_FLAG_TX_STARTED          BIT(4)
#define GB_I2S_FLAG_TX_DELAYING         BIT(5)

#ifndef SIGKILL
#define SIGKILL     9
#endif

enum gb_i2s_cport_type {
    GB_I2S_CPORT_TYPE_INVALID,
    GB_I2S_CPORT_TYPE_TRANSMITTER,
    GB_I2S_CPORT_TYPE_RECEIVER,
};

enum gb_i2s_cport_state {
    GB_I2S_CPORT_STATE_INVALID,
    GB_I2S_CPORT_STATE_INACTIVE,
    GB_I2S_CPORT_STATE_ACTIVE,
};

struct gb_i2s_cport_list_entry {
    struct list_head        list;
    struct gb_i2s_info      *info;
    uint16_t                cport;
    enum gb_i2s_cport_type  type;
    enum gb_i2s_cport_state state;
};

struct gb_i2s_info { /* One per I2S Bundle */
    uint16_t            mgmt_cport;
    uint32_t            flags;
    struct device       *dev;
    uint32_t            delay;
    uint32_t            sample_frequency;
    uint16_t            samples_per_message;
    unsigned int        next_rx_sample;
    unsigned int        next_tx_sample;
    unsigned int        sample_size;
    unsigned int        msg_data_size;
    uint8_t             *dummy_data;
    unsigned int        active_tx_cports;
    unsigned int        active_rx_cports;
    struct list_head    cport_list;
    struct ring_buf     *rx_rb;
    uint32_t            rx_rb_count;
    struct ring_buf     *tx_rb;
    sem_t               active_cports_lock;
    sem_t               tx_rb_sem;
    atomic_t            tx_rb_count;
    pthread_t           tx_rb_thread;
    WDOG_ID             tx_wdog;
};

struct gb_i2s_dev_info {
    uint16_t            bundle_id;
    char                *dev_type;
    unsigned int        dev_id;
    struct gb_i2s_info  *info;
};

static struct gb_i2s_dev_info gb_i2s_dev_info_map[] = {
    {
        .bundle_id  = GB_I2S_BUNDLE_0_ID,
        .dev_type   = DEVICE_TYPE_I2S_HW,
        .dev_id     = GB_I2S_BUNDLE_0_DEV_ID,
    },
};

static struct gb_i2s_dev_info *gb_i2s_get_dev_info(uint16_t bundle_id)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++)
        if (gb_i2s_dev_info_map[i].bundle_id == bundle_id)
            return &gb_i2s_dev_info_map[i];

    return NULL;
}

static struct gb_i2s_info *gb_i2s_get_info(uint16_t mgmt_cport)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++)
        if (gb_i2s_dev_info_map[i].info &&
            (gb_i2s_dev_info_map[i].info->mgmt_cport == mgmt_cport)) {

            return gb_i2s_dev_info_map[i].info;
        }

     return NULL;
}

static struct gb_i2s_cport_list_entry *gb_i2s_get_cple(struct gb_i2s_info *info,
                                                       uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;
    struct list_head *iter;

    list_foreach(&info->cport_list, iter) {
        cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

        if (cple->cport == cport)
            return cple;
    }

    return NULL;
}

static struct gb_i2s_cport_list_entry *gb_i2s_find_cple(uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++) {
        if (gb_i2s_dev_info_map[i].info) {
            cple = gb_i2s_get_cple(gb_i2s_dev_info_map[i].info, cport);
            if (cple)
                return cple;
        }
    }

    return NULL;
}

static struct gb_i2s_info *gb_i2s_get_info_by_cport(uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;

    cple = gb_i2s_find_cple(cport);
    if (cple)
        return cple->info;

    return NULL;
}

static void gb_i2s_report_event(struct gb_i2s_info *info, uint32_t event)
{
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;
    struct gb_i2s_report_event_request *request;

    operation = gb_operation_create(info->mgmt_cport,
                                    GB_I2S_MGMT_TYPE_REPORT_EVENT,
                                    sizeof(*hdr) + sizeof(*request));
    if (!operation)
        return;

    request = gb_operation_get_request_payload(operation);
    request->event = event;

    /* TODO: What to do when this fails? */
    gb_operation_send_request(operation, NULL, false);
    gb_operation_destroy(operation);
}

static void gb_i2s_ll_tx(struct gb_i2s_info *info, uint8_t *data)
{
    ring_buf_reset(info->rx_rb);

    /*
     * Unfortunately, we have to copy the data because the unipro
     * susbystem reuses the buffer immediately and the data may
     * not be sent out yet.
     */
    memcpy(ring_buf_get_tail(info->rx_rb), data, info->msg_data_size);

    ring_buf_put(info->rx_rb, info->msg_data_size);
    ring_buf_pass(info->rx_rb);

    info->next_rx_sample += info->samples_per_message;
    info->rx_rb = ring_buf_get_next(info->rx_rb);

    info->rx_rb_count++;
}

/* Callback for low-level i2s transmit operations (GB receives) */
static void gb_i2s_ll_tx_cb(struct ring_buf *rb,
                            enum device_i2s_event event, void *arg)
{
    struct gb_i2s_info *info = arg;
    uint32_t gb_event = 0;

    switch (event) {
    case DEVICE_I2S_EVENT_TX_COMPLETE:
        info->rx_rb_count--;

        /* TODO: Replace with smarter underrun prevention */
        if (info->rx_rb_count < 2)
            gb_i2s_ll_tx(info, info->dummy_data);

        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        gb_event = GB_I2S_EVENT_UNDERRUN;
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_event = GB_I2S_EVENT_OVERRUN;
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_event = GB_I2S_EVENT_CLOCKING;
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_event = GB_I2S_EVENT_DATA_LEN;
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_event = GB_I2S_EVENT_UNSPECIFIED;
        break;
    default:
        gb_event = GB_I2S_EVENT_INTERNAL_ERROR;
    }

    if (gb_event) {
        gb_i2s_report_event(info, gb_event);
        /* All driver error events halt streaming right now */
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
    }
}

static int gb_i2s_prepare_receiver(struct gb_i2s_info *info)
{
    unsigned int entries;
    int ret;

    if (info->flags & GB_I2S_FLAG_RX_PREPARED)
        return -EPROTO;

    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((info->sample_frequency * info->delay) /
               (info->samples_per_message * 1000000)) +
               GB_I2S_RX_RING_BUF_PAD;

    info->rx_rb = ring_buf_alloc_ring(entries,
                                      sizeof(struct gb_operation_hdr) +
                                        sizeof(struct gb_i2s_send_data_request),
                                      info->msg_data_size, 0, NULL, NULL, NULL);
    if (!info->rx_rb)
        return -ENOMEM;

    /* Greybus i2s message receiver is local i2s transmitter */
    ret = device_i2s_prepare_transmitter(info->dev, info->rx_rb,
                                         gb_i2s_ll_tx_cb, info);
    if (ret) {
        ring_buf_free_ring(info->rx_rb, NULL, NULL);
        info->rx_rb = NULL;

        return -EIO;
    }

    info->flags |= GB_I2S_FLAG_RX_PREPARED;

    return 0;
}

static uint8_t gb_i2s_receiver_send_data_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_send_data_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;
    irqstate_t flags;
    int ret;

    info = gb_i2s_get_info_by_cport(operation->cport);
    if (!info) {
        gb_i2s_report_event(info, GB_I2S_EVENT_PROTOCOL_ERROR);
        return GB_OP_SUCCESS;
    }

    sem_wait(&info->active_cports_lock);

    if (!info->active_rx_cports)
        goto err_exit;

    if (le32_to_cpu(request->size) != info->msg_data_size) {
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        goto err_exit;
    }

    flags = irqsave();

    if (!ring_buf_is_producers(info->rx_rb)) {
        irqrestore(flags);
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
        goto err_exit; /* Discard the message */
    }

    /* Fill in any missing data */
    while (ring_buf_is_producers(info->rx_rb) &&
           (le32_to_cpu(request->sample_number) > info->next_rx_sample))
        gb_i2s_ll_tx(info, info->dummy_data);

    gb_i2s_ll_tx(info, request->data);

    irqrestore(flags);

    ret = device_i2s_start_transmitter(info->dev);
    if (ret) {
        gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
        goto err_exit;
    }

    info->flags |= GB_I2S_FLAG_RX_STARTED;

err_exit:
    sem_post(&info->active_cports_lock);

    return GB_OP_SUCCESS;
}

static int gb_i2s_shutdown_receiver(struct gb_i2s_info *info)
{
    if (!(info->flags & GB_I2S_FLAG_RX_PREPARED))
        return -EPROTO;

    /*
     * Any audio data sitting in ring buffer is discarded.  This is intentional
     * since the AP will expect the audio to stop immediately when a receive
     * CPort is shutdown.
     */
    if (info->flags & GB_I2S_FLAG_RX_STARTED) {
        device_i2s_stop_transmitter(info->dev);
        info->flags &= ~GB_I2S_FLAG_RX_STARTED;
    }

    device_i2s_shutdown_transmitter(info->dev);

    ring_buf_free_ring(info->rx_rb, NULL, NULL);
    info->rx_rb = NULL;
    info->rx_rb_count = 0;
    info->next_rx_sample = 0;

    info->flags &= ~GB_I2S_FLAG_RX_PREPARED;

    return 0;
}

/*
 * Use 'tx_rb_sem' semaphore to regulate when a sample is sent to the
 * receiver.  When the start delay time passes, 'GB_I2S_TX_SEND_DOWNSTREAM'
 * samples will be sent to the receiver to help with jitter (see
 * gb_i2s_tx_delay_timeout()).  The rest will remain on transmitter so
 * receiver isn't overrun.
 */
static void *gb_i2s_tx_rb_thread(void *data)
{
    struct gb_i2s_info *info = data;
    struct gb_operation *operation;
    struct gb_i2s_send_data_request *request;
    struct ring_buf *rb;
    struct list_head *iter;
    struct gb_i2s_cport_list_entry *cple;
    int ret;

    /* TODO: Waits indefinitely when gb_i2s_ll_rx_cb() not called. Is that ok?*/
    while (1) {
        sem_wait(&info->tx_rb_sem);

        if (!atomic_get(&info->tx_rb_count))
            continue;

        sem_wait(&info->active_cports_lock);

        if (!(info->flags & GB_I2S_FLAG_TX_STARTED)) {
            sem_post(&info->active_cports_lock);
            continue;
        }

        rb = info->tx_rb;
        operation = rb->priv;

        request = gb_operation_get_response_payload(operation);
        request->sample_number = cpu_to_le32(info->next_tx_sample);
        info->next_tx_sample += info->samples_per_message;

        list_foreach(&info->cport_list, iter) {
            cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

            if ((cple->type != GB_I2S_CPORT_TYPE_TRANSMITTER) ||
                (cple->state != GB_I2S_CPORT_STATE_ACTIVE))
                continue;

            operation->cport = cple->cport;

            ret = gb_operation_send_request(operation, NULL, 0);
            if (ret) /* TODO: Add report_event throttling */
                gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        }

        sem_post(&info->active_cports_lock);

        ring_buf_reset(rb);
        ring_buf_pass(rb);

        ret = device_i2s_start_receiver(info->dev);
        if (ret)
            gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);

        atomic_dec(&info->tx_rb_count);
        info->tx_rb = ring_buf_get_next(rb);
    }

    /* NOTREACHED */
    return NULL;
}

/* Callback for low-level i2s receive operations (GB transmits) */
static void gb_i2s_ll_rx_cb(struct ring_buf *rb,
                            enum device_i2s_event event, void *arg)
{
    struct gb_i2s_info *info = arg;
    uint32_t gb_event = 0;

    if (!(info->flags & GB_I2S_FLAG_TX_STARTED))
        return;

    switch (event) {
    case DEVICE_I2S_EVENT_RX_COMPLETE:
        atomic_inc(&info->tx_rb_count);

        if (!(info->flags & GB_I2S_FLAG_TX_DELAYING))
            sem_post(&info->tx_rb_sem);
        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        gb_event = GB_I2S_EVENT_UNDERRUN;
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_event = GB_I2S_EVENT_OVERRUN;
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_event = GB_I2S_EVENT_CLOCKING;
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_event = GB_I2S_EVENT_DATA_LEN;
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_event = GB_I2S_EVENT_UNSPECIFIED;
        break;
    default:
        gb_event = GB_I2S_EVENT_INTERNAL_ERROR;
        break;
    }

    if (gb_event) {
        gb_i2s_report_event(info, gb_event);
        /* All driver error events halt streaming right now */
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
    }
}

static void gb_i2s_tx_delay_timeout(int argc, uint32_t arg, ...)
{
    struct gb_i2s_info *info = (struct gb_i2s_info *)arg;
    irqstate_t flags;
    uint32_t i, limit;

    flags = irqsave();

    if ((info->flags & GB_I2S_FLAG_TX_STARTED) &&
        (info->flags & GB_I2S_FLAG_TX_DELAYING)) {

        limit = atomic_get(&info->tx_rb_count);
        limit = MIN(limit, GB_I2S_TX_SEND_DOWNSTREAM);

        for (i = 0; i < limit; i++)
            sem_post(&info->tx_rb_sem);

        info->flags &= ~GB_I2S_FLAG_TX_DELAYING;
    }

    irqrestore(flags);
}

static int gb_i2s_rb_alloc_gb_op(struct ring_buf *rb, void *arg)
{
    struct gb_i2s_info *info = arg;
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;
    struct gb_i2s_send_data_request *request;

    operation = gb_operation_create(info->mgmt_cport,
                                    GB_I2S_DATA_TYPE_SEND_DATA,
                                    sizeof(*request) + info->msg_data_size);
    if (!operation)
        return -ENOMEM;

    ring_buf_init(rb, operation->request_buffer,
                  sizeof(*hdr) + sizeof(*request), info->msg_data_size);

    request = gb_operation_get_response_payload(operation);
    request->size = cpu_to_le32(info->msg_data_size);

    ring_buf_set_priv(rb, operation);

    return 0;
}

static void gb_i2s_rb_free_gb_op(struct ring_buf *rb, void *arg)
{
    gb_operation_destroy(ring_buf_get_priv(rb));
}

static int gb_i2s_prepare_transmitter(struct gb_i2s_info *info)
{
    unsigned int entries;
    int ret;

    if (info->flags & GB_I2S_FLAG_TX_PREPARED)
        return -EPROTO;

    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((info->sample_frequency * info->delay) /
               (info->samples_per_message * 1000000)) +
               GB_I2S_TX_RING_BUF_PAD;

    info->tx_rb = ring_buf_alloc_ring(entries, 0, 0, 0,
                                      gb_i2s_rb_alloc_gb_op,
                                      gb_i2s_rb_free_gb_op, info);
    if (!info->tx_rb)
        return -ENOMEM;

    /* Greybus i2s message transmitter is local i2s receiver */
    ret = device_i2s_prepare_receiver(info->dev, info->tx_rb, gb_i2s_ll_rx_cb,
                                      info);
    if (ret) {
        ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
        info->tx_rb = NULL;

        return ret;
    }

    info->flags |= GB_I2S_FLAG_TX_PREPARED;

    return 0;
}

static int gb_i2s_start_transmitter(struct gb_i2s_info *info)
{
    irqstate_t flags;
    int ret;

    if (!(info->flags & GB_I2S_FLAG_TX_PREPARED) ||
        (info->flags & GB_I2S_FLAG_TX_STARTED)) {

        return -EPROTO;
    }

    flags = irqsave();

    if (info->delay) {
        info->flags |= GB_I2S_FLAG_TX_DELAYING;

        /* TODO: Subtract out estimate of time it took to get here */
        wd_start(info->tx_wdog, info->delay / CLOCKS_PER_SEC,
                 gb_i2s_tx_delay_timeout, 1, info);
    }

    ret = device_i2s_start_receiver(info->dev);
    if (ret) {
        if (info->flags & GB_I2S_FLAG_TX_DELAYING) {
            wd_cancel(info->tx_wdog);
            info->flags &= ~GB_I2S_FLAG_TX_DELAYING;
        }

        goto err_exit;
    }

    info->flags |= GB_I2S_FLAG_TX_STARTED;

err_exit:
    irqrestore(flags);
    return ret;
}

static int gb_i2s_stop_transmitter(struct gb_i2s_info *info)
{
    irqstate_t flags;

    if (!(info->flags & GB_I2S_FLAG_TX_STARTED))
        return -EPROTO;

    info->flags &= ~GB_I2S_FLAG_TX_STARTED;

    device_i2s_stop_receiver(info->dev);

    flags = irqsave();

    if (info->flags & GB_I2S_FLAG_TX_DELAYING) {
        wd_cancel(info->tx_wdog);
        info->flags &= ~GB_I2S_FLAG_TX_DELAYING;
    }

    irqrestore(flags);

    return 0;
}

static int gb_i2s_shutdown_transmitter(struct gb_i2s_info *info)
{
    if (!(info->flags & GB_I2S_FLAG_TX_PREPARED) ||
        (info->flags & GB_I2S_FLAG_TX_STARTED)) {

        return -EPROTO;
    }

    device_i2s_shutdown_receiver(info->dev);

    atomic_add(&info->tx_rb_count, -atomic_get(&info->tx_rb_count));

    ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
    info->tx_rb = NULL;
    info->next_tx_sample = 0;

    info->flags &= ~GB_I2S_FLAG_TX_PREPARED;

    return 0;
}

static void gb_i2s_copy_cfg_gb2dev(struct gb_i2s_configuration *gb_cfg,
                                   struct device_i2s_configuration *dev_cfg)
{
    dev_cfg->sample_frequency = le32_to_cpu(gb_cfg->sample_frequency);
    dev_cfg->num_channels = gb_cfg->num_channels;
    dev_cfg->bytes_per_channel = gb_cfg->bytes_per_channel;
    dev_cfg->byte_order = gb_cfg->byte_order;
    dev_cfg->spatial_locations = le32_to_cpu(gb_cfg->spatial_locations);
    dev_cfg->ll_protocol = le32_to_cpu(gb_cfg->ll_protocol);
    dev_cfg->ll_mclk_role = gb_cfg->ll_mclk_role;
    dev_cfg->ll_bclk_role = gb_cfg->ll_bclk_role;
    dev_cfg->ll_wclk_role = gb_cfg->ll_wclk_role;
    dev_cfg->ll_wclk_polarity = gb_cfg->ll_wclk_polarity;
    dev_cfg->ll_wclk_change_edge = gb_cfg->ll_wclk_change_edge;
    dev_cfg->ll_data_tx_edge = gb_cfg->ll_data_tx_edge;
    dev_cfg->ll_data_rx_edge = gb_cfg->ll_data_rx_edge;
    dev_cfg->ll_data_offset = gb_cfg->ll_data_offset;
}

static void gb_i2s_copy_cfg_dev2gb(const struct device_i2s_configuration
                                                                    *dev_cfg,
                                   struct gb_i2s_configuration *gb_cfg)
{
    gb_cfg->sample_frequency = cpu_to_le32(dev_cfg->sample_frequency);
    gb_cfg->num_channels = dev_cfg->num_channels;
    gb_cfg->bytes_per_channel = dev_cfg->bytes_per_channel;
    gb_cfg->byte_order = dev_cfg->byte_order;
    gb_cfg->spatial_locations = cpu_to_le32(dev_cfg->spatial_locations);
    gb_cfg->ll_protocol = cpu_to_le32(dev_cfg->ll_protocol);
    gb_cfg->ll_mclk_role = dev_cfg->ll_mclk_role;
    gb_cfg->ll_bclk_role = dev_cfg->ll_bclk_role;
    gb_cfg->ll_wclk_role = dev_cfg->ll_wclk_role;
    gb_cfg->ll_wclk_polarity = dev_cfg->ll_wclk_polarity;
    gb_cfg->ll_wclk_change_edge = dev_cfg->ll_wclk_change_edge;
    gb_cfg->ll_data_tx_edge = dev_cfg->ll_data_tx_edge;
    gb_cfg->ll_data_rx_edge = dev_cfg->ll_data_rx_edge;
    gb_cfg->ll_data_offset = dev_cfg->ll_data_offset;
}

/* Greybus Operation Handlers */

static uint8_t gb_i2s_protocol_version_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_I2S_VERSION_MAJOR;
    response->minor = GB_I2S_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_get_supported_configurations_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_get_supported_configurations_response *response;
    struct gb_i2s_info *info;
    const struct device_i2s_configuration *dev_cfg;
    struct gb_i2s_configuration *gb_cfg;
    uint16_t dev_cfg_cnt;
    size_t size;
    uint16_t i;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    ret = device_i2s_get_supported_configurations(info->dev, &dev_cfg_cnt,
                                                  &dev_cfg);
    if (ret)
        return GB_OP_MALFUNCTION;

    dev_cfg_cnt = MIN(dev_cfg_cnt, GB_I2S_CONFIG_MAX);
    size = dev_cfg_cnt * sizeof(*gb_cfg);

    response = gb_operation_alloc_response(operation, sizeof(*response) + size);
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config_count = dev_cfg_cnt;
    gb_cfg = response->config;

    for (i = 0; i < dev_cfg_cnt; i++)
        gb_i2s_copy_cfg_dev2gb(dev_cfg++, gb_cfg++);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_configuration_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_configuration_request *request =
                gb_operation_get_request_payload(operation);
    struct device_i2s_configuration dev_cfg;
    struct gb_i2s_info *info;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    gb_i2s_copy_cfg_gb2dev(&request->config, &dev_cfg);

    ret = device_i2s_set_configuration(info->dev, &dev_cfg);
    if (ret)
        return GB_OP_MALFUNCTION;

    info->sample_frequency = request->config.sample_frequency;
    info->sample_size = request->config.num_channels *
                        request->config.bytes_per_channel;

    info->flags |= GB_I2S_FLAG_CONFIGURED;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_samples_per_message_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_samples_per_message_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    info->samples_per_message = le16_to_cpu(request->samples_per_message);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_get_processing_delay_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_get_processing_delay_response *response;
    struct gb_i2s_info *info;
    uint32_t microseconds;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_i2s_get_processing_delay(info->dev, &microseconds);
    if (ret)
        return GB_OP_MALFUNCTION;

    /* TODO Figure out a real delay value */
    response->microseconds = cpu_to_le32(microseconds + 0);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_start_delay_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_start_delay_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    info->delay = le32_to_cpu(request->microseconds);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_errno2gb(int ret)
{
    uint8_t rc;

    switch (-ret) {
    case 0:
        rc = GB_OP_SUCCESS;
        break;
    case EIO:
        rc = GB_OP_MALFUNCTION;
        break;
    case ENOMEM:
        rc = GB_OP_NO_MEMORY;
        break;
    case EBUSY:
        rc = GB_OP_RETRY;
        break;
    case EINVAL:
        rc = GB_OP_INVALID;
        break;
    case EPROTO:
        rc = GB_OP_PROTOCOL_BAD;
        break;
    default:
        rc = GB_OP_UNKNOWN_ERROR;
    }

    return rc;
}

static uint8_t gb_i2s_activate_cport_req_handler(struct gb_operation *operation)
{
    struct gb_i2s_activate_cport_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_cport_list_entry *cple;
    struct gb_i2s_info *info;
    int allocated_dummy_data = 0;
    int ret = 0;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if (!(info->flags & GB_I2S_FLAG_CONFIGURED))
        return GB_OP_PROTOCOL_BAD;

    cple = gb_i2s_get_cple(info, le16_to_cpu(request->cport));
    if (!cple)
        return GB_OP_INVALID;

    if (cple->state != GB_I2S_CPORT_STATE_INACTIVE)
        return GB_OP_PROTOCOL_BAD;

    if ((info->active_tx_cports + info->active_rx_cports) == 0) {
        info->msg_data_size = info->sample_size * info->samples_per_message;

        info->dummy_data = zalloc(info->msg_data_size);
        if (!info->dummy_data)
            return GB_OP_NO_MEMORY;

        allocated_dummy_data = 1;
    }

    sem_wait(&info->active_cports_lock);

    switch (cple->type) {
    case GB_I2S_CPORT_TYPE_RECEIVER:
        if (!info->active_rx_cports) {
            ret = gb_i2s_prepare_receiver(info);
            if (ret)
                break;
            /* i2s driver started when first Send Data Request arrives */
        }

        cple->state = GB_I2S_CPORT_STATE_ACTIVE;
        info->active_rx_cports++;
        break;
    case GB_I2S_CPORT_TYPE_TRANSMITTER:
        if (!info->active_tx_cports) {
            ret = gb_i2s_prepare_transmitter(info);
            if (ret)
                break;

            ret = gb_i2s_start_transmitter(info);
            if (ret) {
                gb_i2s_shutdown_transmitter(info);
                break;
            }
        }

        cple->state = GB_I2S_CPORT_STATE_ACTIVE;
        info->active_tx_cports++;
        break;
    default:
        ret = -EINVAL;
    }

    sem_post(&info->active_cports_lock);

    if (ret && allocated_dummy_data) {
        free(info->dummy_data);
        info->dummy_data = NULL;
    }

    return gb_i2s_errno2gb(ret);
}

static uint8_t gb_i2s_deactivate_cport_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_deactivate_cport_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_cport_list_entry *cple;
    struct gb_i2s_info *info;
    int ret = 0;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    cple = gb_i2s_get_cple(info, le16_to_cpu(request->cport));
    if (!cple)
        return GB_OP_INVALID;

    if (cple->state != GB_I2S_CPORT_STATE_ACTIVE)
        return GB_OP_PROTOCOL_BAD;

    sem_wait(&info->active_cports_lock);

    cple->state = GB_I2S_CPORT_STATE_INACTIVE;

    switch (cple->type) {
    case GB_I2S_CPORT_TYPE_RECEIVER:
        info->active_rx_cports--;

        if (!info->active_rx_cports)
            gb_i2s_shutdown_receiver(info);
        break;
    case GB_I2S_CPORT_TYPE_TRANSMITTER:
        info->active_tx_cports--;

        if (!info->active_tx_cports) {
            gb_i2s_stop_transmitter(info);
            gb_i2s_shutdown_transmitter(info);
        }
        break;
    default:
        ret = -EINVAL;
    }

    sem_post(&info->active_cports_lock);

    if (info->dummy_data &&
        (info->active_tx_cports + info->active_rx_cports) == 0) {

        free(info->dummy_data);
        info->dummy_data = NULL;
    }

    return gb_i2s_errno2gb(ret);
}

/* TODO: Fix rx or tx init/exit called before/after mgmt init/exit */
static int gb_i2s_mgmt_init(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_info *info;
    int ret;

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID);
    if (!dev_info)
        return -EINVAL;

    if (dev_info->info)
        return -EBUSY;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    info->mgmt_cport = cport;
    info->samples_per_message = GB_I2S_SAMPLES_PER_MSG_DEFAULT;
    info->tx_wdog = wd_create();

    list_init(&info->cport_list);
    sem_init(&info->active_cports_lock, 0, 1);
    sem_init(&info->tx_rb_sem, 0, 0);
    atomic_init(&info->tx_rb_count, 0);

    ret = pthread_create(&info->tx_rb_thread, NULL, gb_i2s_tx_rb_thread, info);
    if (ret) {
        ret = -ret;
        goto err_free_resources;
    }

    info->dev = device_open(dev_info->dev_type, dev_info->dev_id);
    if (!info->dev) {
        ret = -EIO;
        goto err_kill_pthread;
    }

    dev_info->info = info;

    return 0;

err_kill_pthread:
    pthread_kill(info->tx_rb_thread, SIGKILL);
err_free_resources:
    sem_destroy(&info->tx_rb_sem);
    sem_destroy(&info->active_cports_lock);
    wd_delete(info->tx_wdog);
    memset(info, 0, sizeof(*info));
    free(info);

    return ret;
}

static void gb_i2s_mgmt_exit(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;

    dev_info = gb_i2s_get_dev_info(cport);
    if (!dev_info && !dev_info->info)
        return;

    device_close(dev_info->info->dev);

    pthread_kill(dev_info->info->tx_rb_thread, SIGKILL);

    wd_cancel(dev_info->info->tx_wdog);
    sem_destroy(&dev_info->info->tx_rb_sem);
    sem_destroy(&dev_info->info->active_cports_lock);
    wd_delete(dev_info->info->tx_wdog);

    memset(dev_info->info, 0, sizeof(struct gb_i2s_info));
    free(dev_info->info);
    dev_info->info = NULL;
}

static struct gb_operation_handler gb_i2s_mgmt_handlers[] = {
    GB_HANDLER(GB_I2S_MGMT_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS,
               gb_i2s_get_supported_configurations_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_CONFIGURATION,
               gb_i2s_set_configuration_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE,
               gb_i2s_set_samples_per_message_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_PROCESSING_DELAY,
               gb_i2s_get_processing_delay_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_START_DELAY,
               gb_i2s_set_start_delay_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_ACTIVATE_CPORT,
               gb_i2s_activate_cport_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT,
               gb_i2s_deactivate_cport_req_handler),
    /* GB_I2S_TYPE_REPORT_EVENT should only be received by the AP */
};

static struct gb_driver i2s_mgmt_driver = {
    .init               = gb_i2s_mgmt_init,
    .exit               = gb_i2s_mgmt_exit,
    .op_handlers        = gb_i2s_mgmt_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_mgmt_handlers),
};

void gb_i2s_mgmt_register(int cport)
{
    gb_register_driver(cport, &i2s_mgmt_driver);
}

static int gb_i2s_cple_init(unsigned int cport, enum gb_i2s_cport_type type)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_cport_list_entry *cple;

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID);
    if (!dev_info || !dev_info->info)
        return -EINVAL;

    cple = zalloc(sizeof(*cple));
    if (!cple)
        return -ENOMEM;

    cple->info = dev_info->info;
    cple->cport = cport;
    cple->type = type;
    cple->state = GB_I2S_CPORT_STATE_INACTIVE;

    list_add(&dev_info->info->cport_list, &cple->list);

    return 0;
}

static int gb_i2s_cple_destroy(unsigned int cport)
{
    struct gb_i2s_info *info;
    struct gb_i2s_cport_list_entry *cple;

    info = gb_i2s_get_info_by_cport(cport);
    if (!info)
        return -EINVAL;

    cple = gb_i2s_find_cple(cport);
    if (!cple)
        return -EINVAL;

    list_del(&cple->list);
    memset(cple, 0, sizeof(*cple));
    free(cple);

    return 0;
}

static int gb_i2s_receiver_init(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_cport_list_entry *cple;
    struct list_head *iter;

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID);
    if (!dev_info || !dev_info->info)
        return -EINVAL;

    /* Can have only one Receiver CPort in a bundle */
    list_foreach(&dev_info->info->cport_list, iter) {
        cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

        if (cple->type == GB_I2S_CPORT_TYPE_RECEIVER)
            return -EINVAL;
    }

    return gb_i2s_cple_init(cport, GB_I2S_CPORT_TYPE_RECEIVER);
}

static void gb_i2s_receiver_exit(unsigned int cport)
{
    gb_i2s_cple_destroy(cport);
}

static struct gb_operation_handler gb_i2s_receiver_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
    GB_HANDLER(GB_I2S_DATA_TYPE_SEND_DATA,
               gb_i2s_receiver_send_data_req_handler),
};

static struct gb_driver i2s_receiver_driver = {
    .init               = gb_i2s_receiver_init,
    .exit               = gb_i2s_receiver_exit,
    .op_handlers        = gb_i2s_receiver_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_receiver_handlers),
};

void gb_i2s_receiver_register(int cport)
{
    gb_register_driver(cport, &i2s_receiver_driver);
}

static int gb_i2s_transmitter_init(unsigned int cport)
{
    return gb_i2s_cple_init(cport, GB_I2S_CPORT_TYPE_TRANSMITTER);
}

static void gb_i2s_transmitter_exit(unsigned int cport)
{
    gb_i2s_cple_destroy(cport);
}

static struct gb_operation_handler gb_i2s_transmitter_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
};

static struct gb_driver i2s_transmitter_driver = {
    .init               = gb_i2s_transmitter_init,
    .exit               = gb_i2s_transmitter_exit,
    .op_handlers        = gb_i2s_transmitter_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_transmitter_handlers),
};

void gb_i2s_transmitter_register(int cport)
{
    gb_register_driver(cport, &i2s_transmitter_driver);
}
