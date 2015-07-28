/*
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

#include <nuttx/list.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/loopback.h>
#include <nuttx/greybus/loopback-gb.h>
#include <arch/tsb/unipro.h>
#include <arch/byteorder.h>

#define GB_LOOPBACK_VERSION_MAJOR 0
#define GB_LOOPBACK_VERSION_MINOR 1

struct gb_loopback {
    struct list_head list;
    int ms;
    int type;
    int enomem;
    size_t size;
    unsigned int error;
    unsigned int cportid;
    pthread_t thread;
    pthread_mutex_t mutex;
};

LIST_DECLARE(gb_loopback_list);

void *gb_loopback_fn(void *data)
{
    int ms;
    int type;
    size_t size;
    struct gb_loopback *loopback = data;

    while(1) {
        if (loopback->type == GB_LOOPBACK_TYPE_NONE) {
            sleep(1);
            continue;
        }

        /* mutex lock */
        ms = loopback->ms;
        type = loopback->type;
        size = loopback->size;
        /* mutex unlock */

        if (type == GB_LOOPBACK_TYPE_PING) {
            if (gb_loopback_send_req(loopback, 1,
                                     GB_LOOPBACK_TYPE_PING)) {
                loopback->enomem++;
            }
        }
        if (type == GB_LOOPBACK_TYPE_TRANSFER) {
            if (gb_loopback_send_req(loopback, size,
                                     GB_LOOPBACK_TYPE_TRANSFER)) {
                loopback->enomem++;
            }
        }
        if (type == GB_LOOPBACK_TYPE_SINK) {
            if (gb_loopback_send_req(loopback, size,
                                     GB_LOOPBACK_TYPE_SINK)) {
                loopback->enomem++;
            }
        }
        if (ms) {
            usleep(ms * 1000);
        }
    }
    return NULL;
}

struct gb_loopback *gb_loopback_from_list(struct list_head *iter)
{
    return list_entry(iter, struct gb_loopback, list);
}

struct gb_loopback *gb_loopback_from_cport(unsigned int cportid)
{
    struct list_head *iter;
    struct gb_loopback *loopback;

    list_foreach(&gb_loopback_list, iter) {
        loopback = list_entry(iter, struct gb_loopback, list);
        if (loopback->cportid == cportid)
            return loopback;
    }

    return NULL;
}

unsigned int gb_loopback_to_cport(struct gb_loopback *loopback)
{
    return loopback->cportid;
}

static int gb_loopback_reset(struct gb_loopback *loopback)
{
    if (!loopback) {
        return -EINVAL;
    }

    loopback->error = 0;
    loopback->enomem = 0;

    return 0;
}

int gb_loopback_cport_conf(struct gb_loopback *loopback,
                           int type, size_t size, int ms)
{
    gb_loopback_reset(loopback);
    /* mutex lock*/
    if (loopback == NULL)
        return 1;
    loopback->type = type;
    loopback->size = size;
    loopback->ms = ms;
    /* mutex unlock */
    return 0;
}

static uint8_t gb_loopback_protocol_version(struct gb_operation *operation)
{
    struct gb_loopback_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if(!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_LOOPBACK_VERSION_MAJOR;
    response->minor = GB_LOOPBACK_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_loopback_transfer(struct gb_operation *operation)
{
    struct gb_loopback_transfer_response *response;
    struct gb_loopback_transfer_request *request =
        gb_operation_get_request_payload(operation);
    size_t request_length = le32_to_cpu(request->len);

    response = gb_operation_alloc_response(operation,
                                           sizeof(*response) + request_length);
    if(!response)
        return GB_OP_NO_MEMORY;
    memcpy(response->data, request->data, request_length);
    return GB_OP_SUCCESS;
}

static uint8_t gb_loopback_ping(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

/**
 * @brief           Called upon reception of a 'sink' operation request
 *                  (receiving end)
 * @param[in]       operation: greybus loopback operation
 */
static uint8_t gb_loopback_sink_req_cb(struct gb_operation *operation)
{
    /* Ignore data payload, just acknowledge the operation. */
    return GB_OP_SUCCESS;
}

static void gb_loopback_transfer_sync(struct gb_operation *operation)
{
    struct gb_loopback *loopback;
    struct gb_loopback_transfer_response *response;
    struct gb_loopback_transfer_request *request;

    request = gb_operation_get_request_payload(operation);
    response = gb_operation_get_request_payload(operation->response);

    loopback = gb_loopback_from_cport(operation->cport);
    if (!loopback) {
        return;
    }

    if (memcmp(request->data, response->data, le32_to_cpu(request->len)))
        loopback->error += 1;
}

/**
 * @brief           called upon reception of a 'sink' operation response
 *                  (sending end)
 * @param[in]       operation: received greybus loopback operation
 */
static void gb_loopback_sink_resp_cb(struct gb_operation *operation)
{
    /*
     * FIXME: operation result shall be verified, but bug #826 implementing
     * this feature is still under development/review. To be completed.
     */
}

/**
 * @brief           Send loopback operation request
 * @return          OK in case of success, <0 otherwise
 * @param[in]       loopback: gb_loopback private driver data
 * @param[in]       size: request payload size in bytes
 * @param[in]       type: operation type (ping / transfer / sink)
 */
int gb_loopback_send_req(struct gb_loopback *loopback,
                         size_t size, uint8_t type)
{
    int i;
    struct gb_operation *operation;
    struct gb_loopback_transfer_request *request;

    if (!loopback) {
        return -EINVAL;
    }

    switch(type) {
    case GB_LOOPBACK_TYPE_PING:
        operation = gb_operation_create(loopback->cportid,
                                        GB_LOOPBACK_TYPE_PING, 1);
        break;
    case GB_LOOPBACK_TYPE_TRANSFER:
    case GB_LOOPBACK_TYPE_SINK:
        operation = gb_operation_create(loopback->cportid,
                                        type,
                                        sizeof(*request) + size);
        break;
    default:
        return -EINVAL;

    }
    if (!operation)
        return -ENOMEM;

    switch(type) {
    case GB_LOOPBACK_TYPE_PING:
        gb_operation_send_request_sync(operation);
        break;
    case GB_LOOPBACK_TYPE_TRANSFER:
    case GB_LOOPBACK_TYPE_SINK:
        request = gb_operation_get_request_payload(operation);
        request->len = cpu_to_le32(size);
        if (type == GB_LOOPBACK_TYPE_TRANSFER) {
            for (i = 0; i < size; i++) {
                request->data[i] = rand() & 0xFF;
            }
            gb_operation_send_request(operation,
                                      gb_loopback_transfer_sync, true);
        } else {
            /*
             * Data payload is ignored on receiver end.
             * No need to fill the buffer with some data.
             */
            gb_operation_send_request(operation,
                                      gb_loopback_sink_resp_cb, true);
        }
        break;
    default:
        return -EINVAL;

    }

    gb_operation_destroy(operation);
    return OK;
}

int gb_loopback_status(struct gb_loopback *loopback)
{
    if (!loopback) {
        return -EINVAL;
    }

    return loopback->error + loopback->enomem;
}

static struct gb_operation_handler gb_loopback_handlers[] = {
    GB_HANDLER(GB_LOOPBACK_TYPE_PROTOCOL_VERSION,
               gb_loopback_protocol_version),
    GB_HANDLER(GB_LOOPBACK_TYPE_PING, gb_loopback_ping),
    GB_HANDLER(GB_LOOPBACK_TYPE_TRANSFER, gb_loopback_transfer),
    GB_HANDLER(GB_LOOPBACK_TYPE_SINK, gb_loopback_sink_req_cb),
};

struct gb_driver loopback_driver = {
    .op_handlers = (struct gb_operation_handler *)gb_loopback_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_loopback_handlers),
};

void gb_loopback_register(int cport)
{
    struct gb_loopback *loopback = zalloc(sizeof(*loopback));
    if (loopback) {
        loopback->cportid = cport;
        list_add(&gb_loopback_list, &loopback->list);
        pthread_create(&loopback->thread, NULL, gb_loopback_fn,
                       (pthread_addr_t)loopback);
    }
    gb_register_driver(cport, &loopback_driver);
}
