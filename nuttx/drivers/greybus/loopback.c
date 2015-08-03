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

struct list_head gb_loopback_list = LIST_INIT(gb_loopback_list);
static pthread_mutex_t gb_loopback_list_mutex = PTHREAD_MUTEX_INITIALIZER;

void gb_loopback_list_lock(void)
{
    pthread_mutex_lock(&gb_loopback_list_mutex);
}

void gb_loopback_list_unlock(void)
{
    pthread_mutex_unlock(&gb_loopback_list_mutex);
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
                           int type, size_t size, unsigned wait)
{
    if (loopback == NULL)
        return 1;

    gb_loopback_reset(loopback);

    loopback->type = type;
    loopback->size = size;
    loopback->wait = wait;

    return 0;
}

/* Callbacks for gb_operation_send_request(). */

static void gb_loopback_ping_sink_resp_cb(struct gb_operation *operation)
{
    struct gb_loopback *loopback;
    int ret;

    loopback = gb_loopback_from_cport(operation->cport);
    if (!loopback) {
        return;
    }

    ret = gb_operation_get_request_result(operation);
    if (ret != OK)
        loopback->error += 1;
}

static void gb_loopback_transfer_resp_cb(struct gb_operation *operation)
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
        gb_operation_send_request(operation,
                                  gb_loopback_ping_sink_resp_cb, true);
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
                                      gb_loopback_transfer_resp_cb, true);
        } else {
            /*
             * Data payload is ignored on receiver end.
             * No need to fill the buffer with some data.
             */
            gb_operation_send_request(operation,
                                      gb_loopback_ping_sink_resp_cb, true);
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

/*
 * The below functions are called by greybus-core upon
 * reception of inbound packets.
 */

static uint8_t gb_loopback_protocol_ver_cb(struct gb_operation *operation)
{
    struct gb_loopback_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if(!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_LOOPBACK_VERSION_MAJOR;
    response->minor = GB_LOOPBACK_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_loopback_transfer_req_cb(struct gb_operation *operation)
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

static uint8_t gb_loopback_ping_sink_req_cb(struct gb_operation *operation)
{
    /* Ignore data payload, just acknowledge the operation. */
    return GB_OP_SUCCESS;
}

static struct gb_operation_handler gb_loopback_handlers[] = {
    GB_HANDLER(GB_LOOPBACK_TYPE_PROTOCOL_VERSION, gb_loopback_protocol_ver_cb),
    GB_HANDLER(GB_LOOPBACK_TYPE_PING, gb_loopback_ping_sink_req_cb),
    GB_HANDLER(GB_LOOPBACK_TYPE_TRANSFER, gb_loopback_transfer_req_cb),
    GB_HANDLER(GB_LOOPBACK_TYPE_SINK, gb_loopback_ping_sink_req_cb),
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
        gb_loopback_list_lock();
        list_add(&gb_loopback_list, &loopback->list);
        gb_loopback_list_unlock();

    }
    gb_register_driver(cport, &loopback_driver);
}
