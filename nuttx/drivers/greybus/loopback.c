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

#include "loopback-gb.h"

#include <nuttx/list.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/loopback.h>
#include <arch/tsb/unipro.h>
#include <arch/byteorder.h>

#define GB_LOOPBACK_VERSION_MAJOR 0
#define GB_LOOPBACK_VERSION_MINOR 1

struct gb_loopback {
    struct list_head list;
    pthread_mutex_t lock;
    int cport;
    int err;
    unsigned recv;
};

struct list_head gb_loopback_list = LIST_INIT(gb_loopback_list);
static pthread_mutex_t gb_loopback_list_mutex = PTHREAD_MUTEX_INITIALIZER;

static void loopback_list_lock(void)
{
    pthread_mutex_lock(&gb_loopback_list_mutex);
}

static void loopback_list_unlock(void)
{
    pthread_mutex_unlock(&gb_loopback_list_mutex);
}

static void loopback_lock(struct gb_loopback *loopback)
{
    pthread_mutex_lock(&loopback->lock);
}

static void loopback_unlock(struct gb_loopback *loopback)
{
    pthread_mutex_unlock(&loopback->lock);
}

/**
 * @brief Iterate through all registered cports and call a function
 * @param cb callback function to be called for each cport
 * @param data additional private data to be passed to cb
 * @return 0 on success, return value of cb if a call to it fails
 */
int gb_loopback_get_cports(gb_loopback_cport_cb cb, void *data)
{
    struct gb_loopback *loopback;
    struct list_head *iter;
    int status = 0;

    loopback_list_lock();
    list_foreach(&gb_loopback_list, iter) {
        loopback = list_entry(iter, struct gb_loopback, list);
        status = cb(loopback->cport, data);
        if (status < 0)
            break;
    }
    loopback_list_unlock();

    return status;
}

static struct gb_loopback *loopback_from_cport(int cport)
{
    struct gb_loopback *loopback;
    struct list_head *iter;

    loopback_list_lock();
    list_foreach(&gb_loopback_list, iter) {
        loopback = list_entry(iter, struct gb_loopback, list);
        if (loopback->cport == cport) {
            loopback_list_unlock();
            return loopback;
        }
    }
    loopback_list_unlock();

    return NULL;
}

/**
 * @brief Get the number of reception errors for given cport
 * @param cport cport number
 * @return Number of errors since the last loopback reset
 */
int gb_loopback_get_error_count(int cport)
{
    struct gb_loopback *loopback = loopback_from_cport(cport);
    int err = -1;

    if (loopback != NULL) {
        loopback_lock(loopback);
        err = loopback->err;
        loopback_unlock(loopback);
    }

    return err;
}

static void loopback_error_notify(int cport)
{
    struct gb_loopback *loopback = loopback_from_cport(cport);

    if (loopback != NULL) {
        loopback_lock(loopback);
        loopback->err++;
        loopback_unlock(loopback);
    }
}

/**
 * @brief Get the number of received responses on given cport
 * @param cport cport number
 * @return Number of received responses since the last reset
 */
unsigned gb_loopback_get_recv_count(int cport)
{
    struct gb_loopback *loopback = loopback_from_cport(cport);
    unsigned recv = 0;

    if (loopback != NULL) {
        loopback_lock(loopback);
        recv = loopback->recv;
        loopback_unlock(loopback);
    }

    return recv;
}

static void loopback_recv_inc(int cport)
{
    struct gb_loopback *loopback = loopback_from_cport(cport);

    if (loopback != NULL) {
        loopback_lock(loopback);
        loopback->recv++;
        loopback_unlock(loopback);
    }
}

/**
 * @brief Reset statistics acquisition for given cport.
 * @param cport cport number
 */
void gb_loopback_reset(int cport)
{
    struct gb_loopback *loopback = loopback_from_cport(cport);

    if (loopback != NULL) {
        loopback_lock(loopback);
        loopback->err = 0;
        loopback->recv = 0;
        loopback_unlock(loopback);
    }
}

/**
 * @brief Check if given cport has been registered in the loopback driver
 * @param cport cport number
 * @return 1 if there's a loopback instance for this cport, 0 otherwise
 */
int gb_loopback_cport_valid(int cport)
{
    return loopback_from_cport(cport) != NULL;
}

/* Callbacks for gb_operation_send_request(). */

static void gb_loopback_ping_sink_resp_cb(struct gb_operation *operation)
{
    int ret;

    ret = gb_operation_get_request_result(operation);
    if (ret != OK)
        loopback_error_notify(operation->cport);
    else
        loopback_recv_inc(operation->cport);
}

static void gb_loopback_transfer_resp_cb(struct gb_operation *operation)
{
    struct gb_loopback_transfer_response *response;
    struct gb_loopback_transfer_request *request;

    request = gb_operation_get_request_payload(operation);
    response = gb_operation_get_request_payload(operation->response);

    if (memcmp(request->data, response->data, le32_to_cpu(request->len)))
        loopback_error_notify(operation->cport);
    else
        loopback_recv_inc(operation->cport);
}

/**
 * @brief           Send loopback operation request
 * @return          OK in case of success, <0 otherwise
 * @param[in]       cport: cport number
 * @param[in]       size: request payload size in bytes
 * @param[in]       type: operation type (ping / transfer / sink)
 */
int gb_loopback_send_req(int cport, size_t size, uint8_t type)
{
    struct gb_loopback_transfer_request *request;
    struct gb_operation *operation;
    int i, status, retval = OK;

    switch(type) {
    case GB_LOOPBACK_TYPE_PING:
        operation = gb_operation_create(cport, GB_LOOPBACK_TYPE_PING, 1);
        break;
    case GB_LOOPBACK_TYPE_TRANSFER:
    case GB_LOOPBACK_TYPE_SINK:
        operation = gb_operation_create(cport, type, sizeof(*request) + size);
        break;
    default:
        return -EINVAL;

    }
    if (!operation)
        return -ENOMEM;

    switch(type) {
    case GB_LOOPBACK_TYPE_PING:
        status = gb_operation_send_request(operation,
                                           gb_loopback_ping_sink_resp_cb,
                                           true);
        break;
    case GB_LOOPBACK_TYPE_TRANSFER:
    case GB_LOOPBACK_TYPE_SINK:
        request = gb_operation_get_request_payload(operation);
        request->len = cpu_to_le32(size);
        if (type == GB_LOOPBACK_TYPE_TRANSFER) {
            for (i = 0; i < size; i++) {
                request->data[i] = rand() & 0xFF;
            }
            status = gb_operation_send_request(operation,
                                               gb_loopback_transfer_resp_cb,
                                               true);
        } else {
            /*
             * Data payload is ignored on receiver end.
             * No need to fill the buffer with some data.
             */
            status = gb_operation_send_request(operation,
                                               gb_loopback_ping_sink_resp_cb,
                                               true);
        }
        break;
    default:
        return -EINVAL;

    }

    if (status != OK)
        retval = ERROR;

    gb_operation_destroy(operation);
    return retval;
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
        loopback->cport = cport;
        pthread_mutex_init(&loopback->lock, NULL);
        loopback_list_lock();
        list_add(&gb_loopback_list, &loopback->list);
        loopback_list_unlock();

    }
    gb_register_driver(cport, &loopback_driver);
}
