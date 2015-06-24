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
#include "loopback-gb.h"
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

struct list_head gb_loopbacks = {
    .next = &gb_loopbacks,
    .prev = &gb_loopbacks,
};

void *gb_loopback_fn(void *data)
{
    int ms;
    int type;
    size_t size;
    struct gb_loopback *gb_loopback = data;

    while(1) {
        if (gb_loopback->type == 0) {
            sleep(1);
            continue;
        }

        /* mutex lock */
        ms = gb_loopback->ms;
        type = gb_loopback->type;
        size = gb_loopback->size;
        /* mutex unlock */
        if (type == 1) {
            if (gb_loopback_ping_host(gb_loopback)) {
                gb_loopback->enomem++;
            }
        }
        if (type == 2) {
            if (gb_loopback_transfer_host(gb_loopback, size)) {
                gb_loopback->enomem++;
            }
        }
        if (ms) {
            usleep(ms*1000);
        }
    }
    return NULL;
}

struct gb_loopback *list_to_loopback(struct list_head *iter)
{
    return list_entry(iter, struct gb_loopback, list);
}

struct gb_loopback *cport_to_loopback(unsigned int cportid)
{
    struct list_head *iter;
    struct gb_loopback *gb_loopback;

    list_foreach(&gb_loopbacks, iter) {
        gb_loopback = list_entry(iter, struct gb_loopback, list);
        if (gb_loopback->cportid == cportid)
            return gb_loopback;
    }

    return NULL;
}

unsigned int gb_loopback_to_cport(struct gb_loopback *gb_loopback)
{
    return gb_loopback->cportid;
}

static int gb_loopback_reset(struct gb_loopback *gb_loopback)
{
    if (!gb_loopback) {
        return -EINVAL;
    }

    gb_loopback->error = 0;
    gb_loopback->enomem = 0;

    return 0;
}

int gb_loopback_cport_conf(struct gb_loopback *gb_loopback,
                           int type, size_t size, int ms)
{
    gb_loopback_reset(gb_loopback);
    /* mutex lock*/
    if (gb_loopback == NULL)
        return 1;
    gb_loopback->type = type;
    gb_loopback->size = size;
    gb_loopback->ms = ms;
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

static void gb_loopback_transfer_sync(struct gb_operation *operation)
{
    struct gb_loopback *gb_loopback;
    struct gb_loopback_transfer_response *response;
    struct gb_loopback_transfer_request *request;

    request = gb_operation_get_request_payload(operation);
    response = gb_operation_get_request_payload(operation->response);

    gb_loopback = cport_to_loopback(operation->cport);
    if (!gb_loopback) {
        return;
    }

    if (memcmp(request->data, response->data, le32_to_cpu(request->len)))
        gb_loopback->error += 1;
}

int gb_loopback_transfer_host(struct gb_loopback *gb_loopback, size_t size)
{
    int i;
    struct gb_operation *operation;
    struct gb_loopback_transfer_request *request;

    if (!gb_loopback) {
        return -EINVAL;
    }

    operation = gb_operation_create(gb_loopback->cportid,
                                    GB_LOOPBACK_TYPE_TRANSFER,
                                    sizeof(*request) + size);
    if (!operation)
        return -ENOMEM;

    request = gb_operation_get_request_payload(operation);
    request->len = cpu_to_le32(size);
    for (i = 0; i < size; i++) {
        request->data[i] = rand() & 0xff;
    }
    gb_operation_send_request(operation,
                              gb_loopback_transfer_sync, true);
    gb_operation_destroy(operation);

    return OK;
}

int gb_loopback_ping_host(struct gb_loopback *gb_loopback)
{
    struct gb_operation *operation;

    operation = gb_operation_create(gb_loopback->cportid,
                                    GB_LOOPBACK_TYPE_PING, 1);
    if (!operation)
        return -ENOMEM;
    gb_operation_send_request_sync(operation);
    gb_operation_destroy(operation);

    return OK;
}

int gb_loopback_status(struct gb_loopback *gb_loopback)
{
    if (!gb_loopback) {
        return -EINVAL;
    }

    return gb_loopback->error + gb_loopback->enomem;
}

static struct gb_operation_handler gb_loopback_handlers[] = {
    GB_HANDLER(GB_LOOPBACK_TYPE_PROTOCOL_VERSION,
               gb_loopback_protocol_version),
    GB_HANDLER(GB_LOOPBACK_TYPE_PING, gb_loopback_ping),
    GB_HANDLER(GB_LOOPBACK_TYPE_TRANSFER, gb_loopback_transfer),
};

struct gb_driver loopback_driver = {
    .op_handlers = (struct gb_operation_handler *)gb_loopback_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_loopback_handlers),
};

void gb_loopback_register(int cport)
{
    struct gb_loopback *gb_loopback = zalloc(sizeof(*gb_loopback));
    if (gb_loopback) {
        gb_loopback->cportid = cport;
        list_add(&gb_loopbacks, &gb_loopback->list);
        pthread_create(&gb_loopback->thread, NULL, gb_loopback_fn,
                       (pthread_addr_t)gb_loopback);
    }
    gb_register_driver(cport, &loopback_driver);
}
