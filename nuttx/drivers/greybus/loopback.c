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

#include <string.h>

#include <nuttx/greybus/greybus.h>
#include "loopback-gb.h"

#define GB_LOOPBACK_VERSION_MAJOR 0
#define GB_LOOPBACK_VERSION_MINOR 1

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

    response = gb_operation_alloc_response(operation,
                                           sizeof(*response) + request->len);
    if(!response)
        return GB_OP_NO_MEMORY;
    memcpy(response->data, request->data, request->len);
    return GB_OP_SUCCESS;
}

static uint8_t gb_loopback_ping(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
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
    gb_register_driver(cport, &loopback_driver);
}
