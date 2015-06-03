/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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
#include <debug.h>
#include <stdlib.h>

#include <arch/board/slice.h>

#include <nuttx/greybus/greybus.h>

#include "vendor-moto-gb.h"

#define GB_VENDOR_MOTO_VERSION_MAJOR     0
#define GB_VENDOR_MOTO_VERSION_MINOR     1

static uint8_t gb_vendor_moto_protocol_version(struct gb_operation *operation)
{
    struct gb_vendor_moto_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_VENDOR_MOTO_VERSION_MAJOR;
    response->minor = GB_VENDOR_MOTO_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_vendor_moto_charge_base(struct gb_operation *operation)
{
    struct gb_vendor_moto_charge_base_request *request =
        gb_operation_get_request_payload(operation);

    lowsyslog("charge_base: enable=%d\n", request->enable);

    slice_vbus_en_sw(request->enable > 0);
    return GB_OP_SUCCESS;
}

static int gb_vendor_moto_init(unsigned int cport)
{
    // Nothing to do: GPIO is already configured
    return 0;
}

static struct gb_operation_handler gb_vendor_moto_handlers[] = {
    GB_HANDLER(GB_VENDOR_MOTO_PROTOCOL_VERSION, gb_vendor_moto_protocol_version),
    GB_HANDLER(GB_VENDOR_MOTO_CHARGE_BASE, gb_vendor_moto_charge_base),
};

static struct gb_driver gb_vendor_moto_driver = {
    .init = gb_vendor_moto_init,
    .op_handlers = gb_vendor_moto_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_vendor_moto_handlers),
};

void gb_vendor_register(int cport)
{
    gb_register_driver(cport, &gb_vendor_moto_driver);
}

