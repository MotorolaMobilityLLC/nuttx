/*
 * Copyright (c) 2015 Motorola Mobility LLC.
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
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>
#include "i2s-gb.h"

#define GB_I2S_DIRECT_DATA_VERSION_MAJOR        0x00
#define GB_I2S_DIRECT_DATA_VERSION_MINOR        0x01

#define GB_I2S_DIRECT_DATA_STACK_SIZE           512

extern void gb_i2s_set_rx_cport(int cport);
extern void gb_i2s_set_tx_cport(int cport);

static uint8_t gb_i2s_direct_data_protocol_version(struct gb_operation *operation)
{
    struct gb_i2s_proto_version_response *response;

    gb_debug("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;


    response->major = GB_I2S_DIRECT_DATA_VERSION_MAJOR;
    response->minor = GB_I2S_DIRECT_DATA_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static int gb_i2s_direct_data_rx_init(unsigned int cport)
{
    gb_debug("%s()\n cport %d", __func__, cport);

    gb_i2s_set_rx_cport(cport);
    return 0;
}

static int gb_i2s_direct_data_tx_init(unsigned int cport)
{
    gb_debug("%s()\n cport %d", __func__, cport);

    gb_i2s_set_tx_cport(cport);
    return 0;
}

static struct gb_operation_handler gb_i2s_direct_data_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION, gb_i2s_direct_data_protocol_version),
};

static struct gb_driver gb_i2s_direct_data_tx_driver = {
    .init = gb_i2s_direct_data_tx_init,
    .op_handlers = (struct gb_operation_handler*)gb_i2s_direct_data_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2s_direct_data_handlers),
    .stack_size = GB_I2S_DIRECT_DATA_STACK_SIZE,
};

static struct gb_driver gb_i2s_direct_data_rx_driver = {
    .init = gb_i2s_direct_data_rx_init,
    .op_handlers = (struct gb_operation_handler*)gb_i2s_direct_data_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2s_direct_data_handlers),
    .stack_size = GB_I2S_DIRECT_DATA_STACK_SIZE,
};

void gb_i2s_direct_rx_register(int cport, int protocol_id)
{
    gb_debug("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_direct_data_rx_driver);
}

void gb_i2s_direct_tx_register(int cport, int protocol_id)
{
    gb_debug("%s()\n", __func__);

    gb_register_driver(cport, &gb_i2s_direct_data_tx_driver);
}
