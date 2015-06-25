/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 * Author: Benoit Cousson <bcousson@baylibre.com>
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

#include <arch/byteorder.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include "battery-gb.h"

/* Version of the Greybus battery protocol we support */
#define GB_BATTERY_VERSION_MAJOR    0x00
#define GB_BATTERY_VERSION_MINOR    0x01


static uint8_t gb_battery_protocol_version(struct gb_operation *operation)
{
    struct gb_battery_proto_version_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_BATTERY_VERSION_MAJOR;
    response->minor = GB_BATTERY_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_technology(struct gb_operation *operation)
{
    struct gb_battery_technology_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->technology = cpu_to_le32(GB_BATTERY_TECH_LIPO);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_status(struct gb_operation *operation)
{
    struct gb_battery_status_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->status = cpu_to_le16(GB_BATTERY_STATUS_FULL);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_max_voltage(struct gb_operation *operation)
{
    struct gb_battery_max_voltage_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->voltage = cpu_to_le32(3800);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_percent_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->capacity = cpu_to_le32(100);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_temperature(struct gb_operation *operation)
{
    struct gb_battery_temperature_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->temperature = cpu_to_le32(25);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_voltage(struct gb_operation *operation)
{
    struct gb_battery_voltage_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->voltage = cpu_to_le32(3000);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_current(struct gb_operation *operation)
{
    gb_info("%s()\n", __func__);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->capacity = cpu_to_le32(150);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_shutdowntemp(struct gb_operation *operation)
{
    struct gb_battery_temperature_response *response;

    gb_info("%s()\n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->temperature = cpu_to_le32(50);

    return GB_OP_SUCCESS;
}


static int gb_battery_init(unsigned int cport)
{
    return 0;
}

static struct gb_operation_handler gb_battery_handlers[] = {
    GB_HANDLER(GB_BATTERY_TYPE_PROTOCOL_VERSION, gb_battery_protocol_version),
    GB_HANDLER(GB_BATTERY_TYPE_TECHNOLOGY, gb_battery_technology),
    GB_HANDLER(GB_BATTERY_TYPE_STATUS, gb_battery_status),
    GB_HANDLER(GB_BATTERY_TYPE_MAX_VOLTAGE, gb_battery_max_voltage),
    GB_HANDLER(GB_BATTERY_TYPE_PERCENT_CAPACITY, gb_battery_percent_capacity),
    GB_HANDLER(GB_BATTERY_TYPE_TEMPERATURE, gb_battery_temperature),
    GB_HANDLER(GB_BATTERY_TYPE_VOLTAGE, gb_battery_voltage),
    GB_HANDLER(GB_BATTERY_TYPE_CURRENT, gb_battery_current),
    GB_HANDLER(GB_BATTERY_TYPE_CAPACITY, gb_battery_capacity),
    GB_HANDLER(GB_BATTERY_TYPE_SHUTDOWN_TEMP, gb_battery_shutdowntemp),
};

static struct gb_driver gb_battery_driver = {
    .init = gb_battery_init,
    .op_handlers = gb_battery_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_battery_handlers),
};

void gb_battery_register(int cport)
{
    gb_register_driver(cport, &gb_battery_driver);
}
