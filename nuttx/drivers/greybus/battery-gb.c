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

#include <nuttx/greybus/greybus.h>

#include "battery-gb.h"

#define GB_BATTERY_VERSION_MAJOR     0
#define GB_BATTERY_VERSION_MINOR     1

extern uint8_t gb_battery_driver_technology(__le32 *technology);
extern uint8_t gb_battery_driver_status(__le16 *status);
extern uint8_t gb_battery_driver_max_voltage(__le32 *max_voltage);
extern uint8_t gb_battery_driver_percent_capacity(__le32 *capacity);
extern uint8_t gb_battery_driver_temperature(__le32 *temperature);
extern uint8_t gb_battery_driver_voltage(__le32 *voltage);
extern uint8_t gb_battery_driver_current(__le32 *current);
extern uint8_t gb_battery_driver_capacity(__le32 *capacity);
extern uint8_t gb_battery_driver_shutdown_temperature(__le32 *temperature);
extern int     gb_battery_driver_init(void);

static uint8_t gb_battery_protocol_version(struct gb_operation *operation)
{
    struct gb_battery_proto_version_response *response;

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

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_technology(&response->technology);
}

static uint8_t gb_battery_status(struct gb_operation *operation)
{
    struct gb_battery_status_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_status(&response->status);
}

static uint8_t gb_battery_max_voltage(struct gb_operation *operation)
{
    struct gb_battery_max_voltage_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_max_voltage(&response->max_voltage);
}

static uint8_t gb_battery_percent_capacity(struct gb_operation *operation)
{
    struct gb_battery_percent_capacity_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_percent_capacity(&response->capacity);
}

static uint8_t gb_battery_temperature(struct gb_operation *operation)
{
    struct gb_battery_temperature_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_temperature(&response->temperature);
}

static uint8_t gb_battery_voltage(struct gb_operation *operation)
{
    struct gb_battery_voltage_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_voltage(&response->voltage);
}

static uint8_t gb_battery_current(struct gb_operation *operation)
{
    struct gb_battery_current_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_current(&response->current);
}

static uint8_t gb_battery_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_capacity(&response->capacity);
}

static uint8_t gb_battery_shutdown_temperature(struct gb_operation *operation)
{
    struct gb_battery_shutdown_temp_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    return gb_battery_driver_shutdown_temperature(&response->temperature);
}

static int gb_battery_init(unsigned int cport)
{
    // No common initialization required. Call driver specific init.
    return gb_battery_driver_init();
}

static struct gb_operation_handler gb_battery_handlers[] = {
    GB_HANDLER(GB_BATTERY_PROTOCOL_VERSION, gb_battery_protocol_version),
    GB_HANDLER(GB_BATTERY_TECHNOLOGY, gb_battery_technology),
    GB_HANDLER(GB_BATTERY_STATUS, gb_battery_status),
    GB_HANDLER(GB_BATTERY_MAX_VOLTAGE, gb_battery_max_voltage),
    GB_HANDLER(GB_BATTERY_PERCENT_CAPACITY, gb_battery_percent_capacity),
    GB_HANDLER(GB_BATTERY_TEMPERATURE, gb_battery_temperature),
    GB_HANDLER(GB_BATTERY_VOLTAGE, gb_battery_voltage),
    GB_HANDLER(GB_BATTERY_CURRENT, gb_battery_current),
    GB_HANDLER(GB_BATTERY_CAPACITY, gb_battery_capacity),
    GB_HANDLER(GB_BATTERY_SHUTDOWN_TEMPERATURE, gb_battery_shutdown_temperature),
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

