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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <nuttx/greybus/greybus.h>
#include "gpio-gb.h"

#include <arch/tsb/gpio.h>

#define GB_GPIO_VERSION_MAJOR 0
#define GB_GPIO_VERSION_MINOR 1

static uint8_t gb_gpio_protocol_version(struct gb_operation *operation)
{
    struct gb_gpio_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_GPIO_VERSION_MAJOR;
    response->minor = GB_GPIO_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_line_count(struct gb_operation *operation)
{
    struct gb_gpio_line_count_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->count = gpio_line_count();
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_activate(struct gb_operation *operation)
{
    struct gb_gpio_activate_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    gpio_activate(request->which);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_deactivate(struct gb_operation *operation)
{
    struct gb_gpio_deactivate_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    gpio_deactivate(request->which);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_get_direction(struct gb_operation *operation)
{
    struct gb_gpio_get_direction_response *response;
    struct gb_gpio_get_direction_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->direction = gpio_get_direction(request->which);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_direction_in(struct gb_operation *operation)
{
    struct gb_gpio_direction_in_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    gpio_direction_in(request->which);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_direction_out(struct gb_operation *operation)
{
    struct gb_gpio_direction_out_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    gpio_direction_out(request->which, request->value);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_get_value(struct gb_operation *operation)
{
    struct gb_gpio_get_value_response *response;
    struct gb_gpio_get_value_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->value = gpio_get_value(request->which);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_set_value(struct gb_operation *operation)
{
    struct gb_gpio_set_value_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    gpio_set_value(request->which, request->value);
    return GB_OP_SUCCESS;
}

static uint8_t gb_gpio_set_debounce(struct gb_operation *operation)
{
    struct gb_gpio_set_debounce_request *request =
        gb_operation_get_request_payload(operation);

    if (request->which >= gpio_line_count())
        return GB_OP_INVALID;

    return gpio_set_debounce(request->which, request->usec);
}

static struct gb_operation_handler gb_gpio_handlers[] = {
    GB_HANDLER(GB_GPIO_TYPE_PROTOCOL_VERSION, gb_gpio_protocol_version),
    GB_HANDLER(GB_GPIO_TYPE_LINE_COUNT, gb_gpio_line_count),
    GB_HANDLER(GB_GPIO_TYPE_ACTIVATE, gb_gpio_activate),
    GB_HANDLER(GB_GPIO_TYPE_DEACTIVATE, gb_gpio_deactivate),
    GB_HANDLER(GB_GPIO_TYPE_GET_DIRECTION, gb_gpio_get_direction),
    GB_HANDLER(GB_GPIO_TYPE_DIRECTION_IN, gb_gpio_direction_in),
    GB_HANDLER(GB_GPIO_TYPE_DIRECTION_OUT, gb_gpio_direction_out),
    GB_HANDLER(GB_GPIO_TYPE_GET_VALUE, gb_gpio_get_value),
    GB_HANDLER(GB_GPIO_TYPE_SET_VALUE, gb_gpio_set_value),
    GB_HANDLER(GB_GPIO_TYPE_SET_DEBOUNCE, gb_gpio_set_debounce),
};

struct gb_driver gpio_driver = {
    .op_handlers = (struct gb_operation_handler*) gb_gpio_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_gpio_handlers),
};

void gb_gpio_register(int cport)
{
    gb_register_driver(cport, &gpio_driver);
}
