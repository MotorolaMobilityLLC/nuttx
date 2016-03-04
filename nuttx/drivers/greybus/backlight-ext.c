/*
 * Copyright (c) 2016 Motorola, LLC.
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
#include <string.h>
#include <queue.h>

#include <nuttx/device.h>
#include <nuttx/device_backlight.h>
#include <nuttx/greybus/greybus.h>

#include <apps/greybus-utils/utils.h>

/* Greybus Backlight operation types */
#define GB_BACKLIGHT_EXT_TYPE_PROTOCOL_VERSION       0x01
#define GB_BACKLIGHT_EXT_TYPE_SET_MODE               0x02
#define GB_BACKLIGHT_EXT_TYPE_GET_MODE               0x03
#define GB_BACKLIGHT_EXT_TYPE_SET_BRIGHTNESS         0x04
#define GB_BACKLIGHT_EXT_TYPE_GET_BRIGHTNESS         0x05

#define GB_BACKLIGHT_EXT_VERSION_MAJOR               0
#define GB_BACKLIGHT_EXT_VERSION_MINOR               1

/**
 * Greybus Backlight Protocol Version Response
 */
struct gb_backlight_ext_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

/**
 * Greybus Backlight Set Mode
 */
struct gb_backlight_ext_set_mode_request {
    __u8 mode;
} __packed;

/**
 * Greybus Backlight Get Mode
 */
struct gb_backlight_ext_get_mode_response {
    __u8 mode;
} __packed;

/**
 * Greybus Backlight Set Brightness
 */
struct gb_backlight_ext_set_brightness_request {
    __u8 brightness;
} __packed;

/**
 * Greybus Backlight Get Brightness
 */
struct gb_backlight_ext_get_brightness_response {
    __u8 brightness;
} __packed;

struct gb_backlight_info {
    /** opened device driver handler */
    struct device *dev;
    /** assigned CPort number */
    unsigned int cport;
};

static struct gb_backlight_info *backlight_info;

/**
 * @brief Get this firmware supported Backlight protocol version.
 *
 * This function is called when the Backlight is initialized in Greybus kernel.
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_backlight_ext_protocol_version(struct gb_operation *operation)
{
    struct gb_backlight_ext_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_BACKLIGHT_EXT_VERSION_MAJOR;
    response->minor = GB_BACKLIGHT_EXT_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Set the backlight mode
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_backlight_ext_set_mode(struct gb_operation *operation)
{
    struct gb_backlight_ext_set_mode_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_backlight_set_mode(backlight_info->dev, request->mode);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the Backlight mode
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_backlight_ext_get_mode(struct gb_operation *operation)
{
    struct gb_backlight_ext_get_mode_response *response;
    uint8_t mode;
    int ret;

    ret = device_backlight_get_mode(backlight_info->dev, &mode);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->mode = mode;

    return GB_OP_SUCCESS;
}

/**
 * @brief Set the Backlight brightness
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_backlight_ext_set_brightness(struct gb_operation *operation)
{
    struct gb_backlight_ext_set_brightness_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_backlight_set_brightness(backlight_info->dev, request->brightness);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the Backligth brightness
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_backlight_ext_get_brightness(struct gb_operation *operation)
{
    struct gb_backlight_ext_get_brightness_response *response;
    uint8_t brightness;
    int ret;

    ret = device_backlight_get_brightness(backlight_info->dev, &brightness);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->brightness = brightness;

    return GB_OP_SUCCESS;
}

/**
 * @brief called on initialization of backlight interface
 *
 * @param cport our cport used for greybus operations
 */
static int gb_backlight_ext_init(unsigned int cport)
{
    int ret = 0;

    backlight_info = zalloc(sizeof(*backlight_info));
    if (!backlight_info) {
        ret = -ENOMEM;
        goto err_out;
    }

    backlight_info->cport = cport;

    backlight_info->dev = device_open(DEVICE_TYPE_BACKLIGHT_HW, 0);
    if (!backlight_info->dev) {
        gb_info("failed to open %s device!\n", DEVICE_TYPE_BACKLIGHT_HW);
        ret = -EIO;
        goto err_free;
    }

    return 0;

err_free:
    free(backlight_info);
    backlight_info = NULL;
err_out:
    return ret;
}

/**
 * @brief called on teardown of backlight interface
 *
 * @param cport our cport used for greybus operations
 */
static void gb_backlight_ext_exit(unsigned int cport)
{
    if (backlight_info) {
        device_close(backlight_info->dev);
        free(backlight_info);
        backlight_info = NULL;
    }
}

/**
 * @brief Greybus Backlight protocol operation handler
 */
static struct gb_operation_handler gb_backlight_ext_handlers[] = {
    GB_HANDLER(GB_BACKLIGHT_EXT_TYPE_PROTOCOL_VERSION, gb_backlight_ext_protocol_version),
    GB_HANDLER(GB_BACKLIGHT_EXT_TYPE_SET_MODE, gb_backlight_ext_set_mode),
    GB_HANDLER(GB_BACKLIGHT_EXT_TYPE_GET_MODE, gb_backlight_ext_get_mode),
    GB_HANDLER(GB_BACKLIGHT_EXT_TYPE_SET_BRIGHTNESS, gb_backlight_ext_set_brightness),
    GB_HANDLER(GB_BACKLIGHT_EXT_TYPE_GET_BRIGHTNESS, gb_backlight_ext_get_brightness),
};

static struct gb_driver gb_backlight_ext_driver = {
    .init = gb_backlight_ext_init,
    .exit = gb_backlight_ext_exit,
    .op_handlers = gb_backlight_ext_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_backlight_ext_handlers),
};

/**
 * @brief Register Greybus Backlight protocol
 *
 * @param cport CPort number
 */
void gb_backlight_ext_register(int cport)
{
    gb_register_driver(cport, &gb_backlight_ext_driver);
}
