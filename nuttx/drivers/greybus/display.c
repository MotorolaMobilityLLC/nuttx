/*
 * Copyright (c) 2015 Motorola, LLC.
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

#include <arch/byteorder.h>

#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/greybus/greybus.h>

#include <apps/greybus-utils/utils.h>

#include "display-gb.h"

struct gb_mods_display_info {
    /** opened device driver handler */
    struct device *dev;
    /** assigned CPort number */
    unsigned int cport;
};

static struct gb_mods_display_info *display_info;

/**
 * @brief Get this firmware supported Display protocol version.
 *
 * This function is called when the Display is initialized in Greybus kernel.
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_protocol_version(struct gb_operation *operation)
{
    struct gb_mods_display_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_MODS_DISPLAY_VERSION_MAJOR;
    response->minor = GB_MODS_DISPLAY_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the size of the configuration data
 *
 * This function is called before gb_mods_display_get_config() to get the
 * size of the data returned by that function.
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_get_config_size(struct gb_operation *operation)
{
    struct gb_mods_display_get_display_config_size_response *response;
    size_t config_size;
    int ret;

    ret = device_display_get_config_size(display_info->dev,  &config_size);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->size = cpu_to_le32(config_size);

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the currently selected configuration
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_get_config(struct gb_operation *operation)
{
    struct gb_mods_display_get_display_config_response *response;
    uint8_t type;
    size_t config_size;
    uint8_t *config_data;
    int ret;

    ret = device_display_get_config(display_info->dev,  &type, &config_size,
            &config_data);
    if (ret || !config_size)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation,
            sizeof(*response) + config_size);
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config_type = type;
    memcpy(response->config_data, config_data, config_size);

    return GB_OP_SUCCESS;
}

/**
 * @brief Select the configuration
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_set_config(struct gb_operation *operation)
{
    struct gb_mods_display_set_display_config_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_display_set_config(display_info->dev, request->index);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Set the state of the Display (on/off)
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_get_state(struct gb_operation *operation)
{
    struct gb_mods_display_get_display_state_response *response;
    uint8_t state;
    int ret;

    ret = device_display_get_state(display_info->dev,  &state);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->state = state;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the state of the Display (on/off)
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_set_state(struct gb_operation *operation)
{
    struct gb_mods_display_set_display_state_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_display_set_state(display_info->dev, request->state);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get the backlight configuration
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_get_backlight_config(struct gb_operation *operation)
{
    struct gb_mods_display_get_backlight_config_response *response;
    uint8_t config;
    int ret;

    ret = device_display_get_backlight_config(display_info->dev,  &config);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config = config;

    return GB_OP_SUCCESS;
}

/**
 * @brief Set the backlight configuration
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_set_backlight_config(struct gb_operation *operation)
{
    struct gb_mods_display_set_backlight_config_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_display_set_backlight_config(display_info->dev, request->config);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Set the backlight brightness to value between 0-255
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_set_backlight_brightness(struct gb_operation *operation)
{
    struct gb_mods_display_set_backlight_brightness_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    ret = device_display_set_backlight_brightness(display_info->dev, request->brightness);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Send notification of state change
 *
 * @param event to send
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_mods_display_notification(uint8_t event)
{
    struct gb_mods_display_notification_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(display_info->cport,
            GB_MODS_DISPLAY_TYPE_NOTIFICATION, sizeof(*request));
    if (!operation)
        return GB_OP_NO_MEMORY;

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_operation_destroy(operation);
        return GB_OP_INVALID;
    }
    request->event = event;

    ret = gb_operation_send_request(operation, NULL, false);

    gb_operation_destroy(operation);

    return ret;
}

/**
 * @brief Callback for sending display notification
 *
 * @param dev Pointer to structure of device data.
 * @param len buffer length.
 * @return None.
 */
static int _display_notification_cb(struct device *dev, uint8_t event)
{
    return (int) gb_mods_display_notification(event);
}

/**
 * @brief called on initialization of display interface
 *
 * @param cport our cport used for greybus operations
 */
static int gb_mods_display_init(unsigned int cport)
{
    int ret = 0;

    display_info = zalloc(sizeof(*display_info));
    if (!display_info) {
        ret = -ENOMEM;
        goto err_out;
    }

    display_info->cport = cport;

    display_info->dev = device_open(DEVICE_TYPE_DISPLAY_HW, 0);
    if (!display_info->dev) {
        gb_info("failed to open %s device!\n", DEVICE_TYPE_DISPLAY_HW);
        ret = -EIO;
        goto err_free;
    }

    ret = device_display_register_callback(display_info->dev,
                                       _display_notification_cb);
    if (ret) {
        gb_info("failed to register callback for %s device!\n",
                DEVICE_TYPE_DISPLAY_HW);
        ret = -EIO;
        goto err_close;
    }

    return 0;

err_close:
    device_close(display_info->dev);
err_free:
    free(display_info);
err_out:
    return ret;
}

/**
 * @brief called on teardown of display interface
 *
 * @param cport our cport used for greybus operations
 */
static void gb_mods_display_exit(unsigned int cport)
{
    if (display_info) {
        device_display_unregister_callback(display_info->dev);
        device_close(display_info->dev);
        free(display_info);
        display_info = NULL;
    }
}

/**
 * @brief Greybus Display protocol operation handler
 */
static struct gb_operation_handler gb_mods_display_handlers[] = {
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_PROTOCOL_VERSION, gb_mods_display_protocol_version),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_GET_CONFIG_SIZE, gb_mods_display_get_config_size),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_GET_CONFIG, gb_mods_display_get_config),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_SET_CONFIG, gb_mods_display_set_config),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_SET_STATE, gb_mods_display_set_state),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_GET_STATE, gb_mods_display_get_state),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_GET_BACKLIGHT_CONFIG, gb_mods_display_get_backlight_config),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_SET_BACKLIGHT_CONFIG, gb_mods_display_set_backlight_config),
    GB_HANDLER(GB_MODS_DISPLAY_TYPE_SET_BACKLIGHT_BRIGHTNESS, gb_mods_display_set_backlight_brightness),
};

static struct gb_driver gb_mods_display_driver = {
    .init = gb_mods_display_init,
    .exit = gb_mods_display_exit,
    .op_handlers = gb_mods_display_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_mods_display_handlers),
};

/**
 * @brief Register Greybus Display protocol
 *
 * @param cport CPort number
 */
void gb_mods_display_register(int cport)
{
    gb_register_driver(cport, &gb_mods_display_driver);
}
