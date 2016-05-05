/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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
#include <nuttx/device_raw.h>
#include <nuttx/greybus/greybus.h>

#include <apps/greybus-utils/utils.h>

/* Greybus RAW operation types */
#define GB_RAW_TYPE_PROTOCOL_VERSION   0x01
#define GB_RAW_TYPE_SEND               0x02

#define GB_RAW_VERSION_MAJOR              0
#define GB_RAW_VERSION_MINOR              1

/**
 * Greybus Raw Protocol Version Response
 */
struct gb_raw_proto_version_response {
    __u8 major; /**< Greybus Raw Protocol major version */
    __u8 minor; /**< Greybus Raw Protocol minor version */
};

/**
 * Greybus Raw Protocol Send Request
 */
struct gb_raw_send_request {
        __le32  len;
        __u8    data[0];
};

struct gb_raw_info {
    /** opened device driver handler */
    struct device *dev;
    /** assigned CPort number */
    unsigned int cport;
};

static struct gb_raw_info *raw_info;

/**
 * @brief Get this firmware supported Raw protocol version.
 *
 * This function is called when Raw operates initialize in Greybus kernel.
 *
 * @param operation Pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_raw_protocol_version(struct gb_operation *operation)
{
    struct gb_raw_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_RAW_VERSION_MAJOR;
    response->minor = GB_RAW_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Send message from module
 *
 * This function sends the raw data to the AP, internal version
 * called by gb_raw_send.
 *
 * @param length of data field
 * @param data data to send
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_raw_protocol_send(uint32_t len, uint8_t data[])
{
    struct gb_raw_send_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(raw_info->cport, GB_RAW_TYPE_SEND,
            sizeof(*request) + len);
    if (!operation)
        return GB_OP_NO_MEMORY;

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_operation_destroy(operation);
        return GB_OP_INVALID;
    }
    request->len = cpu_to_le32(len);
    memcpy(request->data, data, len);

    ret = gb_operation_send_request(operation, NULL, false);

    gb_operation_destroy(operation);

    return ret;
}

/**
 * @brief Callback for data sending
 *
 * @param dev Pointer to structure of device data.
 * @param len buffer length.
 * @return None.
 */
static int raw_callback_routine(struct device *dev,
                                uint32_t len, uint8_t data[])
{
    return (int) gb_raw_protocol_send(len, data);
}

/**
 * @brief Called on receive of message send by device
 *
 * @param operation Pointer to structure of gb_operation
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_raw_protocol_recv(struct gb_operation *operation)
{
    struct gb_raw_send_request *request;
    int ret;

    request = (struct gb_raw_send_request *)
            gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    ret = device_raw_recv(raw_info->dev, le32_to_cpu(request->len),
            request->data);
    if (ret) {
        return GB_OP_UNKNOWN_ERROR;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief called on initialization of raw interface
 *
 * @param cport our cport used for greybus operations
 */
static int gb_raw_init(unsigned int cport)
{
    int ret = 0;

    raw_info = zalloc(sizeof(*raw_info));
    if (!raw_info) {
        ret = -ENOMEM;
        goto err_out;
    }

    raw_info->cport = cport;

    raw_info->dev = device_open(DEVICE_TYPE_RAW_HW, 0);
    if (!raw_info->dev) {
        gb_info("failed to open %s device!\n", DEVICE_TYPE_RAW_HW);
        ret = -EIO;
        goto err_free;
    }

    ret = device_raw_register_callback(raw_info->dev,
                                       raw_callback_routine);
    if (ret) {
        gb_info("failed to register callback for %s device!\n",
                DEVICE_TYPE_RAW_HW);
        ret = -EIO;
        goto err_close;
    }

    return 0;

err_close:
    device_close(raw_info->dev);
err_free:
    free(raw_info);
err_out:
    return ret;
}

/**
 * @brief called on teardown of raw interface
 *
 * @param cport our cport used for greybus operations
 */
static void gb_raw_exit(unsigned int cport)
{
    if (raw_info) {
        device_raw_unregister_callback(raw_info->dev);
        device_close(raw_info->dev);
        free(raw_info);
        raw_info = NULL;
    }
}

/**
 * @brief Greybus HID protocol operation handler
 */
static struct gb_operation_handler gb_raw_handlers[] = {
    GB_HANDLER(GB_RAW_TYPE_PROTOCOL_VERSION, gb_raw_protocol_version),
    GB_HANDLER(GB_RAW_TYPE_SEND, gb_raw_protocol_recv),
};

static struct gb_driver gb_raw_driver = {
    .init = gb_raw_init,
    .exit = gb_raw_exit,
    .op_handlers = gb_raw_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_raw_handlers),
};

/**
 * @brief Register Greybus RAW protocol
 *
 * @param cport CPort number
 */
void gb_raw_register(int cport)
{
    gb_register_driver(cport, &gb_raw_driver);
}
