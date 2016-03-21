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

#include <arch/byteorder.h>

#include <nuttx/device.h>
#include <nuttx/device_ptp.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/util.h>

#include <stdbool.h>
#include <stdlib.h>

#include "ptp-gb.h"

#define GB_PTP_VERSION_MAJOR 0
#define GB_PTP_VERSION_MINOR 1

struct gb_ptp_info {
    unsigned int cport;
    struct device *dev;
    bool connected;
};

static struct gb_ptp_info *ptp_info = NULL;

static uint8_t gb_ptp_protocol_version(struct gb_operation *operation)
{
    struct gb_ptp_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_PTP_VERSION_MAJOR;
    response->minor = GB_PTP_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gp_ptp_get_functionality(struct gb_operation *operation)
{
    struct gb_ptp_get_functionality_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

#if defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER)
    response->int_snd = PTP_INT_SND_NEVER;
#elif defined (CONFIG_GREYBUS_PTP_INT_SND_SUPPLEMENTAL)
    response->int_snd = PTP_INT_SND_SUPPLEMENTAL;
#elif defined (CONFIG_GREYBUS_PTP_INT_SND_LOW_BATT_SAVER)
    response->int_snd = PTP_INT_SND_LOW_BATT_SAVER;
#else
    #error "define send power capabilities"
#endif

#if defined (CONFIG_GREYBUS_PTP_INT_RCV_NEVER)
    response->int_rcv = PTP_INT_RCV_NEVER;
#elif defined (CONFIG_GREYBUS_PTP_INT_RCV_FIRST)
    response->int_rcv = PTP_INT_RCV_FIRST;
#elif defined (CONFIG_GREYBUS_PTP_INT_RCV_SECOND)
    response->int_rcv = PTP_INT_RCV_SECOND;
#elif defined (CONFIG_GREYBUS_PTP_INT_RCV_PARALLEL)
    response->int_rcv = PTP_INT_RCV_PARALLEL;
#else
    #error "define receive power capabilities"
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    response->int_rcv_max_v = cpu_to_le32(CONFIG_GREYBUS_PTP_INT_RCV_MAX_V);
#else
    response->int_rcv_max_v = 0;
#endif

#if defined (CONFIG_GREYBUS_PTP_EXT_NONE)
    response->ext = PTP_EXT_NONE;
#elif defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    response->ext = PTP_EXT_SUPPORTED;
#else
    #error "define external power sources capabilities"
#endif

    return GB_OP_SUCCESS;
}

static uint8_t gb_ptp_set_current_flow(struct gb_operation *operation)
{
    struct gb_ptp_set_current_flow_request *request;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

#ifdef GREYBUS_PTP_INT_RCV_NEVER
    if (request->direction == PTP_CURRENT_TO_MOD)
        return GB_OP_INVALID;
#endif
#if defined(GREYBUS_PTP_INT_SND_NEVER) && defined(GREYBUS_PTP_EXT_NONE)
    if (request->direction == PTP_CURRENT_FROM_MOD)
        return GB_OP_INVALID;
#endif

    ret = device_ptp_set_current_flow(ptp_info->dev, request->direction);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_ptp_set_max_input_current(struct gb_operation *operation)
{
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    struct gb_ptp_set_max_input_current_request *request;
    uint32_t current;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("%s(): dropping short message\n", __func__);
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    current = le32_to_cpu(request->current);
    ret = device_ptp_set_max_input_current(ptp_info->dev, current);
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    return GB_OP_SUCCESS;
#else
    return GB_OP_INVALID;
#endif
}

static uint8_t gb_ptp_ext_power_present(struct gb_operation *operation)
{
    struct gb_ptp_ext_power_present_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

#if defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    if (device_ptp_ext_power_present(ptp_info->dev, &response->present))
        return GB_OP_UNKNOWN_ERROR;
#else
    response->present = PTP_EXT_POWER_NOT_PRESENT;
#endif

    return GB_OP_SUCCESS;
}

#if defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
static int gb_ptp_ext_power_changed(void)
{
    struct gb_operation *operation;
    int ret;

    /* Do not notify core until connected */
    if (!ptp_info->connected)
        return 0;

    operation = gb_operation_create(ptp_info->cport,
                                    GB_PTP_TYPE_EXT_POWER_CHANGED, 0);
    if (!operation) {
        gb_error("%s(): failed to create operation\n", __func__);
        return -ENOMEM;
    }

    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        gb_error("%s(): failed to send request\n", __func__);

    gb_operation_destroy(operation);

    return ret;
}
#endif

static uint8_t gb_ptp_power_required(struct gb_operation *operation)
{
    struct gb_ptp_power_required_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    if (device_ptp_power_required(ptp_info->dev, &response->required))
        return GB_OP_UNKNOWN_ERROR;
#else
        response->required = PTP_POWER_NOT_REQUIRED;
#endif

    return GB_OP_SUCCESS;
}

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
static int gb_ptp_power_required_changed(void)
{
    struct gb_operation *operation;
    int ret;

   /* Do not notify core until connected */
    if (!ptp_info->connected)
        return 0;

    operation = gb_operation_create(ptp_info->cport,
                                    GB_PTP_TYPE_POWER_REQUIRED_CHANGED, 0);
    if (!operation) {
        gb_error("%s(): failed to create operation\n", __func__);
        return -ENOMEM;
    }

    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        gb_error("%s(): failed to send request\n", __func__);

    gb_operation_destroy(operation);

    return ret;
}
#endif

static void gb_ptp_connected(unsigned int cport)
{
    ptp_info->connected = true;
}

static void gb_ptp_disconnected(unsigned int cport)
{
    ptp_info->connected = false;
}

static int gb_ptp_init(unsigned int cport)
{
    int ret = 0;

    ptp_info = zalloc(sizeof(*ptp_info));
    if (!ptp_info) {
        gb_info("%s(): failed to allocate memory\n", __func__);
        ret = -ENOMEM;
        goto done;
    }

    ptp_info->cport = cport;

    ptp_info->dev = device_open(DEVICE_TYPE_PTP_HW, 0);
    if (!ptp_info->dev) {
        gb_info("%s(): failed to open device\n", __func__);
        ret = -EIO;
        goto dev_err;
    }

#if defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    ret = device_ptp_register_ext_power_cb(ptp_info->dev,
                                           gb_ptp_ext_power_changed);
    if (ret) {
        gb_info("%s(): failed to register external power callback\n", __func__);
        goto cb_err;
    }
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    ret = device_ptp_register_power_required_cb(ptp_info->dev,
                                                gb_ptp_power_required_changed);
    if (ret) {
        gb_info("%s(): failed to register power required callback\n", __func__);
        goto cb_err;
    }
#endif

    return 0;

cb_err:
    device_close(ptp_info->dev);
dev_err:
    free(ptp_info);
done:
    return ret;
}

static void gb_ptp_exit(unsigned int cport)
{
    device_close(ptp_info->dev);
    free(ptp_info);
}

static struct gb_operation_handler gb_ptp_handlers[] = {
    GB_HANDLER(GB_PTP_TYPE_PROTOCOL_VERSION, gb_ptp_protocol_version),
    GB_HANDLER(GB_PTP_TYPE_GET_FUNCTIONALITY, gp_ptp_get_functionality),
    GB_HANDLER(GB_PTP_TYPE_SET_CURRENT_FLOW, gb_ptp_set_current_flow),
    GB_HANDLER(GB_PTP_TYPE_SET_MAX_INPUT_CURRENT, gb_ptp_set_max_input_current),
    GB_HANDLER(GB_PTP_TYPE_EXT_POWER_PRESENT, gb_ptp_ext_power_present),
    GB_HANDLER(GB_PTP_TYPE_POWER_REQUIRED, gb_ptp_power_required),
};

static struct gb_driver gb_ptp_driver = {
    .init = gb_ptp_init,
    .exit = gb_ptp_exit,
    .connected = gb_ptp_connected,
    .disconnected = gb_ptp_disconnected,
    .op_handlers = gb_ptp_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_ptp_handlers),
};

void gb_ptp_register(int cport)
{
    gb_register_driver(cport, &gb_ptp_driver);
}
