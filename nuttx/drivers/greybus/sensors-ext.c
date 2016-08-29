/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arch/byteorder.h>
#include <nuttx/device.h>
#include <nuttx/device_sensors_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/list.h>
#include <nuttx/util.h>

#include "sensors-ext-gb.h"

#define GB_SENSORS_EXT_VERSION_MAJOR 0
#define GB_SENSORS_EXT_VERSION_MINOR 2


struct gb_sensors_ext_info {
    struct list_head node;
    struct device *dev;
    uint8_t sensor_id;
};

static unsigned int sensors_ext_cport;
static struct list_head s_device_list;

static struct gb_sensors_ext_info *get_dev_from_id(uint8_t sensor_id)
{
    struct gb_sensors_ext_info *se_info;
    struct list_head *iter;

    list_foreach(&s_device_list, iter) {
        se_info = list_entry(iter, struct gb_sensors_ext_info, node);
        if (se_info->sensor_id == sensor_id) {
            return se_info;
        }
    }
    return NULL;
}

static int event_callback(uint8_t se_id, struct report_info_data *rinfo_data,
                            uint16_t payload_size)
{
    struct gb_operation *operation;
    struct gb_sensors_ext_report_data *request;

    operation = gb_operation_create(sensors_ext_cport, GB_SENSORS_EXT_TYPE_EVENT,
                                    (sizeof(*request) + payload_size));
    if (!operation) {
        gb_error("%s: gb_operation_create failed !\n",  __func__);
        return -ENOMEM;
    }

    request = gb_operation_get_request_payload(operation);

    request->num_sensors_reporting = rinfo_data->num_sensors_reporting;

    memcpy((void *)&request->event_report[0],
            (void *)&rinfo_data->reportinfo[0], payload_size);

    gb_operation_send_request(operation, NULL, false);
    gb_operation_destroy(operation);

    return 0;
}


static uint8_t gb_sensors_ext_protocol_version(struct gb_operation *operation)
{
    struct gb_sensors_ext_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SENSORS_EXT_VERSION_MAJOR;
    response->minor = GB_SENSORS_EXT_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_sensors_ext_sensor_count(struct gb_operation *operation)
{
    struct gb_sensors_ext_sensor_count_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->count = list_count(&s_device_list);
    gb_debug("sensor count %d\n", response->count);

    return GB_OP_SUCCESS;
}

static uint8_t gb_sensors_ext_get_sensor_info(struct gb_operation *operation)
{
    struct gb_sensors_ext_sensor_info_request *request;
    struct gb_sensors_ext_sensor_info_response *response;
    struct sensor_info *s_info;
    struct gb_sensors_ext_info *se_info;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    se_info = get_dev_from_id(request->id);
    if (!se_info) {
        gb_error("No sensor device with id: %d\n", request->id);
        return GB_OP_INVALID;
    }

    s_info = zalloc(sizeof(struct sensor_info));
    if(!s_info)
        return GB_OP_NO_MEMORY;

    ret = device_sensors_ext_get_sensor_info(se_info->dev,
                                              request->id, s_info);
    if (ret) {
        free(s_info);
        return gb_errno_to_op_result(ret);
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        free(s_info);
        return GB_OP_NO_MEMORY;
    }

    response->version = cpu_to_le32(s_info->version);
    response->type = cpu_to_le32(s_info->type);
    response->max_range = cpu_to_le32(s_info->max_range);
    response->resolution = cpu_to_le32(s_info->resolution);
    response->power = cpu_to_le32(s_info->power);
    response->min_delay = cpu_to_le32(s_info->min_delay);
    response->max_delay = cpu_to_le32(s_info->max_delay);
    response->fifo_rec = cpu_to_le32(s_info->fifo_rec);
    response->fifo_mec = cpu_to_le32(s_info->fifo_mec);
    response->flags = cpu_to_le32(s_info->flags);
    response->scale_int = cpu_to_le32(s_info->scale_int);
    response->scale_nano = cpu_to_le32(s_info->scale_nano);
    response->offset_int = cpu_to_le32(s_info->offset_int);
    response->offset_nano = cpu_to_le32(s_info->offset_nano);
    response->channels = s_info->channels;

    if (s_info->name_len) {
        if (s_info->name_len > sizeof(response->name))
            s_info->name_len = sizeof(response->name);

        memcpy(&response->name[0], &s_info->name[0], s_info->name_len);
    } else {
        memset(&response->name[0], 0, sizeof(response->name));
    }

    if (s_info->vendor_len) {
        if (s_info->vendor_len > sizeof(response->vendor))
            s_info->vendor_len = sizeof(response->vendor);

        memcpy(&response->vendor[0], &s_info->vendor[0], s_info->vendor_len);
    } else {
        memset(&response->vendor[0], 0, sizeof(response->vendor));
    }

    if (s_info->string_type_len) {
        if (s_info->string_type_len > sizeof(response->string_type))
            s_info->string_type_len = sizeof(response->string_type);

        memcpy(&response->string_type[0], &s_info->string_type[0],
            s_info->string_type_len);
    } else {
        memset(&response->string_type[0], 0, sizeof(response->string_type));
    }

    response->name_len = cpu_to_le16(s_info->name_len);
    response->vendor_len = cpu_to_le16(s_info->vendor_len);
    response->string_type_len = cpu_to_le16(s_info->string_type_len);

    free(s_info);

    return GB_OP_SUCCESS;
}

static uint8_t gb_sensors_ext_start_reporting(struct gb_operation *operation)
{
    struct gb_sensors_ext_start_reporting_request *request;
    struct gb_sensors_ext_info *se_info;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    se_info = get_dev_from_id(request->id);
    if (!se_info) {
        gb_error("No sensor device with id: %d\n", request->id);
        return GB_OP_INVALID;
    }

    ret = device_sensors_ext_start_reporting(se_info->dev, request->id,
                    request->sampling_period, request->max_report_latency);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    return GB_OP_SUCCESS;
}


static uint8_t gb_sensors_ext_flush(struct gb_operation *operation)
{
    struct gb_sensors_ext_flush_request *request;
    struct gb_sensors_ext_info *se_info;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    se_info = get_dev_from_id(request->id);
    if (!se_info) {
        gb_error("No sensor device with id: %d\n", request->id);
        return GB_OP_INVALID;
    }

    ret = device_sensors_ext_flush(se_info->dev, request->id);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_sensors_ext_stop_reporting(struct gb_operation *operation)
{
    struct gb_sensors_ext_stop_reporting_request *request;
    struct gb_sensors_ext_info *se_info;
    int ret = 0;

    request = gb_operation_get_request_payload(operation);

    gb_debug("%s() %d \n",  __func__, request->id);

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    se_info = get_dev_from_id(request->id);
    if (!se_info) {
        gb_error("No sensor device with id: %d\n", request->id);
        return GB_OP_INVALID;
    }

    ret = device_sensors_ext_stop_reporting(se_info->dev, request->id);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    return GB_OP_SUCCESS;
}

static void gb_sensors_ext_disconnected(unsigned int cport)
{
    struct gb_sensors_ext_info *se_info;
    struct list_head *iter;

    /* stop all the sensors */
    list_foreach(&s_device_list, iter) {
        se_info = list_entry(iter, struct gb_sensors_ext_info, node);
        device_sensors_ext_stop_reporting(se_info->dev, se_info->sensor_id);
    }
}

static int gb_sensors_ext_init(unsigned int cport)
{
    int ret = 0;
    int i;
    struct gb_sensors_ext_info *se_info;
    struct list_head *iter, *next;

    list_init(&s_device_list);

    sensors_ext_cport = cport;

    i = 0;
    do {
        se_info = zalloc(sizeof(*se_info));
        if (!se_info) {
            gb_error("%s(): failed to allocate memory\n", __func__);
            ret = -ENOMEM;
            goto err_close;
        }

        se_info->dev = device_open(DEVICE_TYPE_SENSORS_HW, i);
        if (!se_info->dev) {
            gb_debug("End of sensor devices %s:%d device!\n",
                        DEVICE_TYPE_SENSORS_HW, i);
        } else {
            se_info->sensor_id = i;
            list_add(&s_device_list, &se_info->node);
            ret = device_sensors_ext_register_callback(se_info->dev,
                                            se_info->sensor_id, event_callback);
            if (ret)
                goto err_close;
            i++;
        }
    } while(se_info->dev);

    free(se_info);

    if (i == 0) {
        ret = -ENODEV;
        goto err_out;
    }

    if (i != list_count(&s_device_list)) {
        gb_error("%s(): Sensor count mismatch i = %d, list = %d\n", __func__,
                i, list_count(&s_device_list));
        goto err_close;
    }

    return 0;

err_close:
    list_foreach_safe(&s_device_list, iter, next) {
        se_info = list_entry(iter, struct gb_sensors_ext_info, node);
        device_sensors_ext_register_callback(se_info->dev,
                                                se_info->sensor_id, NULL);
        device_close(se_info->dev);
        free(se_info);
        se_info = NULL;
    }
err_out:
    return ret;
}

static void gb_sensors_ext_exit(unsigned int cport)
{
    struct gb_sensors_ext_info *se_info;
    struct list_head *iter, *next;

    list_foreach_safe(&s_device_list, iter, next) {
        se_info = list_entry(iter, struct gb_sensors_ext_info, node);
        device_sensors_ext_register_callback(se_info->dev,
                                                se_info->sensor_id, NULL);
        device_close(se_info->dev);
        free(se_info);
        se_info = NULL;
    }
}

static struct gb_operation_handler gb_sensors_ext_handlers[] = {
    GB_HANDLER(GB_SENSORS_EXT_TYPE_PROTOCOL_VERSION, gb_sensors_ext_protocol_version),
    GB_HANDLER(GB_SENSORS_EXT_TYPE_SENSOR_COUNT, gb_sensors_ext_sensor_count),
    GB_HANDLER(GB_SENSORS_EXT_TYPE_SENSOR_INFO, gb_sensors_ext_get_sensor_info),
    GB_HANDLER(GB_SENSORS_EXT_TYPE_START_REPORTING, gb_sensors_ext_start_reporting),
    GB_HANDLER(GB_SENSORS_EXT_TYPE_FLUSH, gb_sensors_ext_flush),
    GB_HANDLER(GB_SENSORS_EXT_TYPE_STOP_REPORTING, gb_sensors_ext_stop_reporting),
};

static struct gb_driver gb_sensors_ext_driver = {
    .init = gb_sensors_ext_init,
    .exit = gb_sensors_ext_exit,
    .disconnected = gb_sensors_ext_disconnected,
    .op_handlers = gb_sensors_ext_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_sensors_ext_handlers),
};

void gb_sensors_ext_register(int cport)
{
    gb_register_driver(cport, &gb_sensors_ext_driver);
}
