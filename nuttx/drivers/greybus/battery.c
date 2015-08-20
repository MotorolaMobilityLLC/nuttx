/*
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
 *
 * Author: Benoit Cousson <bcousson@baylibre.com>
 * Author: Winnie Wang <wang_winnie@projectara.com>
 */

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <arch/byteorder.h>
#include <nuttx/greybus/greybus.h>
#include <apps/greybus-utils/utils.h>

#include "battery-gb.h"
#include <nuttx/device_battery.h>

/* Version of the Greybus battery protocol we support */
#define GB_BATTERY_VERSION_MAJOR    0x00
#define GB_BATTERY_VERSION_MINOR    0x01

/* Allocated in initial function for internal data store */
static struct device *batt_dev = NULL;

/**
 * @brief Get this firmware supported BATTERY protocol vsersion.
 *
 * This function is called when battery operates initialize in Greybus kernel.
 *
 * @param operation Pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
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

/**
 * @brief Get the battery adapter control technology type.
 *
 * @param operation The pointer to structure of gb_operation.
 *
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_battery_technology(struct gb_operation *operation)
{
    struct gb_battery_technology_response *response;
    uint32_t tech = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_technology(batt_dev, &tech);

    response->technology = cpu_to_le32(tech);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery status.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_status(struct gb_operation *operation)
{
    struct gb_battery_status_response *response;
    uint16_t status = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_status(batt_dev, &status);

    response->status = cpu_to_le16(status);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery maximum voltage in microvolt .
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_max_voltage(struct gb_operation *operation)
{
    struct gb_battery_max_voltage_response *response;
    uint32_t voltage = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_max_voltage(batt_dev, &voltage);

    response->voltage = cpu_to_le32(voltage);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery percentage of total capacity.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_percent_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_response *response;
    uint32_t capacity = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_percent_capacity(batt_dev, &capacity);

    response->capacity = cpu_to_le32(capacity);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery temperature in 0.1 Celsius.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_temperature(struct gb_operation *operation)
{
    struct gb_battery_temperature_response *response;
    int temp = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_temperature(batt_dev, &temp);

    response->temperature = cpu_to_le32(temp);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery current voltage in microvolt .
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_voltage(struct gb_operation *operation)
{
    struct gb_battery_voltage_response *response;
    uint32_t voltage = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_voltage(batt_dev, &voltage);

    response->voltage = cpu_to_le32(voltage);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery current in microampere.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_current(struct gb_operation *operation)
{
    struct gb_battery_current_response *response;
    int current = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_current(batt_dev, &current);

    response->current = cpu_to_le32(current);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery total capacity in mAh.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_response *response;
    uint32_t capacity = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_total_capacity(batt_dev, &capacity);

    response->capacity = cpu_to_le32(capacity);

    return gb_errno_to_op_result(ret);
}

/**
* @brief Get battery shutdown temperature in 0.1 Celsius.
*
* @param operation The pointer to structure of gb_operation.
*
* @return GB_OP_SUCCESS on success, error code on failure.
*/
static uint8_t gb_battery_shutdown_temp(struct gb_operation *operation)
{
    struct gb_battery_shutdown_temperature_response *response;
    int temp = 0;
    int ret = 0;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = device_battery_shutdown_temp(batt_dev, &temp);

    response->temperature = cpu_to_le32(temp);

    return gb_errno_to_op_result(ret);
}

/**
 * @brief Greybus battery protocol initialize function
 *
 * @param cport CPort number
 *
 * @return GB_OP_SUCCESS on success, negative errno on error
 */
static int gb_battery_init(unsigned int cport)
{
    batt_dev = device_open(DEVICE_TYPE_BATTERY_DEVICE, 0);
    if (!batt_dev) {
        return -ENODEV;
    }

    return 0;
}

/**
 * @brief Greybus BATTERY protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_battery_exit(unsigned int cport)
{
    device_close(batt_dev);
    batt_dev = NULL;
}

/**
 * @brief Greybus BATTERY protocol operation handler
 */
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
    GB_HANDLER(GB_BATTERY_TYPE_SHUTDOWN_TEMP, gb_battery_shutdown_temp),
};

static struct gb_driver gb_battery_driver = {
    .init              = gb_battery_init,
    .exit              = gb_battery_exit,
    .op_handlers       = gb_battery_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_battery_handlers),
};

/**
 * @brief Register Greybus BATTERY protocol
 *
 * @param cport CPort number
 */
void gb_battery_register(int cport)
{
    gb_register_driver(cport, &gb_battery_driver);
}
