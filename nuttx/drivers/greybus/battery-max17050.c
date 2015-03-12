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

#include <nuttx/i2c.h>
#include <nuttx/greybus/greybus.h>

#include "battery-gb.h"

#define GB_BATTERY_VERSION_MAJOR     0
#define GB_BATTERY_VERSION_MINOR     1

#define MAX17050_I2C_ADDR         0x36

#define MAX17050_REG_REP_SOC      0x06
#define MAX17050_REG_TEMP         0x08
#define MAX17050_REG_VCELL        0x09
#define MAX17050_REG_CURRENT      0x0A
#define MAX17050_REG_FULL_CAP     0x10
#define MAX17050_REG_MAX_VOLT     0x1B
#define MAX17050_REG_DEV_NAME     0x21

#define MAX17050_IC_VERSION     0x00AC

#define MAX17050_SNS_RESISTOR    10000

struct i2c_dev_s *i2c_dev;

static int gb_battery_read_reg(uint16_t reg, uint8_t *buf, int size)
{
    struct i2c_msg_s msgs[2];

    msgs[0].addr = MAX17050_I2C_ADDR;
    msgs[0].flags = 0;
    msgs[0].length = sizeof(reg);
    msgs[0].buffer = (uint8_t *)&reg;

    msgs[1].addr = MAX17050_I2C_ADDR;
    msgs[1].flags = I2C_M_READ;
    msgs[1].length = size;
    msgs[1].buffer = buf;

    return I2C_TRANSFER(i2c_dev, msgs, 2);
}

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

    // TODO: Better to hard-code something here, or just return unknown?
    response->technology = GB_BATTERY_TECH_UNKNOWN;
    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_status(struct gb_operation *operation)
{
    struct gb_battery_status_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    // TODO: implement when charging supported
    response->status = GB_BATTERY_STATUS_UNKNOWN;
    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_max_voltage(struct gb_operation *operation)
{
    struct gb_battery_max_voltage_response *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_MAX_VOLT, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->max_voltage = reg_val >> 8;
    response->max_voltage *= 20000; /* Units of LSB = 20mV */

    lowsyslog("%s: %d mV\n", __func__, response->max_voltage);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_percent_capacity(struct gb_operation *operation)
{
    struct gb_battery_percent_capacity_response *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_REP_SOC, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->capacity = reg_val >> 8;

    lowsyslog("%s: %d percent\n", __func__, response->capacity);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_temperature(struct gb_operation *operation)
{
    struct gb_battery_temperature_response *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_TEMP, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->temperature = reg_val;
    /* The value is signed. */
    if (response->temperature & 0x8000) {
        response->temperature = (0x7fff & ~response->temperature) + 1;
        response->temperature *= -1;
    }

    /* The value is converted into deci-centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    response->temperature = response->temperature * 10 / 256;

    lowsyslog("%s: %d\n", __func__, response->temperature);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_voltage(struct gb_operation *operation)
{
    struct gb_battery_voltage_rsp *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_VCELL, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->voltage = reg_val * 625 / 8;

    lowsyslog("%s: %d mV\n", __func__, response->voltage);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_current(struct gb_operation *operation)
{
    struct gb_battery_current_rsp *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_CURRENT, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->current = reg_val;
    if (response->current & 0x8000) {
        /* Negative */
        response->current = ~response->current & 0x7fff;
        response->current++;
        response->current *= -1;
    }
    response->current *= 1562500 / MAX17050_SNS_RESISTOR;

    lowsyslog("%s: %d uA\n", __func__, response->current);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_capacity(struct gb_operation *operation)
{
    struct gb_battery_capacity_rsp *response;
    uint16_t reg_val;
    int ret;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    ret = gb_battery_read_reg(MAX17050_REG_FULL_CAP, (uint8_t *)&reg_val, sizeof(reg_val));
    if (ret)
        return GB_OP_UNKNOWN_ERROR;

    response->capacity = reg_val * 1000 / 2;

    lowsyslog("%s: %d mAh\n", __func__, response->capacity);

    return GB_OP_SUCCESS;
}

static uint8_t gb_battery_shutdown_temperature(struct gb_operation *operation)
{
    struct gb_battery_shutdown_temp_rsp *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    // TODO: return a better value?
    response->temperature = 0;
    return GB_OP_SUCCESS;
}

static int gb_battery_init(unsigned int cport)
{
    uint16_t version;
    int ret;

    i2c_dev = up_i2cinitialize(CONFIG_GREYBUS_BATTERY_MAX17050_I2C_BUS);
    if (!i2c_dev)
        return -ENODEV;

    ret = gb_battery_read_reg(MAX17050_REG_DEV_NAME, (uint8_t *)&version, sizeof(version));
    if (ret)
        return -ENODEV;

    if (version != MAX17050_IC_VERSION)
        return -EINVAL;

    // TODO: Initialize IC registers as needed

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
    GB_HANDLER(GB_BATTERY_TYPE_SHUTDOWN_TEMPERATURE, gb_battery_shutdown_temperature),
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

