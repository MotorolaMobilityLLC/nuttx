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

static int max17050_reg_read(uint16_t reg)
{
    uint16_t reg_val;
    int ret;

    ret = I2C_WRITEREAD(i2c_dev, (uint8_t *)&reg, sizeof(reg),
                        (uint8_t *)&reg_val, sizeof(reg_val));
    return ret ? ret : reg_val;
}

uint8_t gb_battery_driver_technology(__le32 *technology)
{
    // TODO: Better to hard-code something here, or just return unknown?
    *technology = GB_BATTERY_TECH_UNKNOWN;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_status(__le16 *status)
{
    // TODO: implement when charging supported
    *status = GB_BATTERY_STATUS_UNKNOWN;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_max_voltage(__le32 *max_voltage)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_MAX_VOLT);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *max_voltage = ret >> 8;
    *max_voltage *= 20000; /* Units of LSB = 20mV */

    lowsyslog("%s: %d mV\n", __func__, *max_voltage);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_percent_capacity(__le32 *capacity)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_REP_SOC);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *capacity = ret >> 8;

    lowsyslog("%s: %d percent\n", __func__, *capacity);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_temperature(__le32 *temperature)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_TEMP);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *temperature = ret;
    /* The value is signed. */
    if (*temperature & 0x8000) {
        *temperature = (0x7fff & ~*temperature) + 1;
        *temperature *= -1;
    }

    /* The value is converted into deci-centigrade scale */
    /* Units of LSB = 1 / 256 degree Celsius */
    *temperature = *temperature * 10 / 256;

    lowsyslog("%s: %d\n", __func__, *temperature);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_voltage(__le32 *voltage)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_VCELL);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *voltage = ret * 625 / 8;

    lowsyslog("%s: %d mV\n", __func__, *voltage);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_current(__le32 *current)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_CURRENT);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *current = ret;
    if (*current & 0x8000) {
        /* Negative */
        *current = ~*current & 0x7fff;
        *current += 1;
        *current *= -1;
    }
    *current *= 1562500 / MAX17050_SNS_RESISTOR;

    lowsyslog("%s: %d uA\n", __func__, *current);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_capacity(__le32 *capacity)
{
    int ret;

    ret = max17050_reg_read(MAX17050_REG_FULL_CAP);
    if (ret < 0)
        return GB_OP_UNKNOWN_ERROR;

    *capacity = ret * 1000 / 2;

    lowsyslog("%s: %d mAh\n", __func__, *capacity);

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_shutdown_temperature(__le32 *temperature)
{
    // TODO: return a better value?
    *temperature = 0;
    return GB_OP_SUCCESS;
}

int gb_battery_driver_init(void)
{
    int ret;

    i2c_dev = up_i2cinitialize(CONFIG_GREYBUS_BATTERY_MAX17050_I2C_BUS);
    if (!i2c_dev)
        return -ENODEV;

    ret = I2C_SETADDRESS(i2c_dev, MAX17050_I2C_ADDR, 7);
    if (ret)
        return ret;

    I2C_SETFREQUENCY(i2c_dev, 400000);

    ret = max17050_reg_read(MAX17050_REG_DEV_NAME);
    if (ret < 0)
        return ret;

    if (ret != MAX17050_IC_VERSION)
        return -EINVAL;

    // TODO: Initialize IC registers as needed

    return 0;
}
