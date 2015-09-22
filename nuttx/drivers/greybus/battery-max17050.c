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
#include <nuttx/power/battery.h>

#include "battery-mods.h"

struct battery_dev_s *bdev;

static uint8_t to_gb_error(int ret)
{
    switch (ret) {
        case -EAGAIN:
            return GB_OP_RETRY;
        default:
            return GB_OP_UNKNOWN_ERROR;
    }
}

uint8_t gb_battery_driver_technology(__le32 *technology)
{
    // The MAX17050 is for lithium-ion batteries only
    *technology = GB_BATTERY_TECH_LION;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_status(__le16 *status)
{
    int ret;
    int value;

    ret = bdev->ops->state(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    // Convert to Greybus values
    switch (value) {
        case BATTERY_IDLE:
            *status = GB_BATTERY_STATUS_NOT_CHARGING;
            break;
        case BATTERY_FULL:
            *status = GB_BATTERY_STATUS_FULL;
            break;
        case BATTERY_CHARGING:
            *status = GB_BATTERY_STATUS_CHARGING;
            break;
        case BATTERY_DISCHARGING:
            *status = GB_BATTERY_STATUS_DISCHARGING;
            break;
        case BATTERY_UNKNOWN:
        default:
            *status = GB_BATTERY_STATUS_UNKNOWN;
            break;
    }

    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_max_voltage(__le32 *max_voltage)
{
    int ret;
    b16_t value;

    ret = bdev->ops->max_voltage(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *max_voltage = value;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_percent_capacity(__le32 *capacity)
{
    int ret;
    b16_t value;

    ret = bdev->ops->capacity(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *capacity = value;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_temperature(__le32 *temperature)
{
    int ret;
    b16_t value;

    ret = bdev->ops->temperature(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *temperature = value;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_voltage(__le32 *voltage)
{
    int ret;
    b16_t value;

    ret = bdev->ops->voltage(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *voltage = value;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_current(__le32 *current)
{
    int ret;
    b16_t value;

    ret = bdev->ops->current(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *current = value;
    return GB_OP_SUCCESS;
}

uint8_t gb_battery_driver_capacity(__le32 *capacity)
{
    int ret;
    b16_t value;

    ret = bdev->ops->full_capacity(bdev, &value);
    if (ret < 0)
        return to_gb_error(ret);

    *capacity = value;
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
    FAR struct i2c_dev_s *i2c;

    i2c = up_i2cinitialize(CONFIG_GREYBUS_BATTERY_MAX17050_I2C_BUS);
    if (!i2c)
        return -ENODEV;

    bdev = max17050_initialize(i2c, 400000);
    if (!bdev)
        return -ENODEV;

    return 0;
}
