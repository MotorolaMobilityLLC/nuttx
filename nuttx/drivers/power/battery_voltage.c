/*
 * Copyright (C) 2016 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>

#include <nuttx/device.h>
#include <nuttx/device_battery_voltage.h>
#include <nuttx/power/battery_state.h>

#include "battery_voltage.h"

struct battery_voltage_info_s {
    struct device *dev;             /* device for voltage measurements */
    bool available;                 /* are voltage measurements available ? */
    enum batt_voltage_e zone;       /* current zone */
};

static struct battery_voltage_info_s *g_info; /* for battery_voltage_stop() */

/* Helper function */
static void battery_voltage_set_zone(struct battery_voltage_info_s *info,
                                  enum batt_voltage_e new_zone)
{
    int min, max;              /* new voltage limits */

    info->zone = new_zone;

    /* Determine new battery state voltage and limits */
    switch (new_zone) {
    case BATTERY_QUICK_CHARGE:
        min = INT_MIN;
        max = CONFIG_BATTERY_VOLTAGE_QUICK_TO_SLOW;
        break;
    case BATTERY_SLOW_CHARGE:
        min = CONFIG_BATTERY_VOLTAGE_QUICK_TO_SLOW - CONFIG_BATTERY_VOLTAGE_SLOW_TO_QUICK_HYST;
        max = INT_MAX;
        break;
     default:
        /* Should never be here! */
        return;
    }

    battery_state_set_voltage(new_zone);
    device_batt_voltage_set_limits(info->dev, min, max);
}

/* Device driver callback */
static void battery_voltage_limits_cb(void *arg, bool min)
{
    struct battery_voltage_info_s *info = arg;
    enum batt_voltage_e new_zone = BATTERY_QUICK_CHARGE;

    /* Transition to a new voltage zone */
    switch (info->zone) {
    case BATTERY_QUICK_CHARGE:
        new_zone = BATTERY_SLOW_CHARGE;
        break;
    case BATTERY_SLOW_CHARGE:
        new_zone = BATTERY_QUICK_CHARGE;
        break;
    default:
        /* Should never be here !*/
        break;
    }

    battery_voltage_set_zone(info, new_zone);
}

/* Device driver callback */
static void battery_voltage_available_cb(void *arg, bool available)
{
    struct battery_voltage_info_s *info = arg;
    int voltage;

    if (!info->available && available) {
        /*
         * Determine initial zone based on current voltage.
         */
        info->available = true;

        if (device_batt_voltage_get_voltage(info->dev, &voltage))
            battery_voltage_set_zone(info, BATTERY_SLOW_CHARGE);
        else if (voltage <= CONFIG_BATTERY_VOLTAGE_QUICK_TO_SLOW )
            battery_voltage_set_zone(info, BATTERY_QUICK_CHARGE);
        else
            battery_voltage_set_zone(info, BATTERY_SLOW_CHARGE);
    } else if (info->available && !available) {
        info->available = false;
        battery_state_set_voltage(BATTERY_SLOW_CHARGE);
    }
}

int battery_voltage_start(void)
{
    struct battery_voltage_info_s *info;
    int ret;
    battery_state_set_voltage(BATTERY_SLOW_CHARGE);

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    info->dev = device_open(DEVICE_TYPE_BATTERY_VOLTAGE_HW, 0);
    if (!info->dev) {
        dbg("failed to open battery voltage device\n");
        ret = EIO;
        goto error;
    }

    ret = device_batt_voltage_register_limits_cb(info->dev, battery_voltage_limits_cb, info);
    if (ret) {
        dbg("failed to register battery voltage limits callback\n");
        goto error;
    }
    ret = device_batt_voltage_register_available_cb(info->dev, battery_voltage_available_cb,
                                               info);
    if (ret) {
        dbg("failed to register battery voltage available callback\n");
        goto error;
    }

    g_info = info;

    return 0;

error:
    if (info->dev)
        device_close(info->dev);
    free(info);
    return ret;
}

void battery_voltage_stop(void)
{
    if (g_info && g_info->dev)
        device_close(g_info->dev);
    free(g_info);
}
