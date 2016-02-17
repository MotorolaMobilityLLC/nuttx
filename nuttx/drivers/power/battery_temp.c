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
#include <nuttx/device_battery_temp.h>
#include <nuttx/power/battery_state.h>

#include "battery_temp.h"

/* Battery temperature zones */
enum battery_temp_zone_e {
    BATTERY_COLD,
    BATTERY_COOL,
    BATTERY_NORMAL,
    BATTERY_WARM,
    BATTERY_HOT,
    BATTERY_COOL_DOWN
};

struct battery_temp_info_s {
    struct device *dev;             /* device for temp measurements */
    bool available;                 /* are temp measurements available ? */
    enum battery_temp_zone_e zone;  /* current zone */
};

static struct battery_temp_info_s *g_info; /* for battery_temp_stop() */

/* Helper function */
static void battery_temp_set_zone(struct battery_temp_info_s *info,
                                  enum battery_temp_zone_e new_zone)
{
    enum batt_temp_e state; /* new battery state temp */
    int min, max;           /* new temp limits */

    info->zone = new_zone;

    /* Determine new battery state temp and limits */
    switch (new_zone) {
    case BATTERY_COLD:
        state = BATTERY_TEMP_NO_CHARGING;
        min = INT_MIN;
        max = CONFIG_BATTERY_TEMP_COLD + CONFIG_BATTERY_TEMP_HYST;
        break;
    case BATTERY_COOL:
        state = BATTERY_TEMP_REDUCED_CHARGING;
        min = CONFIG_BATTERY_TEMP_COLD;
        max = CONFIG_BATTERY_TEMP_COOL + CONFIG_BATTERY_TEMP_HYST;
        break;
    case BATTERY_NORMAL:
        state = BATTERY_TEMP_NORMAL;
        min = CONFIG_BATTERY_TEMP_COOL;
        max = CONFIG_BATTERY_TEMP_WARM;
        break;
    case BATTERY_WARM:
        state = BATTERY_TEMP_REDUCED_CHARGING;
        min = CONFIG_BATTERY_TEMP_WARM - CONFIG_BATTERY_TEMP_HYST;
        max = CONFIG_BATTERY_TEMP_HOT;
        break;
    case BATTERY_HOT:
        state = BATTERY_TEMP_NO_CHARGING;
        min = CONFIG_BATTERY_TEMP_HOT - CONFIG_BATTERY_TEMP_HYST;
        max = CONFIG_BATTERY_TEMP_COOL_DOWN;
        break;
     case BATTERY_COOL_DOWN:
        state  = BATTERY_TEMP_COOL_DOWN;
        min = CONFIG_BATTERY_TEMP_COOL_DOWN - CONFIG_BATTERY_TEMP_HYST;
        max = INT_MAX;
        break;
     default:
        /* Should never be here! */
        return;
    }

    battery_state_set_temp(state);
    device_batt_temp_set_limits(info->dev, min, max);
}

/* Device driver callback */
static void battery_limits_cb(void *arg, bool min)
{
    struct battery_temp_info_s *info = arg;
    enum battery_temp_zone_e new_zone = BATTERY_NORMAL;

    /* Transition to a new temp zone */
    switch (info->zone) {
    case BATTERY_COLD:
        new_zone = BATTERY_COOL;
        break;
    case BATTERY_COOL:
        new_zone = min ? BATTERY_COLD : BATTERY_NORMAL;
        break;
    case BATTERY_NORMAL:
        new_zone = min ? BATTERY_COOL : BATTERY_WARM;
        break;
    case BATTERY_WARM:
        new_zone = min ? BATTERY_NORMAL : BATTERY_HOT;
        break;
    case BATTERY_HOT:
        new_zone = min ? BATTERY_WARM : BATTERY_COOL_DOWN;
        break;
    case BATTERY_COOL_DOWN:
        new_zone = BATTERY_HOT;
        break;
    default:
        /* Should never be here !*/
        break;
    }

    battery_temp_set_zone(info, new_zone);
}

/* Device driver callback */
static void battery_available_cb(void *arg, bool available)
{
    struct battery_temp_info_s *info = arg;
    int temperature;

    if (!info->available && available) {
        /*
         * Determine initial zone based on current temperature. Choose stricter
         * zone if temp is equal to the threshold. Device driver temp callbacks
         * will be used to transition between zones.
         */
        info->available = true;
        if (device_batt_temp_get_temperature(info->dev, &temperature))
            battery_temp_set_zone(info, BATTERY_COOL_DOWN);
        else if (temperature <= CONFIG_BATTERY_TEMP_COLD )
            battery_temp_set_zone(info, BATTERY_COLD);
        else if (temperature <= CONFIG_BATTERY_TEMP_COOL)
            battery_temp_set_zone(info, BATTERY_COOL);
        else if (temperature < CONFIG_BATTERY_TEMP_WARM)
            battery_temp_set_zone(info, BATTERY_NORMAL);
        else if (temperature < CONFIG_BATTERY_TEMP_HOT)
            battery_temp_set_zone(info, BATTERY_WARM);
        else if (temperature < CONFIG_BATTERY_TEMP_COOL_DOWN)
            battery_temp_set_zone(info, BATTERY_HOT);
        else
            battery_temp_set_zone(info, BATTERY_COOL_DOWN);
    } else if (info->available && !available) {
        info->available = false;
        battery_state_set_temp(BATTERY_TEMP_UNAVAILABLE);
    }
}

int battery_temp_start(void)
{
    struct battery_temp_info_s *info;
    int ret;

    battery_state_set_temp(BATTERY_TEMP_UNAVAILABLE);

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    info->dev = device_open(DEVICE_TYPE_BATTERY_TEMP_HW, 0);
    if (!info->dev) {
        dbg("failed to open battery temp device\n");
        ret = EIO;
        goto error;
    }

    ret = device_batt_temp_register_limits_cb(info->dev, battery_limits_cb, info);
    if (ret) {
        dbg("failed to register battery limits callback\n");
        goto error;
    }

    ret = device_batt_temp_register_available_cb(info->dev, battery_available_cb,
                                               info);
    if (ret) {
        dbg("failed to register battery available callback\n");
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

void battery_temp_stop(void)
{
    if (g_info && g_info->dev)
        device_close(g_info->dev);
    free(g_info);
}
