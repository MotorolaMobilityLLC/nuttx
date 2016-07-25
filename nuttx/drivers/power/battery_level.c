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
#include <nuttx/device_battery_level.h>
#include <nuttx/power/battery_state.h>

#include "battery_level.h"

struct battery_level_info_s {
    struct device *dev;         /* device for battery capacity measurements */
    bool available;             /* are measurements available ? */
    enum batt_level_e level;    /* current level */
};

static struct battery_level_info_s *g_info; /* for battery_level_stop() */


/* Helper function */
static void battery_level_set(struct battery_level_info_s *info,
                              enum batt_level_e level)
{
    int min, max; /* new capacity limits */

    /* Determine new capacity limits */
    switch (level) {
    case BATTERY_LEVEL_EMPTY:
        min = INT_MIN;
        max = DEVICE_BATTERY_LEVEL_EMPTY + CONFIG_BATTERY_LEVEL_EMPTY_HYST;
        break;
    case BATTERY_LEVEL_LOW:
        min = DEVICE_BATTERY_LEVEL_EMPTY;
        max = CONFIG_BATTERY_LEVEL_LOW + CONFIG_BATTERY_LEVEL_LOW_HYST;
        break;
    case BATTERY_LEVEL_NORMAL:
        min = CONFIG_BATTERY_LEVEL_LOW;
        max = DEVICE_BATTERY_LEVEL_FULL;
        break;
    case BATTERY_LEVEL_FULL:
        min = DEVICE_BATTERY_LEVEL_FULL - CONFIG_BATTERY_LEVEL_FULL_HYST;
        max = INT_MAX;
        break;
     default:
        /* Should never be here! */
        return;
    }

    info->level = level;
    battery_state_set_level(level);
    device_batt_level_set_limits(info->dev, min, max);
}

/* Device driver callback */
static void battery_limits_cb(void *arg, bool min)
{
    struct battery_level_info_s *info = arg;
    enum batt_level_e new_level;

    /* Transition to a new level */
    switch (info->level) {
    case BATTERY_LEVEL_EMPTY:
        if (min) /* no change */
            return;
        new_level = BATTERY_LEVEL_LOW;
        break;
    case BATTERY_LEVEL_LOW :
        new_level = min ?  BATTERY_LEVEL_EMPTY : BATTERY_LEVEL_NORMAL;
        break;
    case BATTERY_LEVEL_NORMAL:
        new_level = min ? BATTERY_LEVEL_LOW : BATTERY_LEVEL_FULL;
        break;
    case BATTERY_LEVEL_FULL:
        if (!min) /* no change */
            return;
        new_level = BATTERY_LEVEL_NORMAL;
        break;
    default:
        /* Should never be here !*/
        return;
    }

    battery_level_set(info, new_level);
}

/* Device driver callback */
static void battery_available_cb(void *arg, bool available)
{
    struct battery_level_info_s *info = arg;
    int capacity;

    if (!info->available && available) {
        /*
         * Determine initial level based on current capacity. Device driver
         * limit callbacks will be used to transition between levels.
         */
        info->available = true;
        if (device_batt_level_get_capacity(info->dev, &capacity))
            battery_level_set(info, BATTERY_LEVEL_EMPTY);
        else if (capacity <= DEVICE_BATTERY_LEVEL_EMPTY)
            battery_level_set(info, BATTERY_LEVEL_EMPTY);
        else if (capacity <= CONFIG_BATTERY_LEVEL_LOW)
            battery_level_set(info, BATTERY_LEVEL_LOW);
        else if (capacity < DEVICE_BATTERY_LEVEL_FULL)
            battery_level_set(info, BATTERY_LEVEL_NORMAL);
        else
            battery_level_set(info, BATTERY_LEVEL_FULL);
    } else if (info->available && !available) {
        info->available = false;
        battery_state_set_level(BATTERY_LEVEL_EMPTY);
    }
}

int battery_level_start(void)
{
    struct battery_level_info_s *info;
    int ret;

    battery_state_set_level(BATTERY_LEVEL_EMPTY);

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    info->dev = device_open(DEVICE_TYPE_BATTERY_LEVEL_HW, 0);
    if (!info->dev) {
        dbg("failed to open battery level device\n");
        ret = EIO;
        goto error;
    }

    ret = device_batt_level_register_limits_cb(info->dev, battery_limits_cb, info);
    if (ret) {
        dbg("failed to register battery level limits callback\n");
        goto error;
    }

    ret = device_batt_level_register_available_cb(info->dev, battery_available_cb,
                                               info);
    if (ret) {
        dbg("failed to register battery level available callback\n");
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

void battery_level_stop(void)
{
    if (g_info && g_info->dev)
        device_close(g_info->dev);
    free(g_info);
}
