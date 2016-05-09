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
#include <string.h>
#include <unistd.h>

#include <nuttx/device.h>
#include <nuttx/device_lights.h>
#include <nuttx/util.h>

#include <nuttx/mhb/mhb_dsi_display.h>

static struct dcs_backlight
{
    struct device *self;
    struct light_info *light_info_array;
} g_backlight;

static struct channel_info dcs_backlight_channel_info[] = {
    {
        .id = 0,
        .cfg = {
            .max_brightness = 255,
            .flags = 0,
            .color = 0,
            .color_name = "",
            .mode = LIGHTS_CHANNEL_MODE_VENDOR,
            .mode_name = "backlight",
        },
    },
};

static struct light_info dcs_light_info[] = {
    {
        .id = 0,
        .cfg = {
            .channel_count = 1,
            .name = "mod_light0",
        },
        .channels = dcs_backlight_channel_info,
    },
};

#define DCS_NUM_LIGHTS (ARRAY_SIZE(dcs_light_info))

static int _dcs_backlight_set_brightness(struct dcs_backlight *backlight,
    uint8_t brightness)
{
    vdbg("%d\n", brightness);
    return mhb_dsi_display_set_brightness(brightness);
}

/**
 * @brief Get number of lights
 *
 * @param dev pointer to structure of device data
 * @param lights pointer to the number of lights
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_get_lights(struct device *dev, uint8_t *lights)
{
    *lights = DCS_NUM_LIGHTS;

    return 0;
}

/**
 * @brief Get light config
 *
 * @param dev pointer to structure of device data
 * @param id light ID number
 * @param cfg pointer to the light_config struct
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_get_light_config(struct device *dev, uint8_t id,
                                          struct light_config *cfg)
{
    struct dcs_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    info = backlight->light_info_array;

    if (id < DCS_NUM_LIGHTS)
        *cfg = info[id].cfg;
    else
        ret = -EINVAL;

    return ret;
}

/**
 * @brief Get channel config
 *
 * @param dev pointer to structure of device data
 * @param light_id light ID number
 * @param channel_id channel ID number
 * @param cfg pointer to the channel_config struct
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_get_channel_config(struct device *dev,
                                            uint8_t light_id,
                                            uint8_t channel_id,
                                            struct channel_config *cfg)
{
    struct dcs_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    info = backlight->light_info_array;

    if (light_id < DCS_NUM_LIGHTS &&
        channel_id < info[light_id].cfg.channel_count)
        *cfg = info[light_id].channels[channel_id].cfg;
    else
        ret = -EINVAL;

    return ret;
}

/**
 * @brief Set brightness
 *
 * @param dev pointer to structure of device data
 * @param light_id light ID number
 * @param channel_id channel ID number
 * @param brightness backlight brightness to set
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_set_brightness(struct device *dev,
                                        uint8_t light_id,
                                        uint8_t channel_id,
                                        uint8_t brightness)
{
    struct dcs_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    vdbg("light_id=%d channel_id=%d brightness=%d\n",
        light_id, channel_id, brightness);

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    info = backlight->light_info_array;

    if (light_id >= DCS_NUM_LIGHTS ||
        channel_id >= info[light_id].cfg.channel_count) {
        return -EINVAL;
    }

    ret = _dcs_backlight_set_brightness(backlight, brightness);
    if (!ret) {
        /* Success, save value */
        info[light_id].channels[channel_id].brightness = brightness;
    }

    return ret;
}

/**
 * @brief Register callback
 *
 * @param dev pointer to structure of device data
 * @param callback pointer to event handler function
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_register_callback(struct device *dev,
                                           lights_event_callback callback)
{
    return 0;
}

/**
 * @brief Unregister callback
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int dcs_backlight_unregister_callback(struct device *dev)
{
    return 0;
}


/**
 * @brief open device
 *
 * @param dev pointer to the UART device structure
 * dev->type must be DEVICE_TYPE_UART_HW.
 * dev->id will be mapped to /dev/ttySx (where x is the ID between 0 and 9).
 * @return Address of structure representing device or NULL on failure
 */
static int dcs_backlight_dev_open(struct device *dev)
{
    struct dcs_backlight *backlight;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    return 0;
}

/**
 * @brief Close device
 * @param dev pointer to the UART device structure
 */
static void dcs_backlight_dev_close(struct device *dev)
{
    struct dcs_backlight *backlight = device_get_private(dev);
    if (!backlight) {
        return;
    }
}

/**
 * @brief probe device
 * @param dev pointer to the device structure
 * @return 0 for success, -errno for failures.
 */
static int dcs_backlight_probe(struct device *dev)
{
    struct dcs_backlight *backlight = &g_backlight;

    backlight->self = dev;

    /* add the light info array to the private data */
    backlight->light_info_array = dcs_light_info;

    device_set_private(dev, backlight);
    return 0;
}

/**
 * @brief remove device
 * @param dev pointer to the UART device structure
 */
static void dcs_backlight_remove(struct device *dev)
{
    struct dcs_backlight *backlight = device_get_private(dev);
    if (backlight) {
        device_set_private(dev, NULL);
        memset(backlight, 0, sizeof(*backlight));
    }
}

const static struct device_lights_type_ops dcs_backlight_ops = {
    .get_lights          = dcs_backlight_get_lights,
    .get_light_config    = dcs_backlight_get_light_config,
    .get_channel_config  = dcs_backlight_get_channel_config,
    .set_brightness      = dcs_backlight_set_brightness,
    .register_callback   = dcs_backlight_register_callback,
    .unregister_callback = dcs_backlight_unregister_callback,
};

const static struct device_driver_ops dcs_backlight_driver_ops = {
    .probe    = dcs_backlight_probe,
    .remove   = dcs_backlight_remove,
    .open     = dcs_backlight_dev_open,
    .close    = dcs_backlight_dev_close,
    .type_ops = (struct device_light_type_ops *)&dcs_backlight_ops,
};

const struct device_driver dcs_backlight_driver = {
    .type = DEVICE_TYPE_LIGHTS_HW,
    .name = "dcs_backlight",
    .desc = "DCS Backlight",
    .ops  = (struct device_driver_ops *)&dcs_backlight_driver_ops,
};
