/*
 * Copyright (c) 2015 Google, Inc.
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

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_lights.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

/**
 * @brief lights device state
 */
enum lights_state {
    LIGHTS_STATE_INVALID,
    LIGHTS_STATE_OPEN,
    LIGHTS_STATE_CLOSED,
};

#define COLOR_WHITE     0x00000000
#define COLOR_RED       0x00FF0000
#define COLOR_GREEN     0x0000FF00
#define COLOR_BLUE      0x000000FF
#define COLOR_YELLOW    0x00FFFF00

/* [FAKE] A fake light count */
#define LIGHTS_COUNT    3
#define CHANNEL_COUNT   4

/**
 * @brief private lights device information
 */
struct lights_info {
    struct device           *dev;
    enum lights_state       state;
    uint8_t                 lights_count;
    struct light_info       *light_attri[LIGHTS_COUNT];
    lights_event_callback   event_callback;
};

/* [FAKE] Fake channel data of lights - Beginning */
static const struct channel_info channel_rgb = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = (LIGHTS_CHANNEL_FLAG_MULTICOLOR |
                                   LIGHTS_CHANNEL_FLAG_BLINK |
                                   LIGHTS_CHANNEL_FLAG_FADER),
                .color          = COLOR_RED,
                .color_name     = "rgb",
                .mode           = LIGHTS_CHANNEL_MODE_VENDOR,
                .mode_name      = "vendor_notification",
              },
};

static const struct channel_info channel_green = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = LIGHTS_CHANNEL_FLAG_BLINK,
                .color          = COLOR_GREEN,
                .color_name     = "green",
              },
};

static const struct channel_info channel_yellow = {
    .cfg    = {
                .max_brightness = 1,
                .flags          = 0,
                .color          = COLOR_YELLOW,
                .color_name     = "yellow",
              },
};

static const struct channel_info channel_red = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = 0,
                .color          = COLOR_RED,
                .color_name     = "red",
              },
};

static const struct channel_info channel_blue = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = 0,
                .color          = COLOR_BLUE,
                .color_name     = "blue",
              },
};

static const struct channel_info channel_flash = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = 0,
                .color_name     = "white",
                .mode           = LIGHTS_CHANNEL_MODE_FLASH,
                .mode_name      = "flash",
              },
    .fcfg   = {
                /* example values taken from as3645a */
                .intensity_min_uA   = 200000,
                .intensity_max_uA   = 4000000,
                .intensity_step_uA  = 20000,
                .timeout_min_us     = 100000,
                .timeout_max_us     = 800000,
                .timeout_step_us    = 50000,
              },
};

static const struct channel_info channel_indicator = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = 0,
                .color_name     = "red",
                .color          = COLOR_RED,
                .mode           = LIGHTS_CHANNEL_MODE_INDICATOR,
                .mode_name      = "indicator",
              },
    .fcfg   = {
                /* example values taken from as3645a */
                .intensity_min_uA   = 0,
                .intensity_max_uA   = 10000,
                .intensity_step_uA  = 2500,
              },
};

static const struct channel_info channel_torch = {
    .cfg    = {
                .max_brightness = 255,
                .flags          = 0,
                .color_name     = "white",
                .color          = COLOR_WHITE,
                .mode           = LIGHTS_CHANNEL_MODE_TORCH,
                .mode_name      = "torch",
              },
    .fcfg   = {
                /* example values taken from as3645a */
                .intensity_min_uA   = 20000,
                .intensity_max_uA   = 160000,
                .intensity_step_uA  = 20000,
              },
};
/* [FAKE] Fake channel data of lights - End */

/**
 * @brief Initialize channel configuration
 *
 * This function is called from the lights_init().
 * It initializes a non light type channel configuration.
 *
 * @param light pointer to a private light devices information
 */
static int light_example_channels_init(struct light_info *light)
{
    light->cfg.channel_count = CHANNEL_COUNT;

    light->channels = zalloc(light->cfg.channel_count *
                             sizeof(struct channel_info));
    if (light->channels == NULL) {
        return -ENOMEM;
    }

    /* channel 0 */
    light->channels[0] = channel_rgb;

    /* channel 1 */
    light->channels[1] = channel_green;
    light->channels[1].cfg.mode = LIGHTS_CHANNEL_MODE_BATTERY;
    snprintf(light->channels[1].cfg.mode_name,
             sizeof(light->channels[1].cfg.mode_name), "battery");

    /* channel 2 */
    light->channels[2] = channel_yellow;
    light->channels[2].cfg.mode = LIGHTS_CHANNEL_MODE_BATTERY;
    snprintf(light->channels[2].cfg.mode_name,
             sizeof(light->channels[2].cfg.mode_name), "battery");

    /* channel 3 */
    light->channels[3] = channel_red;
    light->channels[3].cfg.mode = LIGHTS_CHANNEL_MODE_BATTERY;
    snprintf(light->channels[3].cfg.mode_name,
             sizeof(light->channels[3].cfg.mode_name), "battery");

    return 0;
}

/**
 * @brief Initialize channel with light type.
 *
 * This function is called from the lights_init(). It initializes a light
 * type channel configuration.
 *
 * @param light pointer to a private Lights device information
 */
static int light_example_channels_flash_init(struct light_info *light)
{
    light->cfg.channel_count = CHANNEL_COUNT;

    light->channels = zalloc(light->cfg.channel_count *
                             sizeof(struct channel_info));
    if (light->channels == NULL) {
        return -ENOMEM;
    }

    /* channel 0 */
    light->channels[0] = channel_flash;

    /* channel 1 */
    light->channels[1] = channel_green;
    light->channels[1].cfg.mode = LIGHTS_CHANNEL_MODE_NOTIFICATION;
    snprintf(light->channels[1].cfg.mode_name,
             sizeof(light->channels[1].cfg.mode_name), "notification");

    /* channel 2 */
    light->channels[2] = channel_indicator;

    /* channel 3 */
    light->channels[3] = channel_torch;

    return 0;
}

/**
 * @brief Initialize light device
 *
 * Initialize light device depends on light type.
 *
 * @param id the ID of specific light
 * @return light pointer to a private light devices information
 */
static struct light_info *lights_init(uint8_t id)
{
    struct light_info *light;

    light = zalloc(sizeof(*light));
    if (light ==  NULL) {
        return NULL;
    }

    light->id = id;
    snprintf(light->cfg.name, sizeof(light->cfg.name), "ara_light%d", id);

    switch (id) {
    case 0:
        if (light_example_channels_init(light)) {
            goto free_light;
        }
        break;
    case 1:
    default:
        if (light_example_channels_flash_init(light)) {
            goto free_light;
        }
        break;
    }

    return light;

free_light:
    free(light);
    light = NULL;

    return light;
}

/**
 * @brief Free resource of light devices.
 *
 * This function is called when the caller no longer using this driver.
 * It should release light device resources in the close() function.
 *
 * @param light pointer to a private Lights device information
 */
static void lights_free(struct light_info *light)
{
    if(light) {
       if (light->channels) {
           free(light->channels);
           light->channels = NULL;
       }
       free(light);
       light = NULL;
    }
}

/**
 * @brief Check if channel is valid.
 *
 * This function check if specified channel is valid.
 *
 * @param info pointer to a private lights device information
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @return true for the valid channel.
 */
static bool is_channel_valid(struct lights_info *info, uint8_t light_id,
                             uint8_t channel_id)
{
    if (light_id < info->lights_count) {
        if (channel_id < info->light_attri[light_id]->cfg.channel_count) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Check if channel supports flash type.
 *
 * This function check if channel supports flash type.
 *
 * @param info pointer to a private Lights device information
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @return 0 on supporting flash type.
 */
static bool is_channel_flash(struct lights_info *info, uint8_t light_id,
                             uint8_t channel_id)
{
    struct channel_info *channel;

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    channel = &(info->light_attri[light_id]->channels[channel_id]);

    return (channel->cfg.mode & (LIGHTS_CHANNEL_MODE_FLASH |
              LIGHTS_CHANNEL_MODE_TORCH | LIGHTS_CHANNEL_MODE_INDICATOR));
}

/**
 * @brief Get the flash light fault
 *
 * This function get flash fault of specific channel.
 * Note that is function is only called when the channel type is flash.
 * i.e. the mode of channel_config is LIGHTS_CHANNEL_MODE_FLASH,
 * LIGHTS_CHANNEL_MODE_TORCH or LIGHTS_CHANNEL_MODE_INDICATOR.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param fault pointer to the variable of flash fault
 *
 * @return 0: Success, error code on failure
*/
static int lights_get_flash_fault(struct device *dev, uint8_t light_id,
                                  uint8_t channel_id, uint32_t *fault)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !fault) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    if (!is_channel_flash(info, light_id, channel_id)) {
        return -EINVAL;
    }

    *fault = LIGHTS_FLASH_FAULT_INDICATOR;   /* return hardcode now */

    return 0;
}

/**
 * @brief Configure flash light timeout
 *
 * This function set the flash timeout configuration to specific channel.
 * Note that is function is only called when the channel type is flash.
 * i.e. the mode of channel_config is LIGHTS_CHANNEL_MODE_FLASH or
 * LIGHTS_CHANNEL_MODE_TORCH or LIGHTS_CHANNEL_MODE_INDICATOR.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param timeout flash light timeout to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_flash_timeout(struct device *dev, uint8_t light_id,
                                    uint8_t channel_id, uint32_t timeout)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    if (!is_channel_flash(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].timeout = timeout;

    return 0;
}

/**
 * @brief Configure flash light strobe
 *
 * This function set flash strobe to specific channel.
 * Note that this function is only called when the channel type is flash.
 * i.e. the mode of channel_config is LIGHTS_CHANNEL_MODE_FLASH or
 * LIGHTS_CHANNEL_MODE_TORCH or LIGHTS_CHANNEL_MODE_INDICATOR.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param strobe flash light strobe to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_flash_strobe(struct device *dev, uint8_t light_id,
                                   uint8_t channel_id, uint8_t strobe)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    if (!is_channel_flash(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].strobe = strobe;

    return 0;
}

/**
 * @brief Configure flash light intensity
 *
 * This function set flash intensity to specific channel.
 * Note that this function is only called when the channel type is flash.
 * i.e. the mode of channel_config is LIGHTS_CHANNEL_MODE_FLASH or
 * LIGHTS_CHANNEL_MODE_TORCH or LIGHTS_CHANNEL_MODE_INDICATOR.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param intensity flash light intensity to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_flash_intensity(struct device *dev, uint8_t light_id,
                                      uint8_t channel_id, uint32_t intensity)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    if (!is_channel_flash(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].current = intensity;

    return 0;
}

/**
 * @brief Remove lights event callback function
 *
 * Unregister lights event callback function from the lights device driver.
 * After called this function, the lights device will not inform the host again
 * for any event.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int lights_unregister_callback(struct device *dev)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    info->event_callback = NULL;

    return 0;
}

/**
 * @brief Register lights event callback function
 *
 * Register lights event callback function to lights device driver.
 * If the lights device has event need to notify host in some situation, the
 * lights device driver will perform the event callback function.
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static int lights_register_callback(struct device *dev,
                                    lights_event_callback callback)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    info->event_callback = callback;

    return 0;
}

/**
 * @brief Configure light fade
 *
 * This function set the fade configuration to specific channel.
 * When setting this function the light change is not instant, instead it fades
 * smooth from one state to another.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param fade_in light fade in to be set
 * @param fade_out light fade out to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_fade(struct device *dev, uint8_t light_id,
                           uint8_t channel_id, uint8_t fade_in,
                           uint8_t fade_out)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].fade_in = fade_in;
    info->light_attri[light_id]->channels[channel_id].fade_out = fade_out;

    return 0;
}

/**
 * @brief Configure light color
 *
 * This function set color to specific channel
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param color It is a 32 bits color representation which indicates a color
 * space
 * @return 0 on success, negative errno on error
 */
static int lights_set_color(struct device *dev, uint8_t light_id,
                            uint8_t channel_id, uint32_t color)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].cfg.color = color;

    return 0;
}

/**
 * @brief Configure light blink
 *
 * This function set the blink to specific channel. It can adjust the timings
 * to activate blink, delays are in milliseconds.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param time_on_ms light time on in milliseconds to be set
 * @param time_off_ms light time off in milliseconds to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_blink(struct device *dev, uint8_t light_id,
                            uint8_t channel_id, uint16_t time_on_ms,
                            uint16_t time_off_ms)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    info->light_attri[light_id]->channels[channel_id].time_on_ms = time_on_ms;
    info->light_attri[light_id]->channels[channel_id].time_off_ms = time_off_ms;

    return 0;
}

/**
 * @brief Configure light brightness
 *
 * This function set brightness to specific channel
 * Note that if blink is active, this function will disable blink mode if the
 * brightness value is 0.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param brightness light brightness to be set
 * @return 0 on success, negative errno on error
 */
static int lights_set_brightness(struct device *dev, uint8_t light_id,
                                 uint8_t channel_id, uint8_t brightness)
{
    struct lights_info *info = NULL;
    struct channel_info *channel = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }
    channel = &(info->light_attri[light_id]->channels[channel_id]);

    channel->brightness = (brightness > channel->cfg.max_brightness)
                          ? channel->cfg.max_brightness : brightness;

    /* disable blinking mode if brignthess is 0 */
    if (brightness == 0) {
        if (channel->time_on_ms || channel->time_off_ms) {
            /* TODO: disable blinking mode */
        }
    }

    /* Test hack: send release event if brightness is set to 254 */
    if (brightness == 254) {
        if (info->event_callback) {
            info->event_callback(light_id, LIGHTS_EVENT_LIGHT_CONFIG);
        }
    }

    return 0;
}

/**
 * @brief Get flash config of specific channel
 *
 * This function provides the channel flash configuration of light of specific
 * light ID and channel ID.
 * This function is called if the caller would like to know the configuration
 * of flash type channel ID in lights device driver.
 * Note that is function is only called when the channel support flash type.
 * i.e. the mode of channel_config is LIGHTS_CHANNEL_MODE_FLASH or
 * LIGHTS_CHANNEL_MODE_TORCH or LIGHTS_CHANNEL_MODE_INDICATOR.
 *
 * @param dev pointer to structure of device data
 * @param light_id The ID of specific light
 * @param channel_id The ID of specific channel
 * @param fcfg A pointer to the channel_flash_config structure to receive
 *             channel flash configuration information
 *
 * @return 0: Success, error code on failure
*/
static int lights_get_channel_flash_config(struct device *dev,
                                           uint8_t light_id,
                                           uint8_t channel_id,
                                           struct channel_flash_config *fcfg)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !fcfg) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    if (!is_channel_flash(info, light_id, channel_id)) {
        return -EINVAL;
    }

    memcpy(fcfg, &(info->light_attri[light_id]->channels[channel_id].fcfg),
           sizeof(struct channel_flash_config));

    return 0;
}

/**
 * @brief Get configuration of specific channel
 *
 * This function provides the channel configuration of specific light ID and
 * channel ID.
 * This function is called if the caller would like to know the configuration
 * of each channel ID in lights device driver.
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param cfg pointer to the channel_config structure to receive channel
 *            configuration information
 *
 * @return 0: Success, error code on failure
*/
static int lights_get_channel_config(struct device *dev, uint8_t light_id,
                                     uint8_t channel_id,
                                     struct channel_config *cfg)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !cfg) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, light_id, channel_id)) {
        return -EINVAL;
    }

    memcpy(cfg, &(info->light_attri[light_id]->channels[channel_id].cfg),
           sizeof(struct channel_config));

    return 0;
}

/**
 * @brief Get configuration of light
 *
 * This function returns the light configuration of specific light ID.
 * After invoking this function, the caller will get the both light name and
 * the number of channels, which are from the light driver.
 * This function is called if the caller would like to know the configuration
 * of each light ID in lights device driver.
 *
 * @param dev pointer to structure of device data
 * @param id the ID of specific light
 * @param cfg pointer to the light_config structure to receive light
 *            configuration information
 *
 * @return 0: Success, error code on failure
*/
static int lights_get_light_config(struct device *dev, uint8_t id,
                                   struct light_config *cfg)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !cfg) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    if (!is_channel_valid(info, id, 0)) {
        return -EINVAL;
    }

    memcpy(cfg, &(info->light_attri[id]->cfg), sizeof(struct light_config));

    return 0;
}

/**
 * @brief Get count of lights
 *
 * This function is called if the caller would like to know how many lights
 * are supported in the lights device Driver
 *
 * @param dev pointer to structure of device data
 * @param lights pointer to the variable of lights count
 *
 * @return 0: Success, error code on failure
*/
static int lights_get_lights(struct device *dev, uint8_t *lights)
{
    struct lights_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !lights) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != LIGHTS_STATE_OPEN) {
        return -EPERM;
    }

    *lights = info->lights_count;

    return 0;
}

/**
 * @brief Open lights device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int lights_dev_open(struct device *dev)
{
    struct lights_info *info = NULL;
    int ret = 0;
    int i;
    int success_cnt;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if(info->state == LIGHTS_STATE_OPEN) {
        return -EBUSY;
    }

    info->lights_count = LIGHTS_COUNT;
    for (i = 0; i < info->lights_count; i++) {
        info->light_attri[i] = lights_init(i);
        if (info->light_attri[i] == NULL) {
            ret = -ENOMEM;
            success_cnt = i;
            goto free_lights;
        }
    }

    info->event_callback = NULL;

    info->state = LIGHTS_STATE_OPEN;

    return ret;

free_lights:

    for (i = 0; i < success_cnt; i++) {
         lights_free(info->light_attri[i]);
    }

    return ret;
}

/**
 * @brief Close lights device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void lights_dev_close(struct device *dev)
{
    struct lights_info *info = NULL;
    int i;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    for (i = 0; i < info->lights_count; i++) {
         lights_free(info->light_attri[i]);
    }
    info->lights_count = 0;

    info->event_callback = NULL;

    info->state = LIGHTS_STATE_CLOSED;
}

/**
 * @brief Probe lights device
 *
 * This function is called by the system when the system boot up. This function
 * allocates memory for the light device information.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int lights_dev_probe(struct device *dev)
{
    struct lights_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->state = LIGHTS_STATE_CLOSED;
    info->dev = dev;
    device_set_private(dev, info);

    return 0;
}

/**
 * @brief Remove lights device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void lights_dev_remove(struct device *dev)
{
    struct lights_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    if (info->state == LIGHTS_STATE_OPEN) {
        lights_dev_close(dev);
    }

    device_set_private(dev, NULL);
    free(info);
}

static struct device_lights_type_ops lights_type_ops = {
    .get_lights                 = lights_get_lights,
    .get_light_config           = lights_get_light_config,
    .get_channel_config         = lights_get_channel_config,
    .get_channel_flash_config   = lights_get_channel_flash_config,
    .set_brightness             = lights_set_brightness,
    .set_blink                  = lights_set_blink,
    .set_color                  = lights_set_color,
    .set_fade                   = lights_set_fade,
    .register_callback          = lights_register_callback,
    .unregister_callback        = lights_unregister_callback,
    .set_flash_intensity        = lights_set_flash_intensity,
    .set_flash_strobe           = lights_set_flash_strobe,
    .set_flash_timeout          = lights_set_flash_timeout,
    .get_flash_fault            = lights_get_flash_fault,
};

static struct device_driver_ops lights_driver_ops = {
    .probe              = lights_dev_probe,
    .remove             = lights_dev_remove,
    .open               = lights_dev_open,
    .close              = lights_dev_close,
    .type_ops           = &lights_type_ops,
};

struct device_driver lights_driver = {
    .type       = DEVICE_TYPE_LIGHTS_HW,
    .name       = "lights",
    .desc       = "Lights Driver",
    .ops        = &lights_driver_ops,
};
