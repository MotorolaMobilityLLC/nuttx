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

#include <nuttx/greybus/mods.h>

#include <nuttx/device.h>
#include <nuttx/device_lights.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/util.h>

#define LM27965_INVALID_RESOURCE         (0xffffffff)

#define LM27965_BACKLIGHT_POWER_DELAY_US (1000)
#define LM27965_BACKLIGHT_RESET_DELAY_US (1000)

#define LM27965_GENERAL_PURPOSE          (0x10)
#define LM27965_BANK_A_CONTROL           (0xa0)
#define LM27965_BANK_B_CONTROL           (0xb0)
#define LM27965_BANK_C_CONTROL           (0xc0)

const static uint8_t LM27965_BACKLIGHT_INIT[] = { \
    /* register              value */
    LM27965_GENERAL_PURPOSE, 0x02,
    LM27965_BANK_B_CONTROL,  0x1f,
};

static struct lm27965_backlight
{
    struct device *self;
    struct i2c_dev_s *i2c_dev;
    uint32_t i2c_bus;
    uint32_t i2c_addr;
    uint32_t gpio_power;
    uint32_t gpio_reset;
    struct light_info *light_info_array;
    int on;
} g_backlight;

static struct channel_info lm27965_backlight_channel_info[] = {
    {
        .id = 0,
        .cfg = {
            .max_brightness = 31,
            .flags = 0,
            .color = 0,
            .color_name = "",
            .mode = LIGHTS_CHANNEL_MODE_VENDOR,
            .mode_name = "backlight",
        },
    },
};

static struct light_info lm27965_light_info[] = {
    {
        .id = 0,
        .cfg = {
            .channel_count = 1,
            .name = "mod_light0",
        },
        .channels = lm27965_backlight_channel_info,
    },
};

#define LM27965_NUM_LIGHTS (ARRAY_SIZE(lm27965_light_info))

static int _lm27965_backlight_i2c_write(struct i2c_dev_s *dev, uint16_t addr,
        uint8_t regaddr, uint8_t value)
{
    int result;
    struct i2c_msg_s msg;
    uint8_t wbuf[] = { regaddr, value };

    msg.addr   = addr;
    msg.flags  = 0;
    msg.buffer = wbuf;
    msg.length = sizeof(wbuf)/sizeof(wbuf[0]);

    result = I2C_TRANSFER(dev, &msg, 1);
    return result;
}

static void _lm27965_backlight_power_on(struct lm27965_backlight *backlight)
{
    vdbg("\n");

    int i;

    if (!backlight->i2c_dev) {
        return;
    }

    if (backlight->gpio_power != LM27965_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_power, 1);

        usleep(LM27965_BACKLIGHT_POWER_DELAY_US);
    }

    if (backlight->gpio_reset != LM27965_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_reset, 1);

        usleep(LM27965_BACKLIGHT_POWER_DELAY_US);
    }

    /* Initialize */
    for (i = 0; i < ARRAY_SIZE(LM27965_BACKLIGHT_INIT); i += 2) {
        _lm27965_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
            LM27965_BACKLIGHT_INIT[i], LM27965_BACKLIGHT_INIT[i+1]);
    }

    backlight->on = 1;
}

static void _lm27965_backlight_power_off(struct lm27965_backlight *backlight)
{
    vdbg("\n");

    backlight->on = 0;

    if (backlight->gpio_reset != LM27965_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_reset, 0);
    }

    if (backlight->gpio_power != LM27965_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_power, 0);
    }
}

static int _lm27965_backlight_attach(void *arg, const void *data)
{
    if (!arg || !data) {
        return -EINVAL;
    }

    struct lm27965_backlight *backlight = (struct lm27965_backlight *)arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    vdbg("backlight=%p, state=%d\n", backlight, state);

    switch (state) {
    case BASE_ATTACHED_OFF:
    case BASE_DETACHED:
        _lm27965_backlight_power_off(backlight);
        break;
    case BASE_ATTACHED:
    case BASE_INVALID:
        /* Ignore */
        break;
    }

    vdbg("done\n");
    return 0;
}

static int
_lm27965_backlight_set_brightness(struct lm27965_backlight *backlight,
    uint8_t brightness)
{
    vdbg("brightness=%d\n", brightness);

    if (!backlight->i2c_dev) {
        return -ENOENT;
    }

    _lm27965_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
        LM27965_BANK_B_CONTROL, brightness);

    return 0;
}

/**
 * @brief Get number of lights
 *
 * @param dev pointer to structure of device data
 * @param lights pointer to the number of lights
 * @return 0 on success, negative errno on error
 */
static int lm27965_backlight_get_lights(struct device *dev, uint8_t *lights)
{
    *lights = LM27965_NUM_LIGHTS;
    vdbg("lights=%d\n", *lights);

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
static int lm27965_backlight_get_light_config(struct device *dev, uint8_t id,
                                              struct light_config *cfg)
{
    struct lm27965_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    vdbg("\n");

    if (!dev) {
        return -ENODEV;
    }

    backlight = device_get_private(dev);
    if (!backlight) {
        return -ENOENT;
    }

    info = backlight->light_info_array;

    if (id < LM27965_NUM_LIGHTS) {
        *cfg = info[id].cfg;
    } else {
        ret = -EINVAL;
    }

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
static int lm27965_backlight_get_channel_config(struct device *dev,
                                                uint8_t light_id,
                                                uint8_t channel_id,
                                                struct channel_config *cfg)
{
    struct lm27965_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    vdbg("\n");

    if (!dev) {
        return -ENODEV;
    }

    backlight = device_get_private(dev);
    if (!backlight) {
        return -ENOENT;
    }

    info = backlight->light_info_array;

    if (light_id < LM27965_NUM_LIGHTS &&
        channel_id < info[light_id].cfg.channel_count) {
        *cfg = info[light_id].channels[channel_id].cfg;
    } else {
        ret = -EINVAL;
    }

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
static int lm27965_backlight_set_brightness(struct device *dev,
                                            uint8_t light_id,
                                            uint8_t channel_id,
                                            uint8_t brightness)
{
    struct lm27965_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    vdbg("light_id=%d, channel_id=%d, brightness=%d\n",
        light_id, channel_id, brightness);

    if (!dev) {
        return -ENODEV;
    }

    backlight = device_get_private(dev);
    if (!backlight) {
        return -ENOENT;
    }

    info = backlight->light_info_array;

    if (light_id >= LM27965_NUM_LIGHTS ||
        channel_id >= info[light_id].cfg.channel_count) {
        return -EINVAL;
    }

    if (!backlight->on) {
        _lm27965_backlight_power_on(backlight);
    }

    ret = _lm27965_backlight_set_brightness(backlight, brightness);
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
static int lm27965_backlight_register_callback(struct device *dev,
                                               lights_event_callback callback)
{
    vdbg("\n");
    return 0;
}

/**
 * @brief Unregister callback
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int lm27965_backlight_unregister_callback(struct device *dev)
{
    vdbg("\n");
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
static int lm27965_backlight_dev_open(struct device *dev)
{
    struct lm27965_backlight *backlight;

    vdbg("\n");

    if (!dev) {
        return -ENODEV;
    }

    backlight = device_get_private(dev);
    if (!backlight) {
        return -ENOENT;
    }

    /* Open I2C */
    backlight->i2c_dev = up_i2cinitialize(backlight->i2c_bus);
    if (!backlight->i2c_dev) {
        dbg("ERROR: Failed to get bus %d\n", backlight->i2c_bus);
        return -EIO;
    }

    return 0;
}

/**
 * @brief Close device
 * @param dev pointer to the UART device structure
 */
static void lm27965_backlight_dev_close(struct device *dev)
{
    vdbg("\n");

    struct lm27965_backlight *backlight = device_get_private(dev);
    if (!backlight) {
        return;
    }

    /* Release I2C */
    (void)up_i2cuninitialize(backlight->i2c_dev);
    backlight->i2c_dev = NULL;

    /* Power off */
    _lm27965_backlight_power_off(backlight);
}

/**
 * @brief probe device
 * @param dev pointer to the device structure
 * @return 0 for success, -errno for failures.
 */
static int lm27965_backlight_probe(struct device *dev)
{
    struct lm27965_backlight *backlight = &g_backlight;

    vdbg("\n");

    backlight->self = dev;

    backlight->i2c_bus = CONFIG_BACKLIGHT_LM27965_I2C_BUS;
    backlight->i2c_addr = CONFIG_BACKLIGHT_LM27965_I2C_ADDR;

    /* GPIO power (optional) */
    backlight->gpio_power = LM27965_INVALID_RESOURCE;
    struct device_resource *rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "gpio_power");
    if (rsrc) {
        backlight->gpio_power = rsrc->start;
        gpio_direction_out(backlight->gpio_power, 0);
    }

    /* GPIO reset (optional) */
    backlight->gpio_reset = LM27965_INVALID_RESOURCE;
    rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "gpio_reset");
    if (rsrc) {
        backlight->gpio_reset = rsrc->start;
        gpio_direction_out(backlight->gpio_reset, 0);
    }

    vdbg("i2c: bus=%d, addr=%02x, gpio: pwr=%d, rst=%d\n",
        backlight->i2c_bus, backlight->i2c_addr,
        backlight->gpio_power, backlight->gpio_reset);

    /* add the light info array to the private data */
    backlight->light_info_array = lm27965_light_info;
    backlight->on = 0;

    device_set_private(dev, backlight);

    /* Register for attach notifications. This will callback immediately. */
    if (mods_attach_register(_lm27965_backlight_attach, backlight)) {
        dbg("ERROR: failed to register attach notifier\n");
    }

    return 0;
}

/**
 * @brief remove device
 * @param dev pointer to the UART device structure
 */
static void lm27965_backlight_remove(struct device *dev)
{
    vdbg("\n");

    struct lm27965_backlight *backlight = device_get_private(dev);
    if (backlight) {
        device_set_private(dev, NULL);
        memset(backlight, 0, sizeof(*backlight));
    }
}

const static struct device_lights_type_ops lm27965_backlight_ops = {
    .get_lights          = lm27965_backlight_get_lights,
    .get_light_config    = lm27965_backlight_get_light_config,
    .get_channel_config  = lm27965_backlight_get_channel_config,
    .set_brightness      = lm27965_backlight_set_brightness,
    .register_callback   = lm27965_backlight_register_callback,
    .unregister_callback = lm27965_backlight_unregister_callback,
};

const static struct device_driver_ops lm27965_backlight_driver_ops = {
    .probe    = lm27965_backlight_probe,
    .remove   = lm27965_backlight_remove,
    .open     = lm27965_backlight_dev_open,
    .close    = lm27965_backlight_dev_close,
    .type_ops = (struct device_light_type_ops *)&lm27965_backlight_ops,
};

const struct device_driver lm27965_backlight_driver = {
    .type = DEVICE_TYPE_LIGHTS_HW,
    .name = "lm27965_backlight",
    .desc = "LM27965 Backlight",
    .ops  = (struct device_driver_ops *)&lm27965_backlight_driver_ops,
};