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
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/util.h>

#define ISL98611_INVALID_RESOURCE (0xffffffff)

#define ISL98611_BACKLIGHT_POWER_DELAY_US (1000)
#define ISL98611_BACKLIGHT_RESET_DELAY_US (1000)

#define ISL98611_BACKLIGHT_INITIAL_BRIGHTNESS (255)

#define ISL98611_FAULT_STATUS        (0x01)
#define ISL98611_ENABLE              (0x02)
#define ISL98611_VP_VN_HEADROOM      (0x03)
#define ISL98611_VP_VOLTAGE          (0x04)
#define ISL98611_VN_VOLTAGE          (0x05)
#define ISL98611_VBST_CTRL           (0x06)
/* 0x07 - 0x0f reserved */
#define ISL98611_BRIGHTNESS_CTRL_MSB (0x10)
#define ISL98611_BRIGHTNESS_CTRL_LSB (0x11)
#define ISL98611_LED_CURRENT         (0x12)
#define ISL98611_DIMMING_CTRL        (0x13)
#define ISL98611_LED_PWM_CTRL        (0x14)
/* 0x15 reserved */
#define ISL98611_VLED_FREQ           (0x16)
#define ISL98611_LED_CONFIG          (0x17)

const static uint8_t ISL98611_BACKLIGHT_INIT[] = { \
    /* register            value */
    ISL98611_DIMMING_CTRL, 0x00,
    ISL98611_LED_PWM_CTRL, 0x7d,
    ISL98611_VP_VOLTAGE,   0x14,
    ISL98611_VN_VOLTAGE,   0x14,
    ISL98611_LED_CURRENT,  0xbf,
    0x15,                  0x86,
};

static struct isl98611_backlight
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

static struct channel_info isl98611_backlight_channel_info[] = {
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

static struct light_info isl98611_light_info[] = {
    {
        .id = 0,
        .cfg = {
            .channel_count = 1,
            .name = "mod_light0",
        },
        .channels = isl98611_backlight_channel_info,
    },
};

#define ISL98611_NUM_LIGHTS (ARRAY_SIZE(isl98611_light_info))

static int _isl98611_backlight_i2c_write(struct i2c_dev_s *dev, uint16_t addr,
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

static void _isl98611_backlight_power_on(struct isl98611_backlight *backlight)
{
    int i;

    if (!backlight->i2c_dev) {
        return;
    }

    if (backlight->gpio_power != ISL98611_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_power, 1);

        usleep(ISL98611_BACKLIGHT_POWER_DELAY_US);
    }

    if (backlight->gpio_reset != ISL98611_INVALID_RESOURCE) {
        gpio_direction_out(backlight->gpio_reset, 1);

        usleep(ISL98611_BACKLIGHT_POWER_DELAY_US);
    }

    /* Reset and delay */
    _isl98611_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
        ISL98611_ENABLE, 0x3f);
    usleep(ISL98611_BACKLIGHT_RESET_DELAY_US);

    /* Initialize */
    for (i = 0; i < ARRAY_SIZE(ISL98611_BACKLIGHT_INIT); i += 2) {
        _isl98611_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
            ISL98611_BACKLIGHT_INIT[i], ISL98611_BACKLIGHT_INIT[i+1]);
    }

    backlight->on = 1;
}

static void _isl98611_backlight_power_off(struct isl98611_backlight *backlight)
{
    backlight->on = 0;

    if (backlight->gpio_reset != ISL98611_INVALID_RESOURCE)
        gpio_direction_out(backlight->gpio_reset, 0);

    if (backlight->gpio_power != ISL98611_INVALID_RESOURCE)
        gpio_direction_out(backlight->gpio_power, 0);
}

static int
_isl98611_backlight_set_brightness(struct isl98611_backlight *backlight,
    uint8_t brightness)
{
    if (!backlight->i2c_dev) {
        return -ENOENT;
    }

    _isl98611_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
        ISL98611_BRIGHTNESS_CTRL_LSB, 0x00);
    _isl98611_backlight_i2c_write(backlight->i2c_dev, backlight->i2c_addr,
        ISL98611_BRIGHTNESS_CTRL_MSB, brightness);

    return 0;
}

/**
 * @brief Get number of lights
 *
 * @param dev pointer to structure of device data
 * @param lights pointer to the number of lights
 * @return 0 on success, negative errno on error
 */
static int isl98611_backlight_get_lights(struct device *dev, uint8_t *lights)
{
    *lights = ISL98611_NUM_LIGHTS;

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
static int isl98611_backlight_get_light_config(struct device *dev, uint8_t id,
                                               struct light_config *cfg)
{
    struct isl98611_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    info = backlight->light_info_array;

    if (id < ISL98611_NUM_LIGHTS)
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
static int isl98611_backlight_get_channel_config(struct device *dev,
                                                 uint8_t light_id,
                                                 uint8_t channel_id,
                                                 struct channel_config *cfg)
{
    struct isl98611_backlight *backlight;
    struct light_info *info;
    int ret = 0;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

    info = backlight->light_info_array;

    if (light_id < ISL98611_NUM_LIGHTS &&
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
static int isl98611_backlight_set_brightness(struct device *dev,
                                             uint8_t light_id,
                                             uint8_t channel_id,
                                             uint8_t brightness)
{
    struct isl98611_backlight *backlight;
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

    if (light_id >= ISL98611_NUM_LIGHTS ||
        channel_id >= info[light_id].cfg.channel_count) {
        return -EINVAL;
    }

    if (!backlight->on) {
        _isl98611_backlight_power_on(backlight);
    }

    ret = _isl98611_backlight_set_brightness(backlight, brightness);
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
static int isl98611_backlight_register_callback(struct device *dev,
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
static int isl98611_backlight_unregister_callback(struct device *dev)
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
static int isl98611_backlight_dev_open(struct device *dev)
{
    struct isl98611_backlight *backlight;

    if (!dev)
        return -ENODEV;

    backlight = device_get_private(dev);
    if (!backlight)
        return -ENOENT;

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
static void isl98611_backlight_dev_close(struct device *dev)
{
    struct isl98611_backlight *backlight = device_get_private(dev);
    if (!backlight) {
        return;
    }

    /* Release I2C */
    (void)up_i2cuninitialize(backlight->i2c_dev);
    backlight->i2c_dev = NULL;

    /* Power off */
    _isl98611_backlight_power_off(backlight);
}

/**
 * @brief probe device
 * @param dev pointer to the device structure
 * @return 0 for success, -errno for failures.
 */
static int isl98611_backlight_probe(struct device *dev)
{
    struct isl98611_backlight *backlight = &g_backlight;

    backlight->self = dev;

    backlight->i2c_bus = CONFIG_BACKLIGHT_ISL98611_I2C_BUS;
    backlight->i2c_addr = CONFIG_BACKLIGHT_ISL98611_I2C_ADDR;

    /* GPIO power (optional) */
    backlight->gpio_power = ISL98611_INVALID_RESOURCE;
    struct device_resource *rsrc = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "gpio_power");
    if (rsrc) {
        backlight->gpio_power = rsrc->start;
        gpio_direction_out(backlight->gpio_power, 0);
    }

    /* GPIO reset (optional) */
    backlight->gpio_reset = ISL98611_INVALID_RESOURCE;
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
    backlight->light_info_array = isl98611_light_info;
    backlight->on = 0;

    device_set_private(dev, backlight);
    return 0;
}

/**
 * @brief remove device
 * @param dev pointer to the UART device structure
 */
static void isl98611_backlight_remove(struct device *dev)
{
    struct isl98611_backlight *backlight = device_get_private(dev);
    if (backlight) {
        device_set_private(dev, NULL);
        memset(backlight, 0, sizeof(*backlight));
    }
}

const static struct device_lights_type_ops isl98611_backlight_ops = {
    .get_lights          = isl98611_backlight_get_lights,
    .get_light_config    = isl98611_backlight_get_light_config,
    .get_channel_config  = isl98611_backlight_get_channel_config,
    .set_brightness      = isl98611_backlight_set_brightness,
    .register_callback   = isl98611_backlight_register_callback,
    .unregister_callback = isl98611_backlight_unregister_callback,
};

const static struct device_driver_ops isl98611_backlight_driver_ops = {
    .probe    = isl98611_backlight_probe,
    .remove   = isl98611_backlight_remove,
    .open     = isl98611_backlight_dev_open,
    .close    = isl98611_backlight_dev_close,
    .type_ops = (struct device_light_type_ops *)&isl98611_backlight_ops,
};

const struct device_driver isl98611_backlight_driver = {
    .type = DEVICE_TYPE_LIGHTS_HW,
    .name = "isl98611_backlight",
    .desc = "ISL98611 Backlight",
    .ops  = (struct device_driver_ops *)&isl98611_backlight_driver_ops,
};
