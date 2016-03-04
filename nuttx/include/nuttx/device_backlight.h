/*
 * Copyright (c) 2016 Motorola, LLC.
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

#ifndef __DEVICE_BACKLIGHT_H__
#define __DEVICE_BACKLIGHT_H__

#define DEVICE_TYPE_BACKLIGHT_HW          "backlight"

#define GB_BACKLIGHT_EXT_MODE_INVALID     0x00
#define GB_BACKLIGHT_EXT_MODE_MANUAL      0x01
#define GB_BACKLIGHT_EXT_MODE_AUTO        0x02

struct device_backlight_type_ops {
    int (*set_mode)(struct device *dev, uint8_t mode);
    int (*get_mode)(struct device *dev, uint8_t *mode);
    int (*set_brightness)(struct device *dev, uint8_t brightness);
    int (*get_brightness)(struct device *dev, uint8_t *brightness);
};

/**
 * @brief Backlight set_mode() wrap function
 *
 * @param dev pointer to structure of device data
 * @param mode backlight mode to set
 * @return 0 on success, negative errno on error
 */
static inline int device_backlight_set_mode(struct device *dev, uint8_t mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, backlight)->set_mode) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, backlight)->set_mode(dev, mode);
}

/**
 * @brief Backlight get_mode() wrap function
 *
 * @param dev pointer to structure of device data
 * @param mode returned backlight mode
 * @return 0 on success, negative errno on error
 */
static inline int device_backlight_get_mode(struct device *dev, uint8_t *mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, backlight)->get_mode) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, backlight)->get_mode(dev, mode);
}

/**
 * @brief Backlight set_brightness() wrap function
 *
 * @param dev pointer to structure of device data
 * @param brightness backlight brightness to set
 * @return 0 on success, negative errno on error
 */
static inline int device_backlight_set_brightness(struct device *dev, uint8_t brightness)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, backlight)->set_brightness) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, backlight)->set_brightness(dev, brightness);
}

/**
 * @brief Backlight get_brightness() wrap function
 *
 * @param dev pointer to structure of device data
 * @param brightness returned backlight brightness
 * @return 0 on success, negative errno on error
 */
static inline int device_backlight_get_brightness(struct device *dev, uint8_t *brightness)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, backlight)->get_brightness) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, backlight)->get_brightness(dev, brightness);
}

#endif /* __DEVICE_BACKLIGHT_H__ */
