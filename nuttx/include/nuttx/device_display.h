/*
 * Copyright (c) 2015 Motorola, LLC.
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

#ifndef __DEVICE_DISPLAY_H__
#define __DEVICE_DISPLAY_H__

#define DEVICE_TYPE_DISPLAY_HW          "display"

enum display_notification_event {
    DISPLAY_NOTIFICATION_EVENT_INVALID  = 0x00,
    DISPLAY_NOTIFICATION_EVENT_FAILURE  = 0x01,
    DISPLAY_NOTIFICATION_EVENT_ATTACHED = 0x02,
    DISPLAY_NOTIFICATION_EVENT_DETACHED = 0x03,
};

typedef int (*display_notification_cb)(struct device *dev,
    enum display_notification_event event);

struct device_display_type_ops {
    int (*get_config_size)(struct device *dev, uint32_t *size);
    int (*get_config)(struct device *dev, uint8_t *type, uint32_t *size, uint8_t **config);
    int (*set_config)(struct device *dev, uint8_t index);
    int (*get_state)(struct device *dev, uint8_t *state);
    int (*set_state)(struct device *dev, uint8_t state);
    int (*get_backlight_config)(struct device *dev, uint8_t *config);
    int (*set_backlight_config)(struct device *dev, uint8_t config);
    int (*get_backlight_brightness)(struct device *dev, uint8_t *brightness);
    int (*set_backlight_brightness)(struct device *dev, uint8_t brightness);
    int (*register_callback)(struct device *dev, display_notification_cb cb);
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief Display get_config_size() wrap function
 *
 * @param dev pointer to structure of device data
 * @param size the length of the config sent in get_config()
 * @return 0 on success, negative errno on error
 */
static inline int device_display_get_config_size(struct device *dev, uint32_t *size)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_config_size) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_config_size(dev, size);
}

/**
 * @brief Display get_config() wrap function
 *
 * @param dev pointer to structure of device data
 * @param type of the configuration data
 * @param size the length of the data buffer
 * @param config_data the configuration data
 * @return 0 on success, negative errno on error
 */
static inline int device_display_get_config(struct device *dev, uint8_t *type,
    size_t *size, uint8_t **config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_config(dev, type, size, config);
}

/**
 * @brief Display set_config() wrap function
 *
 * @param dev pointer to structure of device data
 * @param index the index into the config set to select
 * @return 0 on success, negative errno on error
 */
static inline int device_display_set_config(struct device *dev, uint8_t index)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->set_config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->set_config(dev, index);
}

/**
 * @brief Display get_state() wrap function
 *
 * @param dev pointer to structure of device data
 * @param state returned state
 * @return 0 on success, negative errno on error
 */
static inline int device_display_get_state(struct device *dev, uint8_t *state)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_state) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_state(dev, state);
}

/**
 * @brief Display set_state() wrap function
 *
 * @param dev pointer to structure of device data
 * @param state display state to select
 * @return 0 on success, negative errno on error
 */
static inline int device_display_set_state(struct device *dev, uint8_t state)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->set_state) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->set_state(dev, state);
}

/**
 * @brief Display get_backlight_config() wrap function
 *
 * @param dev pointer to structure of device data
 * @param config the backlight configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_display_get_backlight_config(struct device *dev,
        uint8_t *config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_backlight_config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_backlight_config(dev, config);
}

/**
 * @brief Display get_backlight_config() wrap function
 *
 * @param dev pointer to structure of device data
 * @param config the backlight configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_display_set_backlight_config(struct device *dev,
        uint8_t config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->set_backlight_config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->set_backlight_config(dev, config);
}

/**
 * @brief Display get_backlight_brightness() wrap function
 *
 * @param dev pointer to structure of device data
 * @param brightness the backlight brightness level
 * @return 0 on success, negative errno on error
 */
static inline int device_display_get_backlight_brightness(struct device *dev,
        uint8_t *brightness)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_backlight_brightness) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_backlight_brightness(dev, brightness);
}

/**
 * @brief Display get_backlight_brightness() wrap function
 *
 * @param dev pointer to structure of device data
 * @param brightness the backlight brightness level
 * @return 0 on success, negative errno on error
 */
static inline int device_display_set_backlight_brightness(struct device *dev,
        uint8_t brightness)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->set_backlight_brightness) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->set_backlight_config(dev, brightness);
}

/**
 * @brief Display register_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static inline int device_display_register_callback(struct device *dev,
        display_notification_cb cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->register_callback(dev, cb);
}


/**
 * @brief Display unregister_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_display_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->unregister_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->unregister_callback(dev);
}
#endif /* __DEVICE_DISPLAY_H__ */
