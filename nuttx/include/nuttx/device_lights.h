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

#ifndef __ARCH_ARM_DEVICE_LIGHTS_H
#define __ARCH_ARM_DEVICE_LIGHTS_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define DEVICE_TYPE_LIGHTS_HW                  "lights"

#define NAME_LENGTH                             32

/* Lights channel mode definition */
#define LIGHTS_CHANNEL_MODE_NONE                0x00000000
#define LIGHTS_CHANNEL_MODE_BATTERY             0x00000001
#define LIGHTS_CHANNEL_MODE_POWER               0x00000002
#define LIGHTS_CHANNEL_MODE_WIRELESS            0x00000004
#define LIGHTS_CHANNEL_MODE_BLUETOOTH           0x00000008
#define LIGHTS_CHANNEL_MODE_KEYBOARD            0x00000010
#define LIGHTS_CHANNEL_MODE_BUTTONS             0x00000020
#define LIGHTS_CHANNEL_MODE_NOTIFICATION        0x00000040
#define LIGHTS_CHANNEL_MODE_ATTENTION           0x00000080
#define LIGHTS_CHANNEL_MODE_FLASH               0x00000100
#define LIGHTS_CHANNEL_MODE_TORCH               0x00000200
#define LIGHTS_CHANNEL_MODE_INDICATOR           0x00000400
#define LIGHTS_CHANNEL_MODE_VENDOR              0x00100000

/* Lights channel mode valid bit values */
#define LIGHTS_CHANNEL_MODE_DEFINED_RANGE       0x000004FF
#define LIGHTS_CHANNEL_MODE_VENDOR_RANGE        0x00F00000

/* Lights channel flags definition */
#define LIGHTS_CHANNEL_FLAG_MULTICOLOR          0x00000001
#define LIGHTS_CHANNEL_FLAG_FADER               0x00000002
#define LIGHTS_CHANNEL_FLAG_BLINK               0x00000004

/* Lights events definition */
#define LIGHTS_EVENT_LIGHT_CONFIG               0x01

/* Lights flash fault values */
#define LIGHTS_FLASH_FAULT_OVER_VOLTAGE         0x00000000
#define LIGHTS_FLASH_FAULT_TIMEOUT              0x00000001
#define LIGHTS_FLASH_FAULT_OVER_TEMPERATURE     0x00000002
#define LIGHTS_FLASH_FAULT_SHORT_CIRCUIT        0x00000004
#define LIGHTS_FLASH_FAULT_OVER_CURRENT         0x00000008
#define LIGHTS_FLASH_FAULT_INDICATOR            0x00000010
#define LIGHTS_FLASH_FAULT_UNDER_VOLTAGE        0x00000020
#define LIGHTS_FLASH_FAULT_INPUT_VOLTAGE        0x00000040
#define LIGHTS_FLASH_FAULT_LED_OVER_TEMPERATURE 0x00000080

/**
 * Lights channel configuration
 */
struct channel_config {
    /** max brightness */
    uint8_t     max_brightness;
    /** light channels flags bits */
    uint32_t    flags;
    /** color code value */
    uint32_t    color;
    /** color name, 31 characters with null character */
    char        color_name[NAME_LENGTH];
    /** lights channel mode bits */
    uint32_t    mode;
    /** mode name, 31 characters with null character */
    char        mode_name[NAME_LENGTH];
};

/**
 * Lights flash channel configuration
 */
struct channel_flash_config {
    /** min current intensity in microamps */
    uint32_t    intensity_min_uA;
    /** max current intensity in microamps */
    uint32_t    intensity_max_uA;
    /** current intensity step in microamps */
    uint32_t    intensity_step_uA;
    /** min strobe flash timeout in microseconds */
    uint32_t    timeout_min_us;
    /** max strobe flash timeout in microseconds */
    uint32_t    timeout_max_us;
    /** strobe flash timeout step in microseconds */
    uint32_t    timeout_step_us;
};

/**
 * Lights channel info
 */
struct channel_info {
    /** the ID of specific light */
    uint8_t                     id;
    /** light configuration information */
    struct channel_config       cfg;
    /** light brightness */
    uint8_t                     brightness;
    /** fade-in level */
    uint8_t                     fade_in;
    /** fade-out level */
    uint8_t                     fade_out;
    /** a current intensity in microamps */
    uint32_t                    current;
    /** a period for stopping strobe */
    uint32_t                    timeout;
    /** light strobe to be ON or OFF */
    bool                        strobe;
    /** light time on period for blink mode */
    uint16_t                    time_on_ms;
    /** light time off period for blink mode */
    uint16_t                    time_off_ms;
    /** channel flash configuration information */
    struct channel_flash_config fcfg;
};

/**
 * Lights light configuration
 */
struct light_config {
    /** the number of channels */
    uint8_t             channel_count;
    /** light name, 31 characters with null character */
    char                name[NAME_LENGTH];
};

/**
 * Lights light info
 */
struct light_info {
    /** the ID of specific light */
    uint8_t             id;
    /** light configuration information */
    struct light_config cfg;
    /** pointer to channel configuration of this light */
    struct channel_info *channels;
};

/**
 * @brief Lights event callback function
 *
 * @param event event type.
 * @return 0 on success, negative errno on error
 */
typedef int (*lights_event_callback)(uint8_t light_id, uint8_t event);

/**
 * Lights device driver operations
 */
struct device_lights_type_ops {
    /** Get lights count */
    int (*get_lights)(struct device *dev, uint8_t *lights);
    /** Get light config */
    int (*get_light_config)(struct device *dev, uint8_t id,
                            struct light_config *cfg);
    /** Get channel config */
    int (*get_channel_config)(struct device *dev, uint8_t light_id,
                              uint8_t channel_id, struct channel_config *cfg);
    /** Get channel flash config */
    int (*get_channel_flash_config)(struct device *dev, uint8_t light_id,
                                    uint8_t channel_id,
                                    struct channel_flash_config *fcfg);
    /** Set light brightness */
    int (*set_brightness)(struct device *dev, uint8_t light_id,
                          uint8_t channel_id, uint8_t brightness);
    /** Set light blink */
    int (*set_blink)(struct device *dev, uint8_t light_id, uint8_t channel_id,
                     uint16_t time_on_ms, uint16_t time_off_ms);
    /** Set light color */
    int (*set_color)(struct device *dev, uint8_t light_id, uint8_t channel_id,
                     uint32_t color);
    /** Set light fade */
    int (*set_fade)(struct device *dev, uint8_t light_id, uint8_t channel_id,
                    uint8_t fade_in, uint8_t fade_out);
    /** Register lights notify event */
    int (*register_callback)(struct device *dev,
                             lights_event_callback callback);
    /** Remove lights notify event */
    int (*unregister_callback)(struct device *dev);
    /** Set flash light intensity */
    int (*set_flash_intensity)(struct device *dev, uint8_t light_id,
                               uint8_t channel_id, uint32_t intensity);
    /** Set flash light strobe */
    int (*set_flash_strobe)(struct device *dev, uint8_t light_id,
                            uint8_t channel_id, uint8_t strobe);
    /** Set flash light timeout */
    int (*set_flash_timeout)(struct device *dev, uint8_t light_id,
                             uint8_t channel_id, uint32_t timeout);
    /** Get flash light fault */
    int (*get_flash_fault)(struct device *dev, uint8_t light_id,
                           uint8_t channel_id, uint32_t *fault);
};

/**
 * @brief Lights get lights count wrap function
 *
 * @param dev pointer to structure of device data
 * @param lights pointer to the variable of lights count
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_get_lights(struct device *dev, uint8_t *lights)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->get_lights) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->get_lights(dev, lights);
    }
    return -ENOSYS;
}

/**
 * @brief Lights get light configuration wrap function
 *
 * @param dev pointer to structure of device data
 * @param id the ID of specific light
 * @param cfg pointer to the light_config structure to receive light
 *            configuration information
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_get_light_config(struct device *dev,
                                                 uint8_t id,
                                                 struct light_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->get_light_config) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->get_light_config(dev,
                                                                    id,
                                                                    cfg);
    }
    return -ENOSYS;
}

/**
 * @brief Lights get channel configuration wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param cfg pointer to the channel_config structure to receive channel
 *            configuration information
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_get_channel_config(struct device *dev,
                                                   uint8_t light_id,
                                                   uint8_t channel_id,
                                                   struct channel_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->get_channel_config) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->get_channel_config(dev,
                                                                    light_id,
                                                                    channel_id,
                                                                    cfg);
    }
    return -ENOSYS;
}

/**
 * @brief Lights get channel flash config wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param fcfg pointer to the channel_flash_config structure to receive channel
 *             flash configuration information
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_get_channel_flash_config(struct device *dev,
                                            uint8_t light_id,
                                            uint8_t channel_id,
                                            struct channel_flash_config *fcfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->get_channel_flash_config) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->get_channel_flash_config(dev,
                                                                    light_id,
                                                                    channel_id,
                                                                    fcfg);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set brightness wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param brightness light brightness to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_brightness(struct device *dev,
                                               uint8_t light_id,
                                               uint8_t channel_id,
                                               uint8_t brightness)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_brightness) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_brightness(dev,
                                                                  light_id,
                                                                  channel_id,
                                                                  brightness);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set blink wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param time_on_ms light time on in milliseconds to be set
 * @param time_off_ms light time off in milliseconds to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_blink(struct device *dev,
                                          uint8_t light_id,
                                          uint8_t channel_id,
                                          uint16_t time_on_ms,
                                          uint16_t time_off_ms)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_blink) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_blink(dev,
                                                             light_id,
                                                             channel_id,
                                                             time_on_ms,
                                                             time_off_ms);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set color wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param color light color to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_color(struct device *dev,
                                          uint8_t light_id,
                                          uint8_t channel_id,
                                          uint32_t color)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_color) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_color(dev,
                                                             light_id,
                                                             channel_id,
                                                             color);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set fade wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param fade_in light fade in to be set
 * @param fade_out light fade out to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_fade(struct device *dev,
                                         uint8_t light_id,
                                         uint8_t channel_id,
                                         uint8_t fade_in,
                                         uint8_t fade_out)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_fade) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_fade(dev,
                                                            light_id,
                                                            channel_id,
                                                            fade_in,
                                                            fade_out);
    }
    return -ENOSYS;
}

/**
 * @brief Lights register_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_register_callback(struct device *dev,
                                                lights_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->register_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->register_callback(dev,
                                                                     callback);
    }
    return -ENOSYS;
}

/**
 * @brief Lights unregister_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->unregister_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->unregister_callback(dev);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set flash intensity wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param intensity flash light intensity to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_flash_intensity(struct device *dev,
                                                    uint8_t light_id,
                                                    uint8_t channel_id,
                                                    uint32_t intensity)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_intensity) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_intensity(dev,
                                                                    light_id,
                                                                    channel_id,
                                                                    intensity);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set flash strobe wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param strobe flash light strobe to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_flash_strobe(struct device *dev,
                                                 uint8_t light_id,
                                                 uint8_t channel_id,
                                                 uint8_t strobe)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_strobe) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_strobe(dev,
                                                                    light_id,
                                                                    channel_id,
                                                                    strobe);
    }
    return -ENOSYS;
}

/**
 * @brief Lights set flash timeout wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param timeout flash light timeout to be set
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_set_flash_timeout(struct device *dev,
                                                  uint8_t light_id,
                                                  uint8_t channel_id,
                                                  uint32_t timeout)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_timeout) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->set_flash_timeout(dev,
                                                                     light_id,
                                                                     channel_id,
                                                                     timeout);
    }
    return -ENOSYS;
}

/**
 * @brief Lights get flash fault wrap function
 *
 * @param dev pointer to structure of device data
 * @param light_id the ID of specific light
 * @param channel_id the ID of specific channel
 * @param fault pointer to the variable of flash fault
 * @return 0 on success, negative errno on error
 */
static inline int device_lights_get_flash_fault(struct device *dev,
                                                uint8_t light_id,
                                                uint8_t channel_id,
                                                uint32_t *fault)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, lights)->get_flash_fault) {
        return DEVICE_DRIVER_GET_OPS(dev, lights)->get_flash_fault(dev,
                                                                   light_id,
                                                                   channel_id,
                                                                   fault);
    }
    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_LIGHTS_H */
