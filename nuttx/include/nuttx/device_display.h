/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#include <errno.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_DISPLAY_HW          "display"

enum display_type {
    DISPLAY_TYPE_INVALID = 0x00,
    DISPLAY_TYPE_DSI     = 0x01,
    DISPLAY_TYPE_DP      = 0x02,
};

enum display_config_type {
    DISPLAY_CONFIG_TYPE_INVALID         = 0x00,
    DISPLAY_CONFIG_TYPE_EDID_1P3        = 0x01,
    DISPLAY_CONFIG_TYPE_DSI             = 0x02,
    DISPLAY_CONFIG_TYPE_EDID_DOWNSTREAM = 0x03,
};

enum display_config_dsi_mode {
    DISPLAY_CONFIG_DSI_MODE_VIDEO   = 0x00,
    DISPLAY_CONFIG_DSI_MODE_COMMAND = 0x01,
};

enum display_config_dsi_swap {
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_RGB = 0x00,
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_RBG = 0x01,
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_BGR = 0x02,
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_BRG = 0x03,
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_GRB = 0x04,
    DISPLAY_CONFIG_DSI_SWAP_RGB_TO_GBR = 0x05,
};

enum display_config_dsi_continuous_clock {
    DISPLAY_CONFIG_DSI_CONTINUOUS_CLOCK_DISABLED = 0x00,
    DISPLAY_CONFIG_DSI_CONTINUOUS_CLOCK_ENABLED  = 0x01,
};

enum display_config_dsi_eot_mode {
    DISPLAY_CONFIG_DSI_EOT_MODE_NONE   = 0x00,
    DISPLAY_CONFIG_DSI_EOT_MODE_APPEND = 0x01,
};

enum display_config_dsi_vsync_mode {
    DISPLAY_CONFIG_DSI_VSYNC_MODE_NONE = 0x00,
    DISPLAY_CONFIG_DSI_VSYNC_MODE_GPIO = 0x01,
    DISPLAY_CONFIG_DSI_VSYNC_MODE_DCS  = 0x02,
};

enum display_config_dsi_traffic_mode {
    DISPLAY_CONFIG_DSI_TRAFFIC_MODE_NON_BURST_SYNC_PULSE = 0x00,
    DISPLAY_CONFIG_DSI_TRAFFIC_MODE_NON_BURST_SYNC_EVENT = 0x01,
    DISPLAY_CONFIG_DSI_TRAFFIC_MODE_BURST                = 0x02,
};

enum display_config_dsi_pixel_packing {
    DISPLAY_CONFIG_DSI_PIXEL_PACKING_UNPACKED = 0x00,
};

struct display_dsi_config {
    /* MIPI manufacturer ID (http://mid.mipi.org) */
    uint16_t manufacturer_id;
    /* display_config_dsi_mode */
    uint8_t mode;
    /* 1-4 lanes */
    uint8_t num_lanes;

    /* pixels */
    uint16_t width;
    /* pixels */
    uint16_t height;

    /* millimeters */
    uint16_t physical_width_dim;
    /* millimeters */
    uint16_t physical_length_dim;

    /* frames-per-second */
    uint8_t framerate;
    /* bits-per-pixel */
    uint8_t bpp;
    /* must be zero */
    uint16_t reserved0;

    /* Hz */
    uint64_t clockrate;

    /* nanoseconds */
    uint16_t t_clk_pre;
    /* nanoseconds */
    uint16_t t_clk_post;

    /* display_config_dsi_continuous_clock */
    uint8_t continuous_clock;
    /* display_config_dsi_eot_mode */
    uint8_t eot_mode;
    /* display_config_dsi_vsync_mode */
    uint8_t vsync_mode;
    /* display_config_dsi_traffic_mode */
    uint8_t traffic_mode;

    /* DSI virtual channel (VC) ID */
    uint8_t virtual_channel_id;
    /* display_config_dsi_swap */
    uint8_t color_order;
    /* display_config_dsi_pixel_packing* */
    uint8_t pixel_packing;
    /* must be zero */
    uint8_t reserved1;

    /* pixels */
    uint16_t horizontal_front_porch;
    uint16_t horizontal_sync_pulse_width;
    uint16_t horizontal_sync_skew;
    uint16_t horizontal_back_porch;
    uint16_t horizontal_left_border;
    uint16_t horizontal_right_border;

    /* lines */
    uint16_t vertical_front_porch;
    uint16_t vertical_sync_pulse_width;
    uint16_t vertical_back_porch;
    uint16_t vertical_top_border;
    uint16_t vertical_bottom_border;
    uint16_t reserved2;
};

struct display_downstream_config {
    uint32_t max_link_bandwidth_khz;
};

enum display_state {
    DISPLAY_STATE_OFF      = 0x00,
    DISPLAY_STATE_ON       = 0x01,
    DISPLAY_STATE_BLANK    = 0x02,
    DISPLAY_STATE_UNBLANK  = 0x03,
};

enum display_notification_event {
    DISPLAY_NOTIFICATION_EVENT_INVALID     = 0x00,
    DISPLAY_NOTIFICATION_EVENT_FAILURE     = 0x01,
    DISPLAY_NOTIFICATION_EVENT_AVAILABLE   = 0x02,
    DISPLAY_NOTIFICATION_EVENT_UNAVAILABLE = 0x03,
    DISPLAY_NOTIFICATION_EVENT_CONNECT     = 0x04,
    DISPLAY_NOTIFICATION_EVENT_DISCONNECT  = 0x05,
};

typedef int (*display_notification_cb)(struct device *dev,
    enum display_notification_event event);

struct device_display_type_ops {
    int (*host_ready)(struct device *dev);
    int (*get_config)(struct device *dev, uint8_t *display_type,
        uint8_t *config_type, uint32_t *size, uint8_t **config);
    int (*set_config)(struct device *dev, uint8_t index);
    int (*get_state)(struct device *dev, uint8_t *state);
    int (*set_state)(struct device *dev, uint8_t state);
    int (*register_callback)(struct device *dev, display_notification_cb cb);
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief Display host_ready() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_display_host_ready(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->host_ready) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->host_ready(dev);
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
static inline int device_display_get_config(struct device *dev,
    uint8_t *display_type, uint8_t *config_type, size_t *size, uint8_t **config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, display)->get_config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, display)->get_config(dev, display_type,
        config_type, size, config);
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
