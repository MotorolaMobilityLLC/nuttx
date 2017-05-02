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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arch/byteorder.h>

#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>

/* Config options */
#define HDMI_DISPLAY_CONNECT_ON_ATTACH  (1)

static struct hdmi_display
{
    display_notification_cb callback;
    struct display_downstream_config cfg;
} g_display;

/**
 * Send a notification to the Core
 */
static int hdmi_display_notification(struct device *dev,
        enum display_notification_event event)
{
    struct hdmi_display *display = device_get_private(dev);

    if (display->callback)
    {
        display->callback(dev, event);
    }

    return OK;
}

static int hdmi_display_host_ready(struct device *dev)
{
    dbg("enter\n");

    hdmi_display_notification(dev, DISPLAY_NOTIFICATION_EVENT_AVAILABLE);

    return OK;
}

static const struct display_downstream_config downstream_config = {
#ifdef CONFIG_HDMI_DISPLAY_MAX_BANDWIDTH_KHZ
    .max_link_bandwidth_khz = CONFIG_HDMI_DISPLAY_MAX_BANDWIDTH_KHZ,
#else
    .max_link_bandwidth_khz = 0,
#endif /* CONFIG_HDMI_DISPLAY_MAX_BANDWIDTH_KHZ */
};

static void _hdmi_display_convert_downstream_config(
        const struct display_downstream_config *src,
        struct display_downstream_config *dst)
{
    dst->max_link_bandwidth_khz = cpu_to_le32(src->max_link_bandwidth_khz);
}

static int hdmi_display_get_config(struct device *dev, uint8_t *display_type,
        uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    struct hdmi_display *display = device_get_private(dev);

    *display_type = DISPLAY_TYPE_DP;
    *config_type = DISPLAY_CONFIG_TYPE_EDID_DOWNSTREAM;
    *size = sizeof(downstream_config);
    if (config) {
        _hdmi_display_convert_downstream_config(&downstream_config, &display->cfg);
        *config = (uint8_t*) &display->cfg;
    }

#if HDMI_DISPLAY_CONNECT_ON_ATTACH
    hdmi_display_notification(dev, DISPLAY_NOTIFICATION_EVENT_CONNECT);
#endif

    return 0;
}

static int hdmi_display_register_callback(struct device *dev,
        display_notification_cb callback)
{
    struct hdmi_display *display = device_get_private(dev);
    dbg("callback=0x%p\n", callback);

    display->callback = callback;
    return 0;
}

static int hdmi_display_unregister_callback(struct device *dev)
{
    struct hdmi_display *display = device_get_private(dev);
    display->callback = NULL;
    return 0;
}

static int hdmi_display_probe(struct device *dev)
{
    struct hdmi_display *display = &g_display;

    dbg("enter\n");

    memset(display, 0, sizeof(*display));
    device_set_private(dev, display);

    return 0;
}

static void hdmi_display_remove(struct device *dev) {
    struct hdmi_display *display = device_get_private(dev);
    if (display) {
        device_set_private(dev, NULL);
        memset(display, 0, sizeof(*display));
    }
}

static struct device_display_type_ops hdmi_display_ops = {
    .host_ready = hdmi_display_host_ready,
    .get_config = hdmi_display_get_config,
    .register_callback = hdmi_display_register_callback,
    .unregister_callback = hdmi_display_unregister_callback,
};

static struct device_driver_ops hdmi_display_driver_ops = {
    .probe = hdmi_display_probe,
    .remove = hdmi_display_remove,
    .type_ops = &hdmi_display_ops,
};

struct device_driver hdmi_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "hdmi_display",
    .desc = "HDMI External Display Driver",
    .ops = &hdmi_display_driver_ops,
};
