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

#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>

static struct device *gDevice;
static display_notification_cb gCallback;

/**
 * Send a notification to the Core
 */
int hdmi_display_notification(enum display_notification_event event)
{
    if (gCallback)
    {
        gCallback(gDevice, event);
    }

    return OK;
}

static int hdmi_display_host_ready(struct device *dev)
{
    dbg("enter\n");

    hdmi_display_notification(DISPLAY_NOTIFICATION_EVENT_AVAILABLE);

    return OK;
}

static int hdmi_display_get_config_size(struct device *dev, uint32_t *size)
{
    *size = 0;

    return OK;
}

static int hdmi_display_get_config(struct device *dev, uint8_t *display_type, uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    *display_type = DISPLAY_TYPE_DP;
    *config_type = DISPLAY_CONFIG_TYPE_EDID_1P3;
    *size = 0;
    *config = NULL;

    return 0;
}

static int hdmi_display_register_callback(struct device *dev,
        display_notification_cb callback)
{
    dbg("callback=0x%p\n", callback);

    gCallback = callback;
    return 0;
}

static int hdmi_display_unregister_callback(struct device *dev)
{
    gCallback = NULL;
    return 0;
}

static int hdmi_display_probe(struct device *dev)
{
    dbg("enter\n");

    gDevice = dev;

    return 0;
}

static struct device_display_type_ops hdmi_display_ops = {
    .host_ready = hdmi_display_host_ready,
    .get_config_size = hdmi_display_get_config_size,
    .get_config = hdmi_display_get_config,
    .register_callback = hdmi_display_register_callback,
    .unregister_callback = hdmi_display_unregister_callback,
};

static struct device_driver_ops hdmi_display_driver_ops = {
    .probe = hdmi_display_probe,
    .type_ops = &hdmi_display_ops,
};

struct device_driver hdmi_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "hdmi_display",
    .desc = "HDMI External Display Driver",
    .ops = &hdmi_display_driver_ops,
};
