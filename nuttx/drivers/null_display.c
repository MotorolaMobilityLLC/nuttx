/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

/* Kinzie panel EID 720x2560 */
static const uint8_t g_config[] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
    0x22, 0xf0, 0x4b, 0x28, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x15, 0x01, 0x03, 0x80, 0x34, 0x20, 0x78,
    0xee, 0x9e, 0xc5, 0xa6, 0x56, 0x4b, 0x9a, 0x25,
    0x13, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x86, 0x35,
    0xd0, 0xa0, 0x20, 0x00, 0x23, 0xa0, 0x30, 0x20,
    0x36, 0x00, 0x06, 0x44, 0x21, 0x00, 0x00, 0x1a,
    0x00, 0x00, 0x00, 0xfd, 0x00, 0x32, 0x3f, 0x18,
    0x4c, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x4c,
    0x41, 0x32, 0x34, 0x30, 0x35, 0x0a, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
    0x00, 0x43, 0x4e, 0x34, 0x31, 0x30, 0x31, 0x30,
    0x30, 0x4c, 0x54, 0x0a, 0x20, 0x20, 0x00, 0x75,
};

/**
 * Send a notification to the Core
 *
 * Not likely for this instance since it is a noop
 */
int null_display_notification(enum display_notification_event event)
{
    if (gCallback)
      {
        gCallback(gDevice, event);
      }
    return OK;
}

static int null_display_get_config(struct device *dev, uint8_t *display_type, uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    *display_type = 0;
    *config_type = 0;
    *size = sizeof(g_config);
    if (config)
        *config = g_config;

    return 0;
}

static int null_display_set_config(struct device *dev, uint8_t index)
{
    dbg("%s %d\n", __func__, index);
    return 0;
}

static int null_display_get_state(struct device *dev, uint8_t *state)
{
    dbg("%s\n", __func__);
    *state = 0;
    return 0;
}

static int null_display_set_state(struct device *dev, uint8_t state)
{
    dbg("%s %d\n", __func__, state);
    return 0;
}

static int null_display_register_callback(struct device *dev,
        display_notification_cb callback)
{
    dbg("callback=0x%p\n", callback);

    gCallback = callback;
    return 0;
}

static int null_display_unregister_callback(struct device *dev)
{
    gCallback = NULL;
    return 0;
}

static int null_display_probe(struct device *dev)
{
    dbg("enter\n");

    gDevice = dev;

    return 0;
}

static struct device_display_type_ops null_display_ops = {
    .get_config = null_display_get_config,
    .set_config = null_display_set_config,
    .get_state = null_display_get_state,
    .set_state = null_display_set_state,
    .register_callback = null_display_register_callback,
    .unregister_callback = null_display_unregister_callback,
};

static struct device_driver_ops null_display_driver_ops = {
    .probe = null_display_probe,
    .type_ops = &null_display_ops,
};

struct device_driver null_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "null_display",
    .desc = "Empty Display Driver",
    .ops = &null_display_driver_ops,
};
