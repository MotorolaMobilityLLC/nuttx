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

#include <arch/board/mods.h>

#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/gpio.h>

#include <nuttx/display/anx7750.h>
#include <nuttx/display/tc358860xbg.h>
#include <nuttx/display/quickvx_bx5bxa.h>
#include <nuttx/display/ti_dlpc3430.h>

/* 854 x 480 Projector */
static const uint8_t PROJECTOR_EDID_CONFIG[] = {
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
    0x22, 0xF0, 0x4B, 0x28, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x15, 0x01, 0x03, 0x80, 0x34, 0x20, 0x78,
    0xEE, 0x9E, 0xC5, 0xA6, 0x56, 0x4B, 0x9A, 0x25,
    0x13, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1C, 0x0C,
    0x56, 0x64, 0x30, 0xE0, 0x40, 0x10, 0x34, 0x20,
    0x00, 0x06, 0xED, 0xED, 0xDD, 0x00, 0x00, 0x1A,
    0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x3F, 0x18,
    0x4C, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x4C,
    0x41, 0x32, 0x34, 0x30, 0x35, 0x0A, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFF,
    0x00, 0x43, 0x4E, 0x34, 0x31, 0x30, 0x31, 0x30,
    0x30, 0x4C, 0x54, 0x0A, 0x20, 0x20, 0x00, 0x21,
};

static struct projector_display
{
    struct device *display_device;
    display_notification_cb callback;
    uint8_t state;
} g_display;


/**
 * Helper Functions
 */
void projector_on(void)
{
    dbg("enter\n");

    if (g_display.state != DISPLAY_STATE_ON)
    {
#ifdef CONFIG_DISPLAY_QLBX5B3A
        qlbx5bxa_power(true);
#endif

#ifdef CONFIG_DISPLAY_DLPC3430
        dlpc3430_power(true);
#endif

#ifdef CONFIG_DISPLAY_TC358860
        tc358860xbg_power(true);
        tc358860xbg_configure(&qlbx5bxa_configure);
#endif

        g_display.state = DISPLAY_STATE_ON;
    }
}

void projector_off(void)
{
    dbg("enter\n");

    if (g_display.state != DISPLAY_STATE_OFF)
    {
#ifdef CONFIG_DISPLAY_DLPC3430
        dlpc3430_power(false);
#endif

#ifdef CONFIG_DISPLAY_QLBX5B3A
        qlbx5bxa_power(false);
#endif

#ifdef CONFIG_DISPLAY_TC358860
        tc358860xbg_power(false);
#endif

        g_display.state = DISPLAY_STATE_OFF;
    }
}

/**
 * Greybus Display Protocol Implementation
 */
int projector_notification(enum display_notification_event event)
{
    dbg("enter\n");

    if (g_display.callback)
    {
        g_display.callback(g_display.display_device, event);
    }

    return OK;
}

static int projector_host_ready(struct device *dev)
{
    dbg("enter\n");

    projector_notification(DISPLAY_NOTIFICATION_EVENT_AVAILABLE);
    projector_notification(DISPLAY_NOTIFICATION_EVENT_CONNECT);

    return OK;
}

static int projector_get_config_size(struct device *dev, uint32_t *size)
{
    dbg("enter\n");

    *size = sizeof(PROJECTOR_EDID_CONFIG);

    return OK;
}

static int projector_get_config(struct device *dev, uint8_t *display_type,
    uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    dbg("enter\n");

    *display_type = DISPLAY_TYPE_DP;
    *config_type = DISPLAY_CONFIG_TYPE_EDID_1P3;
    *size = sizeof(PROJECTOR_EDID_CONFIG);
    *config = (uint8_t*)PROJECTOR_EDID_CONFIG;

    return 0;
}

static int projector_set_config(struct device *dev, uint8_t index)
{
    dbg("index: %d\n", index);

    return 0;
}

static int projector_get_state(struct device *dev, uint8_t *state)
{
    dbg("enter\n");

    *state = g_display.state;

    return 0;
}

static int projector_set_state(struct device *dev, uint8_t state)
{
    dbg("state: %d\n", state);

    if (state == DISPLAY_STATE_ON)
    {
        projector_on();
    }
    else if (state == DISPLAY_STATE_OFF)
    {
        projector_off();
    }

    return 0;
}

static void projector_attach_cb(FAR void *arg, enum base_attached_e state)
{
    dbg("enter state: %d\n", state);

    switch (state)
    {
    case BASE_DETACHED:
    case BASE_ATTACHED_OFF:
        projector_off();

#ifdef CONFIG_DISPLAY_ANX7750
        anx7750_power(false);
#endif

        usleep(3 * 1000);

        gpio_direction_out(GPIO_MODS_PROJ_DC_EN, 0);
        gpio_direction_out(GPIO_MODS_APBE_PWR_EN, 0);
        break;
    case BASE_ATTACHED:
        gpio_direction_out(GPIO_MODS_PROJ_DC_EN, 1);
        gpio_direction_out(GPIO_MODS_APBE_PWR_EN, 1);

        usleep(3 * 1000);

#ifdef CONFIG_DISPLAY_ANX7750
        anx7750_power(true);
#endif
        break;
    default:
        dbg("Invalid attach state: %d\n", state);
        break;
    }
}


static int projector_register_callback(struct device *dev,
        display_notification_cb callback)
{
    dbg("callback=0x%p\n", callback);

    g_display.callback = callback;

    return 0;
}

static int projector_unregister_callback(struct device *dev)
{
    dbg("enter\n");

    g_display.callback = NULL;

    return 0;
}

static int projector_probe(struct device *dev)
{
    dbg("enter\n");

    memset(&g_display, 0, sizeof(g_display));
    g_display.display_device = dev;
    g_display.state = DISPLAY_STATE_OFF;

#ifdef CONFIG_DISPLAY_ANX7750
    anx7750_driver_init(GPIO_MODS_ANX7750_RST_IN_N);
#endif

#ifdef CONFIG_DISPLAY_TC358860
    tc358860xbg_driver_init(GPIO_MODS_TC358860_RST_IN_N,
                            GPIO_MODS_TC358860_CLK_EN);
#endif

#ifdef CONFIG_DISPLAY_QLBX5B3A
    qlbx5bxa_driver_init(GPIO_MODS_QL_RST_IN_N,
                         GPIO_MODS_REF_CLK_EN);
#endif

#ifdef CONFIG_DISPLAY_DLPC3430
    dlpc3430_driver_init(GPIO_MODS_PROJ_ON,
                         GPIO_MODS_DLPC_HOST_IRQ_N);
#endif

    mods_attach_register(projector_attach_cb, &g_display);

    return 0;
}

static struct device_display_type_ops projector_ops = {
    .host_ready = projector_host_ready,
    .get_config_size = projector_get_config_size,
    .get_config = projector_get_config,
    .set_config = projector_set_config,
    .get_state = projector_get_state,
    .set_state = projector_set_state,
    .register_callback = projector_register_callback,
    .unregister_callback = projector_unregister_callback,
};

static struct device_driver_ops projector_driver_ops = {
    .probe = projector_probe,
    .type_ops = &projector_ops,
};

struct device_driver projector_display_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "projector_display",
    .desc = "Projector Display",
    .ops = &projector_driver_ops,
};
