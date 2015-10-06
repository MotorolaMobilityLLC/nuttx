/*
 * Copyright (c) 2015 Google Inc.
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 * Author: Alexandre Bailon <abailon@baylibre.com>
 */

#include <nuttx/config.h>
#include <nuttx/usb.h>
#include <nuttx/gpio.h>
#include <nuttx/arch.h>

#include "tsb_scm.h"

#define HUB_LINE_N_RESET                    3
#define HUB_RESET_ASSERTION_TIME_IN_USEC    5 /* us */
#define HUB_RESET_DEASSERTION_TIME_IN_MSEC  2 /* ms */

/**
 * Init the USB3813 hub
 *
 * Activate the GPIO line
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_open(struct device *dev)
{
    gpio_activate(HUB_LINE_N_RESET);
    return 0;
}

/**
 * Deinit the USB3813 hub
 *
 * Deactivate the GPIO line
 *
 * @param dev Device
 */
static void usb3813_close(struct device *dev)
{
    gpio_deactivate(HUB_LINE_N_RESET);
}

/**
 * Hold the usb hub under reset
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_hold_reset(struct device *dev)
{
    gpio_direction_out(HUB_LINE_N_RESET, 0);
    up_udelay(HUB_RESET_ASSERTION_TIME_IN_USEC);
    return 0;
}

/**
 * Release the usb hub from reset
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_release_reset(struct device *dev)
{
    gpio_direction_out(HUB_LINE_N_RESET, 1);
    up_mdelay(HUB_RESET_DEASSERTION_TIME_IN_MSEC);
    return 0;
}

/**
 * Reset the USB Hub
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb3813_reset(struct device *dev)
{
    int retval;

    retval = usb3813_hold_reset(dev);
    if (!retval) {
        return retval;
    }

    retval = usb3813_release_reset(dev);

    return retval;
}

static struct device_hsic_type_ops usb3813_type_ops = {
    .reset = usb3813_reset,
    .hold_reset = usb3813_hold_reset,
    .release_reset = usb3813_release_reset,
};

static struct device_driver_ops usb3813_driver_ops = {
    .open = usb3813_open,
    .close = usb3813_close,
    .type_ops = &usb3813_type_ops,
};

struct device_driver usb3813_driver = {
    .type = DEVICE_TYPE_HSIC_DEVICE,
    .name = "usb3813",
    .desc = "USB3813 HSIC Hub Driver",
    .ops = &usb3813_driver_ops,
};
