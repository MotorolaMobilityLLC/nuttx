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
 */

#include <nuttx/config.h>
#include <nuttx/usb.h>
#include <nuttx/gpio.h>
#include <nuttx/arch.h>

#include "tsb_scm.h"

#define HUB_LINE_N_RESET                    0
#define HUB_RESET_ASSERTION_TIME_IN_USEC    5 /* us */
#define HUB_RESET_DEASSERTION_TIME_IN_MSEC  1 /* ms */

/**
 * Init the USB4624 hub
 *
 * Activate the GPIO line
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb4624_open(struct device *dev)
{
    int retval;

    retval = tsb_request_pinshare(TSB_PIN_UART_CTSRTS);
    if (retval) {
        lowsyslog("USB4624: cannot get ownership of USB4624 reset pin.\n");
        return retval;
    }

    tsb_clr_pinshare(TSB_PIN_UART_CTSRTS);
    gpio_activate(HUB_LINE_N_RESET);
    return 0;
}

/**
 * Deinit the USB4624 hub
 *
 * Deactivate the GPIO line
 *
 * @param dev Device
 */
static void usb4624_close(struct device *dev)
{
    gpio_deactivate(HUB_LINE_N_RESET);
    tsb_release_pinshare(TSB_PIN_UART_CTSRTS);
}

/**
 * Hold the usb hub under reset
 *
 * @param dev Device
 * @return 0 if successful
 */
static int usb4624_hold_reset(struct device *dev)
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
static int usb4624_release_reset(struct device *dev)
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
static int usb4624_reset(struct device *dev)
{
    int retval;

    retval = usb4624_hold_reset(dev);
    if (!retval) {
        return retval;
    }

    retval = usb4624_release_reset(dev);

    return retval;
}

static struct device_hsic_type_ops usb4624_type_ops = {
    .reset = usb4624_reset,
    .hold_reset = usb4624_hold_reset,
    .release_reset = usb4624_release_reset,
};

static struct device_driver_ops usb4624_driver_ops = {
    .open = usb4624_open,
    .close = usb4624_close,
    .type_ops = &usb4624_type_ops,
};

struct device_driver usb4624_driver = {
    .type = DEVICE_TYPE_HSIC_DEVICE,
    .name = "usb4624",
    .desc = "USB4624 HSIC Hub Driver",
    .ops = &usb4624_driver_ops,
};
