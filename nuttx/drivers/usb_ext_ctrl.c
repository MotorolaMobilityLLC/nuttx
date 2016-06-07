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
 *
 */

/* Simple USB-EXT control driver. This driver does nothing other than sending
   attach or detach indication to registered usb_ext callback listener. */

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_usbtun.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/gpio.h>

struct usb_ext_s {
    usb_ext_event_callback callback;
    uint8_t usb_path_select;
    struct device *usbtun;
};

struct usb_ext_s s_data;

static void do_gb_action(bool attached)
{
    irqstate_t flags;
    flags = irqsave();

    if (s_data.callback) {
        s_data.callback(attached);
    } else {
        lldbg("USB-EXT ctrl driver is not registered\n");
    }

    irqrestore(flags);
}

#ifdef CONFIG_MHB_USBTUN
static int _hsic_status_cb(struct device *dev, uint8_t attached) {
    lldbg("USB: %s\n", attached ? "connected" : "disconnected");
    do_gb_action(attached);
    return 0;
}
#endif

static void do_mhb_action(bool start)
{
    if (start)
        device_usbtun_on(s_data.usbtun);
    else
        device_usbtun_off(s_data.usbtun);
}

void usb_ext_ctrl_notify_attach(void)
{
    if (s_data.usbtun) {
        /*
         * The gb attach indication will be sent when a USB device
         * is connected.
         */
        do_mhb_action(true);
    } else {
        do_gb_action(true);
    }
}

void usb_ext_ctrl_notify_detach(void)
{
    do_gb_action(false);

    if (s_data.usbtun) {
        do_mhb_action(false);
    }
}

static int _register_callback(struct device *dev,
                              usb_ext_event_callback callback)
{
    s_data.callback = callback;

    return 0;
}

static int _unregister_callback(struct device *dev)
{
    s_data.callback = 0;

    return 0;
}

static int _open(struct device *dev)
{
#ifdef CONFIG_MHB_USBTUN
    s_data.usbtun = device_open(DEVICE_TYPE_USBTUN_HW, 0);
    if (!s_data.usbtun)
        return -ENODEV;

    device_usbtun_register_callback(s_data.usbtun, &_hsic_status_cb);
#endif

    return 0;
}

static void _close(struct device *dev)
{
    s_data.callback = 0;

    if (s_data.usbtun) {
        device_usbtun_unregister_callback(s_data.usbtun);
        device_close(s_data.usbtun);
        s_data.usbtun = NULL;
    }

}

static int _probe(struct device *dev)
{
    struct device_resource *res;

    memset(&s_data, 0, sizeof(s_data));

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "USB_PATH_SELECT");
    if (!res) {
        lldbg("Failed to get usb_path_select gpio\n");
        return -ENODEV;
    }
    s_data.usb_path_select = res->start;

#if defined(CONFIG_GREYBUS_USB_EXT_PROTO_2_0) && defined(CONFIG_GREYBUS_USB_EXT_PATH_A)
    gpio_direction_out(s_data.usb_path_select, 1);
#else
    gpio_direction_out(s_data.usb_path_select, 0);
#endif

    return 0;
}

static void _remove(struct device *dev)
{
    gpio_direction_out(s_data.usb_path_select, 0);
    _close(dev);
    memset(&s_data, 0, sizeof(s_data));
}

static struct device_usb_ext_type_ops _type_ops = {
    .register_callback    = _register_callback,
    .unregister_callback  = _unregister_callback,
};

static struct device_driver_ops _driver_ops = {
    .probe              = _probe,
    .remove             = _remove,
    .open               = _open,
    .close              = _close,
    .type_ops           = &_type_ops,
};

struct device_driver usb_ext_ctrl_driver = {
    .type       = DEVICE_TYPE_USB_EXT_HW,
    .name       = "usb-ext ctrl",
    .desc       = "USB EXT control driver",
    .ops        = &_driver_ops,
};
