/*
 * Copyright (c) 2016 Motorola, LLC.
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
#include <nuttx/device_usb_ext.h>
#include <nuttx/greybus/debug.h>

usb_ext_event_callback s_callback;

static void do_action(bool attached)
{
    if (s_callback) {
        s_callback(attached);
    } else {
        lldbg("USB-EXT ctrl driver is not registered\n");
    }
}

void usb_ext_ctrl_notify_attach(void)
{
    do_action(true);
}

void usb_ext_ctrl_notify_detach(void)
{
    do_action(false);
}

static int _register_callback(struct device *dev,
                              usb_ext_event_callback callback)
{
    s_callback = callback;

    return 0;
}

static int _unregister_callback(struct device *dev)
{
    s_callback = 0;

    return 0;
}

static void _close(struct device *dev)
{
    s_callback = 0;
}

static int _probe(struct device *dev)
{
    s_callback = 0;

    return 0;
}

static void _remove(struct device *dev)
{
    s_callback = 0;
}

static struct device_usb_ext_type_ops _type_ops = {
    .register_callback    = _register_callback,
    .unregister_callback  = _unregister_callback,
};

static struct device_driver_ops _driver_ops = {
    .probe              = _probe,
    .remove             = _remove,
    .close              = _close,
    .type_ops           = &_type_ops,
};

struct device_driver usb_ext_ctrl_driver = {
    .type       = DEVICE_TYPE_USB_EXT_HW,
    .name       = "usb-ext ctrl",
    .desc       = "USB EXT control driver",
    .ops        = &_driver_ops,
};
