/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/device.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/usb.h>
#include <arch/board/mods.h>


typedef struct {
    struct device *usb_ext;
    usb_ext_event_callback usb_ext_callback;
} common_data_t;

static common_data_t s_data;


/**
 * probe function called upon device registration.
 */
static int usb2_probe(struct device *dev)
{
    if (!dev)
        return -EINVAL;

    s_data.usb_ext_callback = NULL;

    /* Enable power out the MDK USB2 (TypeC) */
    /* Power is always present on USB3 (microB) */
    gpio_direction_out(GPIO_MODS_VBUS_PWR_EN, 0);
    gpio_set_value(GPIO_MODS_VBUS_PWR_EN, 1);
    return 0;
}

/**
 * remove function called when device is unregistered.
 */
static void usb2_remove(struct device *dev)
{
    if (!dev)
        return;

    s_data.usb_ext_callback = NULL;

    /* Disable power out the MDK USB2 (TypeC) */
    gpio_set_value(GPIO_MODS_VBUS_PWR_EN, 0);
}


static uint8_t usb2_get_attached(void)
{
    /* No hotplug detect for USB2 with the MDK */
    return true;
}

static uint8_t usb2_get_protocol(void)
{
    return GB_USB_EXT_PROTOCOL_2_0;
}

static uint8_t usb2_get_path(void)
{
    return GB_USB_EXT_PATH_A;
}

static uint8_t usb2_get_type(void)
{
    return GB_USB_EXT_REMOTE_DEVICE;
}

int usb2_register_callback(struct device *dev, usb_ext_event_callback callback)
{
    if (!dev)
        return -ENODEV;

    s_data.usb_ext = dev;
    s_data.usb_ext_callback = callback;

    /* Immediately report connected when given the callback */
    callback(dev, true);

    return 0;
}

int usb2_unregister_callback(struct device *dev)
{
    if (!dev)
        return -ENODEV;

    s_data.usb_ext = NULL;
    s_data.usb_ext_callback = NULL;

    return 0;
}


static struct device_usb_ext_type_ops usb2_type_ops = {
    .get_attached         = usb2_get_attached,
    .get_protocol         = usb2_get_protocol,
    .get_path             = usb2_get_path,
    .get_type             = usb2_get_type,
    .register_callback    = usb2_register_callback,
    .unregister_callback  = usb2_unregister_callback,
};

static struct device_driver_ops usb2_driver_ops = {
    .probe              = usb2_probe,
    .remove             = usb2_remove,
    .type_ops           = &usb2_type_ops,
};

struct device_driver usb2_driver = {
    .type       = DEVICE_TYPE_USB_EXT_HW,
    .name       = "usb_ext_usb2",
    .desc       = "USB-EXT USB2 Interface",
    .ops        = &usb2_driver_ops,
};
