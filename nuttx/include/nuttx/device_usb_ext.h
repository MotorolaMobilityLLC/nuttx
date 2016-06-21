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

#ifndef __DEVICE_USB_EXT_H__
#define __DEBICE_USB_EXT_H__

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define DEVICE_TYPE_USB_EXT_HW "usb_ext"

/* greybus attach definitions */
#define GB_USB_EXT_PROTOCOL_2_0           0x00
#define GB_USB_EXT_PROTOCOL_3_1           0x01

#define GB_USB_EXT_PATH_A                 0x00
#define GB_USB_EXT_PATH_B                 0x01

#define GB_USB_EXT_REMOTE_DEVICE          0x00
#define GB_USB_EXT_REMOTE_HOST            0x01

typedef int (*usb_ext_event_callback)(struct device *dev, bool attached);

struct device_usb_ext_type_ops {
    uint8_t (*get_attached)(void);
    uint8_t (*get_protocol)(void);
    uint8_t (*get_path)(void);
    uint8_t (*get_type)(void);
    int (*register_callback)(struct device *dev,
                             usb_ext_event_callback callback);
    int (*unregister_callback)(struct device *dev);
};

static inline int device_usb_ext_get_attached(struct device *dev, uint8_t *attached)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_attached) {
        *attached = DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_attached();
        return 0;
    }
    return -ENOSYS;
}

static inline int device_usb_ext_get_protocol(struct device *dev, uint8_t *protocol)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_protocol) {
        *protocol = DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_protocol();
        return 0;
    }
    return -ENOSYS;
}

static inline int device_usb_ext_get_path(struct device *dev, uint8_t *path)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_path) {
        *path = DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_path();
        return 0;
    }
    return -ENOSYS;
}

static inline int device_usb_ext_get_type(struct device *dev, uint8_t *type)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_type) {
        *type = DEVICE_DRIVER_GET_OPS(dev, usb_ext)->get_type();
        return 0;
    }
    return -ENOSYS;
}

static inline int device_usb_ext_register_callback(struct device *dev,
                                            usb_ext_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->register_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, usb_ext)->register_callback(dev,
                                                                     callback);
    }
    return -ENOSYS;
}

static inline int device_usb_ext_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usb_ext)->unregister_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, usb_ext)->unregister_callback(dev);
    }
    return -ENOSYS;
}

#endif /* __DEVICE_USB_EXT_H__ */
