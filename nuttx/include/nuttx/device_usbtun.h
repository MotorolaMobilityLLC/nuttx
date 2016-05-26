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

#ifndef __DEVICE_USBTUN_H__
#define __DEBICE_USBTUN_H__

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define DEVICE_TYPE_USBTUN_HW "usbtun"

typedef int (*usbtun_status_cb)(struct device *dev, uint8_t attached);

struct device_usbtun_type_ops {
    int (*on)(struct device *dev);
    int (*off)(struct device *dev);
    int (*register_callback)(struct device *dev, usbtun_status_cb cb);
    int (*unregister_callback)(struct device *dev);
};

static inline int device_usbtun_on(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usbtun)->on) {
        return DEVICE_DRIVER_GET_OPS(dev, usbtun)->on(dev);
    }
    return -ENOSYS;
}

static inline int device_usbtun_off(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usbtun)->off) {
        return DEVICE_DRIVER_GET_OPS(dev, usbtun)->off(dev);
    }
    return -ENOSYS;
}

static inline int device_usbtun_register_callback(struct device *dev, usbtun_status_cb cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usbtun)->register_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, usbtun)->register_callback(dev, cb);
    }
    return -ENOSYS;
}

static inline int device_usbtun_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, usbtun)->unregister_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, usbtun)->unregister_callback(dev);
    }
    return -ENOSYS;
}

#endif /* __DEBICE_USBTUN_H__ */
