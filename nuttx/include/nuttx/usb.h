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

#ifndef __NUTTX_USB_H__
#define __NUTTX_USB_H__

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/device.h>

#define DEVICE_TYPE_HSIC_DEVICE "hsic-device"

struct device_hsic_type_ops {
    int (*reset)(struct device *dev);
    int (*hold_reset)(struct device *dev);
    int (*release_reset)(struct device *dev);
};

/**
 * Reset remote HSIC device
 *
 * @param dev HSIC device
 * @return 0 if successful
 */
static inline int device_hsic_reset(struct device *dev)
{
    DEBUGASSERT(dev);
    DEBUGASSERT(dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hsic);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.hsic->reset) {
        return dev->driver->ops->type_ops.hsic->reset(dev);
    }

    return -ENOSYS;
}

/**
 * Hold under reset the remote HSIC device
 *
 * @param dev HSIC device
 * @return 0 if successful
 */
static inline int device_hsic_hold_reset(struct device *dev)
{
    DEBUGASSERT(dev);
    DEBUGASSERT(dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hsic);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.hsic->hold_reset) {
        return dev->driver->ops->type_ops.hsic->hold_reset(dev);
    }

    return -ENOSYS;
}

/**
 * Release from reset the remote HSIC device
 *
 * @param dev HSIC device
 * @return 0 if successful
 */
static inline int device_hsic_release_reset(struct device *dev)
{
    DEBUGASSERT(dev);
    DEBUGASSERT(dev->driver && dev->driver->ops &&
                dev->driver->ops->type_ops.hsic);

    if (dev->state != DEVICE_STATE_OPEN) {
        return -ENODEV;
    }

    if (dev->driver->ops->type_ops.hsic->release_reset) {
        return dev->driver->ops->type_ops.hsic->release_reset(dev);
    }

    return -ENOSYS;
}

#endif /* __NUTTX_USB_H__ */

