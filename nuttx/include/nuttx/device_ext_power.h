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

#ifndef __INCLUDE_NUTTX_DEVICE_EXT_POWER_H
#define __INCLUDE_NUTTX_DEVICE_EXT_POWER_H

#include <nuttx/device.h>

#include <errno.h>
#include <stdbool.h>

#define DEVICE_TYPE_EXT_POWER_HW  "ext_power"

/** The type of the external power changed callback function
 *
 * @param arg user provided value that will be returned in callback
 * @param current external power source max current output in mA.
 */
typedef void (*ext_power_changed)(void *arg, int current);

/**
 * external power device driver operations
 */
struct device_ext_power_type_ops {
    /** Register for external power changes notifications */
    int (*register_callback)(struct device *dev, ext_power_changed cb, void *arg);
};

/**
 * @brief Register external power changed notification callback
 *
 * @param dev pointer to structure of device data
 * @param callback callback function to notify when external power changes
 * @param arg user provided value that will be returned in callback
 * @return 0 on success, negative errno on error
 */
static inline int device_ext_power_register_callback(struct device *dev,
                                                     ext_power_changed cb,
                                                     void* arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ext_power)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ext_power)->register_callback(dev, cb, arg);
}
#endif
