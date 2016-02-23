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

#ifndef __INCLUDE_NUTTX_DEVICE_BATTERY_TEMP_H
#define __INCLUDE_NUTTX_DEVICE_BATTERY_TEMP_H

#include <nuttx/device.h>

#include <errno.h>
#include <stdbool.h>

#define DEVICE_TYPE_BATTERY_TEMP_HW  "batt_temp"

/* The type of the batt temperature limits callback function */
typedef void (*batt_temp_limits)(void *arg, bool min);

/* The type of the batt temperature available callback function */
typedef void (*batt_temp_available)(void *arg, bool available);


struct device_batt_temp_type_ops {
    int (*register_limits_cb)(struct device *dev, batt_temp_limits cb, void *arg);
    int (*register_available_cb)(struct device *dev, batt_temp_available cb, void *arg);
    int (*get_temperature)(struct device *dev, int *temp);
    int (*set_limits)(struct device *dev, int min, int max);
};

static inline int device_batt_temp_register_limits_cb(struct device *dev,
                                                      batt_temp_limits cb,
                                                      void* arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, batt_temp)->register_limits_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, batt_temp)->register_limits_cb(dev, cb,
                                                                     arg);
}

static inline int device_batt_temp_register_available_cb(struct device *dev,
                                                         batt_temp_available cb,
                                                         void* arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, batt_temp)->register_available_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, batt_temp)->register_available_cb(dev,
                                                                        cb,
                                                                        arg);
}

static inline int device_batt_temp_get_temperature(struct device *dev,
                                                   int *temp)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, batt_temp)->get_temperature) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, batt_temp)->get_temperature(dev, temp);
}

static inline int device_batt_temp_set_limits(struct device *dev, int min,
                                              int max)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, batt_temp)->set_limits) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, batt_temp)->set_limits(dev, min, max);
}
#endif
