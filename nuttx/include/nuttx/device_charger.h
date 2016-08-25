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

#ifndef __INCLUDE_NUTTX_DEVICE_CHARGER_H
#define __INCLUDE_NUTTX_DEVICE_CHARGER_H

#include <nuttx/device.h>

#include <errno.h>

#define DEVICE_TYPE_CHARGER_HW  "charger"

/* The type of the charger fault in boost mode callback function */
typedef void (*charger_boost_fault)(void *arg);

/* Limits are in mA and mV */
struct charger_config {
    int input_current_limit;
    int input_voltage_limit;
    int charge_current_limit;
    int charge_voltage_limit;
};

struct device_charger_type_ops {
    int (*send)(struct device *dev, int *current);
    int (*receive)(struct device *dev, const struct charger_config *cfg);
    int (*off)(struct device *dev);
    int (*register_boost_fault_cb)(struct device *dev, charger_boost_fault cb, void *arg);
    int (*max_input_voltage)(struct device *dev, int *voltage);
};

static inline int device_charger_send(struct device *dev, int *current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, charger)->send) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, charger)->send(dev, current);
}

static inline int device_charger_receive(struct device *dev, const struct charger_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, charger)->receive) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, charger)->receive(dev, cfg);
}

static inline int device_charger_off(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, charger)->off) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, charger)->off(dev);
}

static inline int device_charger_register_boost_fault_cb(struct device *dev,
                                              charger_boost_fault cb, void *arg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, charger)->register_boost_fault_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, charger)->register_boost_fault_cb(dev, cb, arg);
}

static inline int device_charger_max_input_voltage(struct device *dev, int *voltage)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, charger)->max_input_voltage) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, charger)->max_input_voltage(dev, voltage);
}
#endif
