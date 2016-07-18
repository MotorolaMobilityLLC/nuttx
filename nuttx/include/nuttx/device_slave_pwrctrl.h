/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#ifndef __DEVICE_SLAVE_PWRCTRL_H__
#define __DEVICE_SLAVE_PWRCTRL_H__

#include <errno.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_SLAVE_PWRCTRL_HW          "slave_pwrctrl"

typedef int (*slave_state_callback)(struct device *dev, uint32_t slave_state);
typedef int (*slave_status_callback)(struct device *dev, uint32_t slave_status);

enum slave_pwrctrl_mode {
    SLAVE_PWRCTRL_POWER_ON          = 0x01,
    SLAVE_PWRCTRL_POWER_OFF         = 0x02,
    SLAVE_PWRCTRL_POWER_FLASH_MODE  = 0x03,
};

enum {
    SLAVE_STATE_DISABLED = 0,
    SLAVE_STATE_ENABLED,
    SLAVE_STATE_RESET,
};

struct device_slave_pwrctrl_type_ops {
    int (*get_mask)(struct device *dev, uint32_t *mask);
    int (*set_mode)(struct device *dev, enum slave_pwrctrl_mode mode);
    int (*register_slave_state_cb)(struct device *dev, slave_state_callback cb);
    int (*unregister_slave_state_cb)(struct device *dev);
    int (*send_slave_state)(struct device *dev, uint32_t slave_state);
    int (*register_slave_status_cb)(struct device *dev, slave_status_callback cb);
    int (*unregister_slave_status_cb)(struct device *dev, slave_status_callback cb);
};

/**
 * @brief SLAVE Power Control get mask
 *
 * @param dev pointer to structure of device data
 * @param mask for the attached SLAVE(s)
 * @return 0 on success, negative errno on error
 */
static inline int device_slave_pwrctrl_get_mask(struct device *dev, uint32_t *mask)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->get_mask) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->get_mask(dev, mask);
}

/**
 * @brief SLAVE Power Control set mode
 *
 * @param dev pointer to structure of device data
 * @param mode to set
 * @return 0 on success, negative errno on error
 */
static inline int device_slave_pwrctrl_set_mode(struct device *dev,
        enum slave_pwrctrl_mode mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->set_mode) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->set_mode(dev, mode);
}

/**
 * @brief SLAVE Power register callback used to send slave state
 *
 * @param dev pointer to structure of device data
 * @param mode to set
 * @return 0 on success, negative errno on error
 */
static inline int device_slave_pwrctrl_register_callback(struct device *dev,
                                       slave_state_callback send_slave_state_cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->register_slave_state_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->register_slave_state_cb(dev,
                                                    send_slave_state_cb);
}

/**
 * @brief  SLAVE Power unregister callback used to send slave state
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_slave_pwrctrl_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->unregister_slave_state_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->unregister_slave_state_cb(dev);
}

/**
 * @brief  SLAVE Power unregister callback used to send slave state
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_slave_pwrctrl_send_slave_state(struct device *dev,
                                                              uint32_t slave_state)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->send_slave_state) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->send_slave_state(dev, slave_state);
}

/**
 * @brief Register a callback to receive status notifications about the APBE.
 *
 * @param dev pointer to structure of device data
 * @param function pointer to the callback to register
 * @return 0 on success, negative errno on error
 */
static inline int
device_slave_pwrctrl_register_status_callback(struct device *dev,
                                              slave_status_callback cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->register_slave_status_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->register_slave_status_cb(dev, cb);
}

/**
 * @brief  Unregister an existing status callback.
 *
 * @param dev pointer to structure of device data
 * @param function pointer to the callback to unregister
 * @return 0 on success, negative errno on error
 */
static inline int
device_slave_pwrctrl_unregister_status_callback(struct device *dev,
                                                slave_status_callback cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->unregister_slave_status_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, slave_pwrctrl)->unregister_slave_status_cb(dev, cb);
}
#endif /* __DEVICE_SLAVE_PWRCTRL_H__ */
