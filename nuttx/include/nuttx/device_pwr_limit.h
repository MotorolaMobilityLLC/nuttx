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

#ifndef __DEVICE_PWR_LIMIT__H__
#define __DEVICE_PWR_LIMIT__H__

#include <errno.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_PWR_LIMIT_HW "pwr_limit"

/* the following should match the greybus specification for the */
/* mods control interface                                       */
#define DEV_PWR_LIMIT_LOW                  0x00
#define DEV_PWR_LIMIT_FULL                 0x01

#define DEV_PWR_LIMIT_CAPABILITY_FULL      0x00
#define DEV_PWR_LIMIT_CAPABILITY_REDUCED   0x01
#define DEV_PWR_LIMIT_CAPABILITY_DISABLED  0x02

#define PWR_LIMIT_REASON_TEMP_MASK         0x01
#define PWR_LIMIT_REASON_BATTERY_MASK      0x02
#define PWR_LIMIT_REASON_BPLUS_MASK        0x04

typedef int (*pwr_limit_send_capability_change_cb)(
    struct device *dev,
    uint8_t level,
    uint8_t reason,
    uint16_t vendor);

typedef int (*pwr_limit_current_rsv_cb)(
    struct device *dev,
    uint8_t rsv);

struct device_pwr_limit_type_ops {
    int (*set_limit)(struct device *dev, uint8_t lmt);
    uint8_t (*get_limit)(struct device *dev);
    void (*current_rsv_ack)(struct device *dev, uint8_t rsv);
    int (*register_capability_change_cb)(struct device *dev,
            pwr_limit_send_capability_change_cb cb);
    int (*unregister_capability_change_cb)(struct device *dev);
    int (*register_current_rsv_cb)(struct device *dev,
            pwr_limit_current_rsv_cb cb);
    int (*unregister_current_rsv_cb)(struct device *dev);
};

/**
 * @brief Power Limit Set Limit
 *
 * @param dev pointer to structure of device data
 * @param limit level
 * @return 0 on success, negative errno on error
 */
static inline int device_pwr_limit_set_pwr_limit(struct device *dev,
        uint8_t limit)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->set_limit) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->set_limit(dev, limit);
}

/**
 * @brief Power Limit Get Limit
 *
 * @param dev pointer to structure of device data
 * @param limit level
 * @return 0 on success, negative errno on error
 */
static inline uint8_t device_pwr_limit_get_pwr_limit(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->get_limit) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->get_limit(dev);
}

/**
 * @brief Power Limit Register Capability Change Callback
 *
 * @param dev pointer to structure of device data
 * @param callback function
 * @return 0 on success, negative errno on error
 */
static inline int
device_pwr_limit_register_capability_change_cb(struct device *dev,
        pwr_limit_send_capability_change_cb cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->register_capability_change_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)
        ->register_capability_change_cb(dev, cb);
}

/**
 * @brief Power Limit Unregister Capability Change Callback
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int
device_pwr_limit_unregister_capability_change_cb(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)
            ->unregister_capability_change_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)
            ->unregister_capability_change_cb(dev);
}

/**
 * @brief Power Limit Register Current Reservation
 *
 * @param dev pointer to structure of device data
 * @param callback function
 * @return 0 on success, negative errno on error
 */
static inline int
device_pwr_limit_register_current_rsv(struct device *dev,
        pwr_limit_current_rsv_cb cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->register_current_rsv_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->register_current_rsv_cb(dev, cb);
}

/**
 * @brief Power Limit Unregister Capability Change Callback
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int
device_pwr_limit_unregister_current_rsv(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->unregister_current_rsv_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->unregister_current_rsv_cb(dev);
}

/**
 * @brief Power Limit Current Reservation Complete
 *
 * @param dev pointer to structure of device data
 * @param limit level
 * @return 0 on success, negative errno on error
 */
static inline void device_pwr_limit_current_rsv_ack(struct device *dev, uint8_t rsv)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->current_rsv_ack) {
        return -ENOSYS;
    }

    DEVICE_DRIVER_GET_OPS(dev, pwr_limit)->current_rsv_ack(dev, rsv);
}
#endif
