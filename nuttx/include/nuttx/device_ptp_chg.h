/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#ifndef __INCLUDE_NUTTX_DEVICE_PTP_CHG_H
#define __INCLUDE_NUTTX_DEVICE_PTP_CHG_H

#include <nuttx/device.h>
#include <nuttx/device_charger.h>

#include <errno.h>

#define DEVICE_TYPE_PTP_CHG_HW  "ptp_chg"

struct device_ptp_chg_type_ops {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    int (*send_wireless_pwr)(struct device *dev);
    int (*send_wired_pwr)(struct device *dev);
    int (*receive_wireless_pwr)(struct device *dev, const struct charger_config *cfg);
    int (*receive_wired_pwr)(struct device *dev, const struct charger_config *cfg);
#endif
    int (*send_batt_pwr)(struct device *dev);
    int (*receive_base_pwr)(struct device *dev, const struct charger_config *cfg);
    int (*off)(struct device *dev);
};

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static inline int device_ptp_chg_send_wireless_pwr(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_wireless_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_wireless_pwr(dev);
}

static inline int device_ptp_chg_send_wired_pwr(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_wired_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_wired_pwr(dev);
}

static inline int device_ptp_chg_receive_wireless_pwr(struct device *dev,
                                               const struct charger_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_wireless_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_wireless_pwr(dev, cfg);
}

static inline int device_ptp_chg_receive_wired_pwr(struct device *dev,
                                               const struct charger_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_wired_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_wired_pwr(dev, cfg);
}
#endif

static inline int device_ptp_chg_send_batt_pwr(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_batt_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->send_batt_pwr(dev);
}

static inline int device_ptp_chg_receive_base_pwr(struct device *dev,
                                              const struct charger_config *cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_base_pwr) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->receive_base_pwr(dev, cfg);
}

static inline int device_ptp_chg_off(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->off) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp_chg)->off(dev);
}
#endif
