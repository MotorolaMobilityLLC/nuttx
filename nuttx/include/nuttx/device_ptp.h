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

#ifndef __INCLUDE_NUTTX_DEVICE_PTP_H
#define __INCLUDE_NUTTX_DEVICE_PTP_H

#include <nuttx/device.h>

#include <errno.h>
#include <stdint.h>

#define DEVICE_TYPE_PTP_HW  "ptp"

enum ptp_int_snd_functionality {
    PTP_INT_SND_NEVER           = 0x00,
    PTP_INT_SND_SUPPLEMENTAL    = 0x01,
    PTP_INT_SND_LOW_BATT_SAVER  = 0x02,
};

enum ptp_int_rcv_functionality {
    PTP_INT_RCV_NEVER           = 0x00,
    PTP_INT_RCV_FIRST           = 0x01,
    PTP_INT_RCV_SECOND          = 0x02,
    PTP_INT_RCV_PARALLEL        = 0x04,
};

enum ptp_ext_functionality {
    PTP_EXT_NONE                = 0x00,
    PTP_EXT_SUPPORTED           = 0x01,
};

enum ptp_current_flow {
    PTP_CURRENT_OFF             = 0x00,
    PTP_CURRENT_TO_MOD          = 0x01,
    PTP_CURRENT_FROM_MOD        = 0x02,
};

enum ptp_ext_power {
    PTP_EXT_POWER_NOT_PRESENT   = 0x00,
    PTP_EXT_POWER_PRESENT       = 0x01
};

typedef int (*ptp_ext_power_changed_cb)(void);

struct device_ptp_type_ops {
    int (*set_current_flow)(struct device *dev, uint8_t direction);
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    int (*ext_power_present)(struct device *dev, uint8_t *present);
    int (*register_ext_power_changed_cb)(struct device *dev,
                                       ptp_ext_power_changed_cb cb);
    int (*unregister_ext_power_changed_cb)(struct device *dev);
#endif
};

static int device_ptp_set_current_flow(struct device *dev, uint8_t direction)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_current_flow) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_current_flow(dev, direction);
}

#if defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
static int device_ptp_ext_power_present(struct device *dev, uint8_t *present)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->ext_power_present) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->ext_power_present(dev, present);
}

static int device_ptp_register_ext_power_changed_cb(struct device *dev,
                                                  ptp_ext_power_changed_cb cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->register_ext_power_changed_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->register_ext_power_changed_cb(dev, cb);
}

static int device_ptp_unregister_ext_power_changed_cb(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->unregister_ext_power_changed_cb) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->unregister_ext_power_changed_cb(dev);
}
#endif

#endif
