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
    PTP_INT_RCV_PARALLEL        = 0x03,
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
    PTP_EXT_POWER_NOT_PRESENT            = 0x00,
    PTP_EXT_POWER_PRESENT                = 0x01,    /* removed in ver 00.02 */
    PTP_EXT_POWER_WIRELESS_PRESENT       = 0x02,    /* added in ver 00.02 */
    PTP_EXT_POWER_WIRED_PRESENT          = 0x03,    /* added in ver 00.02 */
    PTP_EXT_POWER_WIRED_WIRELESS_PRESENT = 0x04,    /* added in ver 00.02 */
};

/* Last version 'PTP_EXT_POWER_PRESENT' response was supported */
#define PTP_EXT_POWER_PRESENT_LAST_SUPPORTED_MAJOR  0x00
#define PTP_EXT_POWER_PRESENT_LAST_SUPPORTED_MINOR  0x01

enum ptp_power_available {
    PTP_POWER_AVAILABLE_NONE    = 0x00,
    PTP_POWER_AVAILABLE_EXT     = 0x01,
    PTP_POWER_AVAILABLE_INT     = 0x02,
};

enum ptp_power_source {
    PTP_POWER_SOURCE_NONE       = 0x00,
    PTP_POWER_SOURCE_BATTERY    = 0x01,
    PTP_POWER_SOURCE_WIRED      = 0x02,
    PTP_POWER_SOURCE_WIRELESS   = 0x03,
};

enum ptp_power_required {
    PTP_POWER_NOT_REQUIRED      = 0x00,
    PTP_POWER_REQUIRED          = 0x01
};

/* Callback function */
enum ptp_change {
    POWER_PRESENT,
    POWER_REQUIRED,
    POWER_AVAILABLE,
};

typedef int (*ptp_changed)(enum ptp_change change);

struct device_ptp_type_ops {
    int (*set_current_flow)(struct device *dev, uint8_t direction);
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    int (*ext_power_present)(struct device *dev, uint8_t *present);
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    int (*get_max_output_current)(struct device *dev, uint32_t *current);
    int (*power_available)(struct device *dev, uint8_t *available);
    int (*power_source)(struct device *dev, uint8_t *source);
#endif
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    int (*set_max_input_current)(struct device *dev, uint32_t current);
    int (*power_required)(struct device *dev, uint8_t *required);
#endif
    int (*register_callback)(struct device *dev, ptp_changed cb);
};

static inline int device_ptp_set_current_flow(struct device *dev,
                                              uint8_t direction)
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
static inline int device_ptp_ext_power_present(struct device *dev,
                                               uint8_t *present)
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
#endif

#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
static inline int device_ptp_get_max_output_current(struct device *dev,
                                                    uint32_t *current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_output_current) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->get_max_output_current(dev, current);
}

static inline int device_ptp_power_available(struct device *dev,
                                            uint8_t *available)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_available) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_available(dev, available);
}

static inline int device_ptp_power_source(struct device *dev,
                                            uint8_t *source)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_source) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_source(dev, source);
}
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
static inline int device_ptp_set_max_input_current(struct device *dev,
                                                   uint32_t current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_input_current) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->set_max_input_current(dev, current);
}

static inline int device_ptp_power_required(struct device *dev,
                                            uint8_t *required)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->power_required) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->power_required(dev, required);
}
#endif

static inline int device_ptp_register_callback(struct device *dev,
                                               ptp_changed cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, ptp)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, ptp)->register_callback(dev, cb);
}
#endif /* __INCLUDE_NUTTX_DEVICE_PTP_H */
