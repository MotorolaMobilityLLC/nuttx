/*
 * Copyright (c) 2015 Motorola Mobility LLC.
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

#ifndef __INCLUDE_NUTTX_DEVICE_AUDIO_H
#define __INCLUDE_NUTTX_DEVICE_AUDIO_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_MUC_AUD_HW   "aud_dev"

struct device_aud_vol_range {
    int min;
    int step;
};

struct device_aud_devices {
    uint32_t in_devices;
    uint32_t out_devices;
};

struct device_aud_data {
    void (*report_devices)(struct device *dev, struct device_aud_devices *devices);
};

struct device_aud_dev_type_ops {
    int (*get_volume_db_range)(struct device *dev,
                      struct device_aud_vol_range *vol_range);
    int (*get_supported_use_cases)(struct device *dev, uint32_t *use_cases);
    int (*set_current_use_case)(struct device *dev, uint32_t use_case);
    int (*set_volume)(struct device *dev, uint32_t vol_step);
    int (*set_sys_volume)(struct device *dev, int vol_db);
    int (*get_supp_devices)(struct device *dev, struct device_aud_devices  *devices);
    int (*enable_devices)(struct device *dev, struct device_aud_devices  *devices);
};

static inline int device_audio_get_volume_db_range(struct device *dev,
                                         struct device_aud_vol_range *vol_range)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_volume_db_range)
       return DEVICE_DRIVER_GET_OPS(dev,aud_dev)->get_volume_db_range(dev, vol_range);

    return -ENOSYS;

}

static inline int device_audio_get_supported_use_cases(struct device *dev,
                                                               uint32_t *use_cases)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

     if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supported_use_cases)
         return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supported_use_cases(dev, use_cases);


    return -ENOSYS;

}

static inline int device_audio_set_use_case(struct device *dev,
                                                                 uint32_t use_case)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

     if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supported_use_cases)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_current_use_case(dev, use_case);

    return -ENOSYS;

}

static inline int device_audio_set_volume(struct device *dev,
                                                uint32_t vol_step)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_volume)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_volume(dev, vol_step);

    return -ENOSYS;
}

static inline int device_audio_set_sys_volume(struct device *dev,
                                                int vol_db)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_sys_volume)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->set_sys_volume(dev, vol_db);

    return -ENOSYS;
}

static inline int device_audio_get_supp_devices(struct device *dev,
                                           struct device_aud_devices *devices)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_devices)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->get_supp_devices(dev, devices);

    return -ENOSYS;
}

static inline int device_audio_enable_devices(struct device *dev,
                                            struct device_aud_devices *devices)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, aud_dev)->enable_devices)
        return DEVICE_DRIVER_GET_OPS(dev, aud_dev)->enable_devices(dev, devices);

    return -ENOSYS;
}
#endif
