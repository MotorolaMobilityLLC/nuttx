/*
 * Copyright (c) 2016 Motorola Mobility LLC.
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

#ifndef __DEVICE_MHB_CAMERA_H
#define __DEVICE_MHB_CAMERA_H

#include <errno.h>
#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_MHB_CAMERA_HW "mhb_camera_dev"

#define MHB_CAM_DRIVER_ID   0
#define MHB_CAM_RAW_ID      1
#define MHB_CAM_PWRLIMIT_ID 2

#define DEV_INVOKE_OPS(dev, op, ...) \
    do { \
        DEVICE_DRIVER_ASSERT_OPS(dev); \
        if (!device_is_open(dev)) return -ENODEV; \
        if (DEVICE_DRIVER_GET_OPS(dev, mhb_camera_dev)->op) \
            return DEVICE_DRIVER_GET_OPS(dev, mhb_camera_dev)->op(dev, \
                ##__VA_ARGS__); \
        return 0; \
    } while (0)

#define IMPL_MHB_CAMERA_DEV_OP_ARG0(op) \
    static inline int device_mhb_camera_##op(struct device *dev) \
    { \
        DEV_INVOKE_OPS(dev, op); \
    }

#define IMPL_MHB_CAMERA_DEV_OP_ARG1(op, t0, a0) \
    static inline int device_mhb_camera_##op(struct device *dev, t0 a0) \
    { \
        DEV_INVOKE_OPS(dev, op, a0); \
    }

#define MHB_CAM_DEV_OP(dev, op, ...) \
    dev == NULL? -ENODEV : device_mhb_camera_##op(dev, ##__VA_ARGS__)


typedef void (*mhb_cam_cb)(int reason);

struct device_mhb_camera_dev_type_ops {
    int (*soc_enable)(struct device *dev, uint8_t bootmode);
    int (*soc_disable)(struct device *dev);
    int (*stream_configure)(struct device *dev);
    int (*stream_enable)(struct device *dev);
    int (*stream_disable)(struct device *dev);
    int (*stream_reset)(struct device *dev);
    int (*lens_retract)(struct device *dev);
    int (*lens_extend)(struct device *dev);
    int (*power_limit)(struct device *dev, uint8_t enable);
    int (*get_csi_config)(struct device *dev, void *config);
    int (*get_fw_version)(struct device *dev, uint32_t *fw_ver);
    int (*set_err_callback)(struct device *dev, mhb_cam_cb callback);
};

IMPL_MHB_CAMERA_DEV_OP_ARG0(soc_disable)
IMPL_MHB_CAMERA_DEV_OP_ARG0(stream_enable)
IMPL_MHB_CAMERA_DEV_OP_ARG0(stream_disable)

IMPL_MHB_CAMERA_DEV_OP_ARG1(soc_enable, uint8_t, bootmode)
IMPL_MHB_CAMERA_DEV_OP_ARG1(get_fw_version, uint32_t*, fw_ver)
IMPL_MHB_CAMERA_DEV_OP_ARG1(get_csi_config, struct mhb_cdsi_config**, config)
IMPL_MHB_CAMERA_DEV_OP_ARG1(power_limit, uint8_t, enable)
IMPL_MHB_CAMERA_DEV_OP_ARG1(set_err_callback, mhb_cam_cb, callback)

/* OPTIONAL */
IMPL_MHB_CAMERA_DEV_OP_ARG0(stream_configure)
IMPL_MHB_CAMERA_DEV_OP_ARG0(lens_retract)
IMPL_MHB_CAMERA_DEV_OP_ARG0(lens_extend)
IMPL_MHB_CAMERA_DEV_OP_ARG0(stream_reset)

#endif
