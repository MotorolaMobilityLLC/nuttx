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

#ifndef __DEVICE_CAMERA_EXT_H
#define __DEVICE_CAMERA_EXT_H

#include <errno.h>
#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>

#include "camera_ext_defs.h"

#define DEVICE_TYPE_CAMERA_EXT_HW "camera_ext_dev"

#define DEV_INVOKE_OPS(dev, op, ...) \
    do { \
        DEVICE_DRIVER_ASSERT_OPS(dev); \
        if (!device_is_open(dev)) return -ENODEV; \
        if (DEVICE_DRIVER_GET_OPS(dev, camera_ext_dev)->op) \
            return DEVICE_DRIVER_GET_OPS(dev, camera_ext_dev)->op(dev, \
                ##__VA_ARGS__); \
    } while (0)

#define IMPL_CAMERA_EXT_DEV_OP_ARG0(op) \
    static inline int device_camera_ext_##op(struct device *dev) \
    { \
        DEV_INVOKE_OPS(dev, op); \
    }

#define IMPL_CAMERA_EXT_DEV_OP_ARG1(op, t0, a0) \
    static inline int device_camera_ext_##op(struct device *dev, t0 a0) \
    { \
        DEV_INVOKE_OPS(dev, op, a0); \
    }

#define IMPL_CAMERA_EXT_DEV_OP_ARG2(op, t0, a0, t1, a1) \
    static inline int device_camera_ext_##op(struct device *dev, t0 a0, t1 a1) \
    { \
        DEV_INVOKE_OPS(dev, op, a0, a1); \
    }

#define CALL_CAM_DEV_OP(dev, op, ...) \
    dev == NULL? -ENODEV : device_camera_ext_##op(dev, ##__VA_ARGS__)

struct device_camera_ext_dev_type_ops {
    int (*power_on)(struct device *dev);
    int (*power_off)(struct device *dev);
    int (*stream_on)(struct device *dev);
    int (*stream_off)(struct device *dev);

    int (*input_enum)(struct device *dev,
            struct camera_ext_input *input);
    int (*input_get)(struct device *dev, int *index);
    int (*input_set)(struct device *dev, int index);

    int (*format_enum)(struct device *dev,
            struct camera_ext_fmtdesc *fmtdesc);
    int (*format_get)(struct device *dev, struct camera_ext_format *format);
    int (*format_set)(struct device *dev, struct camera_ext_format *format);

    int (*frmsize_enum)(struct device *dev,
            struct camera_ext_frmsize *frmsize);
    int (*frmival_enum)(struct device *dev,
            struct camera_ext_frmival *frmival);

    int (*stream_set_parm)(struct device *dev,
            struct camera_ext_streamparm *parm);
};

IMPL_CAMERA_EXT_DEV_OP_ARG0(power_on)
IMPL_CAMERA_EXT_DEV_OP_ARG0(power_off)
IMPL_CAMERA_EXT_DEV_OP_ARG0(stream_on)
IMPL_CAMERA_EXT_DEV_OP_ARG0(stream_off)

IMPL_CAMERA_EXT_DEV_OP_ARG1(input_enum, \
    struct camera_ext_input*, input)
IMPL_CAMERA_EXT_DEV_OP_ARG1(input_get, int*, index)
IMPL_CAMERA_EXT_DEV_OP_ARG1(input_set, int, index)

IMPL_CAMERA_EXT_DEV_OP_ARG1(format_enum, \
    struct camera_ext_fmtdesc*, fmtdesc)
IMPL_CAMERA_EXT_DEV_OP_ARG1(format_get, struct camera_ext_format*, format)
IMPL_CAMERA_EXT_DEV_OP_ARG1(format_set, struct camera_ext_format*, format)

IMPL_CAMERA_EXT_DEV_OP_ARG1(frmsize_enum, \
    struct camera_ext_frmsize*, frmsize)
IMPL_CAMERA_EXT_DEV_OP_ARG1(frmival_enum, \
    struct camera_ext_frmival*, frmival)

IMPL_CAMERA_EXT_DEV_OP_ARG1(stream_set_parm,\
    struct camera_ext_streamparm*, parm)

#endif
