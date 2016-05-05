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

#ifndef __MODS_RAW_GB_H__
#define __MODS_RAW_GB_H__

#define DEVICE_TYPE_RAW_HW          "raw"

typedef int (*raw_send_callback)(struct device *dev, uint32_t len, uint8_t data[]);

struct device_raw_type_ops {
    int (*recv)(struct device *dev, uint32_t len, uint8_t data[]);
    int (*register_callback)(struct device *dev, raw_send_callback cb);
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief RAW raw_recv() wrap function
 *
 * @param dev pointer to structure of device data
 * @param len the length of the data buffer
 * @param data the raw data
 * @return 0 on success, negative errno on error
 */
static inline int device_raw_recv(struct device *dev,
    uint32_t len, uint8_t data[])
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, raw)->recv) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, raw)->recv(dev, len, data);
}

/**
 * @brief RAW register_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static inline int device_raw_register_callback(struct device *dev,
                                               raw_send_callback cb)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, raw)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, raw)->register_callback(dev, cb);
}


/**
 * @brief RAW unregister_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_raw_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, raw)->unregister_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, raw)->unregister_callback(dev);
}
#endif
