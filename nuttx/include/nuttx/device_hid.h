/*
 * Copyright (c) 2015 Google, Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_HID_H
#define __INCLUDE_NUTTX_DEVICE_HID_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>

#define DEVICE_TYPE_HID_HW          "hid"

/** HID Report type */
#define HID_INPUT_REPORT            0
#define HID_OUTPUT_REPORT           1
#define HID_FEATURE_REPORT          2

/**
 * HID Device Descriptor
 */
struct hid_descriptor {
    /** Length of this descriptor */
    uint8_t length;
    /** Length of the report descriptor */
    uint16_t report_desc_length;
    /** Version of the HID Protocol */
    uint16_t hid_version;
    /** Product ID of the device */
    uint16_t product_id;
    /** Vendor ID of the device */
    uint16_t vendor_id;
    /** Country code of the localized hardware */
    uint8_t country_code;
};

/**
 * @brief HID event callback function
 *
 * @param dev: pointer to structure of device data
 * @param report_type: HID report type
 * @param report: the data buffer that sent by HID device
 * @param len: the length of the data received
 * @return 0 on success, negative errno on error
 */
typedef int (*hid_event_callback)(struct device *dev, uint8_t report_type,
                                  uint8_t *report, uint16_t len);

/**
 * HID device driver operations
 */
struct device_hid_type_ops {
    /** Power-on the HID device */
    int (*power_on)(struct device *dev);
    /** Power-off the HID device */
    int (*power_off)(struct device *dev);
    /** Get HID Descriptor */
    int (*get_descriptor)(struct device *dev, struct hid_descriptor *desc);
    /** Get HID Report Descriptor */
    int (*get_report_descriptor)(struct device *dev, uint8_t *desc);
    /** Get HID report length */
    int (*get_report_length)(struct device *dev, uint8_t report_type,
                             uint8_t report_id);
    /** Get maximum report size in all Report ID for each Report type */
    int (*get_maximum_report_length)(struct device *dev, uint8_t report_type);
    /** Get HID Input / Feature report data */
    int (*get_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    /** Set HID Output / Feature report data */
    int (*set_report)(struct device *dev, uint8_t report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    /** Register HID Report notify event */
    int (*register_callback)(struct device *dev, hid_event_callback callback);
    /** Remove HID Report notify event */
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief HID power_on() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_power_on(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->power_on) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->power_on(dev);
}

/**
 * @brief HID power_off() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_power_off(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->power_off) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->power_off(dev);
}

/**
 * @brief HID get_descriptor() wrap function
 *
 * @param dev pointer to structure of device data
 * @param desc pointer to structure of HID device descriptor
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_get_descriptor(struct device *dev,
                                            struct hid_descriptor *desc)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_descriptor) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_descriptor(dev, desc);
}

/**
 * @brief HID get_report_length() wrap function
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @return the report size on success, negative errno on error
 */
static inline int device_hid_get_report_length(struct device *dev,
                                               uint8_t report_type,
                                               uint8_t report_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_length) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_length(dev, report_type,
                                                              report_id);
}

/**
 * @brief HID get_maximum_report_length() wrap function
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @return the report size on success, negative errno on error
 */
static inline int device_hid_get_max_report_length(struct device *dev,
                                                   uint8_t report_type)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_maximum_report_length) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_maximum_report_length(dev,
                                                             HID_INPUT_REPORT);
}

/**
 * @brief HID get_report_descriptor() wrap function
 *
 * @param dev pointer to structure of device data
 * @param desc pointer to HID report descriptor
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_get_report_descriptor(struct device *dev,
                                                   uint8_t *desc)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_descriptor) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_descriptor(dev, desc);
}

/**
 * @brief HID get_report() wrap function
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @param data pointer of input buffer size
 * @param len max input buffer size
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_get_report(struct device *dev,
                                        uint8_t report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report(dev, report_type,
                                                       report_id, data, len);
}

/**
 * @brief HID set_report() wrap function
 *
 * @param dev pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @param data pointer of output buffer size
 * @param len max output buffer size
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_set_report(struct device *dev,
                                        uint8_t report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->set_report) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->set_report(dev, report_type,
                                                       report_id, data, len);
}

/**
 * @brief HID register_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @param callback callback function for notify event
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_register_callback(struct device *dev,
                                               hid_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->register_callback(dev, callback);
}

/**
 * @brief HID unregister_callback() wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_hid_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->unregister_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->unregister_callback(dev);
}

#endif
