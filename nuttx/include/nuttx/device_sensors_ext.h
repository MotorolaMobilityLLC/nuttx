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

#ifndef __DEVICE_SENSORS_EXT_H__
#define __DEVICE_SENSORS_EXT_H__

#define DEVICE_TYPE_SENSORS_HW          "sensors_ext"

#include <stdint.h>
#include <nuttx/greybus/types.h>

#define SENSOR_EXT_FLAG_CONTINUOUS_MODE 0x00000000
#define SENSOR_EXT_FLAG_NON_MASKABLE    0x00000001
#define SENSOR_EXT_FLAG_ON-CHANGE_MODE  0x00000002
#define SENSOR_EXT_FLAG_ONE_SHOT_MODE   0x00000004
#define SENSOR_EXT_FLAG_SPECIAL_REPORT  0x00000008

#define SENSOR_DATA_PAYLOAD_SIZE    256

enum {
    UNDEFINED_ID = 0x00,
    ACCELEROMETER_ID = 0x01,
    GEOMAGNETIC_FIELD_ID = 0x02,
    ORIENTATION_ID = 0x03,
    GYROSCOPE_ID = 0x04,
    LIGHT_ID = 0x05,
    PRESSURE_ID = 0x06,
    TEMPERATURE_ID = 0x07,
    PROXIMITY_ID = 0x08,
    GRAVITY_ID = 0x09,
    LINEAR_ACCELERATION_ID = 0x0a,
    ROTATION_VECTOR_ID = 0x0b,
    RELATIVE_HUMIDITY_ID = 0x0c,
    AMBIENT_TEMPERATURE_ID = 0x0d,
    MAGNETIC_FIELD_UNCALIB_ID = 0x0e,
    GAME_RATATION_VECTOR_ID = 0x0f,
    GYROSCOPE_UNCALIDB_ID = 0x10,
    SIGNIFICANT_MOTION_ID = 0x11,
    STEP_DETECTOT_ID = 0x12,
    STEP_COUNTER_ID = 0x13,
    GEOMAGNETIC_ROTATION_VECTOR_ID = 0x14,
    HEART_RATE_ID = 0x15,
    TILT_DETECTORM_ID = 0x16,
    WAKE_GESTURE_ID = 0x17,
    GLANCE_GESTURE_ID = 0x18,
    PICKUP_GESTURE_ID = 0x19,
    WRIST_TILT_GESTURE_ID = 0x1a,
};

struct sensor_info {
    uint32_t    version;
    uint32_t    type;
    uint32_t    max_range;
    uint32_t    resolution;
    uint32_t    power;
    uint32_t    min_delay;
    uint32_t    max_delay;
    uint32_t    fifo_rec;
    uint32_t    fifo_mec;
    uint32_t    flags;
    uint32_t    scale_int;
    uint32_t    scale_nano;
    uint32_t    offset_int;
    uint32_t    offset_nano;
    uint8_t     channels;
    uint8_t     reserved[3];
    uint16_t    name_len;
    uint8_t     name[128];
    uint16_t    vendor_len;
    uint8_t     vendor[128];
    uint16_t    string_type_len;
    uint8_t     string_type[256];
};

struct report_info {
    uint8_t    id;
    uint8_t    flags;
    uint16_t   readings;
    uint64_t   reference_time;
    uint8_t    data_payload[];
} __packed;

struct report_info_data {
    uint8_t    num_sensors_reporting;
    uint8_t    reserved;
    struct report_info reportinfo[];
} __packed;

#define REPORT_INFO_FLAG_FLUSHING        0x01
#define REPORT_INFO_FLAG_FLUSH_COMPLETE  0x02

typedef int (*sensors_ext_event_callback)(uint8_t se_id,
                    struct report_info_data *rinfo_data, uint16_t payload_size);

struct device_sensors_ext_type_ops {
    int (*get_sensor_count)(struct device *dev, uint8_t *count);
    int (*get_sensor_info)(struct device *dev, uint8_t sensor_id,
                            struct sensor_info *sinfo);
    int (*start_reporting)(struct device *dev, uint8_t sensor_id,
                            uint64_t sampling_period,
                            uint64_t max_report_latency);
    int (*flush)(struct device *dev, uint8_t sensor_id);
    int (*stop_reporting)(struct device *dev, uint8_t sensor_id);
    int (*time_sync)(struct device *dev, uint8_t *current_time,
             uint8_t *precesion, uint8_t *accuracy, uint8_t flags);
    int (*register_callback)(struct device *dev, uint8_t se_id,
                           sensors_ext_event_callback callback);
};

/**
 * @brief Sensors Ext get_sensor_count() wrap function
 *
 * @param dev pointer to structure of device data
 * @param count, the number of sensor available
 * @return 0 on success, negative errno on error
 */
static inline int device_sensors_ext_get_sensor_count(struct device *dev,
        uint32_t *count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->get_sensor_count) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->get_sensor_count(dev, count);
}

/**
 * @brief Sensors Ext get_sensor_info() wrap function
 *
 * @param dev pointer to structure of device data
 * @param info, the number of sensor available
 * @return 0 on success, negative errno on error
 */
static inline int device_sensors_ext_get_sensor_info(struct device *dev,
        uint8_t sensor_id, struct sensor_info *sinfo)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->get_sensor_info) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->get_sensor_info(dev,
                                 sensor_id, sinfo);
}

/**
 * @brief Sensors Ext start_reporting() wrap function
 *
 * @param dev pointer to structure of device data
 * @param info, the number of sensor available
 * @return 0 on success, negative errno on error
 */
static inline int device_sensors_ext_start_reporting(struct device *dev,
        uint8_t sensor_id, uint64_t sampling_period, uint64_t max_report_latency)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->start_reporting) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->start_reporting(dev,
                                 sensor_id, sampling_period, max_report_latency);
}

/**
 * @brief Sensors Ext get_report() wrap function
 *
 * @param dev pointer to structure of device data
 * @param info, the number of sensor available
 * @return 0 on success, negative errno on error
 */
static inline int device_sensors_ext_flush(struct device *dev, uint8_t sensor_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->flush) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->flush(dev, sensor_id);
}

/**
 * @brief Sensors Ext stop_reporting() wrap function
 *
 * @param dev pointer to structure of device data
 * @param info, the number of sensor available
 * @return 0 on success, negative errno on error
 */
static inline int device_sensors_ext_stop_reporting(struct device *dev,
        uint8_t sensor_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->stop_reporting) {
        return -ENOSYS;
    }
    return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->stop_reporting(dev,
        sensor_id);
}

static inline int device_sensors_ext_register_callback(struct device *dev,
                                    uint8_t se_id,
                                    sensors_ext_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->register_callback)
        return DEVICE_DRIVER_GET_OPS(dev, sensors_ext)->register_callback(dev,
                                     se_id, callback);

    return -ENOSYS;
}
#endif /* __DEVICE_SENSORS_EXT_H__ */

