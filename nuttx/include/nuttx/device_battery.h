/**
 * Copyright (c) 2015 Google Inc.
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

#ifndef __ARCH_ARM_DEVICE_BATTERY_H
#define __ARCH_ARM_DEVICE_BATTERY_H

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_BATTERY_DEVICE       "BATTERY"

/* Should match up with battery types in linux/power_supply.h */
#define BATTERY_TECH_UNKNOWN             0x0000
#define BATTERY_TECH_NIMH                0x0001
#define BATTERY_TECH_LION                0x0002
#define BATTERY_TECH_LIPO                0x0003
#define BATTERY_TECH_LIFE                0x0004
#define BATTERY_TECH_NICD                0x0005
#define BATTERY_TECH_LIMN                0x0006

/* Should match up with battery status in linux/power_supply.h */
#define BATTERY_STATUS_UNKNOWN           0x0000
#define BATTERY_STATUS_CHARGING          0x0001
#define BATTERY_STATUS_DISCHARGING       0x0002
#define BATTERY_STATUS_NOT_CHARGING      0x0003
#define BATTERY_STATUS_FULL              0x0004

/**
 * battery device driver operations
 */
struct device_battery_type_ops {
    /** battery get_technology() function pointer */
    int (*get_technology)(struct device *dev, uint32_t *tech);

    /** battery get_status() function pointer */
    int (*get_status)(struct device *dev, uint16_t *status);

    /** battery get_max_voltage() function pointer */
    int (*get_max_voltage)(struct device *dev, uint32_t *max_voltage);

    /** battery get_voltage() function pointer */
    int (*get_voltage)(struct device *dev, uint32_t *voltage);

    /** battery get_current() function pointer */
    int (*get_current)(struct device *dev, int *current);

    /** battery get_percent_capacity() function pointer */
    int (*get_percent_capacity)(struct device *dev, uint32_t *percent_cap);

    /** battery get_total_capacity() function pointer */
    int (*get_total_capacity)(struct device *dev, uint32_t *total_cap);

    /** battery get_temperature() function pointer */
    int (*get_temperature)(struct device *dev, int *temperature);

    /** battery get_shutdown_temperature() function pointer */
    int (*get_shutdown_temp)(struct device *dev, int *shutdown_temp);
};

/**
 * @brief battery get technology function.
 * @param dev pointer to structure of device data.
 * @param tech The output value is technology.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_technology(struct device *dev, uint32_t *tech)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_technology)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_technology(dev, tech);

    return -ENOSYS;
}

/**
 * @brief battery get technology function.
 * @param dev pointer to structure of device data.
 * @param status The output value is battery status.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_status(struct device *dev, uint16_t *status)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_status)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_status(dev, status);

    return -ENOSYS;
}

/**
 * @brief battery get maximum voltage in microvolt.
 * @param dev pointer to structure of device data.
 * @param volt The output value is battery maximum voltage.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_max_voltage(struct device *dev, uint32_t *volt)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_max_voltage)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_max_voltage(dev, volt);

    return -ENOSYS;
}

/**
 * @brief battery get voltage in microvolt.
 * @param dev pointer to structure of device data.
 * @param volt The output value is battery voltage.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_voltage(struct device *dev, uint32_t *volt)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_voltage)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_voltage(dev, volt);

    return -ENOSYS;
}

/**
 * @brief battery get current in microampere.
 * @param dev pointer to structure of device data.
 * @param current The output value is battery current.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_current(struct device *dev, int *current)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_current)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_current(dev, current);

    return -ENOSYS;
}

/**
 * @brief battery get percent capacity.
 * @param dev pointer to structure of device data.
 * @param cap The output value is battery percent capacity.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_percent_capacity(struct device *dev,
                                                  uint32_t *capacity)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_percent_capacity)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->
               get_percent_capacity(dev, capacity);

    return -ENOSYS;
}

/**
 * @brief battery get toal capacity.
 * @param dev pointer to structure of device data.
 * @param cap The output value is battery total capacity.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_total_capacity(struct device *dev,
                                                uint32_t *capacity)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_total_capacity)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->
               get_total_capacity(dev, capacity);

    return -ENOSYS;
}

/**
 * @brief battery get temperature in 0.1 Celsius.
 * @param dev pointer to structure of device data.
 * @param temp The output value is battery temperature.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_temperature(struct device *dev, int *temp)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_temperature)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->get_temperature(dev, temp);

    return -ENOSYS;
}

/**
 * @brief battery get shutdown temperature in 0.1 Celsius.
 * @param dev pointer to structure of device data.
 * @param temp The output value is battery shutdown temperature.
 * @return 0 on success, negative errno on error.
 */
static inline int device_battery_shutdown_temp(struct device *dev,
                                                      int *temp)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, battery)->get_shutdown_temp)
        return DEVICE_DRIVER_GET_OPS(dev, battery)->
               get_shutdown_temp(dev, temp);

    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_BATTERY_H */
