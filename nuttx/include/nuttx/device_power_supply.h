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

#ifndef __INCLUDE_NUTTX_DEVICE_POWER_SUPPLY_H
#define __INCLUDE_NUTTX_DEVICE_POWER_SUPPLY_H

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_POWER_SUPPLY_DEVICE "POWER_SUPPLY"

/**
 * @brief Power supply event callback function.
 *
 * @param psy_id Power supply identification number.
 * @param event Event type.
 * @return 0 on success, negative errno on error.
 */
typedef int (*power_supply_event_callback)(uint8_t psy_id, uint8_t event);

/**
 * Power supply description
 */
struct power_supply_description {
    uint8_t  manufacturer[32];
    uint8_t  model[32];
    uint8_t  serial_number[32];
    uint16_t type;
    uint8_t  properties_count;
};

/**
 * Power supply property descriptors
 */
struct power_supply_props_desc {
    uint8_t property;
    uint8_t is_writeable;
};

/**
 * Power supply device driver operations
 */
struct device_power_supply_type_ops {
    /** Gets power supply devices */
    int (*get_supplies)(struct device *dev, uint8_t *supplies_count);
    /** Get a set of configuration parameters */
    int (*get_description)(struct device *dev, uint8_t psy_id,
                           struct power_supply_description *description);
    /** Get number of properties descriptors */
    int (*get_properties_count)(struct device *dev, uint8_t psy_id,
                                uint8_t *properties_count);
    /** Get the set of properties */
    int (*get_prop_descriptors)(struct device *dev, uint8_t psy_id,
                                struct power_supply_props_desc *descriptors);
    /** Get the current value of a property */
    int (*get_property)(struct device *dev, uint8_t psy_id, uint8_t property,
                        uint32_t *prop_val);
    /** Set the current value of a property */
    int (*set_property)(struct device *dev, uint8_t psy_id, uint8_t property,
                        uint32_t prop_val);
    /** Attach callback function to power supply device */
    int (*attach_callback)(struct device *dev,
                           power_supply_event_callback callback);
};

/**
 * @brief Get power supply devices.
 *
 * Get power supply devices controlled by the power supply controller.
 *
 * @param dev Pointer to structure of device.
 * @param supplies_count Number of power supplies controlled.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_get_supplies(struct device *dev,
                                                   uint8_t *supplies_count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_supplies)
        return DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_supplies(dev,
                                                               supplies_count);

    return -ENOSYS;
}

/**
 * @brief Get a set of configuration parameters.
 *
 * Get a set of configuration parameters from a specific power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param psy_id Power supply identification number.
 * @param description A set of parameters representing the configuration of a
 *                    power supply.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_get_description(struct device *dev,
                                                      uint8_t psy_id,
                                  struct power_supply_description *description)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_description)
        return DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_description(dev,
                                                          psy_id, description);

    return -ENOSYS;
}

/**
 * @brief Get number of properties descriptors.
 *
 * Get number of properties descriptors supported by the power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param psy_id Power supply identification number.
 * @param properties_count Number of properties descriptors.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_get_properties_count(struct device *dev,
                                                           uint8_t psy_id,
                                                     uint8_t *properties_count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_properties_count)
        return DEVICE_DRIVER_GET_OPS(dev,
            power_supply)->get_properties_count(dev, psy_id, properties_count);

    return -ENOSYS;
}

/**
 * @brief Get the set of properties.
 *
 * Get the set of properties supported by the power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param psy_id Power supply identification number.
 * @param descriptors The properties descriptors.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_get_prop_descriptors(struct device *dev,
                                                           uint8_t psy_id,
                                   struct power_supply_props_desc *descriptors)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_prop_descriptors)
        return DEVICE_DRIVER_GET_OPS(dev,
                 power_supply)->get_prop_descriptors(dev, psy_id, descriptors);

    return -ENOSYS;
}

/**
 * @brief Get the current value of a property.
 *
 * Get the current value of a property supported by the power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param psy_id Power supply identification number.
 * @param property The defined power supply properties.
 * @param prop_val Property value.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_get_property(struct device *dev,
                                                   uint8_t psy_id,
                                                   uint8_t property,
                                                   uint32_t *prop_val)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_property)
        return DEVICE_DRIVER_GET_OPS(dev, power_supply)->get_property(dev,
                                                   psy_id, property, prop_val);

    return -ENOSYS;
}

/**
 * @brief Set the current value of a property.
 *
 * Set the current value of a property supported by the power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param psy_id Power supply identification number.
 * @param property The defined power supply properties.
 * @param prop_val Property value.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_set_property(struct device *dev,
                                                   uint8_t psy_id,
                                                   uint8_t property,
                                                   uint32_t prop_val)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->set_property)
        return DEVICE_DRIVER_GET_OPS(dev, power_supply)->set_property(dev,
                                                   psy_id, property, prop_val);

    return -ENOSYS;
}

/**
 * @brief Attach callback function to power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static inline int device_power_supply_attach_callback(struct device *dev,
                                          power_supply_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, power_supply)->attach_callback)
        return DEVICE_DRIVER_GET_OPS(dev, power_supply)->attach_callback(dev,
                                                                     callback);

    return -ENOSYS;
}

#endif
