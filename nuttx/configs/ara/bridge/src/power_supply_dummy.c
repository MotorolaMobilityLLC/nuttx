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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_power_supply.h>

/* Power supply flags */
#define POWER_SUPPLY_FLAG_OPEN BIT(0)

/* Power supply type definition */
#define POWER_SUPPLY_BATTERY_TYPE 0x0001

/* Power supply property type definition */
#define POWER_SUPPLY_PROP_STATUS             0x00
#define POWER_SUPPLY_PROP_TECHNOLOGY         0x06
#define POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN 0x0A
#define POWER_SUPPLY_PROP_VOLTAGE_NOW        0x0C
#define POWER_SUPPLY_PROP_CURRENT_NOW        0x11
#define POWER_SUPPLY_PROP_CAPACITY           0x2A
#define POWER_SUPPLY_PROP_TEMP               0x2E
#define POWER_SUPPLY_PROP_TEMP_ALERT_MAX     0x32

/* Power supply property status definition */
#define POWER_SUPPLY_STATUS_CHARGING 0x0001

/* Power supply property technology definition */
#define POWER_SUPPLY_TECH_LION 0x0002

/* Power supply event bit masks definition */
#define POWER_SUPPLY_UPDATE 0x01

/* Power supply device information */
#define SUPPLIES_COUNT     1
#define POWER_SUPPLY_ID    0
#define MANUFACTURER       "POWER_SUPPLY_DRIVER"
#define MODEL              "TEST"
#define SERIAL_NUMBER      "123"
#define PROPERTIES_COUNT   8
#define VOLTAGE_MAX_DESIGN 4200000
#define VOLTAGE_NOW        3900000
#define CURRENT_NOW        2100000
#define CAPACITY           86
#define TEMP               310
#define TEMP_ALERT_MAX     500
#define DESC_LEN           32

/* Property type the power supply device supported */
struct power_supply_props_desc props[] = {
    {POWER_SUPPLY_PROP_STATUS,             0},
    {POWER_SUPPLY_PROP_TECHNOLOGY,         0},
    {POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, 0},
    {POWER_SUPPLY_PROP_VOLTAGE_NOW,        0},
    {POWER_SUPPLY_PROP_CURRENT_NOW,        0},
    {POWER_SUPPLY_PROP_CAPACITY,           0},
    {POWER_SUPPLY_PROP_TEMP,               0},
    {POWER_SUPPLY_PROP_TEMP_ALERT_MAX,     1},
};

/**
 * @brief Power supply device private information.
 */
struct power_supply_info {
    /** Device driver handler */
    struct device *dev;
    /** Power supply device state */
    uint32_t flags;
    /** Power supply status values */
    uint32_t status;
    /** Power supply technologies */
    uint32_t technology;
    /** Maximum value for voltage by design */
    uint32_t voltage_max_design;
    /** Instantaneous voltage value */
    uint32_t voltage_now;
    /** Instantaneous current value */
    uint32_t current_now;
    /** Capacity in percents */
    uint32_t capacity;
    /** Temperature */
    uint32_t temp;
    /** Maximum temperature alert */
    uint32_t temp_alert_max;
    /** Power supply types */
    uint16_t type;
    /** Number of power supplies controlled */
    uint8_t supplies_count;
    /** Manufacturer name */
    uint8_t manufacturer[32];
    /** Model name */
    uint8_t model[32];
    /** Serial number */
    uint8_t serial_number[32];
    /** Number of properties */
    uint8_t properties_count;
    /** Event callback function */
    power_supply_event_callback callback;
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
static int power_supply_get_supplies(struct device *dev,
                                     uint8_t *supplies_count)
{
    struct power_supply_info *info = NULL;

    if (!dev || !supplies_count) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    *supplies_count = info->supplies_count;

    return 0;
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
static int power_supply_get_description(struct device *dev, uint8_t psy_id,
                                  struct power_supply_description *description)
{
    struct power_supply_info *info = NULL;

    if (!dev || !description) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (psy_id == POWER_SUPPLY_ID) {
        memcpy(description->manufacturer, info->manufacturer, DESC_LEN);
        memcpy(description->model, info->model, DESC_LEN);
        memcpy(description->serial_number, info->serial_number, DESC_LEN);
        description->type = info->type;
        description->properties_count = info->properties_count;
    }

    return 0;
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
static int power_supply_get_properties_count(struct device *dev,
                                             uint8_t psy_id,
                                             uint8_t *properties_count)
{
    struct power_supply_info *info = NULL;

    if (!dev || !properties_count) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (psy_id == POWER_SUPPLY_ID) {
        *properties_count = info->properties_count;
    }

    return 0;
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
static int power_supply_get_prop_descriptors(struct device *dev,
                                             uint8_t psy_id,
                                   struct power_supply_props_desc *descriptors)
{
    struct power_supply_info *info = NULL;
    int i = 0;

    if (!dev || !descriptors) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (psy_id == POWER_SUPPLY_ID) {
        for (i = 0; i < info->properties_count; i++) {
            descriptors[i].property = props[i].property;
            descriptors[i].is_writeable = props[i].is_writeable;
        }
    }

    return 0;
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
static int power_supply_get_property(struct device *dev, uint8_t psy_id,
                                     uint8_t property, uint32_t *prop_val)
{
    struct power_supply_info *info = NULL;

    if (!dev || !prop_val) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (psy_id == POWER_SUPPLY_ID) {
        switch (property) {
        case POWER_SUPPLY_PROP_STATUS:
            *prop_val = info->status;
            break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
            *prop_val = info->technology;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
            *prop_val = info->voltage_max_design;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            *prop_val = info->voltage_now;
            break;
        case POWER_SUPPLY_PROP_CURRENT_NOW:
            *prop_val = info->current_now;
            break;
        case POWER_SUPPLY_PROP_CAPACITY:
            *prop_val = info->capacity;
            break;
        case POWER_SUPPLY_PROP_TEMP:
            *prop_val = info->temp;
            break;
        case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
            *prop_val = info->temp_alert_max;
            break;
        default:
            return -EINVAL;
        }
    }

    return 0;
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
static int power_supply_set_property(struct device *dev, uint8_t psy_id,
                                     uint8_t property, uint32_t prop_val)
{
    struct power_supply_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (psy_id == POWER_SUPPLY_ID) {
        switch (property) {
        case POWER_SUPPLY_PROP_STATUS:
            info->status = prop_val;
            break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
            info->technology = prop_val;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
            info->voltage_max_design = prop_val;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            info->voltage_now = prop_val;
            break;
        case POWER_SUPPLY_PROP_CURRENT_NOW:
            info->current_now = prop_val;
            break;
        case POWER_SUPPLY_PROP_CAPACITY:
            info->capacity = prop_val;
            break;
        case POWER_SUPPLY_PROP_TEMP:
            info->temp = prop_val;
            break;
        case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
            info->temp_alert_max = prop_val;
            break;
        default:
            return -EINVAL;
        }
        info->callback(POWER_SUPPLY_ID, POWER_SUPPLY_UPDATE);
    }

    return 0;
}

/**
 * @brief Attach callback function to power supply device.
 *
 * @param dev Pointer to structure of device.
 * @param callback Pointer to event callback function.
 * @return 0 on success, negative errno on error.
 */
static int power_supply_attach_callback(struct device *dev,
                                        power_supply_event_callback callback)
{
    struct power_supply_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    info->callback = callback;

    return 0;
}

/**
* @brief The device open function.
*
* This function is called when protocol preparing to use the driver ops
* in initial stage. It is called after probe() was invoked by the system.
* The function checks whether the driver is already open or not. If it was
* opened, it returns driver busy error number, otherwise it keeps a flag to
* identify the driver was opened and returns success.
*
* @param dev Pointer to the power supply device structure.
* @return 0 for success, negative errno on error
*/
static int power_supply_dev_open(struct device *dev)
{
    struct power_supply_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & POWER_SUPPLY_FLAG_OPEN) {
        return -EBUSY;
    }

    /* Set initial value for the power supply device */
    info->flags |= POWER_SUPPLY_FLAG_OPEN;
    info->supplies_count = SUPPLIES_COUNT;
    memcpy(info->manufacturer, MANUFACTURER, sizeof(info->manufacturer));
    memcpy(info->model, MODEL, sizeof(info->model));
    memcpy(info->serial_number, SERIAL_NUMBER, sizeof(info->serial_number));
    info->type = POWER_SUPPLY_BATTERY_TYPE;
    info->properties_count = PROPERTIES_COUNT;
    info->status = POWER_SUPPLY_STATUS_CHARGING;
    info->technology = POWER_SUPPLY_TECH_LION;
    info->voltage_max_design = VOLTAGE_MAX_DESIGN;
    info->voltage_now = VOLTAGE_NOW;
    info->current_now = CURRENT_NOW;
    info->capacity = CAPACITY;
    info->temp = TEMP;
    info->temp_alert_max = TEMP_ALERT_MAX;

    return 0;
}

/**
* @brief The device close function.
*
* This function is called when protocol no longer using this driver.
* The driver must be opened before calling this function.
*
* @param dev Pointer to the power supply device structure.
* @return None.
*/
static void power_supply_dev_close(struct device *dev)
{
    struct power_supply_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (!(info->flags & POWER_SUPPLY_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~POWER_SUPPLY_FLAG_OPEN;
}

/**
* @brief The device probe function.
*
* This function is called by the system to probe the power supply hardware and
* do power supply early initialization when the system boots up. This function
* allocates memory for saving driver internal information data.
*
* @param dev Pointer to the power supply device structure.
* @return 0 for success, negative errno on error.
*/
static int power_supply_dev_probe(struct device *dev)
{
    struct power_supply_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    device_set_private(dev, info);

    return 0;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It must be
* called after probe() and open(). It frees the internal information memory
* space.
*
* @param dev Pointer to the power supply device structure.
* @return None.
*/
static void power_supply_dev_remove(struct device *dev)
{
    struct power_supply_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & POWER_SUPPLY_FLAG_OPEN) {
        power_supply_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static struct device_power_supply_type_ops power_supply_type_ops = {
    .get_supplies         = power_supply_get_supplies,
    .get_description      = power_supply_get_description,
    .get_properties_count = power_supply_get_properties_count,
    .get_prop_descriptors = power_supply_get_prop_descriptors,
    .get_property         = power_supply_get_property,
    .set_property         = power_supply_set_property,
    .attach_callback      = power_supply_attach_callback,
};

static struct device_driver_ops power_supply_driver_ops = {
    .probe    = power_supply_dev_probe,
    .remove   = power_supply_dev_remove,
    .open     = power_supply_dev_open,
    .close    = power_supply_dev_close,
    .type_ops = &power_supply_type_ops,
};

struct device_driver power_supply_driver = {
    .type = DEVICE_TYPE_POWER_SUPPLY_DEVICE,
    .name = "power_supply",
    .desc = "Power Supply Driver",
    .ops  = &power_supply_driver_ops,
};
