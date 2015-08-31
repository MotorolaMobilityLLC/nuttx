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
#include <nuttx/device_battery.h>

/**
 * @brief battery device state
 */
enum batt_state {
    BATT_STATE_INVALID,
    BATT_STATE_OPEN,
    BATT_STATE_CLOSED,
};

/**
 * @brief private battery device information
 */
struct batt_info {
    /** Driver model representation of the device */
    struct device       *dev;

    /** battery driver flags */
    enum batt_state  state;

    /** battery driver info */
    uint32_t max_voltage;

    uint32_t total_capacity;

    int shutdown_temperature;
};

/**
 * @brief record and initial the battery info value
 *
 * These are fake values for test only, please replace with the values
 * that your battery needs.
 */
#define VOLTAGE                  3900000
#define MAX_VOLTAGE              4200000
#define CURRENT                  2100000
#define PERCENT_CAPACITY              86
#define TOTAL_CAPACITY              2100
#define TEMPERATURE                  310
#define SHUTDOWN_TEMPERATURE         500

/**
 * @brief Get battery technology.
 * @param dev The pointer to the battery device structure.
 * @param tech The output value as technology.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_technology(struct device *dev, uint32_t *tech)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !tech) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *tech = BATTERY_TECH_LION;

    return 0;
}

/**
 * @brief Get battery status.
 * @param dev The pointer to the battery device structure.
 * @param status The output value as battery status.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_status(struct device *dev, uint16_t *status)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !status ) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *status = BATTERY_STATUS_CHARGING;

    return 0;
}

/**
 * @brief Get battery maximum voltage.
 * @param dev The pointer to the battery device structure.
 * @param max_voltage The output value in microvolt as battery maximum voltage.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_max_voltage(struct device *dev, uint32_t *max_voltage)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !max_voltage) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *max_voltage = info->max_voltage;

    return 0;
}

/**
 * @brief Get battery current voltage.
 * @param dev The pointer to the battery device structure.
 * @param voltage The output value in microvolt as battery voltage.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_voltage(struct device *dev, uint32_t *voltage)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !voltage) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *voltage = VOLTAGE;

    return 0;
}

/**
 * @brief Get battery current.
 * @param dev The pointer to the battery device structure.
 * @param current The output value in microampere as battery current.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_current(struct device *dev, int *current)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !current) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *current = CURRENT;

    return 0;
}

/**
 * @brief Get battery percent capacity.
 * @param dev The pointer to the battery device structure.
 * @param capacity The output value expressed as a percentage of total
 *                 capacity as battery percent capacity.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_percent_capacity(struct device *dev, uint32_t *capacity)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !capacity) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *capacity = PERCENT_CAPACITY;

    return 0;
}

/**
 * @brief Get battery total capacity.
 * @param dev The pointer to the battery device structure.
 * @param capacity The output value in mAh as battery total capacity.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_total_capacity(struct device *dev, uint32_t *capacity)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !capacity) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *capacity = info->total_capacity;

    return 0;
}

/**
 * @brief Get battery temperature.
 * @param dev The pointer to the battery device structure.
 * @param temperature The output value in 0.1 Celsius as battery temperature.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_temperature(struct device *dev, int *temperature)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !temperature) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *temperature = TEMPERATURE;

    return 0;
}

/**
 * @brief Get battery shutdown temperature.
 * @param dev The pointer to the battery device structure.
 * @param shutdown_temp The output value in 0.1 Celsius as battery shutdown
 *                      temperature.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_shutdown_temp(struct device *dev, int *shutdown_temp)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !shutdown_temp) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != BATT_STATE_OPEN) {
        return -EPERM;
    }

    *shutdown_temp = info->shutdown_temperature;

    return 0;
}

/**
 * @brief Open battery device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int batt_dev_open(struct device *dev)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

     if(info->state == BATT_STATE_OPEN) {
        return -EBUSY;
    }
    info->state = BATT_STATE_OPEN;

    /* record the battery info */
    info->max_voltage = MAX_VOLTAGE;
    info->total_capacity = TOTAL_CAPACITY;
    info->shutdown_temperature = SHUTDOWN_TEMPERATURE;

    return 0;
}

/**
 * @brief Close battery device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function.
 *
 * @param dev pointer to structure of device data
 */
static void batt_dev_close(struct device *dev)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    info->state = BATT_STATE_CLOSED;

}

/**
* @brief The device probe function.
*
* This function is called by the system to register this driver when the
* system boots up.This function allocates memory for saving driver internal
* information data.
*
* @param dev The pointer to the battery device structure.
* @return 0 for success, -errno for failure.
*/
static int batt_dev_probe(struct device *dev)
{
    struct batt_info *info;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    info->state = BATT_STATE_CLOSED;
    info->dev = dev;
    device_set_private(dev, info);

    return 0;
}

/**
 * @brief The device remove function.
 *
 * This function is called by the system to unregister this driver. It
 * must be called after probe() and open(). It frees the internal information
 * memory space.
 *
 * @param dev The pointer to the battery device structure.
 * @return None.
 */
static void batt_dev_remove(struct device *dev)
{
    struct batt_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->state != BATT_STATE_CLOSED) {
        batt_dev_close(dev);
    }

    free(info);
    device_set_private(dev, NULL);
}

static struct device_battery_type_ops batt_type_ops = {
    .get_technology         = batt_get_technology,
    .get_status             = batt_get_status,
    .get_max_voltage        = batt_get_max_voltage,
    .get_voltage            = batt_get_voltage,
    .get_current            = batt_get_current,
    .get_percent_capacity   = batt_get_percent_capacity,
    .get_total_capacity     = batt_get_total_capacity,
    .get_temperature        = batt_get_temperature,
    .get_shutdown_temp      = batt_get_shutdown_temp,
};

static struct device_driver_ops batt_driver_ops = {
    .probe              = batt_dev_probe,
    .remove             = batt_dev_remove,
    .open               = batt_dev_open,
    .close              = batt_dev_close,
    .type_ops           = &batt_type_ops,
};

struct device_driver batt_driver = {
    .type       = DEVICE_TYPE_BATTERY_DEVICE,
    .name       = "ara_bridge_battery",
    .desc       = "ARA BRIDGE BATTERY Driver",
    .ops        = &batt_driver_ops,
};
