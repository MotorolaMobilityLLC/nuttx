/**
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>

#include <arch/board/mods.h>

#include <nuttx/device.h>
#include <nuttx/device_battery.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/battery.h>
#include <nuttx/power/battery_state.h>

#define SHUTDOWN_TEMPERATURE         500

/* Defaults to use if driver returns error */
#define DEFAULT_BATT_STATUS         BATTERY_STATUS_UNKNOWN
#define DEFAULT_BATT_MAX_VOLTAGE    0                         /* uV   */
#define DEFAULT_BATT_VOLTAGE        0                         /* uV   */
#define DEFAULT_BATT_CURRENT        0                         /* uA   */
#define DEFAULT_BATT_CAPACITY       0                         /* %    */
#define DEFAULT_BATT_TOTAL_CAPACITY 0                         /* mAh  */
#define DEFAULT_BATT_TEMP           250                       /* 0.1C */

/**
 * @brief private battery device information
 */
struct batt_info {
    struct i2c_dev_s *i2c;
    struct battery_dev_s *bdev;
    enum batt_level_e current_level;
};

static struct batt_info *g_batt_info;
static int get_capacity_internal(struct batt_info *info, b16_t *capacity);

#ifdef CONFIG_ARCH_BUTTONS
int battery_indicator_probe(void);
int battery_indicator_open(void);
void battery_indicator_remove(void);
#endif

/**
 * @brief Get battery technology.
 * @param dev The pointer to the battery device structure.
 * @param tech The output value as technology.
 * @return 0 for success, -errno for failures.
 */
static int batt_get_technology(struct device *dev, uint32_t *tech)
{
    /* check input parameter */
    if (!tech)
        return -EINVAL;

    *tech = BATTERY_TECH_LION;

    return OK;
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
    int ret;
    int value;

    /* check input parameters */
    if (!dev || !status)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->state(info->bdev, &value);
    if (ret < 0) {
        *status = DEFAULT_BATT_STATUS;
        return OK;
    }

    /* Convert to battery device values */
    switch (value) {
        case BATTERY_IDLE:
            *status = BATTERY_STATUS_NOT_CHARGING;
            break;
        case BATTERY_FULL:
            *status = BATTERY_STATUS_FULL;
            break;
        case BATTERY_CHARGING:
            *status = BATTERY_STATUS_CHARGING;
            break;
        case BATTERY_DISCHARGING:
            *status = BATTERY_STATUS_DISCHARGING;
            break;
        case BATTERY_UNKNOWN:
        default:
            *status = BATTERY_STATUS_UNKNOWN;
            break;
    }

    return OK;
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
    int ret;

    /* check input parameters */
    if (!dev || !max_voltage)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->max_voltage(info->bdev, (b16_t *)max_voltage);
    if (ret < 0)
        *max_voltage = DEFAULT_BATT_MAX_VOLTAGE;

    return OK;
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
    int ret;

    /* check input parameters */
    if (!dev || !voltage)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->voltage(info->bdev, (b16_t *)voltage);
    if (ret < 0)
        *voltage = DEFAULT_BATT_VOLTAGE;

    return OK;
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
    int ret;

    /* check input parameters */
    if (!dev || !current)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->current(info->bdev, (b16_t *)current);
    if (ret < 0)
        *current = DEFAULT_BATT_CURRENT;

    return OK;
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

    /* check input parameters */
    if (!dev || !capacity)
        return -EINVAL;

    info = device_get_private(dev);
    return get_capacity_internal(info, (b16_t *)capacity);
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
    int ret;

    /* check input parameters */
    if (!dev || !capacity)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->full_capacity(info->bdev, (b16_t *)capacity);
    if (ret < 0)
        *capacity = DEFAULT_BATT_TOTAL_CAPACITY;

    return OK;
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
    int ret;

    /* check input parameters */
    if (!dev || !temperature)
        return -EINVAL;

    info = device_get_private(dev);
    ret = info->bdev->ops->temperature(info->bdev, (b16_t *)temperature);
    if (ret < 0)
        *temperature = DEFAULT_BATT_TEMP;

    return OK;
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
    /* check input parameters */
    if (!dev || !shutdown_temp)
        return -EINVAL;

    *shutdown_temp = SHUTDOWN_TEMPERATURE;
    return OK;
}

static void battery_state_changed_cb(void *arg,
        const struct batt_state_s *batt)
{
    struct batt_info *info = arg;
    if (info->current_level == batt->level)
        return;

    info->current_level = batt->level;
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
    int retval = OK;

    /* check input parameters */
    if (!dev || !device_get_private(dev))
        return -EINVAL;

    info = device_get_private(dev);
    if (!info->bdev) {
        info->bdev = get_battery();
        if (!info->bdev)
            return -ENODEV;
    }

    retval = battery_state_register(battery_state_changed_cb, info);
    if (retval) {
        dbg("failed to register battery_state callback\n");
        return retval;
    }

#ifdef CONFIG_ARCH_BUTTONS
    retval = battery_indicator_open();
#endif

    return retval;
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

    info->current_level = BATTERY_LEVEL_EMPTY;

    device_set_private(dev, info);
    g_batt_info = info;

#ifdef CONFIG_ARCH_BUTTONS
    battery_indicator_probe();
#endif

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

#ifdef CONFIG_ARCH_BUTTONS
    battery_indicator_remove();
#endif

    /* check input parameters */
    if (!dev || !device_get_private(dev))
        return;

    info = device_get_private(dev);

    free(info);
    g_batt_info = NULL;
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
    .type_ops           = &batt_type_ops,
};

struct device_driver batt_driver = {
    .type       = DEVICE_TYPE_BATTERY_DEVICE,
    .name       = "max17050_battery",
    .desc       = "MAX17050 Battery Driver",
    .ops        = &batt_driver_ops,
};

static int get_capacity_internal(struct batt_info *info, b16_t *capacity)
{
    int ret;

    /* check input parameters */
    if (!info || !capacity)
        return -EINVAL;

    ret = info->bdev->ops->capacity(info->bdev, capacity);
    if (ret < 0)
        *capacity = DEFAULT_BATT_CAPACITY;

    return OK;
}

int stm32_battery_get_capacity(b16_t *capacity)
{
    return get_capacity_internal(g_batt_info, capacity);
}

enum batt_level_e stm32_battery_get_level(void)
{
    if (g_batt_info)
        return g_batt_info->current_level;
    else
        return BATTERY_LEVEL_EMPTY;
}
