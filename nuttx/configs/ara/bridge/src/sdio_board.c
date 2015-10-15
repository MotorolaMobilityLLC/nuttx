/**
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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_sdio_board.h>

#include <nuttx/gpio.h>
#include "tsb_scm.h"

/* SDIO board flags */
#define SDIO_BOARD_FLAG_OPEN BIT(0)

/**
 * @brief SDIO board device private information
 */
struct sdio_board_info {
    /** Device driver handler */
    struct device *dev;
    /** SDIO board driver state */
    uint32_t flags;
    /** SDIO power pin number */
    uint32_t sdio_gpio_power;
    /** SDIO card detect pin number */
    uint32_t sdio_gpio_cd;
};

/**
 * @brief Retrieve SDIO board resource from driver core.
 *
 * This function gets the SDIO power pin number and card detect pin number from
 * driver core infrastructure.
 *
 * @param dev The pointer to device structure.
 * @param info The SDIO board driver information.
 * @return 0 on success, negative errno on error.
 */
static int sdio_board_extract_resources(struct device *dev,
                                        struct sdio_board_info *info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO,
                                    "sdio_gpio_power");
    if (!r) {
        return -EINVAL;
    }
    info->sdio_gpio_power = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO,
                                    "sdio_gpio_cd");
    if (!r) {
        return -EINVAL;
    }
    info->sdio_gpio_cd = r->start;

    return 0;
}

/**
 * @brief Get card detect pin number of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @param pin_number Pin number.
 * @return 0 on success, negative errno on error.
 */
static int sdio_board_get_cd_pin_number(struct device *dev,
                                        uint32_t *pin_number)
{
    struct sdio_board_info *info = device_get_private(dev);

    *pin_number = info->sdio_gpio_cd;

    return 0;
}

/**
 * @brief Configure power of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @param power Power enable.
 * @return 0 on success, negative errno on error.
 */
static int sdio_board_config_power_enable(struct device *dev, bool power)
{
    struct sdio_board_info *info = device_get_private(dev);
    int retval;

    retval = tsb_request_pinshare(TSB_PIN_GPIO9);
    if (retval) {
        lowsyslog("SDIO: cannot get ownership of GPIO9 pin\n");
        return retval;
    }

    /* Switch the pin share mode for GPB2_SD_POWER_EN pin */
    tsb_set_pinshare(TSB_PIN_GPIO9);
    gpio_activate(info->sdio_gpio_power);
    if (power) {
        /* Pin high for GPB2_SD_POWER_EN pin */
        gpio_direction_out(info->sdio_gpio_power, 1);
    } else {
        /* Pin low for GPB2_SD_POWER_EN pin */
        gpio_direction_out(info->sdio_gpio_power, 0);
    }

    return 0;
}

/**
 * @brief Configure card detect pin of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @return 0 on success, negative errno on error.
 */
static int sdio_board_config_card_detect(struct device *dev)
{
    struct sdio_board_info *info = device_get_private(dev);
    int retval;

    retval = tsb_request_pinshare(TSB_PIN_GPIO22);
    if (retval) {
        lowsyslog("SDIO: cannot get ownership of GPIO22 pin\n");
        return retval;
    }

    /* Switch the pin share mode for card detect pin */
    tsb_set_pinshare(TSB_PIN_GPIO22);
    gpio_activate(info->sdio_gpio_cd);
    gpio_direction_in(info->sdio_gpio_cd);
    set_gpio_triggering(info->sdio_gpio_cd, IRQ_TYPE_EDGE_BOTH);

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
* @param dev Pointer to the SDIO board device structure.
* @return 0 for success, negative errno on error
*/
static int sdio_board_dev_open(struct device *dev)
{
    struct sdio_board_info *info = NULL;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SDIO_BOARD_FLAG_OPEN) {
        ret = -EBUSY;
        return ret;
    }

    info->flags |= SDIO_BOARD_FLAG_OPEN;

    return 0;
}

/**
* @brief The device close function.
*
* This function is called when protocol no longer using this driver.
* The driver must be opened before calling this function.
*
* @param dev Pointer to the SDIO board device structure.
* @return None.
*/
static void sdio_board_dev_close(struct device *dev)
{
    struct sdio_board_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (!(info->flags & SDIO_BOARD_FLAG_OPEN)) {
        return;
    }

    info->flags &= ~SDIO_BOARD_FLAG_OPEN;
}

/**
* @brief The device probe function.
*
* This function is called by the system to allocate memory for saving driver
* internal information data when the system boots up.
*
* @param dev Pointer to the SDIO board device structure.
* @return 0 for success, negative errno on error.
*/
static int sdio_board_dev_probe(struct device *dev)
{
    struct sdio_board_info *info = NULL;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    ret = sdio_board_extract_resources(dev, info);
    if (ret) {
        goto err_free_info;
    }

    info->dev = dev;
    device_set_private(dev, info);

    return 0;

err_free_info:
    free(info);

    return ret;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It must be
* called after probe() and open(). It frees the internal information memory
* space.
*
* @param dev Pointer to the SDIO board device structure.
* @return None.
*/
static void sdio_board_dev_remove(struct device *dev)
{
    struct sdio_board_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & SDIO_BOARD_FLAG_OPEN) {
        sdio_board_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static struct device_sdio_board_type_ops sdio_board_type_ops = {
    .get_cd_pin_number = sdio_board_get_cd_pin_number,
    .config_power_enable = sdio_board_config_power_enable,
    .config_card_detect  = sdio_board_config_card_detect,
};

static struct device_driver_ops sdio_board_driver_ops = {
    .probe    = sdio_board_dev_probe,
    .remove   = sdio_board_dev_remove,
    .open     = sdio_board_dev_open,
    .close    = sdio_board_dev_close,
    .type_ops = &sdio_board_type_ops,
};

struct device_driver sdio_board_driver = {
    .type = DEVICE_TYPE_SDIO_BOARD_HW,
    .name = "sdio_board",
    .desc = "SDIO Board Driver",
    .ops  = &sdio_board_driver_ops,
};
