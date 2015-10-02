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

#ifndef __INCLUDE_NUTTX_DEVICE_SDIO_BOARD_H
#define __INCLUDE_NUTTX_DEVICE_SDIO_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>

#define DEVICE_TYPE_SDIO_BOARD_HW "sdio_board"

/**
 * SDIO board device driver operations
 */
struct device_sdio_board_type_ops {
    /** Get card detect pin number of SD host controller */
    int (*get_cd_pin_number)(struct device *dev, uint32_t *pin_number);
    /** Configure power of SD host controller */
    int (*config_power_enable)(struct device *dev, bool power);
    /** Configure card detect pin of SD host controller */
    int (*config_card_detect)(struct device *dev);
};

/**
 * @brief Get card detect pin number of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @param pin_number Pin number.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_board_get_cd_pin_number(struct device *dev,
                                                      uint32_t *pin_number)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, sdio_board)->get_cd_pin_number)
        return DEVICE_DRIVER_GET_OPS(dev,
                                sdio_board)->get_cd_pin_number(dev, pin_number);

    return -ENOSYS;
}

/**
 * @brief Configure power of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @param power Power enable.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_config_power_enable(struct device *dev,
                                                  bool power)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, sdio_board)->config_power_enable)
        return DEVICE_DRIVER_GET_OPS(dev,
                                   sdio_board)->config_power_enable(dev, power);

    return -ENOSYS;
}

/**
 * @brief Configure card detect pin of SD host controller.
 *
 * @param dev Pointer to structure of device.
 * @return 0 on success, negative errno on error.
 */
static inline int device_sdio_config_card_detect(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, sdio_board)->config_card_detect)
        return DEVICE_DRIVER_GET_OPS(dev, sdio_board)->config_card_detect(dev);

    return -ENOSYS;
}

#endif
