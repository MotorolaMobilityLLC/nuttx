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
 *
 * @author Mark Greer
 */

#ifndef __INCLUDE_NUTTX_DEVICE_PLL_H
#define __INCLUDE_NUTTX_DEVICE_PLL_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_PLL_HW                 "pll"

struct device_pll_type_ops {
    int (*start)(struct device *dev);
    int (*stop)(struct device *dev);
    int (*set_frequency)(struct device *dev, uint32_t frequency);
    int (*query_frequency)(uint32_t frequency);
};

/**
 * @brief Start generating clock signal
 * @param dev PLL device to start
 * @return 0: PLL clock generation started successfully
 *         -errno: Cause of failure
 */
static inline int device_pll_start(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, pll)->start)
        return DEVICE_DRIVER_GET_OPS(dev, pll)->start(dev);

    return -ENOSYS;
}

/**
 * @brief Stop generating clock signal
 * @param dev PLL device to stop
 * @return 0: PLL clock generation stopped
 *         -errno: Cause of failure
 */
static inline int device_pll_stop(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, pll)->stop)
        return DEVICE_DRIVER_GET_OPS(dev, pll)->stop(dev);

    return -ENOSYS;
}

/**
 * @brief Set frequency of PLL (must be done while PLL stopped)
 * @param dev PLL device to set frequency of
 * @param frequency PLL frequency to generate
 * @return 0: PLL frequency set successfully
 *         -errno: Cause of failure
 */
static inline int device_pll_set_frequency(struct device *dev,
                                           uint32_t frequency)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, pll)->set_frequency)
        return DEVICE_DRIVER_GET_OPS(dev, pll)->set_frequency(dev, frequency);

    return -ENOSYS;
}

/**
 * @brief Test if frequency is supported
 * @param dev PLL device to test frequency of
 * @param frequency PLL frequency to support
 * @return 0: PLL frequency set successfully
 *         -errno: Cause of failure
 */
static inline int device_pll_query_frequency(struct device *dev,
                                             uint32_t frequency)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, pll)->query_frequency)
        return DEVICE_DRIVER_GET_OPS(dev, pll)->query_frequency(frequency);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_PLL_H */
