/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <nuttx/device.h>
#include <nuttx/device_wrls_tx.h>
#include <nuttx/gpio.h>

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

/* TODO: GPIO polarity is hard-coded, should be configurable instead */
#define WCHG_TX_ENABLE    (0)
#define WCHG_TX_DISABLE   (1)

struct gpio_wrls_tx_info {
    uint8_t gpio;
};

static int gpio_wrls_tx_enable(struct device *dev, bool en)
{
    struct gpio_wrls_tx_info *info = device_get_private(dev);
    gpio_set_value(info->gpio, en ? WCHG_TX_ENABLE : WCHG_TX_DISABLE);
    return 0;
}

static int gpio_wrls_tx_probe(struct device *dev)
{
    struct gpio_wrls_tx_info *info = zalloc(sizeof(*info));
    struct device_resource *r;

    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO,
                                    "wrls_tx_cntrl");
    if (!r) {
        dbg("failed to get wrls_tx_cntrl gpio\n");
        return -EINVAL;
    }

    info->gpio = r->start;
    gpio_direction_out(info->gpio, WCHG_TX_ENABLE);

    device_set_private(dev, info);
    return 0;
}

static struct device_wrls_tx_type_ops gpio_wrls_tx_type_ops = {
    .enable = gpio_wrls_tx_enable,
};

static struct device_driver_ops gpio_wrls_tx_driver_ops = {
    .probe          = gpio_wrls_tx_probe,
    .type_ops       = &gpio_wrls_tx_type_ops,
};

struct device_driver gpio_wrls_tx_driver = {
    .type = DEVICE_TYPE_WRLS_TX_HW,
    .name = "gpio_wrls_tx",
    .desc = "GPIO wireless TX control device driver",
    .ops = &gpio_wrls_tx_driver_ops,
};
