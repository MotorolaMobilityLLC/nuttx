/*
 * Copyright (C) 2016 Motorola Mobility, LLC.
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
#include <limits.h>
#include <stdlib.h>

#include <nuttx/device.h>
#include <nuttx/device_pwr_limit.h>

static struct device *g_dev;
static pwr_limit_send_capability_change_cb g_pwr_limit_cb;
static uint8_t g_pwr_limit;

/*
 * In a real implementation this might be generated
 * internally and should be static
 */
int pwr_limit_send_capability_change(
        uint8_t level,
        uint8_t reason,
        uint16_t vendor)
{
    pwr_limit_send_capability_change_cb cb = g_pwr_limit_cb;
    int ret = -ENOENT;

    if (cb)
        ret = cb(g_dev, level, reason, vendor);

    return ret;
}

static int pwr_limit_register_capability_change_cb(struct device *dev,
            pwr_limit_send_capability_change_cb cb)
{
    g_pwr_limit_cb = cb;
    return 0;
}

static int pwr_limit_unregister_capability_change_cb(struct device *dev)
{
    g_pwr_limit_cb = NULL;
    return 0;
}

static int pwr_limit_set_limit(struct device *dev, uint8_t limit)
{
    g_pwr_limit = limit;
    return 0;
}

static uint8_t pwr_limit_get_limit(struct device *dev)
{
    return g_pwr_limit;
}

static int pwr_limit_probe(struct device *dev)
{
    g_dev = dev;
    return 0;
}

static struct device_pwr_limit_type_ops pwr_limit_type_ops = {
    .set_limit = pwr_limit_set_limit,
    .get_limit = pwr_limit_get_limit,
    .register_capability_change_cb = pwr_limit_register_capability_change_cb,
    .unregister_capability_change_cb = pwr_limit_unregister_capability_change_cb,
};

static struct device_driver_ops pwr_limit_driver_ops = {
    .probe = pwr_limit_probe,
    .type_ops = &pwr_limit_type_ops,
};

struct device_driver pwr_limit_dummy_driver = {
    .type = DEVICE_TYPE_PWR_LIMIT_HW,
    .name = "pwr_limit_sample",
    .desc = "Sample Power Limit Driver",
    .ops = &pwr_limit_driver_ops,
};
