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
#include <nuttx/device_pad_det.h>
#include <nuttx/gpio.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>

/*
 * Priority to report to PM framework when reporting activity. Report high
 * activity since user-initiated.
 */
#define PM_ACTIVITY 10

struct gpio_pad_detect_info {
    uint8_t gpio; /* gpio to detect docked state */
    bool active_low; /* active when gpio is low */
    bool docked; /* current state */
    pad_detect cb; /* callback */
    void *arg; /*callback arg */
    struct work_s work; /* isr work */
};

/*
 * Global pointer is needed because Nuttx IRQs do not provide a pointer to
 * driver private data
 */
static struct gpio_pad_detect_info *g_info = NULL;


static bool docked(void)
{
    uint8_t val;
    val = gpio_get_value(g_info->gpio);
    return (g_info->active_low) ? !val : val;
}

static void gpio_pad_detect_worker(FAR void *arg)
{
    if (g_info->docked == docked())
        return;

    g_info->docked = !g_info->docked;
    /* Call back with current state */
    g_info->cb(g_info->arg, g_info->docked);
}

static int gpio_pad_detect_isr(int irq, void *context)
{
    pm_activity(PM_ACTIVITY);

    if (work_available(&g_info->work))
        work_queue(HPWORK, &g_info->work, gpio_pad_detect_worker, NULL, 0);

    return OK;
}

static int gpio_pad_detect_register_callback(struct device *dev,
                                             pad_detect cb, void *arg)
{
    if (g_info->cb)
        return -EEXIST;

    g_info->cb = cb;
    g_info->arg = arg;
    g_info->docked = docked();
    /* Immediately call back with current state */
    g_info->cb(g_info->arg, g_info->docked);

    /* Enable IRQ */
    gpio_irqattach(g_info->gpio, gpio_pad_detect_isr);
    set_gpio_triggering(g_info->gpio, IRQ_TYPE_EDGE_BOTH);

    /* In case state changed before IRQ was enabled */
    if (work_available(&g_info->work))
        work_queue(HPWORK, &g_info->work, gpio_pad_detect_worker, NULL, 0);

    return 0;
}

static int gpio_pad_detect_probe(struct device *dev)
{
    struct device_resource *r;
    int retval;

    g_info = zalloc(sizeof(*g_info));
    if (!g_info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "pad_det");
    if (!r) {
        r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "pad_det_n");
        if (!r) {
            dbg("failed to get pad det gpio\n");
            retval = -EINVAL;
            goto probe_err;
        } else
            g_info->active_low = true;
    } else
        g_info->active_low = false;

    g_info->gpio = r->start;
    gpio_direction_in(g_info->gpio);

    return 0;

probe_err:
    free(g_info);
    return retval;
}

static struct device_pad_det_type_ops gpio_pad_detect_type_ops = {
    .register_callback = gpio_pad_detect_register_callback,
};

static struct device_driver_ops gpio_pad_detect_driver_ops = {
    .probe          = gpio_pad_detect_probe,
    .type_ops       = &gpio_pad_detect_type_ops,
};

struct device_driver gpio_pad_detect_driver = {
    .type = DEVICE_TYPE_PAD_DET_HW,
    .name = "gpio_pad_detect",
    .desc = "Wireless pad detect with gpio",
    .ops = &gpio_pad_detect_driver_ops,
};
