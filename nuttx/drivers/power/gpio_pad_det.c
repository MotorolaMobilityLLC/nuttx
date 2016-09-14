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
#include <nuttx/device_ext_power.h>
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
#define PM_ACTIVITY     10

/* Fixed output voltage */
#define OUTPUT_VOLTAGE  (5000)  /* mV */

struct gpio_pad_detect_info {
    uint8_t gpio; /* gpio to detect docked state */
    bool active_low; /* active when gpio is low */
    device_ext_power_notification cb; /* callback */
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
    /* Run callback */
    g_info->cb(g_info->arg);
}

static int gpio_pad_detect_isr(int irq, void *context)
{
    pm_activity(PM_ACTIVITY);

    if (work_available(&g_info->work))
        work_queue(HPWORK, &g_info->work, gpio_pad_detect_worker, NULL, 0);

    return OK;
}

static int gpio_pad_detect_register_callback(struct device *dev,
                                    device_ext_power_notification cb, void *arg)
{
    if (g_info->cb)
        return -EEXIST;

    g_info->cb = cb;
    g_info->arg = arg;

    /* Enable IRQ */
    gpio_irqattach(g_info->gpio, gpio_pad_detect_isr);
    set_gpio_triggering(g_info->gpio, IRQ_TYPE_EDGE_BOTH);

    return 0;
}

static int gpio_pad_detect_get_output(struct device *dev,
                                      device_ext_power_output_s *output)
{
    if (!output)
        return -EINVAL;

    if (docked()) {
        output->current = CONFIG_PAD_DETECT_DEVICE_GPIO_MAX_OUTPUT_CURRENT;
        output->voltage = OUTPUT_VOLTAGE;
    } else {
        output->current = output->voltage = 0;
    }

    return 0;
}

static int gpio_pad_detect_set_max_output_voltage(struct device *dev, int voltage)
{
    return voltage >= OUTPUT_VOLTAGE ? 0 : -EINVAL;
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

static struct device_ext_power_type_ops gpio_pad_detect_type_ops = {
    .register_callback = gpio_pad_detect_register_callback,
    .set_max_output_voltage = gpio_pad_detect_set_max_output_voltage,
    .get_output = gpio_pad_detect_get_output,
};

static struct device_driver_ops gpio_pad_detect_driver_ops = {
    .probe          = gpio_pad_detect_probe,
    .type_ops       = &gpio_pad_detect_type_ops,
};

struct device_driver gpio_pad_detect_driver = {
    .type = DEVICE_TYPE_EXT_POWER_HW,
    .name = "gpio_pad_detect",
    .desc = "Wireless pad detect with gpio",
    .ops = &gpio_pad_detect_driver_ops,
};
