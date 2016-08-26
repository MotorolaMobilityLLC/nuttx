/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#include <nuttx/device_battery_good.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <up_arch.h>

#include <arch/board/board.h>
#include <arch/board/mods.h>

/* Priority to report to PM framework when reporting activity */
#define PM_ACTIVITY 10

struct comp_batt_good_info {
    bool good;          /* current state */
    batt_good cb;       /* callback */
    void *arg;          /*callback arg */
    struct work_s work; /* isr work */
};

/*
 * Global pointer is needed because Nuttx IRQs do not provide a pointer to
 * driver private data
 */
static struct comp_batt_good_info *g_info = NULL;

/* Is battery voltage above the threshold? */
static bool good(void)
{
    return BATT_COMP_INV ? !stm32_compread(BATT_COMP) :
                            stm32_compread(BATT_COMP);
}

static void comp_batt_good_worker(FAR void *arg)
{
    if (g_info->good == good())
        return;

    g_info->good = !g_info->good;
    /* Call back with current state */
    g_info->cb(g_info->arg, g_info->good);
}

static int comp_batt_good_isr(int irq, void *context)
{
    pm_activity(PM_ACTIVITY);

    if (work_available(&g_info->work))
        work_queue(LPWORK, &g_info->work, comp_batt_good_worker, NULL, 0);

    return OK;
}

static int comp_batt_good_register_callback(struct device *dev, batt_good cb,
                                            void *arg)
{
    if (g_info->cb)
        return -EEXIST;

    g_info->cb = cb;
    g_info->arg = arg;
    g_info->good = good();
    /* Immediately call back with current state */
    g_info->cb(g_info->arg, g_info->good);

    /* Enable IRQ */
    stm32_exti_comp(BATT_COMP, true, true, true, comp_batt_good_isr);

    /* In case state changed before IRQ was enabled */
    if (work_available(&g_info->work))
        work_queue(LPWORK, &g_info->work, comp_batt_good_worker, NULL, 0);

    return 0;
}

static int comp_batt_good_probe(struct device *dev)
{
    stm32_comp_config_s batt_comp_cfg = {
        .inp = BATT_COMP_INP,
        .inm = BATT_COMP_INM,
        .hyst = BATT_COMP_HYST,
        .speed = BATT_COMP_SPEED,
        .inverted = BATT_COMP_INV,
    };

    int ret;

    g_info = zalloc(sizeof(*g_info));
    if (!g_info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    /* Configure comparator */
    ret = stm32_compconfig(BATT_COMP, &batt_comp_cfg);
    if (ret) {
        dbg("Failed to configure battery comparator: %d\n", ret);
        goto err;
    }

    /* Enable comparator */
    ret = stm32_compenable(BATT_COMP, true);
    if (ret) {
        dbg("Failed to enable battery comparator: %d\n", ret);
        goto err;
    }

    return 0;

err:
    free(g_info);
    return ret;
}

static struct device_batt_good_type_ops comp_batt_good_type_ops = {
    .register_callback = comp_batt_good_register_callback,
};

static struct device_driver_ops comp_batt_good_driver_ops = {
    .probe      = comp_batt_good_probe,
    .type_ops   = &comp_batt_good_type_ops,
};

struct device_driver comp_batt_good_driver = {
    .type = DEVICE_TYPE_BATTERY_GOOD_HW,
    .name = "comp_batt_good",
    .desc = "Battery good detection with voltage comparator",
    .ops = &comp_batt_good_driver_ops,
};
