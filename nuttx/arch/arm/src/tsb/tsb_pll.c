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
 * @brief TSB PLL device driver
 */
#include <errno.h>
#include <string.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pll.h>

#define TSB_PLL_INIT                    0x00
#define TSB_PLL_DIVIDE_MODE             0x04
#define TSB_PLL_CLK_FACTOR              0x08
#define TSB_PLL_CLKFREQ_MODE            0x0c
#define TSB_PLL_SSCG_MODE               0x10
#define TSB_PLL_SSCG_PARAM              0x14

#define TSB_PLL_INIT_RESET              BIT(2)
#define TSB_PLL_INIT_BP                 BIT(1)
#define TSB_PLL_INIT_PD                 BIT(0)

#define TSB_PLL_SSCG_MODE_MODETYPE      BIT(2)
#define TSB_PLL_SSCG_MODE_SSCEN         BIT(1)
#define TSB_PLL_SSCG_MODE_DITHEN        BIT(0)

#include "up_arch.h"

enum tsb_pll_state {
    TSB_PLL_STATE_INVALID,
    TSB_PLL_STATE_CLOSED,
    TSB_PLL_STATE_OPEN,
    TSB_PLL_STATE_READY,
    TSB_PLL_STATE_ACTIVE,
};

struct tsb_pll_info {
    struct device       *dev;
    enum tsb_pll_state  state;
    uint32_t            reg_base;
    uint32_t            freq;
    sem_t               lock;
};

struct tsb_pll_freq_table_entry {
    uint32_t            frequency;
    uint32_t            divide_mode;
    uint32_t            clk_factor;
    uint32_t            clkfreq_mode;
};

/*
 * Do not have an adequate description of the PLL registers from Toshiba
 * to figure out how to program them so use a table of known good values
 * for some common frequencies.
 */
static struct tsb_pll_freq_table_entry tsb_pll_freq_tbl[] = {
    {
        .frequency      =   12288000,
        .divide_mode    = 0x01000001,
        .clk_factor     = 0x00000020,
        .clkfreq_mode   = 0x02071c31,
    },
    {
        .frequency      =    6144000,
        .divide_mode    = 0x01000001,
        .clk_factor     = 0x00000010,
        .clkfreq_mode   = 0x02071132,
    },
    {
        .frequency      =    4096000,
        .divide_mode    = 0x03000001,
        .clk_factor     = 0x00000020,
        .clkfreq_mode   = 0x02071c31,
    },
    {
        .frequency      =    2048000,
        .divide_mode    = 0x03000001,
        .clk_factor     = 0x00000010,
        .clkfreq_mode   = 0x02071132,
    },
};

static struct tsb_pll_freq_table_entry *tsb_pll_find_pfte(uint32_t frequency)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(tsb_pll_freq_tbl); i++)
        if (tsb_pll_freq_tbl[i].frequency == frequency)
            return &tsb_pll_freq_tbl[i];

    return NULL;
}

static void tsb_pll_write(struct tsb_pll_info *info, unsigned int reg,
                          uint32_t val)
{
    putreg32(val, info->reg_base + reg);
}

static void tsb_pll_init(struct tsb_pll_info *info)
{
    tsb_pll_write(info, TSB_PLL_INIT,
                  TSB_PLL_INIT_RESET | TSB_PLL_INIT_BP | TSB_PLL_INIT_PD);
}

static int tsb_pll_set_freq(struct tsb_pll_info *info, uint32_t frequency)
{
    struct tsb_pll_freq_table_entry *pfte;

    pfte = tsb_pll_find_pfte(frequency);
    if (!pfte)
        return -EINVAL;

    tsb_pll_write(info, TSB_PLL_DIVIDE_MODE, pfte->divide_mode);
    tsb_pll_write(info, TSB_PLL_CLK_FACTOR, pfte->clk_factor);
    tsb_pll_write(info, TSB_PLL_CLKFREQ_MODE, pfte->clkfreq_mode);
    tsb_pll_write(info, TSB_PLL_SSCG_MODE, TSB_PLL_SSCG_MODE_MODETYPE);

    /*
     * Latest Toshiba App Note and Functional Spec. say to write 0x01000000
     * to SSCG_PARAM here but Toshiba code writes zero and both values are
     * invalid according to the APB/GPB Register Map doc.
     * Both 0x00000000 and 0x01000000 seem to work.
     */
    tsb_pll_write(info, TSB_PLL_SSCG_PARAM, 0x01000000);
    up_udelay(100); /* Toshiba docs say to wait 100us here */

    return 0;
}

static void tsb_pll_start(struct tsb_pll_info *info)
{
    tsb_pll_write(info, TSB_PLL_INIT, TSB_PLL_INIT_BP | TSB_PLL_INIT_PD);
    __asm("nop"); /* Toshiba docs say to wait 1 CPU cycle here */
    tsb_pll_write(info, TSB_PLL_INIT, TSB_PLL_INIT_BP);
    up_udelay(100); /* Toshiba docs say to wait 100us here */
    tsb_pll_write(info, TSB_PLL_INIT, 0);
}

static int tsb_pll_op_start(struct device *dev)
{
    struct tsb_pll_info *info = device_get_private(dev);
    int ret = 0;

    sem_wait(&info->lock);

    if (info->state != TSB_PLL_STATE_READY) {
        ret = -EPROTO;
        goto err_unlock;
    }

    tsb_pll_start(info);

    info->state = TSB_PLL_STATE_ACTIVE;

err_unlock:
    sem_post(&info->lock);

    return ret;
}

static int tsb_pll_op_stop(struct device *dev)
{
    struct tsb_pll_info *info = device_get_private(dev);
    int ret = 0;

    sem_wait(&info->lock);

    if (info->state != TSB_PLL_STATE_ACTIVE) {
        ret = -EPROTO;
        goto err_unlock;
    }

    tsb_pll_init(info);

    info->state = TSB_PLL_STATE_OPEN;

err_unlock:
    sem_post(&info->lock);

    return ret;
}

static int tsb_pll_op_set_frequency(struct device *dev, uint32_t frequency)
{
    struct tsb_pll_info *info = device_get_private(dev);
    int ret;

    sem_wait(&info->lock);

    if (info->state == TSB_PLL_STATE_ACTIVE) {
        ret = -EBUSY;
        goto err_unlock;
    }

    if ((info->state != TSB_PLL_STATE_OPEN) &&
        (info->state != TSB_PLL_STATE_READY)) {
        ret = -EPROTO;
        goto err_unlock;
    }

    ret = tsb_pll_set_freq(info, frequency);
    if (ret) {
        tsb_pll_init(info);
        info->state = TSB_PLL_STATE_OPEN;
        goto err_unlock;
    }

    info->state = TSB_PLL_STATE_READY;

err_unlock:
    sem_post(&info->lock);

    return ret;
}

static int tsb_pll_op_query_frequency(uint32_t frequency)
{
    struct tsb_pll_freq_table_entry *pfte;

    /* test if frequency is supported */
    pfte = tsb_pll_find_pfte(frequency);
    if (!pfte)
        return -EINVAL;

    return 0;
}

static int tsb_pll_dev_open(struct device *dev)
{
    struct tsb_pll_info *info = device_get_private(dev);
    int ret = 0;

    sem_wait(&info->lock);

    if (info->state != TSB_PLL_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_unlock;
    }

    tsb_pll_init(info);

    info->state = TSB_PLL_STATE_OPEN;

err_unlock:
    sem_post(&info->lock);

    return ret;
}

static void tsb_pll_dev_close(struct device *dev)
{
    struct tsb_pll_info *info = device_get_private(dev);

    sem_wait(&info->lock);

    if ((info->state != TSB_PLL_STATE_OPEN) &&
        (info->state != TSB_PLL_STATE_READY))
        goto err_unlock;

    tsb_pll_init(info);

    info->state = TSB_PLL_STATE_CLOSED;

err_unlock:
    sem_post(&info->lock);
}

static int tsb_pll_extract_resources(struct device *dev,
                                     struct tsb_pll_info *info)
{

    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "reg_base");
    if (!r)
        return -EINVAL;

    info->reg_base = r->start;

    return 0;
}

static int tsb_pll_dev_probe(struct device *dev)
{
    struct tsb_pll_info *info;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    ret = tsb_pll_extract_resources(dev, info);
    if (ret)
        goto err_free_info;

    info->state = TSB_PLL_STATE_CLOSED;
    sem_init(&info->lock, 0, 1);
    info->dev = dev;
    device_set_private(dev, info);

    return 0;

err_free_info:
    free(info);

    return ret;
}

static void tsb_pll_dev_remove(struct device *dev)
{
    struct tsb_pll_info *info = device_get_private(dev);

    sem_destroy(&info->lock);
    free(info);
    device_set_private(dev, NULL);
}

static struct device_pll_type_ops tsb_pll_type_ops = {
    .start          = tsb_pll_op_start,
    .stop           = tsb_pll_op_stop,
    .set_frequency  = tsb_pll_op_set_frequency,
    .query_frequency  = tsb_pll_op_query_frequency
};

static struct device_driver_ops tsb_pll_driver_ops = {
    .probe          = tsb_pll_dev_probe,
    .remove         = tsb_pll_dev_remove,
    .open           = tsb_pll_dev_open,
    .close          = tsb_pll_dev_close,
    .type_ops       = &tsb_pll_type_ops,
};

struct device_driver tsb_pll_driver = {
    .type   = DEVICE_TYPE_PLL_HW,
    .name   = "tsb_pll",
    .desc   = "TSB PLL Driver",
    .ops    = &tsb_pll_driver_ops,
};
