/*
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
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pwm.h>

#include "tsb_scm.h"
#include "tsb_pwm.h"

#define TSB_PWM_DRIVER_NAME     "tsb pwm driver"
#define TSB_PWM_COUNTS          2

struct pwm_ctlr_info {
    struct device    *dev;      /* device driver handler */
    uint32_t         ctl_flags; /* controller status */
    uint16_t         counts;    /* The number of generators Supported */
    sem_t            mutex;     /* Only one thread can access at a time */
};

static int tsb_pwm_op_setup(struct device *dev);
static int tsb_pwm_op_shutdown(struct device *dev, bool force_off);

static bool tsb_pwm_device_is_open(struct pwm_ctlr_info *info)
{
    return (info->ctl_flags & TSB_PWM_FLAG_OPENED) ? true : false;
}

static int tsb_pwm_op_count(struct device *dev, uint16_t *count)
{
    *count = TSB_PWM_COUNTS;
    lldbg("be called to return PWM counts %d\n", *count);
    return 0;
}

static int tsb_pwm_op_activate(struct device *dev, uint16_t which)
{
    lldbg("be called to activate generator(%d)\n", which);
    return 0;
}

static int tsb_pwm_op_deactivate(struct device *dev, uint16_t which)
{
    lldbg("be called to deactivate generator(%d)\n", which);
    return 0;
}

static int tsb_pwm_op_config(struct device *dev, uint16_t which, uint32_t duty,
                             uint32_t period)
{
    lldbg("be called to config generator(%d) within duty = 0x%x,"
            "period = 0x%x\n", which, duty, period);
    return 0;
}

static int tsb_pwm_op_enable(struct device *dev, uint16_t which)
{
    lldbg("be called to enable generator(%d)\n", which);
    return 0;
}

static int tsb_pwm_op_disable(struct device *dev, uint16_t which)
{
    lldbg("be called to disable generator(%d)\n", which);
    return 0;
}

static int tsb_pwm_op_set_polarity(struct device *dev, uint16_t which,
                                   uint8_t polarity)
{
    lldbg("be called to set generator(%d) of polarity to %s\n", which,
          polarity ? "HIGH" : "LOW");
    return 0;
}

static int tsb_pwm_op_set_mode(struct device *dev, uint16_t which,
                               uint32_t mode)
{
     lldbg("be called to set generator(%d) mode is mode(%d)\n", which, mode);
    return 0;
}

static int tsb_pwm_op_setup(struct device *dev)
{
    lldbg("be called to setup PWM controller of Power and Clock\n");
    return 0;
}
static int tsb_pwm_op_shutdown(struct device *dev, bool force_off)
{
    lldbg("be called to shutdown PWM controller of Power and Clock "
            "whith force_off(%d)\n", force_off);
    return 0;
}

static int tsb_pwm_dev_open(struct device *dev)
{
    struct pwm_ctlr_info *info = dev->private;
    int ret = 0;

    sem_wait(&info->mutex);

    if (tsb_pwm_device_is_open(info)) {
        lldbg("PWM device driver is already opened!\n");
        ret = -EBUSY;
        goto open_err;
    }

    info->ctl_flags = TSB_PWM_FLAG_OPENED;
    lldbg("open successed!\n");

open_err:
    sem_post(&info->mutex);

    return ret;
}

static void tsb_pwm_dev_close(struct device *dev)
{
    struct pwm_ctlr_info *info = dev->private;

    sem_wait(&info->mutex);

    if (!tsb_pwm_device_is_open(info)) {
        lldbg("PWM device driver is already closed!\n");
        goto exit_close;
    }

    info->ctl_flags &= ~TSB_PWM_FLAG_OPENED;
    lldbg("close successed!\n");
exit_close:
    sem_post(&info->mutex);
}

static int tsb_pwm_dev_probe(struct device *dev)
{
    struct pwm_ctlr_info *info;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    sem_init(&info->mutex, 0, 1);

    info->dev = dev;
    dev->private = info;

    return OK;
}

static void tsb_pwm_dev_remove(struct device *dev)
{
    struct pwm_ctlr_info *info = dev->private;

    dev->private = NULL;

    sem_destroy(&info->mutex);

    free(info);
}

static struct device_pwm_type_ops tsb_pwm_type_ops = {
    /** Get number of generstor supported in system */
    .count        = tsb_pwm_op_count,

    /** Get register base, add to link and record assign status */
    .activate     = tsb_pwm_op_activate,

    /** Release link and unassign */
    .deactivate   = tsb_pwm_op_deactivate,

    /** Start pulse output by configed freq, duty or mode */
    .enable       = tsb_pwm_op_enable,

    /** Stop pulse output */
    .disable      = tsb_pwm_op_disable,

    /** Set Freq and duty for specific pulse output */
    .config       = tsb_pwm_op_config,

    /** Set a pulse of polarity output */
    .set_polarity = tsb_pwm_op_set_polarity,

    /** sync mode, pulse count mode, pattern mode.. etc. */
    .set_mode     = tsb_pwm_op_set_mode,

    /** Enable controller power/clock */
    .setup        = tsb_pwm_op_setup,

    /** Disable controller power/clock if need */
    .shutdown     = tsb_pwm_op_shutdown,
};

static struct device_driver_ops tsb_pwm_driver_ops = {
    .probe          = tsb_pwm_dev_probe,
    .remove         = tsb_pwm_dev_remove,
    .open           = tsb_pwm_dev_open,
    .close          = tsb_pwm_dev_close,
    .type_ops.pwm   = &tsb_pwm_type_ops,
};

struct device_driver tsb_pwm_driver = {
    .type       = DEVICE_TYPE_PWM_HW,
    .name       = "tsb_pwm",
    .desc       = "TTSB PWM Controller",
    .ops        = &tsb_pwm_driver_ops,
};
