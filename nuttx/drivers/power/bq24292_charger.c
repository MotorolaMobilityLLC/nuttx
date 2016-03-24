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

#include <nuttx/device_charger.h>
#include <nuttx/power/bq24292.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sys/types.h>

/* Charger Settings */
struct bq24292_chg_settings {
    enum chg chg; /* off, charge, otg */
    struct charger_config cfg; /* limits */
};

struct bq24292_charger_info {
    struct bq24292_chg_settings settings; /* as per last last request */
    sem_t sem; /* mutex to set and read chg settings */
};

static void set_bq24292_limits(const struct charger_config *cfg)
{
    (void) bq24292_set_input_current_limit(cfg->input_current_limit);
    (void) bq24292_set_input_voltage_limit(cfg->input_voltage_limit);
    (void) bq24292_set_charge_current_limit(cfg->charge_current_limit);
    (void) bq24292_set_charge_voltage_limit(cfg->charge_voltage_limit);
}

static void config_bq24292(struct bq24292_charger_info *info, enum chg chg,
                           const struct charger_config *cfg)
{
    /*
     * When battery is dead, charge IC is off until power is applied.
     * Settings need to be programmed in power good callback then
     */
    if (cfg) {
        info->settings.cfg = *cfg; /* for power good callback */
        set_bq24292_limits(cfg);
    }

    info->settings.chg = chg; /* for power good callback */
    (void) bq24292_set_chg(chg);
}

static void bq24292_pwr_good(void *arg)
{
    struct bq24292_charger_info *info = arg;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /*  Configure the IC as per last request in case it was off then */
    switch (info->settings.chg) {
    case BQ24292_CHG_OFF:
        bq24292_set_chg(BQ24292_CHG_OFF);
        break;
    case BQ24292_CHG_BATTERY:
        set_bq24292_limits(&info->settings.cfg);
        bq24292_set_chg(BQ24292_CHG_BATTERY);
        break;
    case BQ24292_OTG_500MA:
    case BQ24292_OTG_1300MA:
    default:
        /* Nothing to be done here */
        break;
    }

    sem_post(&info->sem);
}

static int bq24292_charger_send(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_OTG_1300MA, NULL);
    sem_post(&info->sem);

    return 0;
}

static int bq24292_charger_receive(struct device *dev, const struct charger_config *cfg)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_BATTERY, cfg);
    sem_post(&info->sem);

    return 0;
}

static int bq24292_charger_off(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_OFF, NULL);
    sem_post(&info->sem);

    return 0;
}

static int bq24292_charger_probe(struct device *dev)
{
    struct bq24292_charger_info *info;

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    device_set_private(dev, info);
    return 0;
}

static void bq24292_charger_remove(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    free(info);
    device_set_private(dev, NULL);
}

static int bq24292_charger_open(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);
    int retval;

    retval = sem_init(&info->sem, 0, 1);
    if (retval) {
        dbg("failed to init semaphore\n");
        return retval;
    }

    /* No charging by default */
    config_bq24292(info, BQ24292_CHG_OFF, NULL);

    retval = bq24292_power_good_register(bq24292_pwr_good, info);
    if (retval) {
        dbg("failed to register power good callback\n");
        sem_destroy(&info->sem);
        return retval;
    }

    return 0;
}

static void bq24292_charger_close(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    sem_destroy(&info->sem);
}

static struct device_charger_type_ops bq24292_charger_type_ops = {
    .send = bq24292_charger_send,
    .receive = bq24292_charger_receive,
    .off = bq24292_charger_off,
};

static struct device_driver_ops driver_ops = {
    .probe = bq24292_charger_probe,
    .remove = bq24292_charger_remove,
    .open = bq24292_charger_open,
    .close = bq24292_charger_close,
    .type_ops = &bq24292_charger_type_ops,
};

struct device_driver bq24292_charger_driver = {
    .type = DEVICE_TYPE_CHARGER_HW,
    .name = "bq2429_charger",
    .desc = "Charger driver for TI bq24292 IC",
    .ops = &driver_ops,
};
