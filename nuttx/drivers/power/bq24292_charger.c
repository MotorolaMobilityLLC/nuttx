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
#include <nuttx/power/battery.h>
#include <nuttx/power/bq24292.h>
#include <nuttx/gpio.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sys/types.h>

#define MAX_OUTPUT_CURRENT_OTG  1300 /* mA */

/* Charger Settings */
struct bq24292_chg_settings {
    enum chg chg; /* off, charge, otg */
    struct charger_config cfg; /* limits */
};

struct bq24292_charger_info {
    struct bq24292_chg_settings settings; /* as per last last request */
    sem_t chg_sem; /* mutex to set and read chg settings */
    /* callback to notify faults in boost mode */
    charger_boost_fault boost_fault_cb;
    void *boost_fault_arg;
    sem_t boost_fault_sem; /* mutex to register and run callback */
    /* charge enable gpio that controls BQ24292 CE_N pin */
    int chg_en;
    bool chg_en_active_high;
};

/* Drive charge IC CE_N pin */
static void set_bq24292_ce_pin(const struct bq24292_charger_info *info,
                               bool charge)
{
    if (info->chg_en >= 0) {
        if (info->chg_en_active_high)
            gpio_set_value(info->chg_en, charge ? 1 : 0);
        else
            gpio_set_value(info->chg_en, charge ? 0 : 1);
    }
}

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

    set_bq24292_ce_pin(info, chg == BQ24292_CHG_BATTERY);

    info->settings.chg = chg; /* for power good callback */
    (void) bq24292_set_chg(chg);

    battery_set_status(chg == BQ24292_CHG_BATTERY ? BATTERY_CHARGING :
                                                    BATTERY_DISCHARGING);
}

static void bq24292_pwr_good(struct bq24292_charger_info *info)
{
    while (sem_wait(&info->chg_sem) != OK) {
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

    sem_post(&info->chg_sem);
}

static void bq24292_boost_fault(struct bq24292_charger_info *info)
{
    while (sem_wait(&info->boost_fault_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (info->boost_fault_cb) {
        info->boost_fault_cb(info->boost_fault_arg);
    }

    sem_post(&info->boost_fault_sem);
}

static void bq24292_charger_callback(enum bq24292_event event, void *arg)
{
    struct bq24292_charger_info *info = arg;

    switch(event) {
    case POWER_GOOD:
        bq24292_pwr_good(info);
        break;
    case BOOST_FAULT:
        bq24292_boost_fault(info);
        break;
    default:
        /* Should never be here */
        break;
    }
}

static int bq24292_charger_send(struct device *dev, int *current)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *current = MAX_OUTPUT_CURRENT_OTG;
    config_bq24292(info, BQ24292_OTG_1300MA, NULL);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq24292_charger_receive(struct device *dev, const struct charger_config *cfg)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_BATTERY, cfg);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq24292_charger_off(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_OFF, NULL);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq24292_charger_register_boost_fault_cb(struct device *dev,
                                             charger_boost_fault cb, void *arg)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->boost_fault_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    info->boost_fault_cb = cb;
    info->boost_fault_arg = arg;
    sem_post(&info->boost_fault_sem);

    return 0;
}

static int bq24292_get_gpio(struct device *dev, char* name)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, name);
    return r ? r->start : -1;
}


static int bq24292_charger_probe(struct device *dev)
{
    struct bq24292_charger_info *info;

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    /* Optional CHG_EN signal */
    info->chg_en = bq24292_get_gpio(dev, "chg_en");
    if (info->chg_en >= 0)
        info->chg_en_active_high = true;
    else
        info->chg_en = bq24292_get_gpio(dev, "chg_en_n");

    /* Disable charging by default */
    if (info->chg_en >= 0)
        gpio_direction_out(info->chg_en, info->chg_en_active_high ? 0 : 1);

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

    retval = sem_init(&info->chg_sem, 0, 1);
    if (retval) {
        dbg("failed to init chg semaphore\n");
        return retval;
    }

    retval = sem_init(&info->boost_fault_sem, 0, 1);
    if (retval) {
        dbg("failed to init boost fault semaphore\n");
        return retval;
    }

    /* No charging by default */
    config_bq24292(info, BQ24292_CHG_OFF, NULL);

    retval = bq24292_register_callback(bq24292_charger_callback, info);
    if (retval) {
        dbg("failed to register callback\n");
        sem_destroy(&info->chg_sem);
        sem_destroy(&info->boost_fault_sem);
        return retval;
    }

    return 0;
}

static void bq24292_charger_close(struct device *dev)
{
    struct bq24292_charger_info *info = device_get_private(dev);

    sem_destroy(&info->chg_sem);
    sem_destroy(&info->boost_fault_sem);
}

static struct device_charger_type_ops bq24292_charger_type_ops = {
    .send = bq24292_charger_send,
    .receive = bq24292_charger_receive,
    .off = bq24292_charger_off,
    .register_boost_fault_cb = bq24292_charger_register_boost_fault_cb,
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
