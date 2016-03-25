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

#include <nuttx/device_ptp_chg.h>
#include <nuttx/gpio.h>
#include <nuttx/power/bq24292.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sys/types.h>

/* GPIO states to control FET on pwr path from wireless receiver
 * TODO: Hard-coded to active low, should be configurable instead
*/
#define WCHG_PATH_ON    (0)
#define WCHG_PATH_OFF   (1)

/* Charger Settings */
struct bq24292_chg_settings {
    enum chg chg; /* off, charge, otg */
    struct charger_config cfg; /* limits */
};

struct bq24292_ptp_chg_info {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    uint8_t wls_path_gpio; /* controls power path from wireless receiver */
#endif
    struct bq24292_chg_settings settings; /* as per last last request */
    sem_t sem; /* mutex to set and read chg settings */
};

/* Enable or disable FET on a power path from a wireless receiver */
static void wireless_path(const struct bq24292_ptp_chg_info *info, bool state)
{
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    gpio_set_value(info->wls_path_gpio, state ? WCHG_PATH_ON : WCHG_PATH_OFF);
#else
    /* Do nothing */
#endif
}

static void set_bq24292_limits(const struct charger_config *cfg)
{
    (void) bq24292_set_input_current_limit(cfg->input_current_limit);
    (void) bq24292_set_input_voltage_limit(cfg->input_voltage_limit);
    (void) bq24292_set_charge_current_limit(cfg->charge_current_limit);
    (void) bq24292_set_charge_voltage_limit(cfg->charge_voltage_limit);
}

static void config_bq24292(struct bq24292_ptp_chg_info *info, enum chg chg,
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
    struct bq24292_ptp_chg_info *info = arg;

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

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static int bq24292_ptp_chg_send_wireless_pwr(struct device *dev)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_OFF, NULL);
    wireless_path(info, true);

    sem_post(&info->sem);
    return 0;
}

int bq24292_ptp_chg_receive_wireless_pwr(struct device *dev,
                                         const struct charger_config *cfg)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_BATTERY, cfg);
    wireless_path(info, true);

    sem_post(&info->sem);
    return 0;
}
#endif

static int bq24292_ptp_chg_send_batt_pwr(struct device *dev)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    wireless_path(info, false);
    config_bq24292(info, BQ24292_OTG_1300MA, NULL);

    sem_post(&info->sem);
    return 0;
}

static int bq24292_ptp_chg_receive_base_pwr(struct device *dev,
                                               const struct charger_config *cfg)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }


    wireless_path(info, false);
    config_bq24292(info, BQ24292_CHG_BATTERY, cfg);

    sem_post(&info->sem);
    return 0;
}

static int bq24292_ptp_chg_off(struct device *dev)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq24292(info, BQ24292_CHG_OFF, NULL);
    wireless_path(info, false);

    sem_post(&info->sem);
    return 0;
}

static int bq24292_ptp_chg_probe(struct device *dev)
{
    struct bq24292_ptp_chg_info *info;
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO,
                                    "wls_path");
    if (!r) {
        dbg("failed to get wls_path gpio\n");
        return -EINVAL;
    }
#endif

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    info->wls_path_gpio = r->start;
    gpio_direction_out(info->wls_path_gpio, WCHG_PATH_ON);
#endif

    device_set_private(dev, info);
    return 0;
}

static int bq24292_ptp_chg_open(struct device *dev)
{
    struct bq24292_ptp_chg_info *info = device_get_private(dev);
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

static struct device_ptp_chg_type_ops bq24292_ptp_chg_type_ops = {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    .send_wireless_pwr = bq24292_ptp_chg_send_wireless_pwr,
    .receive_wireless_pwr = bq24292_ptp_chg_receive_wireless_pwr,
#endif
    .send_batt_pwr = bq24292_ptp_chg_send_batt_pwr,
    .receive_base_pwr = bq24292_ptp_chg_receive_base_pwr,
    .off = bq24292_ptp_chg_off,
};


static struct device_driver_ops driver_ops = {
    .probe = bq24292_ptp_chg_probe,
    .open = bq24292_ptp_chg_open,
    .type_ops = &bq24292_ptp_chg_type_ops,
};

struct device_driver bq24292_ptp_chg_driver = {
    .type = DEVICE_TYPE_PTP_CHG_HW,
    .name = "bq2429_ptp_chg",
    .desc = "BQ24292 based charger driver for power transfer protocol",
    .ops = &driver_ops,
};
