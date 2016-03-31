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
#include <nuttx/device_ptp_chg.h>
#include <nuttx/gpio.h>

#include <debug.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>


struct switch_ptp_chg_info {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    int wls;  /* gpio to control switch on a path from wireless charger */
    int wrd;  /* gpio to control switch on a path from wired charger */
    int base; /* gpio to control switch on a path to base */
    bool wls_active_low;
    bool wrd_active_low;
    bool base_active_low;
#endif
    struct device *chg_dev; /* device to control current syncing and sourcing */
};

/* Set switch state */
static inline void set_switch(int gpio, bool state, bool active_low)
{
    if (gpio >= 0) {
        if (active_low)
            gpio_set_value(gpio, (state) ? 0 : 1);
        else
            gpio_set_value(gpio, (state) ? 1 : 0);
    }
}

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static inline void wireless_path(const struct switch_ptp_chg_info *info, bool state)
{
    set_switch(info->wls, state, info->wls_active_low);
}

static inline void wired_path(const struct switch_ptp_chg_info *info, bool state)
{
    set_switch(info->wrd, state, info->wrd_active_low);
}

static inline void base_path(const struct switch_ptp_chg_info *info, bool state)
{
    set_switch(info->base, state, info->base_active_low);
}

static int switch_ptp_chg_send_wireless_pwr(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);
    int retval;

    retval = device_charger_off(info->chg_dev);
    if (retval)
        return retval;

    wired_path(info, false);
    base_path(info, true);
    wireless_path(info, true);

    return 0;
}

static int switch_ptp_chg_send_wired_pwr(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);
    int retval;

    retval = device_charger_off(info->chg_dev);
    if (retval)
        return retval;

    wireless_path(info, false);
    base_path(info, true);
    wired_path(info, true);

    return 0;
}

int switch_ptp_chg_receive_wireless_pwr(struct device *dev,
                                        const struct charger_config *cfg)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);
    int retval;

    base_path(info, false);
    wired_path(info, false);

    retval = device_charger_receive(info->chg_dev, cfg);
    if (retval)
        return retval;

    wireless_path(info, true);

    return 0;
}

int switch_ptp_chg_receive_wired_pwr(struct device *dev,
                                     const struct charger_config *cfg)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);
    int retval;

    base_path(info, false);
    wireless_path(info, false);

    retval = device_charger_receive(info->chg_dev, cfg);
    if (retval)
        return retval;

    wired_path(info, true);

    return 0;
}
#else
static inline void wireless_path(const struct switch_ptp_chg_info *info, bool state)
{
    /* Do nothing */
}

static inline void wired_path(const struct switch_ptp_chg_info *info, bool state)
{
    /* Do nothing */
}

static inline void base_path(const struct switch_ptp_chg_info *info, bool state)
{
    /* Do nothing */
}
#endif

static int switch_ptp_chg_send_batt_pwr(struct device *dev)
{
    int retval;
    struct switch_ptp_chg_info *info = device_get_private(dev);

    wireless_path(info, false);
    wired_path(info, false);

    retval = device_charger_send(info->chg_dev);
    if (retval)
        return retval;

    base_path(info, true);

    return 0;
}

static int switch_ptp_chg_receive_base_pwr(struct device *dev,
                                           const struct charger_config *cfg)
{
    int retval;
    struct switch_ptp_chg_info *info = device_get_private(dev);

    wireless_path(info, false);
    wired_path(info, false);

    retval = device_charger_receive(info->chg_dev, cfg);
    if (retval)
        return retval;

    base_path(info, true);

    return 0;
}

static int switch_ptp_chg_off(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);

    wireless_path(info, false);
    wired_path(info, false);
    base_path(info, false);

    return device_charger_off(info->chg_dev);
}

static int switch_ptp_chg_probe(struct device *dev)
{
    struct switch_ptp_chg_info *info = zalloc(sizeof(*info));

    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    struct ptp_chg_init_data *init_data =
            (struct ptp_chg_init_data *)device_get_init_data(dev);
    if (init_data) {
        info->wls_active_low = init_data->wls_active_low;
        info->wrd_active_low = init_data->wrd_active_low;
        info->base_active_low = init_data->base_active_low;
    } else {
        info->wls_active_low = true;
        info->wrd_active_low = true;
        info->base_active_low = true;
    }

    struct device_resource *r;

   /* Find switches to control and set them to the h/w default states */
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "wls_path");
    if (!r) {
        dbg("no switch on a path from wireless charger\n");
        info->wls = -1;
    } else {
        info->wls = r->start;
        /* enable wireless */
        gpio_direction_out(info->wls, info->wls_active_low ? 0 : 1);
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "wrd_path");
    if (!r) {
        dbg("no switch on a path from wired charger\n");
        info->wrd = -1;
    } else {
        info->wrd = r->start;
        /* enable wired */
        gpio_direction_out(info->wrd, info->wrd_active_low ? 0 : 1);
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "base_path");
    if (!r) {
        dbg("no switch on a path to core\n");
        info->base = -1;
    } else {
        info->base = r->start;
        /* disable base */
        gpio_direction_out(info->base, info->base_active_low ? 1 : 0);
    }
#endif

    device_set_private(dev, info);
    return 0;
}

static void switch_ptp_chg_remove(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);

    free(info);
    device_set_private(dev, NULL);
}

static int switch_ptp_chg_open(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);

    info->chg_dev = device_open(DEVICE_TYPE_CHARGER_HW, 0);
    if (!info->chg_dev) {
        dbg("failed to open charger device\n");
        return -EIO;
    }

    return 0;
}

static void switch_ptp_chg_close(struct device *dev)
{
    struct switch_ptp_chg_info *info = device_get_private(dev);
    device_close(info->chg_dev);
}

static struct device_ptp_chg_type_ops switch_ptp_chg_type_ops = {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    .send_wireless_pwr = switch_ptp_chg_send_wireless_pwr,
    .send_wired_pwr = switch_ptp_chg_send_wired_pwr,
    .receive_wireless_pwr = switch_ptp_chg_receive_wireless_pwr,
    .receive_wired_pwr = switch_ptp_chg_receive_wired_pwr,
#endif
    .send_batt_pwr = switch_ptp_chg_send_batt_pwr,
    .receive_base_pwr = switch_ptp_chg_receive_base_pwr,
    .off = switch_ptp_chg_off,
};

static struct device_driver_ops driver_ops = {
    .probe = switch_ptp_chg_probe,
    .remove = switch_ptp_chg_remove,
    .open = switch_ptp_chg_open,
    .close = switch_ptp_chg_close,
    .type_ops = &switch_ptp_chg_type_ops,
};

struct device_driver switch_ptp_chg_driver = {
    .type = DEVICE_TYPE_PTP_CHG_HW,
    .name = "switch_ptp_chg",
    .desc = "PTP charger driver controlling power path through switches",
    .ops = &driver_ops,
};
