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
#include <nuttx/device_ptp.h>
#include <nuttx/device_ptp_chg.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/power/battery_state.h>
#include <nuttx/power/pad_det.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sys/types.h>

#define DEFAULT_BASE_INPUT_CURRENT  500 /* mA */

/* Charging/discharging settings */
struct batt_ptp {
    bool chg_allowed; /* flag */
    bool dischg_allowed; /* flag */
    /* Limits are in mA and mV */
    int input_current; /* as per command from base */
    int charge_current; /* as per battery temp */
    int charge_voltage; /* as per battery temp */
};

/* Internal state */
struct ptp_state {
    enum base_attached_e attached; /* base attachment */
    enum ptp_current_flow direction; /* per command from base */
    struct batt_ptp battery; /* battery chg/dischg settings */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    bool docked; /* on a wireless dock ? */
#endif
};

struct ptp_info {
    struct ptp_state state; /* current state */
    sem_t sem; /* mutual exclusion to set and process state */
    struct device *batt_dev; /*battery temp and level notifications */
    struct device *chg_dev; /* current control */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    ptp_ext_power_cb ext_power_changed_cb; /* callback to client*/
#endif
    ptp_power_required_cb power_required_changed_cb; /* callback to client */
};

/*
 * Charge base with power from wireless receiver if docked, otherwise from a
 * battery if conditions permit
 */
static int do_charge_base(struct device *chg, const struct ptp_state *state)
{
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    if (state->docked) {
        return device_ptp_chg_send_wireless_pwr(chg);
    }
#endif
    if (state->battery.dischg_allowed) {
        return device_ptp_chg_send_batt_pwr(chg);
    } else {
        return device_ptp_chg_off(chg);
    }
}

/*
 * Charge battery with power from wireless receiver if docked and conditions
 * permit
 */
static int do_charge_battery_if_docked(struct device *chg,
                                       const struct ptp_state *state)
{
#ifndef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    return device_ptp_chg_off(chg);
#else
    struct ptp_chg cfg;

    if (!state->battery.chg_allowed) {
        return device_ptp_chg_off(chg);
    } else if (state->docked) {
        cfg.input_current_limit = CONFIG_GREYBUS_MODS_WIRELESS_INPUT_CURRENT;
        cfg.input_voltage_limit = CONFIG_GREYBUS_MODS_INPUT_VOLTAGE;
        cfg.charge_current_limit = state->battery.charge_current;
        cfg.charge_voltage_limit = state->battery.charge_voltage;
        return device_ptp_chg_receive_wireless_pwr(chg, &cfg);
    } else {
        return device_ptp_chg_off(chg);
    }
#endif
}

/*
 * Charge battery with power from base if conditions permit
 */
static int do_charge_battery_with_base_power(struct device *chg,
                                                const struct ptp_state *state)
{
    struct ptp_chg cfg;

    if (!state->battery.chg_allowed) {
        return device_ptp_chg_off(chg);
    } else {
        cfg.input_current_limit = state->battery.input_current;
        cfg.input_voltage_limit = CONFIG_GREYBUS_MODS_INPUT_VOLTAGE;
        cfg.charge_current_limit = state->battery.charge_current;
        cfg.charge_voltage_limit = state->battery.charge_voltage;
        return device_ptp_chg_receive_base_pwr(chg, &cfg);
    }
}

static int batt_ptp_process(struct device *chg, const struct ptp_state *state)
{
    switch (state->attached) {
    case BASE_ATTACHED_OFF:
        return do_charge_base(chg, state);
    case BASE_ATTACHED:
        switch (state->direction) {
        case PTP_CURRENT_OFF:
            return do_charge_battery_if_docked(chg, state);
        case PTP_CURRENT_TO_MOD:
            return do_charge_battery_with_base_power(chg, state);
        case PTP_CURRENT_FROM_MOD:
            return do_charge_base(chg, state);
        default:
            device_ptp_chg_off(chg);
            return -EINVAL;
        }
    case BASE_DETACHED:
        return do_charge_battery_if_docked(chg, state);
    case BASE_INVALID:
    default:
        device_ptp_chg_off(chg);
        return -EINVAL;
    }
}

static void batt_ptp_attach_changed(FAR void *arg, enum base_attached_e state)
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;

    if (init) {
        init = false;
        info->state.attached = state;
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (info->state.attached == state)
        goto attach_done;

    /* Once attached, set current direction and maximum input current to defaults */
    if (state == BASE_ATTACHED) {
        info->state.direction = PTP_CURRENT_OFF;
        info->state.battery.input_current = DEFAULT_BASE_INPUT_CURRENT;
    }

    info->state.attached = state;
    batt_ptp_process(info->chg_dev, &info->state);

attach_done:
    sem_post(&info->sem);
}

static int batt_ptp_set_current_flow(struct device *dev, uint8_t direction)
{
    struct ptp_info *info = device_get_private(dev);
    int retval = 0;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    /* Handset cannot send any requests unless it is attached */
    if (info->state.attached != BASE_ATTACHED) {
        dbg("received current flow request while not attached\n");
        retval = -EPERM;
        goto current_flow_done;
    }

    if (info->state.direction == direction)
        goto current_flow_done;

    info->state.direction = direction;
    /*
     * Always report success to the base. If the direction cannot be set
     * now due to the conditions such as for example, battery level and/or temp,
     * the current flow direction will be set once the conditions allow it.
     */
    (void) batt_ptp_process(info->chg_dev, &info->state);

current_flow_done:
    sem_post(&info->sem);
    return retval;
}

static int batt_ptp_set_max_input_current(struct device *dev, uint32_t current)
{
    struct ptp_info *info = device_get_private(dev);
    int retval = 0;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    /* Handset cannot send any requests unless it is attached */
    if (info->state.attached != BASE_ATTACHED) {
        dbg("received input current request while not attached\n");
        retval = -EPERM;
        goto input_current_done;
    }

    current /= 1000; /* uA to mA */
    if (info->state.battery.input_current == current)
        goto input_current_done;

    info->state.battery.input_current = current;
    /*
     * Always report success to the base. If the limit cannot be set
     * now due to the conditions such as for example, battery level and/or temp,
     * the limit will be set once the conditions allow it.
     */
    (void) batt_ptp_process(info->chg_dev, &info->state);

input_current_done:
    sem_post(&info->sem);
    return retval;
}

static void batt_ptp_set_battery_state(const struct batt_state_s *batt,
                                       struct batt_ptp *state)
{
    state->chg_allowed = (batt->temp != BATTERY_TEMP_NO_CHARGING &&
                          batt->temp != BATTERY_TEMP_COOL_DOWN &&
                          batt->level != BATTERY_LEVEL_FULL);
    state->dischg_allowed = (batt->temp != BATTERY_TEMP_COOL_DOWN &&
                             batt->temp != BATTERY_TEMP_UNAVAILABLE &&
                             batt->level != BATTERY_LEVEL_EMPTY);

    if (state->chg_allowed) {
        if (batt->temp == BATTERY_TEMP_NORMAL) {
            state->charge_current = CONFIG_GREYBUS_MODS_FULL_CHG_CURRENT;
            state->charge_voltage = CONFIG_GREYBUS_MODS_FULL_CHG_VOLTAGE;
        } else {
            state->charge_current = CONFIG_GREYBUS_MODS_REDUCED_CHG_CURRENT;
            state->charge_voltage = CONFIG_GREYBUS_MODS_REDUCED_CHG_VOLTAGE;
        }
    }
}

static void batt_ptp_battery_changed(void *arg,
                                     const struct batt_state_s *batt)
{
    static bool init = true; /* first callback is to initialize */
    static struct batt_state_s current;
    struct ptp_info *info = arg;
    bool chg_allowed;

    if (init) {
        init = false;
        current = *batt;
        batt_ptp_set_battery_state(batt, &info->state.battery);
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (current.temp == batt->temp && current.level == batt->level)
        goto battery_done;

    current = *batt;
    chg_allowed = info->state.battery.chg_allowed;
    batt_ptp_set_battery_state(batt, &info->state.battery);
    if (chg_allowed != info->state.battery.chg_allowed &&
        info->power_required_changed_cb) {
        info->power_required_changed_cb();
    }
    batt_ptp_process(info->chg_dev, &info->state);
battery_done:
    sem_post(&info->sem);
}

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static void batt_ptp_docked_changed(void *arg, bool docked)
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;

    if (init) {
        init = false;
        info->state.docked = docked;
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (info->state.docked == docked)
        goto docked_done;

    if (info->ext_power_changed_cb) {
        info->ext_power_changed_cb();
    }

    info->state.docked = docked;
    batt_ptp_process(info->chg_dev, &info->state);

docked_done:
    sem_post(&info->sem);
}

static int batt_ptp_register_ext_power_changed(struct device *dev,
                                               ptp_ext_power_cb cb)
{
    struct ptp_info *info = device_get_private(dev);
    int retval;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (!info->ext_power_changed_cb) {
        info->ext_power_changed_cb = cb;
        retval = 0;
    } else {
        retval = -EEXIST;
    }

    sem_post(&info->sem);

    return retval;
}

static int batt_ptp_ext_power_present(struct device *dev, uint8_t *present)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *present = info->state.docked ? PTP_EXT_POWER_PRESENT :
                                    PTP_EXT_POWER_NOT_PRESENT;

    sem_post(&info->sem);
    return 0;
}
#endif

static int batt_ptp_register_power_required_changed(struct device *dev,
                                                    ptp_power_required_cb cb)
{
    struct ptp_info *info = device_get_private(dev);
    int retval;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (!info->power_required_changed_cb) {
        info->power_required_changed_cb = cb;
        retval = 0;
    } else {
        retval = -EEXIST;
    }

    sem_post(&info->sem);

    return retval;
}

static int batt_ptp_power_required(struct device *dev, uint8_t *required)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *required = info->state.battery.chg_allowed ? PTP_POWER_REQUIRED :
                                                  PTP_POWER_NOT_REQUIRED;
    sem_post(&info->sem);
    return 0;
}

static int batt_ptp_probe(struct device *dev)
{
    struct ptp_info *info = zalloc(sizeof(*info));

    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    device_set_private(dev, info);

    return 0;
}

static int batt_ptp_open(struct device *dev)
{
    struct ptp_info *info = device_get_private(dev);
    int retval;

    /* Init the state and process it, then post semaphore */
    retval = sem_init(&info->sem, 0, 0);
    if (retval) {
        dbg("failed to init semaphore\n");
        return retval;
    }

    /* Current direction is off until base sends direction request */
    info->state.direction = PTP_CURRENT_OFF;

    /* Input current is default until base changes it */
    info->state.battery.input_current = DEFAULT_BASE_INPUT_CURRENT;

    info->chg_dev = device_open(DEVICE_TYPE_PTP_CHG_HW, 0);
    if (!info->chg_dev) {
        dbg("failed to open charger device\n");
        retval = -EIO;
        goto chg_err;
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    retval = pad_det_register_callback(batt_ptp_docked_changed, info);
    if (retval) {
        dbg("failed to register pad callback\n");
        goto pad_err;
    }
#endif

    retval = battery_state_register(batt_ptp_battery_changed, info);
    if (retval) {
        dbg("failed to register mods_battery callback\n");
        goto batt_err;
    }

    retval = mods_attach_register(batt_ptp_attach_changed, info);
    if (retval) {
        dbg("failed to register mods_attach callback\n");
        goto atch_err;
    }

    batt_ptp_process(info->chg_dev, &info->state);
    sem_post(&info->sem);

    return 0;

atch_err:
batt_err:
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
pad_err:
#endif
    device_close(info->chg_dev);
chg_err:
    sem_destroy(&info->sem);
    return retval;
}

static struct device_ptp_type_ops batt_ptp_type_ops = {
    .set_current_flow = batt_ptp_set_current_flow,
    .set_max_input_current = batt_ptp_set_max_input_current,
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    .ext_power_present = batt_ptp_ext_power_present,
    .register_ext_power_cb = batt_ptp_register_ext_power_changed,
#endif
    .power_required = batt_ptp_power_required,
    .register_power_required_cb = batt_ptp_register_power_required_changed,
};

static struct device_driver_ops batt_ptp_driver_ops = {
    .probe = batt_ptp_probe,
    .open = batt_ptp_open,
    .type_ops = &batt_ptp_type_ops,
};

struct device_driver batt_ptp_driver = {
    .type = DEVICE_TYPE_PTP_HW,
    .name = "battery_ptp",
    .desc = "Power transfer protocol for devices with a battery",
    .ops = &batt_ptp_driver_ops,
};
