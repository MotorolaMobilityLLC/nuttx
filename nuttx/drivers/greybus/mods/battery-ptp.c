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
#include <nuttx/util.h>

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

/* Values reported to the base */
struct ptp_reported {
    enum ptp_ext_power present; /* external power sources */
    enum ptp_power_available available; /* source that will be sent to the base */
    enum ptp_power_source source; /* source that is sent to the base */
    enum ptp_power_required required; /* can power be received from the base ? */
};

/* Internal state */
struct ptp_state {
    enum base_attached_e attached; /* base attachment */
    enum ptp_current_flow direction; /* per command from base */
    struct batt_ptp battery; /* battery chg/dischg settings */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    int wired_current; /* max ouput in mA */
    int wrls_current; /* max output in mA */
    int *ext_power; /* points to external power source or NULL*/
#endif
#ifndef GREYBUS_PTP_INT_SND_NEVER
    int batt_current; /* max output in mA */
#endif
    int *output_current; /* points either to battery, external power source or NULL */
    bool base_powered_off; /* flag to indicate base is powered off with AMP attached */
    struct ptp_reported report; /* what is repoted to the base */
    bool boost_fault; /* flag to indicate fault in boost mode */
};

struct ptp_info {
    struct ptp_state state; /* current state */
    sem_t sem; /* mutual exclusion to set and process state */
    struct device *batt_dev; /*battery temp and level notifications */
    struct device *chg_dev; /* current control */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    struct device *wired_src_dev; /* ext power source device */
#endif
    /* external power present, power available or required have changed */
    ptp_changed changed_cb;
};


/*
 * Charge base from an external power source if it is present, otherwise from a
 * battery if conditions permit
 */
static int do_charge_base(struct device *chg, struct ptp_state *state)
{
    int retval = 0;

    state->output_current = NULL;

    /* Disable base charging if fault in boost mode is detected */
    if (state->boost_fault) {
        retval = device_ptp_chg_off(chg);
        goto done;
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    if (state->ext_power == &state->wired_current) {
        if ((retval = device_ptp_chg_send_wired_pwr(chg)) == 0)
            state->output_current = state->ext_power;
        goto done;
    } else if (state->ext_power == &state->wrls_current) {
        if ((retval = device_ptp_chg_send_wireless_pwr(chg)) == 0)
            state->output_current = state->ext_power;
        goto done;
    }
#endif

#ifndef GREYBUS_PTP_INT_SND_NEVER
    if (state->battery.dischg_allowed && !state->base_powered_off) {
        if ((retval = device_ptp_chg_send_batt_pwr(chg, &state->batt_current)) == 0)
            state->output_current = &state->batt_current;
        goto done;
    }
#endif

    retval = device_ptp_chg_off(chg);

done:
    return retval;
}

/*
 * Charge battery with power from an external power source if it is present and
 * conditions permit
 */
static int do_charge_battery_with_external_power(struct device *chg,
                                                 struct ptp_state *state)
{
    state->output_current = NULL;

#ifndef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    return device_ptp_chg_off(chg);
#else
    struct charger_config cfg;

    if (state->battery.chg_allowed && state->ext_power) {
        cfg.input_current_limit = *state->ext_power;
        cfg.input_voltage_limit = CONFIG_GREYBUS_MODS_INPUT_VOLTAGE;
        cfg.charge_current_limit = state->battery.charge_current;
        cfg.charge_voltage_limit = state->battery.charge_voltage;
        if (state->ext_power == &state->wired_current)
            return device_ptp_chg_receive_wired_pwr(chg, &cfg);
        else
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
                                             struct ptp_state *state)
{
    struct charger_config cfg;

    state->output_current = NULL;

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

/* Disable power transfer */
static void do_off(struct device *chg, struct ptp_state *state)
{
    state->output_current = NULL;
    device_ptp_chg_off(chg);
}

static int batt_ptp_process(struct device *chg, struct ptp_state *state)
{
    switch (state->attached) {
    case BASE_ATTACHED_OFF:
        return do_charge_base(chg, state);
    case BASE_ATTACHED:
        switch (state->direction) {
        case PTP_CURRENT_OFF:
            return do_charge_battery_with_external_power(chg, state);
        case PTP_CURRENT_TO_MOD:
            return do_charge_battery_with_base_power(chg, state);
        case PTP_CURRENT_FROM_MOD:
            return do_charge_base(chg, state);
        default:
            do_off(chg, state);
            return -EINVAL;
        }
    case BASE_DETACHED:
        return do_charge_battery_with_external_power(chg, state);
    case BASE_INVALID:
    default:
        do_off(chg, state);
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
        if (state == BASE_ATTACHED_OFF)
            info->state.base_powered_off = true;
        else
            info->state.base_powered_off = false;
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (info->state.attached == state)
        goto attach_done;

    /* Once attached, set current direction and maximum input current to
       defaults. Once detached, reset boost fault flag */
    if (state == BASE_ATTACHED) {
        info->state.direction = PTP_CURRENT_OFF;
        info->state.battery.input_current = DEFAULT_BASE_INPUT_CURRENT;
    } else if (state == BASE_DETACHED) {
        info->state.boost_fault = false;
    }

    if (state == BASE_ATTACHED_OFF &&
         info->state.attached == BASE_ATTACHED &&
         info->state.direction == PTP_CURRENT_OFF)
        info->state.base_powered_off = true;
    else
        info->state.base_powered_off = false;

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

static void batt_ptp_set_power_availability(struct ptp_info *info)
{
    enum ptp_power_available old_available = info->state.report.available;

    /* Check external power first since this is the preferred source */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    if (info->state.ext_power) {
        info->state.report.available = PTP_POWER_AVAILABLE_EXT;
        goto done;
    }
#endif

#ifndef GREYBUS_PTP_INT_SND_NEVER
    if (info->state.battery.dischg_allowed) {
        info->state.report.available = PTP_POWER_AVAILABLE_INT;
        goto done;
    }
#endif

   info->state.report.available = PTP_POWER_AVAILABLE_NONE;

done:
    /* Notify the change */
    if (info->changed_cb && (info->state.report.available != old_available))
        info->changed_cb(POWER_AVAILABLE);
}

static void batt_ptp_set_power_needs(struct ptp_info *info)
{
   enum ptp_power_required old_required = info->state.report.required;

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    info->state.report.required = info->state.battery.chg_allowed ?
                                  PTP_POWER_REQUIRED : PTP_POWER_NOT_REQUIRED;
#else
    info->state.report.required = PTP_POWER_NOT_REQUIRED;
#endif

    /* Notify the change */
    if (info->changed_cb && (info->state.report.required != old_required))
        info->changed_cb(POWER_REQUIRED);
}

static void batt_ptp_boost_fault(void *arg)
{
    struct ptp_info *info = arg;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /* Fault while sourcing battery power to the base */
    if (!info->state.boost_fault) {
        info->state.boost_fault = true;
        batt_ptp_process(info->chg_dev, &info->state);
        dbg("fault detected while sourcing battery to the base\n");
    }

    sem_post(&info->sem);
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
            if (batt->voltage == BATTERY_QUICK_CHARGE)
                state->charge_current = CONFIG_GREYBUS_MODS_QUICK_CHG_CURRENT;
            else
                state->charge_current = CONFIG_GREYBUS_MODS_SLOW_CHG_CURRENT;
            state->charge_voltage = CONFIG_GREYBUS_MODS_FULL_CHG_VOLTAGE;
        } else {
            if (batt->voltage == BATTERY_QUICK_CHARGE)
                state->charge_current = CONFIG_GREYBUS_MODS_REDUCED_CHG_CURRENT;
            else
                state->charge_current = MIN(CONFIG_GREYBUS_MODS_REDUCED_CHG_CURRENT,
                                             CONFIG_GREYBUS_MODS_SLOW_CHG_CURRENT);
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
    bool chg_allowed, dischg_allowed;

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

    if (current.temp == batt->temp &&
        current.level == batt->level &&
        current.voltage == batt-> voltage)
        goto battery_done;

    current = *batt;
    chg_allowed = info->state.battery.chg_allowed;
    dischg_allowed = info->state.battery.dischg_allowed;
    batt_ptp_set_battery_state(batt, &info->state.battery);

    if (chg_allowed != info->state.battery.chg_allowed)
        batt_ptp_set_power_needs(info);
    if (dischg_allowed != info->state.battery.dischg_allowed)
        batt_ptp_set_power_availability(info);

    batt_ptp_process(info->chg_dev, &info->state);
battery_done:
    sem_post(&info->sem);
}

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static int batt_ptp_ext_power_present(struct device *dev, uint8_t *present)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *present = info->state.report.present;

    sem_post(&info->sem);
    return 0;
}

static void batt_ptp_choose_ext_power_source(struct ptp_state *state)
{
    /* Choose source that provides max output current */
    if (state->wired_current == 0 && state->wrls_current == 0)
        state->ext_power = NULL;
    else if (state->wired_current >= state->wrls_current)
        state->ext_power = &state->wired_current;
    else
        state->ext_power = &state->wrls_current;
}

static void batt_ptp_set_ext_power_presence(struct ptp_info *info)
{
    enum ptp_ext_power old_present = info->state.report.present;

    if (info->state.wired_current > 0 && info->state.wrls_current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRED_WIRELESS_PRESENT;
    else if (info->state.wired_current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRED_PRESENT;
    else if (info->state.wrls_current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRELESS_PRESENT;
    else
        info->state.report.present = PTP_EXT_POWER_NOT_PRESENT;

    /* Notify the change */
    if (info->changed_cb && (info->state.report.present != old_present))
        info->changed_cb(POWER_PRESENT);
}

static bool batt_ptp_ext_power_source_changed(struct ptp_state *state)
{
    int *old_src, *new_src;
    int old_limit, new_limit;

    /* Have source and/or limit changed? */
    old_src = state->ext_power;
    old_limit = state->ext_power ? *state->ext_power : 0;

    batt_ptp_choose_ext_power_source(state);

    new_src = state->ext_power;
    new_limit = state->ext_power ? *state->ext_power : 0;

    return (new_src != old_src || new_limit != old_limit);
}

static void batt_ptp_wireless_changed(void *arg, bool docked)
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;
    int current;

    if (init) {
        init = false;
        info->state.wrls_current = docked ?
                                CONFIG_GREYBUS_MODS_WIRELESS_INPUT_CURRENT : 0;
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    current = docked ? CONFIG_GREYBUS_MODS_WIRELESS_INPUT_CURRENT : 0;
    if (current == info->state.wrls_current)
        goto docked_done;

    info->state.wrls_current = current;

    batt_ptp_set_ext_power_presence(info);

    if (!batt_ptp_ext_power_source_changed(&info->state))
        goto docked_done;

    batt_ptp_set_power_availability(info);

    batt_ptp_process(info->chg_dev, &info->state);

docked_done:
    sem_post(&info->sem);
}

static void batt_ptp_wired_changed(void *arg, int current)
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;

    if (init) {
        init = false;
        info->state.wired_current = current;
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (info->state.wired_current == current)
        goto wired_done;

    info->state.wired_current = current;

    batt_ptp_set_ext_power_presence(info);

    if (!batt_ptp_ext_power_source_changed(&info->state))
        goto wired_done;

    batt_ptp_set_power_availability(info);

    batt_ptp_process(info->chg_dev, &info->state);

wired_done:
    sem_post(&info->sem);
}
#endif

#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
static int batt_ptp_get_max_output_current(struct device *dev, uint32_t *current)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *current = info->state.output_current ? *info->state.output_current : 0;

    sem_post(&info->sem);
    return 0;
}

static int batt_ptp_power_available(struct device *dev, uint8_t *available)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *available = info->state.report.available;

    sem_post(&info->sem);
    return 0;
}

static int batt_ptp_power_source(struct device *dev, uint8_t *source)
{
   struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    if (info->state.output_current == &info->state.wired_current) {
        *source = PTP_POWER_SOURCE_WIRED;
        goto done;
    } else if (info->state.output_current == &info->state.wrls_current) {
         *source = PTP_POWER_SOURCE_WIRELESS;
        goto done;
    }
#endif

#ifndef GREYBUS_PTP_INT_SND_NEVER
    if(info->state.output_current == &info->state.batt_current) {
        *source = PTP_POWER_SOURCE_BATTERY;
        goto done;
    }
#endif

   *source = PTP_POWER_SOURCE_NONE;

done:
    sem_post(&info->sem);
    return 0;
}
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
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

static int batt_ptp_power_required(struct device *dev, uint8_t *required)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *required = info->state.report.required;

    sem_post(&info->sem);
    return 0;
}
#endif

static int batt_ptp_register_callback(struct device *dev, ptp_changed callback)
{
    struct ptp_info *info = device_get_private(dev);
    int retval;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    if (!info->changed_cb) {
        info->changed_cb = callback;
        retval = 0;
    } else {
        retval = -EEXIST;
    }

    sem_post(&info->sem);

    return retval;
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

    info->state.boost_fault = false;
    retval = device_ptp_chg_register_boost_fault_cb(info->chg_dev, batt_ptp_boost_fault, info);
    if (retval)
        dbg("boost fault detection not supported\n");

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    bool wireless;
    retval = pad_det_register_callback(batt_ptp_wireless_changed, info);
    if (retval) {
        wireless = false;
        dbg("wireless power source not supported\n");
    } else
        wireless = true;

    info->wired_src_dev = device_open(DEVICE_TYPE_EXT_POWER_HW, 0);
    if (!info->wired_src_dev) {
        dbg("wired power source not supported\n");
        if (!wireless) {
            dbg("failed to find external power sources\n");
            goto ext_err;
        }
    }

    if (info->wired_src_dev) {
        retval = device_ext_power_register_callback(info->wired_src_dev, batt_ptp_wired_changed,
                                                    info);
        if (retval) {
            dbg("failed to register ext power callback\n");
            goto ext_err;
        }
    }

    batt_ptp_set_ext_power_presence(info);
    batt_ptp_choose_ext_power_source(&info->state);
#endif

    retval = battery_state_register(batt_ptp_battery_changed, info);
    if (retval) {
        dbg("failed to register mods_battery callback\n");
        goto batt_err;
    }

    batt_ptp_set_power_needs(info);        /* based on battery state */
    batt_ptp_set_power_availability(info); /* based on battery state and ext power */

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
ext_err:
    device_close(info->wired_src_dev);
#endif
    device_close(info->chg_dev);
chg_err:
    sem_destroy(&info->sem);
    return retval;
}

static struct device_ptp_type_ops batt_ptp_type_ops = {
    .set_current_flow = batt_ptp_set_current_flow,
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    .ext_power_present = batt_ptp_ext_power_present,
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    .get_max_output_current = batt_ptp_get_max_output_current,
    .power_available = batt_ptp_power_available,
    .power_source = batt_ptp_power_source,
#endif
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    .set_max_input_current = batt_ptp_set_max_input_current,
    .power_required = batt_ptp_power_required,
#endif
    .register_callback = batt_ptp_register_callback,
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
