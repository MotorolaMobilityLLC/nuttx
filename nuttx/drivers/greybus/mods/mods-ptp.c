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
#include <nuttx/power/ext_power.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <nuttx/util.h>

#ifndef CONFIG_GREYBUS_MODS_MAX_VBUS_CURRENT
  #define CONFIG_GREYBUS_MODS_MAX_VBUS_CURRENT      3000    /* mA */
#endif

#define DEFAULT_BASE_CURRENT                        500     /* mA */
#define DEFAULT_BASE_VOLTAGE                        5000    /* mV */
#define DEFAULT_PMIC_MAX_INPUT_VOLTAGE              5000    /* mV */
#define DEFAULT_MAX_OUTPUT_VOLTAGE                  5000    /* mV */

/* TODO: Support variable voltage battery source */
#define BATTERY_SOURCE_OUTPUT_VOLTAGE               5000    /* mV */

/* Maximum output current and voltage of a charging source */
struct mods_ptp_chg_source {
    int current; /* mA */
    int voltage; /* mV */
};

/* Charging/discharging settings */
struct mods_ptp_battery {
    bool chg_allowed; /* flag */
    bool dischg_allowed; /* flag */
    bool low_battery; /* flag */
    /* Limits are in mA and mV */
    int charge_current; /* as per battery temp */
    int charge_voltage; /* as per battery temp */
#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
    struct mods_ptp_chg_source src; /* Mod to Base charging */
#endif
};

/* Commands from the base */
struct mods_ptp_base_commands {
    enum ptp_current_flow direction; /* charging direction */
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    struct mods_ptp_chg_source base; /* Phone to Mod charging */
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    int max_output_voltage; /* maximum voltage that the Mod can send to Base in mV */
#endif
};

/* Values reported to the base */
struct ptp_reported {
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    enum ptp_ext_power present; /* external power sources */
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    enum ptp_power_available available; /* source that will be sent to the base */
    /* Source of power sent to the base */
    enum ptp_power_source source; /* source type */
    struct mods_ptp_chg_source to_base; /* source max output current and voltage */
#endif
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    enum ptp_power_required required; /* can power be received from the base ? */
    int max_input_voltage; /* maximum voltage that the Mod can accept from the base in mV */
#endif
};

/* Internal state */
struct ptp_state {
    enum base_attached_e attached; /* base attachment */
    struct mods_ptp_base_commands commands; /* commands from the base */
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
    struct mods_ptp_battery battery; /* battery chg/dischg settings */
#endif
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    struct mods_ptp_chg_source wired, wireless; /* external power sources */
#endif
    struct mods_ptp_chg_source dongle; /* power adapter over vbus */
    int pmic_max_input_voltage; /* max voltage that that PMIC can take */
    bool base_powered_off; /* flag to indicate base is powered off with AMP attached */
    struct ptp_reported report; /* what is repoted to the base */
    bool boost_fault; /* flag to indicate fault in boost mode */
};

struct ptp_info {
    struct ptp_state state; /* current state */
    sem_t sem; /* mutual exclusion to set and process state */
    struct device *batt_dev; /*battery temp and level notifications */
    struct device *chg_dev; /* current control */
    /* external power present, power available or required have changed */
    ptp_changed changed_cb;
};

/* Set base charging source */
static void do_set_base_source(struct ptp_state *state, enum ptp_power_source source)
{
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
        int current, voltage;

        switch(source)
        {
            case PTP_POWER_SOURCE_NONE:
                current = 0;
                voltage = 0;
                break;
#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
            case PTP_POWER_SOURCE_BATTERY:
                current = state->battery.src.current;
                voltage = state->battery.src.voltage;
                break;
#endif
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
            case PTP_POWER_SOURCE_WIRED:
                current = state->wired.current;
                voltage = state->wired.voltage;
                break;
            case PTP_POWER_SOURCE_WIRELESS:
                current = state->wireless.current;
                voltage = state->wireless.voltage;
                break;
#endif
            default:
                /* Should never be here */
                return;
        }

        state->report.source = source;
        /* Current cannot exceed the limit of the Mod VBUS connector */
        state->report.to_base.current = MIN(CONFIG_GREYBUS_MODS_MAX_VBUS_CURRENT, current);
        state->report.to_base.voltage = voltage;
#endif
}

/* Calculate power of the source. Return 0 if source voltage above the max_v */
static inline int64_t do_calculate_power(const struct mods_ptp_chg_source *src,
                                  int max_v)
{
    return src->voltage <= max_v ? src->current * src->voltage : 0;
}

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
/* Choose the source with the maximum output power */
static const struct mods_ptp_chg_source *do_choose_ext_src(const struct ptp_state *state,
                                                           int max_v)
{
    int i;
    int pwr = 0;
    int selected = 0;
    const struct mods_ptp_chg_source *srcs[] = {
        NULL,
        &state->wired,
        &state->wireless,
        &state->dongle,
    };

    for (i = 1; i < ARRAY_SIZE(srcs); i++) {
        int new_pwr = do_calculate_power(srcs[i], max_v);
        if (new_pwr > pwr) {
            pwr = new_pwr;
            selected = i;
        }
    }
    vdbg("selected = %d\n", selected);

    return srcs[selected];
}
#endif

/*
 * Charge base from an external power source if it is present, otherwise from a
 * battery if conditions permit
 */
static int do_charge_base(struct device *chg, struct ptp_state *state)
{
    int retval = 0;

    do_set_base_source(state, PTP_POWER_SOURCE_NONE);

    /* Disable base charging and set current flow to "off" if fault in boost
       mode is detected */
    if (state->boost_fault) {
        state->commands.direction = PTP_CURRENT_OFF;
        retval = device_ptp_chg_off(chg);
        goto done;
    }

#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    /* The voltage of the external source cannot exceed input and output limits */
    int max_v = MIN(state->commands.max_output_voltage, state->pmic_max_input_voltage);
    const struct mods_ptp_chg_source *src = do_choose_ext_src(state, max_v);

    /* Limit voltage on all sources */
    ext_power_set_max_output_voltage(EXT_POWER_WIRED, max_v);
    ext_power_set_max_output_voltage(EXT_POWER_WIRELESS, max_v);

    if (src == &state->wired) {
        if ((retval = device_ptp_chg_send_wired_pwr(chg)) == 0)
            do_set_base_source(state, PTP_POWER_SOURCE_WIRED);
        goto done;
    } else if (src == &state->wireless) {
        if ((retval = device_ptp_chg_send_wireless_pwr(chg)) == 0)
            do_set_base_source(state, PTP_POWER_SOURCE_WIRELESS);
        goto done;
    }

#endif

#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
    if (state->battery.dischg_allowed && !state->base_powered_off) {
        if ((retval = device_ptp_chg_send_batt_pwr(chg, &state->battery.src.current)) == 0) {
            state->battery.src.voltage = BATTERY_SOURCE_OUTPUT_VOLTAGE;
            if (state->battery.low_battery) {
                state->battery.src.current = MIN(state->battery.src.current,
                                CONFIG_GREYBUS_MODS_LOW_BATTERY_OUTPUT_CURRENT);
            } else {
                state->battery.src.current = MIN(state->battery.src.current,
                             CONFIG_GREYBUS_MODS_NORMAL_BATTERY_OUTPUT_CURRENT);
            }

            do_set_base_source(state, PTP_POWER_SOURCE_BATTERY);
        }
        goto done;
    }
#endif

    retval = device_ptp_chg_off(chg);

done:
    return retval;
}

#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
/*
 * Charge battery with power from an external power source if it is present and
 * conditions permit
 */
static int do_charge_battery_with_external_power(struct device *chg,
                                                 struct ptp_state *state)
{
    do_set_base_source(state, PTP_POWER_SOURCE_NONE);
    int max_v = state->pmic_max_input_voltage;
    struct charger_config cfg;

#ifndef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    const struct mods_ptp_chg_source *src = &state->dongle;

    if ((do_calculate_power(src, max_v) > 0) &&
        (state->battery.chg_allowed && src->current)) {
            return device_ptp_chg_receive_base_pwr(chg, &cfg);
    }

    return device_ptp_chg_off(chg);
#else
    const struct mods_ptp_chg_source *src = do_choose_ext_src(state, max_v);

    /* Limit voltage on all sources */
    ext_power_set_max_output_voltage(EXT_POWER_WIRED, max_v);
    ext_power_set_max_output_voltage(EXT_POWER_WIRELESS, max_v);

    if (state->battery.chg_allowed && src == NULL) {
        return device_ptp_chg_all_paths_open(chg);
    }

    if (state->battery.chg_allowed && src && src->current) {
        cfg.input_current_limit = src->current;
        cfg.input_voltage_limit = src->voltage - CONFIG_GREYBUS_MODS_INPUT_VOLTAGE_OFFSET;
        cfg.charge_current_limit = state->battery.charge_current;
        cfg.charge_voltage_limit = state->battery.charge_voltage;
        if (src == &state->wired)
            return device_ptp_chg_receive_wired_pwr(chg, &cfg);
        else if (src == &state->wireless)
            return device_ptp_chg_receive_wireless_pwr(chg, &cfg);
        else
            return device_ptp_chg_receive_base_pwr(chg, &cfg);
    } else
        return device_ptp_chg_receive_base_pwr(chg, &cfg);
#endif
}

/*
 * Charge battery with power from base if conditions permit
 */
static int do_charge_battery_with_base_power(struct device *chg,
                                             struct ptp_state *state)
{
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    do_set_base_source(state, PTP_POWER_SOURCE_NONE);

    struct charger_config cfg;

    if (!state->battery.chg_allowed) {
        return device_ptp_chg_off(chg);
    } else {
        cfg.input_current_limit = state->commands.base.current;
        cfg.input_voltage_limit = state->commands.base.voltage -
                                  CONFIG_GREYBUS_MODS_INPUT_VOLTAGE_OFFSET;
        cfg.charge_current_limit = state->battery.charge_current;
        cfg.charge_voltage_limit = state->battery.charge_voltage;
        return device_ptp_chg_receive_base_pwr(chg, &cfg);
    }
#else
    /* Should never be here */
    return -EINVAL;
#endif
}
#endif

/* Disable power transfer */
static void do_off(struct device *chg, struct ptp_state *state)
{
    do_set_base_source(state, PTP_POWER_SOURCE_NONE);
    device_ptp_chg_off(chg);
}

static int mods_ptp_process(struct device *chg, struct ptp_state *state)
{
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
    switch (state->attached) {
    case BASE_ATTACHED_OFF:
        return do_charge_base(chg, state);
    case BASE_ATTACHED:
        switch (state->commands.direction) {
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
        return 0;
        break;
    default:
        do_off(chg, state);
        return -EINVAL;
    }
#else
    if (state->attached == BASE_ATTACHED_OFF ||
        (state->attached == BASE_ATTACHED && state->commands.direction == PTP_CURRENT_FROM_MOD)) {
      return do_charge_base(chg, state);
    }

    do_off(chg, state);
    return 0;
#endif
}

static void mods_ptp_reset_base_commands(struct mods_ptp_base_commands *commands)
{

    commands->direction = PTP_CURRENT_OFF;
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    /* Current cannot exceed the limit of the Mod VBUS connector */
    commands->base.current = MIN(CONFIG_GREYBUS_MODS_MAX_VBUS_CURRENT, DEFAULT_BASE_CURRENT);
    commands->base.voltage = DEFAULT_BASE_VOLTAGE;
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    commands->max_output_voltage = DEFAULT_MAX_OUTPUT_VOLTAGE;
#endif
}

static int mods_ptp_attach_changed(FAR void *arg, const void *data)
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    if (init) {
        init = false;
        info->state.attached = state;
        if (state == BASE_ATTACHED_OFF)
            info->state.base_powered_off = true;
        else
            info->state.base_powered_off = false;
        return OK;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -errno;
        }
    }

    if (info->state.attached == state)
        goto attach_done;

    /* Set commands to defaults when attached, reset boost fault flag when
       detached */
    if (state == BASE_ATTACHED)
        mods_ptp_reset_base_commands(&info->state.commands);
    else if (state == BASE_DETACHED)
        info->state.boost_fault = false;

    if (state == BASE_ATTACHED_OFF && info->state.attached == BASE_ATTACHED)
        info->state.base_powered_off = true;
    else
        info->state.base_powered_off = false;

    info->state.attached = state;
    mods_ptp_process(info->chg_dev, &info->state);

attach_done:
    sem_post(&info->sem);

    return OK;
}

static int mods_ptp_set_current_flow(struct device *dev, uint8_t direction)
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

    if (info->state.commands.direction == direction)
        goto current_flow_done;

    info->state.commands.direction = direction;
    /*
     * Always report success to the base. If the direction cannot be set
     * now due to the conditions such as for example, battery level and/or temp,
     * the current flow direction will be set once the conditions allow it.
     */
    (void) mods_ptp_process(info->chg_dev, &info->state);

current_flow_done:
    sem_post(&info->sem);
    return retval;
}

static int mods_ptp_get_current_flow(struct device *dev, uint8_t *direction)
{
    struct ptp_info *info = device_get_private(dev);
    int retval = 0;

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *direction = info->state.commands.direction;

    sem_post(&info->sem);
    return retval;
}

static void mods_ptp_set_power_availability(struct ptp_info *info)
{
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    enum ptp_power_available old_available = info->state.report.available;

    /* Check external power first since this is the preferred source */
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    if (info->state.wired.current > 0 || info->state.wireless.current > 0 ||
        info->state.dongle.current > 0) {
        info->state.report.available = PTP_POWER_AVAILABLE_EXT;
        goto done;
    }
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
    if (info->state.battery.dischg_allowed) {
        info->state.report.available = PTP_POWER_AVAILABLE_INT;
        goto done;
    }
#endif

done:
    /* Notify the change */
    if (info->changed_cb && (info->state.report.available != old_available))
        info->changed_cb(POWER_AVAILABLE);
#endif
}

static void mods_ptp_set_power_needs(struct ptp_info *info)
{
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    enum ptp_power_required old_required = info->state.report.required;

    info->state.report.required = info->state.battery.chg_allowed ?
                                  PTP_POWER_REQUIRED : PTP_POWER_NOT_REQUIRED;

    /* Notify the change */
    if (info->changed_cb && (info->state.report.required != old_required))
        info->changed_cb(POWER_REQUIRED);
#endif
}

#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
static void mods_ptp_boost_fault(void *arg)
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
        mods_ptp_process(info->chg_dev, &info->state);
        dbg("fault detected while sourcing battery to the base\n");
    }

    sem_post(&info->sem);
}
#endif
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
static void mods_ptp_set_battery_state(const struct batt_state_s *batt,
                                       struct mods_ptp_battery *state)
{
    state->chg_allowed = (batt->temp != BATTERY_TEMP_NO_CHARGING &&
                          batt->temp != BATTERY_TEMP_COOL_DOWN &&
                          batt->level != BATTERY_LEVEL_FULL);
    state->dischg_allowed = (batt->temp != BATTERY_TEMP_COOL_DOWN &&
                             batt->temp != BATTERY_TEMP_UNAVAILABLE &&
                             batt->level != BATTERY_LEVEL_EMPTY);

    state->low_battery = (batt->level != BATTERY_LEVEL_FULL && batt->level != BATTERY_LEVEL_NORMAL);

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

static void mods_ptp_battery_changed(void *arg,
                                     const struct batt_state_s *batt)
{
    static bool init = true; /* first callback is to initialize */
    static struct batt_state_s current;
    struct ptp_info *info = arg;
    bool chg_allowed, dischg_allowed, low_battery;

    if (init) {
        init = false;
        current = *batt;
        mods_ptp_set_battery_state(batt, &info->state.battery);
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    if (current.temp == batt->temp &&
        current.level == batt->level &&
        current.voltage == batt->voltage)
        goto battery_done;

    current = *batt;
    chg_allowed = info->state.battery.chg_allowed;
    dischg_allowed = info->state.battery.dischg_allowed;
    low_battery = info->state.battery.low_battery;
    mods_ptp_set_battery_state(batt, &info->state.battery);

    if (!dischg_allowed || low_battery != info->state.battery.low_battery) {
#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
        /* Transition to off if transfering battery power and conditions changed */
        if (info->state.report.source == PTP_POWER_SOURCE_BATTERY)
            info->state.commands.direction = PTP_CURRENT_OFF;
#endif
    }

    if (chg_allowed != info->state.battery.chg_allowed)
        mods_ptp_set_power_needs(info);
    if (dischg_allowed != info->state.battery.dischg_allowed)
        mods_ptp_set_power_availability(info);

    mods_ptp_process(info->chg_dev, &info->state);
battery_done:
    sem_post(&info->sem);
}
#endif
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
static int mods_ptp_ext_power_present(struct device *dev, uint8_t *present)
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

static void mods_ptp_set_ext_power_presence(struct ptp_info *info)
{
    enum ptp_ext_power old_present = info->state.report.present;

    if (info->state.wired.current > 0 && info->state.wireless.current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRED_WIRELESS_PRESENT;
    else if (info->state.wired.current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRED_PRESENT;
    else if (info->state.wireless.current > 0)
        info->state.report.present = PTP_EXT_POWER_WIRELESS_PRESENT;
    else if (info->state.dongle.current > 0)
        info->state.report.present = PTP_EXT_POWER_DONGLE_PRESENT;
    else
        info->state.report.present = PTP_EXT_POWER_NOT_PRESENT;

    /* Notify the change */
    if (info->changed_cb && (info->state.report.present != old_present))
        info->changed_cb(POWER_PRESENT);
}

static void mods_ptp_get_ext_power_output(struct device *dev,
                                          struct mods_ptp_chg_source *output)
{
    device_ext_power_output_s dev_output;

    if (dev && !device_ext_power_get_output(dev, &dev_output)) {
        output->voltage = dev_output.voltage;
        output->current = dev_output.current;
    } else {
        output->current = output->voltage = 0;
    }
}

/* Get external power sources output current and voltage */
static void mods_ptp_get_external_power_sources(struct ptp_state *state,
                                                struct device *const dev[])
{
    mods_ptp_get_ext_power_output(dev[EXT_POWER_WIRED], &state->wired);
    mods_ptp_get_ext_power_output(dev[EXT_POWER_DONGLE], &state->dongle);
    mods_ptp_get_ext_power_output(dev[EXT_POWER_WIRELESS], &state->wireless);
}

/* Have current or voltage changed ? */
static bool mods_ptp_ext_src_changed(const struct mods_ptp_chg_source *before,
                                     const struct mods_ptp_chg_source *now)
{
    return (before->current != now->current || before->voltage != now->voltage);
}

/* Will Base have a new source or has the current source changed */
static bool mods_ptp_base_source_changed(const struct ptp_state *state)
{
    const struct mods_ptp_chg_source *new_src;
    int max_v;

    if (state->report.source == PTP_POWER_SOURCE_NONE)
        return false;

    max_v = MIN(state->commands.max_output_voltage, state->pmic_max_input_voltage);
    new_src = do_choose_ext_src(state, max_v);

    switch(state->report.source) {
    case PTP_POWER_SOURCE_BATTERY:
        return new_src != NULL;
    case PTP_POWER_SOURCE_WIRED:
        return (new_src != &state->wired ||
                mods_ptp_ext_src_changed(&state->report.to_base, &state->wired));
    case PTP_POWER_SOURCE_WIRELESS:
        return (new_src != &state->wireless ||
                mods_ptp_ext_src_changed(&state->report.to_base, &state->wireless));
    default:
        /* Should never be here! */
        return true;
    }
}

static void mods_ptp_ext_power_changed(void *arg, struct device *const dev[])
{
    static bool init = true; /* first callback is to initialize */
    struct ptp_info *info = arg;

    mods_ptp_get_external_power_sources(&info->state, dev);

    if (init) {
        init = false;
        mods_ptp_set_ext_power_presence(info);
        return;
    }

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    mods_ptp_set_ext_power_presence(info);
    mods_ptp_set_power_availability(info);

    /* Transition to off if charging Base and the source has changed */
    if (mods_ptp_base_source_changed(&info->state))
        info->state.commands.direction = PTP_CURRENT_OFF;

    mods_ptp_process(info->chg_dev, &info->state);

    sem_post(&info->sem);
}
#endif

#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
static int mods_ptp_get_max_output_current(struct device *dev, uint32_t *current)
{
    struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    *current = info->state.report.to_base.current * 1000; /* mA to uA */

    sem_post(&info->sem);
    return 0;
}

int mods_ptp_set_max_output_voltage(struct device *dev, uint32_t voltage)
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
        goto output_voltage_done;
    }

    voltage /= 1000; /* uV to mV */

    if (info->state.commands.max_output_voltage == voltage)
        goto output_voltage_done;

    info->state.commands.max_output_voltage = voltage;
    (void) mods_ptp_process(info->chg_dev, &info->state);

output_voltage_done:
    sem_post(&info->sem);
    return retval;
}

int mods_ptp_get_output_voltage(struct device *dev, uint32_t *voltage)
{
   struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

   *voltage = info->state.report.to_base.voltage * 1000; /* mV to uV */

    sem_post(&info->sem);
    return 0;
}

static int mods_ptp_power_available(struct device *dev, uint8_t *available)
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

static int mods_ptp_power_source(struct device *dev, uint8_t *source)
{
   struct ptp_info *info = device_get_private(dev);

    while (sem_wait(&info->sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

   *source = info->state.report.source;

    sem_post(&info->sem);
    return 0;
}
#endif

#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
static int mods_ptp_set_max_input_current(struct device *dev, uint32_t current)
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

    /* Current cannot exceed the limit of the Mod VBUS connector */
    current = MIN(CONFIG_GREYBUS_MODS_MAX_VBUS_CURRENT, current / 1000 /*uA to mA*/);
    if (info->state.commands.base.current == current)
        goto input_current_done;

    info->state.commands.base.current = current;
    /*
     * Always report success to the base. If the limit cannot be set
     * now due to the conditions such as for example, battery level and/or temp,
     * the limit will be set once the conditions allow it.
     */
    (void) mods_ptp_process(info->chg_dev, &info->state);

input_current_done:
    sem_post(&info->sem);
    return retval;
}

static int mods_ptp_get_max_input_voltage(struct device *dev, uint32_t *voltage)
{
    struct ptp_info *info = device_get_private(dev);

    *voltage = info->state.report.max_input_voltage * 1000; /* mV to uV */
    return 0;
}

int mods_ptp_set_input_voltage(struct device *dev, uint32_t voltage)
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
        dbg("received input voltage request while not attached\n");
        retval = -EPERM;
        goto input_voltage_done;
    }

    voltage /= 1000; /* uV to mV */

    if (info->state.commands.base.voltage == voltage)
        goto input_voltage_done;

    info->state.commands.base.voltage = voltage;
    /*
     * Always report success to the base. If the limit cannot be set
     * now due to the conditions such as for example, battery level and/or temp,
     * the limit will be set once the conditions allow it.
     */
    (void) mods_ptp_process(info->chg_dev, &info->state);

input_voltage_done:
    sem_post(&info->sem);
    return retval;
}

static int mods_ptp_power_required(struct device *dev, uint8_t *required)
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

static int mods_ptp_register_callback(struct device *dev, ptp_changed callback)
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

static int mods_ptp_probe(struct device *dev)
{
    struct ptp_info *info = zalloc(sizeof(*info));

    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    device_set_private(dev, info);

    return 0;
}

static int mods_ptp_open(struct device *dev)
{
    struct ptp_info *info = device_get_private(dev);
    int retval;

    /* Init the state and process it, then post semaphore */
    retval = sem_init(&info->sem, 0, 0);
    if (retval) {
        dbg("failed to init semaphore\n");
        return retval;
    }

    /* Set defaults */
    mods_ptp_reset_base_commands(&info->state.commands);

    info->chg_dev = device_open(DEVICE_TYPE_PTP_CHG_HW, 0);
    if (!info->chg_dev) {
        dbg("failed to open charger device\n");
        retval = -EIO;
        goto chg_err;
    }

    info->state.boost_fault = false;
#ifndef CONFIG_GREYBUS_PTP_INT_SND_NEVER
    retval = device_ptp_chg_register_boost_fault_cb(info->chg_dev, mods_ptp_boost_fault, info);
    if (retval)
        dbg("boost fault detection not supported\n");
#endif

    retval = device_ptp_chg_max_input_voltage(info->chg_dev, &info->state.pmic_max_input_voltage);
    if (retval)
        info->state.pmic_max_input_voltage = DEFAULT_PMIC_MAX_INPUT_VOLTAGE;
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    /* Must be less than connector and the charger IC voltage */
    info->state.report.max_input_voltage =
        MIN(info->state.pmic_max_input_voltage, CONFIG_GREYBUS_MODS_MAX_VBUS_VOLTAGE);
#endif


#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    retval = ext_power_register_callback(mods_ptp_ext_power_changed, info);
    if (retval) {
        dbg("failed to register ext power callback\n");
        goto ext_err;
    }
#endif

#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
    retval = battery_state_register(mods_ptp_battery_changed, info);
    if (retval) {
        dbg("failed to register mods_battery callback\n");
        goto batt_err;
    }
#endif
    mods_ptp_set_power_needs(info);        /* based on battery state */
    mods_ptp_set_power_availability(info); /* based on battery state and ext power */

    retval = mods_attach_register(mods_ptp_attach_changed, info);
    if (retval) {
        dbg("failed to register mods_attach callback\n");
        goto atch_err;
    }

    mods_ptp_process(info->chg_dev, &info->state);
    sem_post(&info->sem);

    return 0;

atch_err:
#ifdef CONFIG_GREYBUS_MODS_PTP_DEVICE_HAS_BATTERY
batt_err:
#endif
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
ext_err:
#endif
    device_close(info->chg_dev);
chg_err:
    sem_destroy(&info->sem);
    return retval;
}

static struct device_ptp_type_ops mods_ptp_type_ops = {
    .set_current_flow = mods_ptp_set_current_flow,
    .get_current_flow = mods_ptp_get_current_flow,
#ifdef CONFIG_GREYBUS_PTP_EXT_SUPPORTED
    .ext_power_present = mods_ptp_ext_power_present,
#endif
#if !defined (CONFIG_GREYBUS_PTP_INT_SND_NEVER) || defined (CONFIG_GREYBUS_PTP_EXT_SUPPORTED)
    .get_max_output_current = mods_ptp_get_max_output_current,
    .set_max_output_voltage = mods_ptp_set_max_output_voltage,
    .get_output_voltage = mods_ptp_get_output_voltage,
    .power_available = mods_ptp_power_available,
    .power_source = mods_ptp_power_source,
#endif
#ifndef CONFIG_GREYBUS_PTP_INT_RCV_NEVER
    .set_max_input_current = mods_ptp_set_max_input_current,
    .get_max_input_voltage = mods_ptp_get_max_input_voltage,
    .set_input_voltage = mods_ptp_set_input_voltage,
    .power_required = mods_ptp_power_required,
#endif
    .register_callback = mods_ptp_register_callback,
};

static struct device_driver_ops mods_ptp_driver_ops = {
    .probe = mods_ptp_probe,
    .open = mods_ptp_open,
    .type_ops = &mods_ptp_type_ops,
};

struct device_driver mods_ptp_driver = {
    .type = DEVICE_TYPE_PTP_HW,
    .name = "mods_ptp",
    .desc = "Power transfer protocol for devices with a battery and/or external power source",
    .ops = &mods_ptp_driver_ops,
};
