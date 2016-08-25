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
#include <nuttx/gpio.h>
#include <nuttx/power/battery.h>
#include <nuttx/power/bq25896.h>

#include <debug.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <sys/types.h>

#define BQ25896_MAX_INPUT_VOLTAGE       (14000) /* mV */

/* Boost current in OTG mode */
const static int boost_current =
#if defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_500MA)
    500;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_750MA)
    750;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_1200MA)
    1200;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_1400MA)
    1400;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_1650MA)
    1650;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_1875MA)
    1875;
#elif defined (CONFIG_CHARGER_DEVICE_BQ25896_BOOST_LIM_2150MA)
    2150;
#else
    #error "Boost mode current limit is not selected"
#endif

/* Charger configuration */
enum bq25896_chg {
    BQ25896_CHG_OFF,
    BQ25896_CHG_BATTERY,
    BQ25896_OTG,
};

/* Charger Settings */
struct bq25896_chg_settings {
    enum bq25896_chg chg; /* off, charge, otg */
    struct charger_config cfg; /* limits */
};

struct bq25896_charger_info {
    struct bq25896_config init_data; /* initialization data */
    struct bq25896_chg_settings settings; /* as per last last request */
    sem_t chg_sem; /* mutex to set and read chg settings */
    /* callback to notify faults in boost mode */
    charger_boost_fault boost_fault_cb;
    void *boost_fault_arg;
    sem_t boost_fault_sem; /* mutex to register and run callback */
    /* charge enable gpio that controls BQ25896 CE_N pin */
    int chg_en;
    bool chg_en_active_high;
};

/* Drive charge IC CE_N pin */
static void set_bq25896_ce_pin(const struct bq25896_charger_info *info,
                               bool charge)
{
    if (info->chg_en >= 0) {
        if (info->chg_en_active_high)
            gpio_set_value(info->chg_en, charge ? 1 : 0);
        else
            gpio_set_value(info->chg_en, charge ? 0 : 1);
    }
}

/* Configure charge IC to source or sync current, or off */
static int set_bq25896_chg(const struct bq25896_charger_info *info,
                           enum bq25896_chg config)
{
    int ret;

    set_bq25896_ce_pin(info, config == BQ25896_CHG_BATTERY);

    switch (config) {
    case BQ25896_CHG_OFF:
        return bq25896_reg_modify(BQ25896_REG03, BQ25896_REG03_CONFIG_MASK,
                                  BQ25896_REG03_CONFIG_OFF);
    case BQ25896_CHG_BATTERY:
        return bq25896_reg_modify(BQ25896_REG03, BQ25896_REG03_CONFIG_MASK,
                                  BQ25896_REG03_CONFIG_CHG);
    case BQ25896_OTG:
        /* Set boost current and then enable OTG mode */
        switch (boost_current) {
        case 500:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_500MA);
        break;
        case 750:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_750MA);
            break;
        case 1200:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_1200MA);
            break;
        case 1400:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_1400MA);
            break;
        case 1650:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_1650MA);
            break;
        case 1875:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_1875MA);
            break;
        case 2150:
            ret = bq25896_reg_modify(BQ25896_REG0A,
                                     BQ25896_REG0A_BOOST_LIM_MASK,
                                     BQ25896_REG0A_BOOST_LIM_2150MA);
            break;
        default:
            return -EINVAL;
        }

        if (ret)
            return ret;

        return bq25896_reg_modify(BQ25896_REG03, BQ25896_REG03_CONFIG_MASK,
                                  BQ25896_REG03_CONFIG_OTG);
    default:
        return -EINVAL;
    }
}

#define MIN_INPUT_CURRENT   100
#define MAX_INPUT_CURRENT   3250
#define INPUT_CURRENT_STEP  50
static int set_bq25896_input_current_limit(int limit) /* mA */
{
    int val;
    int delta;

    /* Program max possible setting that is less or equal to the limit */
    if (limit < MIN_INPUT_CURRENT)
        return -EINVAL;

    if (limit > MAX_INPUT_CURRENT)
        limit = MAX_INPUT_CURRENT;

    delta = limit - MIN_INPUT_CURRENT;
    val = delta / INPUT_CURRENT_STEP;

    return bq25896_reg_modify(BQ25896_REG00, BQ25896_REG00_INLIM_MASK,
                              val << BQ25896_REG00_INLIM_SHIFT);
}

#define MIN_INPUT_VOLTAGE       3900
#define MAX_INPUT_VOLTAGE       15300
#define INPUT_VOLTAGE_OFFSET    2600
#define INPUT_VOLTAGE_STEP      100
static int set_bq25896_input_voltage_limit(int limit) /* mV */
{
    int val;
    int delta;

    /* Program min possible setting that is greater or equal to the limit */
    if (limit <= MIN_INPUT_VOLTAGE)
        limit = MIN_INPUT_VOLTAGE;

    if (limit > MAX_INPUT_VOLTAGE)
        return -EINVAL;

    delta = limit - INPUT_VOLTAGE_OFFSET;
    val = delta / INPUT_VOLTAGE_STEP;
    if (delta % INPUT_VOLTAGE_STEP) {
        val +=1;
    }

    /* Program absolute threshold and enable it */
    return bq25896_reg_modify(BQ25896_REG0D,
            BQ25896_REG0D_FORCE_VINDPM_MASK | BQ25896_REG0D_VINDPM_MASK,
             BQ25896_REG0D_VINDPM_ABSOLUTE | (val << BQ25896_REG0D_VINDPM_SHIFT));
}

#define MIN_CHARGE_CURRENT  0
#define MAX_CHARGE_CURRENT  3008
#define CHARGE_CURRENT_STEP 64
static int set_bq25896_charge_current_limit(int limit) /* mA */
{
    int val;
    int delta;

    /* Program max possible setting that is less or equal to the limit */
    if (limit < MIN_CHARGE_CURRENT)
        return -EINVAL;

    if (limit > MAX_CHARGE_CURRENT)
        limit = MAX_CHARGE_CURRENT;

    delta = limit - MIN_CHARGE_CURRENT;
    val = delta / CHARGE_CURRENT_STEP;

    return bq25896_reg_modify(BQ25896_REG04, BQ25896_REG04_ICHG_MASK,
                              val << BQ25896_REG04_ICHG_SHIFT);
}

#define MIN_CHARGE_VOLTAGE  3840
#define MAX_CHARGE_VOLTAGE  4608
#define CHARGE_VOLTAGE_STEP 16
static int set_bq25896_charge_voltage_limit(int limit) /* mV */
{
    int val;
    int delta;

    /* Program max possible setting that is less or equal to the limit */
    if (limit < MIN_CHARGE_VOLTAGE)
        return -EINVAL;

    if (limit > MAX_CHARGE_VOLTAGE)
        limit = MAX_CHARGE_VOLTAGE;

    delta = limit - MIN_CHARGE_VOLTAGE;
    val = delta / CHARGE_VOLTAGE_STEP;

    return bq25896_reg_modify(BQ25896_REG06, BQ25896_REG06_VREG_MASK,
                              val << BQ25896_REG06_VREG_SHIFT);
}

static void set_bq25896_limits(const struct charger_config *cfg)
{
    (void) set_bq25896_input_current_limit(cfg->input_current_limit);
    (void) set_bq25896_input_voltage_limit(cfg->input_voltage_limit);
    (void) set_bq25896_charge_current_limit(cfg->charge_current_limit);
    (void) set_bq25896_charge_voltage_limit(cfg->charge_voltage_limit);
}

static void config_bq25896(struct bq25896_charger_info *info,
                         enum bq25896_chg chg, const struct charger_config *cfg)
{
    /*
     * When battery is dead, charge IC is off until power is applied.
     * Settings need to be programmed in power good callback then.
     */
    if (cfg) {
        info->settings.cfg = *cfg; /* for power good callback */
        set_bq25896_limits(cfg);
    }

    info->settings.chg = chg; /* for power good callback */
    (void) set_bq25896_chg(info, chg);

    battery_set_status(chg == BQ25896_CHG_BATTERY ? BATTERY_CHARGING :
                                                    BATTERY_DISCHARGING);
}

static void bq25896_pwr_good(struct bq25896_charger_info *info)
{
    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return;
        }
    }

    /* Program the IC in case it just powered up. */
    bq25896_configure(&info->init_data);

    switch (info->settings.chg) {
    case BQ25896_CHG_OFF:
        set_bq25896_chg(info, BQ25896_CHG_OFF);
        break;
    case BQ25896_CHG_BATTERY:
        set_bq25896_limits(&info->settings.cfg);
        set_bq25896_chg(info, BQ25896_CHG_BATTERY);
        break;
    default:
        /* Nothing to be done here */
        break;
    }

    sem_post(&info->chg_sem);
}

static void bq25896_boost_fault(struct bq25896_charger_info *info)
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

static void bq25896_charger_callback(enum bq25896_event event, void *arg)
{
    struct bq25896_charger_info *info = arg;

    switch(event) {
    case POWER_GOOD:
        bq25896_pwr_good(info);
        break;
    case BOOST_FAULT:
        bq25896_boost_fault(info);
        break;
    default:
        /* Should never be here */
        break;
    }
}

static int bq25896_charger_send(struct device *dev, int *current)
{
    struct bq25896_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    /* Program the IC in case it lost its settings due to UVLO */
    bq25896_configure(&info->init_data);

    *current = boost_current;
    config_bq25896(info, BQ25896_OTG, NULL);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq25896_charger_receive(struct device *dev, const struct charger_config *cfg)
{
    struct bq25896_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq25896(info, BQ25896_CHG_BATTERY, cfg);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq25896_charger_off(struct device *dev)
{
    struct bq25896_charger_info *info = device_get_private(dev);

    while (sem_wait(&info->chg_sem) != OK) {
        if (errno == EINVAL) {
            return -EINVAL;
        }
    }

    config_bq25896(info, BQ25896_CHG_OFF, NULL);
    sem_post(&info->chg_sem);

    return 0;
}

static int bq25896_charger_register_boost_fault_cb(struct device *dev,
                                             charger_boost_fault cb, void *arg)
{
    struct bq25896_charger_info *info = device_get_private(dev);

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

static int bq25896_charger_max_input_voltage(struct device *dev, int *voltage)
{
    *voltage = BQ25896_MAX_INPUT_VOLTAGE;
    return 0;
}

static int bq25896_get_gpio(struct device *dev, char* name)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, name);
    return r ? r->start : -1;
}

static int bq25896_charger_probe(struct device *dev)
{
    struct bq25896_charger_info *info;
    struct bq25896_config *init_data;
    int int_n, pg_n;
    int i, ret;

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    /* Copy init data */
    init_data = dev->init_data;
    if (init_data && init_data->reg_num) {
        info->init_data.reg = zalloc(
                              sizeof(struct bq25896_reg) * init_data->reg_num);
        if (!info->init_data.reg) {
            dbg("failed to allocate memory for init data\n");
            free(info);
            return -ENOMEM;
        }

        info->init_data.reg_num = init_data->reg_num;
        for (i = 0; i < init_data->reg_num; i++)
            info->init_data.reg[i] = init_data->reg[i];
    }


    /* Optional CHG_EN signal */
    info->chg_en = bq25896_get_gpio(dev, "chg_en");
    if (info->chg_en >= 0)
        info->chg_en_active_high = true;
    else
        info->chg_en = bq25896_get_gpio(dev, "chg_en_n");

    /* Disable charging by default */
    if (info->chg_en >= 0)
        gpio_direction_out(info->chg_en, info->chg_en_active_high? 0 : 1);

    /* Optional IRQ lines */
    int_n = bq25896_get_gpio(dev, "int_n");
    pg_n = bq25896_get_gpio(dev, "pg_n");

    ret = bq25896_driver_init(int_n, pg_n);
    if (ret) {
        dbg("failed to initialize bq25896 driver\n");
        free(info->init_data.reg);
        free(info);
        return ret;
    }

    device_set_private(dev, info);
    return 0;
}

static void bq25896_charger_remove(struct device *dev)
{
    struct bq25896_charger_info *info = device_get_private(dev);

    free(info->init_data.reg);
    free(info);
    device_set_private(dev, NULL);
}

static int bq25896_charger_open(struct device *dev)
{
    struct bq25896_charger_info *info = device_get_private(dev);
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

    /* disable ship mode, allow BATFET turn on */
    bq25896_reg_modify(BQ25896_REG09, BQ25896_REG09_BATFET_MASK,
            BQ25896_REG09_BATFET_ON);

    /* No charging by default */
    config_bq25896(info, BQ25896_CHG_OFF, NULL);

    retval = bq25896_register_callback(bq25896_charger_callback, info);
    if (retval) {
        dbg("failed to register callback\n");
        sem_destroy(&info->chg_sem);
        sem_destroy(&info->boost_fault_sem);
        return retval;
    }

    return 0;
}

static void bq25896_charger_close(struct device *dev)
{
    struct bq25896_charger_info *info = device_get_private(dev);

    sem_destroy(&info->chg_sem);
    sem_destroy(&info->boost_fault_sem);
}

static struct device_charger_type_ops bq25896_charger_type_ops = {
    .send = bq25896_charger_send,
    .receive = bq25896_charger_receive,
    .off = bq25896_charger_off,
    .register_boost_fault_cb = bq25896_charger_register_boost_fault_cb,
    .max_input_voltage = bq25896_charger_max_input_voltage,
};

static struct device_driver_ops driver_ops = {
    .probe = bq25896_charger_probe,
    .remove = bq25896_charger_remove,
    .open = bq25896_charger_open,
    .close = bq25896_charger_close,
    .type_ops = &bq25896_charger_type_ops,
};

struct device_driver bq25896_charger_driver = {
    .type = DEVICE_TYPE_CHARGER_HW,
    .name = "bq25896_charger",
    .desc = "Charger driver for TI bq25896 IC",
    .ops = &driver_ops,
};
