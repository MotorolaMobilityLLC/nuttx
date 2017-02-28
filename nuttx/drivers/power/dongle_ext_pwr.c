/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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
#include <debug.h>
#include <stdint.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/device_ext_power.h>
#include <nuttx/gpio.h>
#include <nuttx/power/bq25896.h>
#include <nuttx/greybus/mods.h>

#ifndef CONFIG_CHARGER_BQ25896
# error "Dongle charging only implemented for BQ25896"
#endif

struct dongle_dev_s {
    struct device *dev;
    enum base_attached_e attach_state;
#ifdef CONFIG_CHARGER_BQ25896
    enum bq25896_event power_good_state;
#endif

    bool open;
    int gpio_capability;  /* dongles current capability */
    gpio_cfg_t gpio_cfgset;

    int current;

    device_ext_power_notification power_changed_cb;
    void *power_changed_arg;
};

/*****
 *  The dongle can support charging at 500mA, 1.5A, or 3A.
 *  It communicates its current capabilities over the SPI Clock
 *  pin (which will be the gpio below).
 *
 *  The generic gpio system currently does not have the capability
 *  of manipulating pullups, so the chipset level gpio configuration
 *  is passed in to this function to switch from output to input with
 *  a pullup.
 */
static int dongle_read_capabilities(int gpio, gpio_cfg_t cfg_in_pu)
{
    gpio_cfg_t *saved = gpio_cfg_save(gpio);
    int current = 0;
    int i;
    int val;

    /* set the direction input with pullup                   */
    /* wait at most 10 ms for the pullup to take effect      */
    gpio_cfg_set(gpio, cfg_in_pu);  /* input with pullup     */
    for (i = 0; i < 100 && !(val = gpio_get_value(gpio)); i++) {
        usleep(100);
    }
    if (!val) {
        dbg("ERROR: gpio never went high\n");
        goto done;
    }

    /*********************************************************/
    /*                 Test for 500 mA                       */
    /*********************************************************/
    /* If after 20 ms, the value goes low we support 500 mA  */
    usleep(20000);
    val = gpio_get_value(gpio);
    if (!val) {
        current = 500;
        goto done;
    }

    /*********************************************************/
    /*                  Test for 1.5 A                       */
    /*********************************************************/
    /* pulse the pin low for  to kick off the next reading   */
    /* If after 10 ms, the value goes low we support 1500 mA */
    gpio_direction_out(gpio, 0);
    usleep(1000);
    gpio_cfg_set(gpio, cfg_in_pu);  /* input with pullup     */
    usleep(10000);

    val = gpio_get_value(gpio);
    if (!val) {
        current = 1500;
        goto done;
    }

    /*********************************************************/
    /*                 Test for 3.0 A                        */
    /*********************************************************/
    /* If after 10 ms, the value goes low we support 3000 mA */
    gpio_direction_out(gpio, 0);
    usleep(1000);
    gpio_cfg_set(gpio, cfg_in_pu);  /* input with pullup     */
    usleep(10000);

    val = gpio_get_value(gpio);
    if (!val) {

        current = 3000;
        goto done;
    }

done:
    gpio_direction_out(gpio, 0);
    dbg("we support %d mA\n", current);
    if (current)
        usleep(100000);
    gpio_cfg_restore(gpio, saved);
    return current;
}

static int dongle_attach_callback(void *arg, const void *data)
{
    struct dongle_dev_s *info = arg;
    enum base_attached_e attach_state = *(enum base_attached_e *)data;

    if (info->attach_state == attach_state)
        return 0;

    vdbg("attached changed from %d => %d\n", info->attach_state, attach_state);

    /* clear current on any change in attach state */
    info->current = 0;

    info->attach_state = attach_state;

    if ((info->power_good_state == POWER_GOOD) &&
        (info->attach_state == BASE_DETACHED)) {
        int current = dongle_read_capabilities(info->gpio_capability,
                info->gpio_cfgset); 
        if (current >= 0)
            info->current = current;
    }

    /* call ext_power_state_changed */
    if (info->power_changed_cb) {
        info->power_changed_cb(info->power_changed_arg);
    }

    return 0;
}

/* FIXME: need to make power good generic */
static void dongle_power_good_callback(enum bq25896_event event, void *dev)
{
    struct dongle_dev_s *info = device_get_private(dev);

    info->current = 0;

    if (info->power_good_state == BOOST_FAULT)
        return;

    if (info->power_good_state == event)
        return;

    info->power_good_state = event;

    if ((info->power_good_state == POWER_GOOD) &&
        (info->attach_state == BASE_DETACHED)) {
        info->current = dongle_read_capabilities(info->gpio_capability,
                info->gpio_cfgset);
    }

    /* call ext_power_state_changed */
    if (info->power_changed_cb) {
        info->power_changed_cb(info->power_changed_arg);
    }
}

static int dongle_ext_power_register_callback(struct device *dev,
        device_ext_power_notification cb, void *arg)
{
    struct dongle_dev_s *info = device_get_private(dev);

    if (!info)
        return 0;

    if (!cb) {
        dbg("invalid parameter\n");
        return -EINVAL;
    }

    if (info->power_changed_cb) {
        dbg("already registered\n");
        return -EINVAL;
    }

    info->power_changed_cb = cb;
    info->power_changed_arg = arg;

    return 0;
}

static int dongle_ext_power_set_max_output_voltage(struct device *dev,
        int voltage)
{
    return 0; /* ignore voltage changes */
}

static int dongle_ext_power_get_output(struct device *dev,
        device_ext_power_output_s *output)
{
    struct dongle_dev_s *info = device_get_private(dev);

    /* BASE_ATTACHED   no_power_good
     * BASE_ATTACHED   power_good
     * !BASE_ATTACHED  no_power_good
     * BASE_ATTACHED   power_good
     */
    if ((info->attach_state == BASE_DETACHED) &&
        (info->power_good_state == POWER_GOOD) &&
        (info->current)) {
        dbg("Dongle Found %d mA\n", info->current);
        output->current = info->current;
        output->voltage = 5000; /* mV */
    } else {
        output->current = 0;  /* mA */
        output->voltage = 0; /* mV */
    }
    vdbg("DONGLE: current = %d voltage = %d\n",
            output->current, output->voltage);
    return 0;
}

static int dongle_ext_power_open(struct device *dev)
{
    struct dongle_dev_s *info = device_get_private(dev);

    info->open = 1;
    return 0;
}

static void dongle_ext_power_close(struct device *dev)
{
    struct dongle_dev_s *info = device_get_private(dev);

    info->open = 0;
}

static void dongle_ext_power_remove(struct device *dev)
{
    struct dongle_dev_s *info = device_get_private(dev);

    info->dev = NULL;
    device_set_private(dev, NULL);
}

static int dongle_ext_power_probe(struct device *dev)
{
    static struct dongle_dev_s dongle_info;

    dongle_info.attach_state = BASE_INVALID;
    dongle_info.power_good_state = NO_POWER_GOOD;

    struct device_resource *r;

    r = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_GPIO, "capability");
    ASSERT(r);
    dongle_info.gpio_capability = r->start;

    r = device_resource_get_by_name(dev,
        DEVICE_RESOURCE_TYPE_REGS, "gpio-cfgset");
    ASSERT(r);
    dongle_info.gpio_cfgset = (gpio_cfg_t)r->start;

    dongle_info.dev = dev;
    device_set_private(dev, &dongle_info);

    /* register to be notified of attach */
    if (mods_attach_register(dongle_attach_callback, &dongle_info) != OK) {
        dongle_ext_power_remove(dev);
        return -ENODEV;
    }

    /* register to be notified of power good */
#ifdef CONFIG_CHARGER_BQ25896
    if (bq25896_register_callback(dongle_power_good_callback, dev) != 0) {
        dongle_ext_power_remove(dev);
        return -ENODEV;
    } 
#endif

    return 0;
}

static struct device_ext_power_type_ops dongle_ext_power_type_ops  = {
    .register_callback = dongle_ext_power_register_callback,
    .set_max_output_voltage = dongle_ext_power_set_max_output_voltage,
    .get_output = dongle_ext_power_get_output,
};

static struct device_driver_ops dongle_ext_power_driver_ops = {
    .probe = dongle_ext_power_probe,
    .remove = dongle_ext_power_remove,
    .open = dongle_ext_power_open,
    .close = dongle_ext_power_close,
    .type_ops = &dongle_ext_power_type_ops,
};

/* device_ext_power */
struct device_driver dongle_ext_power_driver = {
    .type = DEVICE_TYPE_EXT_POWER_HW,
    .name = "dongle_ext_power",
    .desc = "Charging Dongle",
    .ops = &dongle_ext_power_driver_ops,
};
