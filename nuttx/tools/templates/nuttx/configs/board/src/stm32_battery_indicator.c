/**
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

#include <debug.h>
#include <errno.h>
#include <string.h>
#include <arch/board/board.h>
#include <arch/board/mods.h>

#include <nuttx/clock.h>
#include <nuttx/device.h>
#include <nuttx/device_battery.h>
#include <nuttx/device_ext_power.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/mods.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/battery.h>
#include <nuttx/power/battery_state.h>
#include <nuttx/power/ext_power.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>

#include "{{ board }}.h"

#define GPIO_MODS_LED_DRV_R       GPIO_MODS_LED_DRV_1
#define GPIO_MODS_LED_DRV_G       GPIO_MODS_LED_DRV_2

#define BUTTON_ACTIVITY           10

/* count in the delay of system schedule
   the actual on/off interval is about 125ms */
#define BLINK_OFF_INTERVAL        MSEC2TICK(100)
#define BLINK_ON_INTERVAL         MSEC2TICK(100)
#define BLINK_COUNT               12

#define RESET_OFF_INTERVAL        MSEC2TICK(500)
#define RESET_ON_INTERVAL         MSEC2TICK(500)
#define RESET_COUNT               3

#define PULSE_OFF_INTERVAL        MSEC2TICK(2000)
#define PULSE_ON_INTERVAL         MSEC2TICK(1000)

#define BATTERY_CAPACITY_CRITICAL 5
#ifdef CONFIG_BATTERY_LEVEL_LOW
#define BATTERY_CAPACITY_LOW      CONFIG_BATTERY_LEVEL_LOW
#else
#define BATTERY_CAPACITY_LOW      15
#endif
#define BATTERY_CAPACITY_GOOD     50
#define BATTERY_CAPACITY_FULL     100
#define LEDS_TIMEOUT              MSEC2TICK(3000)

#define LED_ON                    0
#define LED_OFF                   1

enum ext_power_state_e {
    EXT_POWER_NOT_PRESENT            = 0x00,
    EXT_POWER_PRESENT                = 0x01,
};

/**
 * @brief private battery indicator device information
 */
struct battery_indicator_info {
    struct work_s button_press_work;
    struct work_s led_timeout_work;
    struct work_s led_blink_work;
    struct work_s led_pulse_work;
    enum base_attached_e attach_state;
    enum ext_power_state_e ext_power_state;
    int led_gpio;
    int count;
    int reset;
};

/* to guarantee there's not race condition, all workers should be put into the same queue
     no semaphore required
*/

#define WORKER_QUEU_ID LPWORK

int stm32_battery_get_capacity(b16_t *capacity);
enum batt_level_e stm32_battery_get_level(void);

static struct battery_indicator_info g_indicator_info;
static void do_start_stop_pulse(struct battery_indicator_info *info,
        enum ext_power_state_e power_state);

#ifdef CONFIG_PM
static int pm_prepare(struct pm_callback_s *cb, enum pm_state_e state)
{
    /*
     * Do not allow STANDBY when handling the button press or when LEDs are ON,
     * else the work queue will not run until the next wake.
     */
    if (state >= PM_STANDBY)
    {
        /* if I am being charged, do not go to sleep */
        if (EXT_POWER_PRESENT == g_indicator_info.ext_power_state)
            return -EBUSY;
        if (!work_available(&g_indicator_info.button_press_work)
                || !work_available(&g_indicator_info.led_timeout_work)
                || !work_available(&g_indicator_info.led_blink_work)
                )
            return -EBUSY;
    }

    return OK;
}

static struct pm_callback_s pm_callback =
{
    .prepare = pm_prepare,
};
#endif

static void reset_led(void)
{
    lldbg("Turn off LEDs\n");

    gpio_set_value(GPIO_MODS_LED_DRV_G, LED_OFF);
    gpio_set_value(GPIO_MODS_LED_DRV_R, LED_OFF);
}

static void battery_led_timeout_worker(FAR void *arg)
{
    struct battery_indicator_info *info = arg;
    do_start_stop_pulse(arg, info->ext_power_state);
}

static void battery_led_blink_worker(FAR void *arg)
{
    struct battery_indicator_info *info = arg;
    uint8_t led = info->led_gpio;
    /* on first blink, count == 1, blink off */
    uint8_t value = (info->count % 2) ? LED_OFF : LED_ON;
    int blink_count = info->reset ? RESET_COUNT : BLINK_COUNT;
    int off_interval = info->reset ? RESET_OFF_INTERVAL: BLINK_OFF_INTERVAL;
    int on_interval = info->reset ? RESET_ON_INTERVAL: BLINK_ON_INTERVAL;

    gpio_set_value(led, value);

    ++(info->count);
    if ((info->count / 2) >= blink_count)
      {
        /* led is off now, no more blink */
        do_start_stop_pulse(arg, info->ext_power_state);
        return;
      }
    work_queue(WORKER_QUEU_ID, &info->led_blink_work,
               battery_led_blink_worker, arg,
               (value == LED_OFF) ? off_interval : on_interval);
}

static void battery_led_pulse_worker(FAR void *arg)
{
    struct battery_indicator_info *info = arg;
    uint8_t on_off;
    uint8_t led;
    enum batt_level_e new_level;
    static enum batt_level_e level = BATTERY_LEVEL_EMPTY;
    if (EXT_POWER_PRESENT != info->ext_power_state) {
        reset_led();
        return;
    }

    new_level = stm32_battery_get_level();
    /* make sure we are initialized on the first pulse */
    if (!info->count || level != new_level) {
        level = new_level;
        info->count = 0;
        if (level == BATTERY_LEVEL_FULL) {
            /* stop pulse, show solid green */
            lldbg("battery full\n");
            gpio_set_value(GPIO_MODS_LED_DRV_G, LED_ON);
            gpio_set_value(GPIO_MODS_LED_DRV_R, LED_OFF);
            return;
        } else {
            info->led_gpio = GPIO_MODS_LED_DRV_G;
            gpio_set_value(GPIO_MODS_LED_DRV_R, LED_OFF);
        }
    }
    /* on the first pulse, count == 0, pulse on */
    on_off = (info->count % 2) ? LED_OFF : LED_ON;
    led = info->led_gpio;

    gpio_set_value(led, on_off);

    ++(info->count);
    work_queue(WORKER_QUEU_ID, &info->led_pulse_work,
               battery_led_pulse_worker, arg,
               (on_off == LED_OFF) ? PULSE_OFF_INTERVAL : PULSE_ON_INTERVAL);
}

/* running in LPWORK */
static void do_start_stop_pulse(struct battery_indicator_info *info,
        enum ext_power_state_e power_state)
{
    if (EXT_POWER_PRESENT == power_state
            && BASE_DETACHED == info->attach_state) {
        if (!work_available(&info->button_press_work)
                || !work_available(&info->led_blink_work)
                || !work_available(&info->led_timeout_work)
                || !work_available(&info->led_pulse_work)
                )
        {
            /* nothing to do if I'm blinking, pulsing or solid on */
            return;
        }
        /* start to pulse */
        info->count = 0;
        work_queue(WORKER_QUEU_ID, &info->led_pulse_work,
                battery_led_pulse_worker, info, 0);
    } else {
        /* stop pulsing */
        if (!work_available(&info->led_pulse_work))
            work_cancel(WORKER_QUEU_ID, &info->led_pulse_work);
        reset_led();
    }
}

/* running in LPWORK */
static void do_button_pressed(FAR void *arg)
{
    struct battery_indicator_info *info = arg;
    b16_t capacity;
    enum batt_level_e level;
    int ret;

    if (!work_available(&info->led_blink_work)
            || !work_available(&info->led_timeout_work))
    {
        /* let blink finish or solid-on timeout */
        return;
    }

    if (!work_available(&info->led_pulse_work))
        work_cancel(WORKER_QUEU_ID, &info->led_pulse_work);

    ret = stm32_battery_get_capacity(&capacity);
    if (ret)
      {
        dbg("Failed to get capacity (ret=%d)\n", ret);
        return;
      }
    level = stm32_battery_get_level();
    lldbg("capacity=%u level=%u\n", capacity, level);

    /* Turn on appropriate number of LEDs based on capacity */
    if (level == BATTERY_LEVEL_FULL) {
        gpio_set_value(GPIO_MODS_LED_DRV_R, LED_OFF);
        gpio_set_value(GPIO_MODS_LED_DRV_G, LED_ON);
        info->count = 1;
        info->led_gpio = GPIO_MODS_LED_DRV_G;
        info->reset = 0;
        work_queue(WORKER_QUEU_ID, &info->led_blink_work,
                   battery_led_blink_worker, arg, BLINK_ON_INTERVAL);
        return;
    } else if (capacity <= BATTERY_CAPACITY_CRITICAL) {
        gpio_set_value(GPIO_MODS_LED_DRV_R, LED_ON);
        gpio_set_value(GPIO_MODS_LED_DRV_G, LED_OFF);
        info->count = 1;
        info->led_gpio = GPIO_MODS_LED_DRV_R;
        info->reset = 0;
        work_queue(WORKER_QUEU_ID, &info->led_blink_work,
                   battery_led_blink_worker, arg, BLINK_ON_INTERVAL);
        return;
    }
    /* At least one LED is ON, so schedule timeout */
    work_queue(WORKER_QUEU_ID, &info->led_timeout_work,
               battery_led_timeout_worker, arg, LEDS_TIMEOUT);
    if (capacity <= BATTERY_CAPACITY_LOW) {
        gpio_set_value(GPIO_MODS_LED_DRV_R, LED_ON);
        gpio_set_value(GPIO_MODS_LED_DRV_G, LED_OFF);
    } else if (capacity <= BATTERY_CAPACITY_GOOD) {
        /* Solid orange */
        gpio_set_value(GPIO_MODS_LED_DRV_R, LED_ON);
        gpio_set_value(GPIO_MODS_LED_DRV_G, LED_ON);
    } else {
        gpio_set_value(GPIO_MODS_LED_DRV_R, LED_OFF);
        gpio_set_value(GPIO_MODS_LED_DRV_G, LED_ON);
    }
}

/* button pressed callback is running in LPWORK */
static void button_pressed_cb(FAR void *arg)
{
    do_button_pressed(arg);
}

/* attach state callback is running in LPWORK */
static int attach_state_changed_cb(FAR void *arg, const void *data)
{
    struct battery_indicator_info *info = arg;
    enum base_attached_e new_state = *((enum base_attached_e *)data);

    if (info->attach_state == new_state)
        return OK;

    if (BASE_DETACHED == new_state && BASE_INVALID == info->attach_state) {
        /* we are still detached */
        info->attach_state = new_state;
        return OK;
    }

    info->attach_state = new_state;
    if (BASE_ATTACHED == new_state || BASE_ATTACHED_OFF == new_state) {
        /* cancel all led worker */
        if (!work_available(&info->button_press_work))
            work_cancel(WORKER_QUEU_ID, &info->button_press_work);
        if (!work_available(&info->led_blink_work))
            work_cancel(WORKER_QUEU_ID, &info->led_blink_work);
        if (!work_available(&info->led_timeout_work))
            work_cancel(WORKER_QUEU_ID, &info->led_timeout_work);
        if (!work_available(&info->led_pulse_work))
            work_cancel(WORKER_QUEU_ID, &info->led_pulse_work);
        reset_led();
    } else {
        do_button_pressed(info);
    }

    return OK;
}

#ifdef CONFIG_EXT_POWER
/* external power state change callback is running in LPWORK */
static void ext_power_state_changed_cb(FAR void *arg,
        struct device *const dev[])
{
    struct battery_indicator_info *info = arg;
    int current;
    enum ext_power_state_e new_power_state;

    device_ext_power_get_current(dev[EXT_POWER_WIRED], &current);
    new_power_state = current > 0 ?
            EXT_POWER_PRESENT : EXT_POWER_NOT_PRESENT;

    if (info->ext_power_state == new_power_state)
        return;

    if (EXT_POWER_PRESENT != new_power_state
            && BASE_DETACHED == info->attach_state) {
        /* when disconnected from charger, treat like detach */
        do_button_pressed(info);
        /* delay update power state upon charger unplugg event to keep us awake
            in case system goes to sleep after external power lost but before lpwork is scheduled
        */
        info->ext_power_state = new_power_state;
    } else {
        info->ext_power_state = new_power_state;
        do_start_stop_pulse(info, new_power_state);
    }
}
#endif

static int battery_button_isr(int irq, FAR void *context)
{
    uint8_t buttons;
    int ret = OK;

    /* Report activity to power management (to keep us out of idle) */
    pm_activity(BUTTON_ACTIVITY);
    buttons = board_buttons();
    if (buttons & BUTTON_POWER_BIT) {
        if (!work_available(&g_indicator_info.button_press_work)) {
            /* Previous press still being handled. Ignore this press. */
            return OK;
        }

        return work_queue(WORKER_QUEU_ID, &g_indicator_info.button_press_work,
                button_pressed_cb, &g_indicator_info, 0);
    }

    return ret;
}

int battery_indicator_probe(void)
{
    int hw_reset;
    memset(&g_indicator_info, 0, sizeof(g_indicator_info));
    g_indicator_info.attach_state = BASE_INVALID;
    g_indicator_info.ext_power_state = EXT_POWER_NOT_PRESENT;
    /* Initialize LED GPIOs to outputs */
    gpio_direction_in(GPIO_MODS_KEY_POWER_PMIC);
    hw_reset = gpio_get_value(GPIO_MODS_KEY_POWER_PMIC);
    if (hw_reset) {
        /* user triggered HW reset */
        gpio_direction_out(GPIO_MODS_LED_DRV_G, LED_ON);
        gpio_direction_out(GPIO_MODS_LED_DRV_R, LED_OFF);
        g_indicator_info.count = 1;
        g_indicator_info.led_gpio = GPIO_MODS_LED_DRV_G;
        g_indicator_info.reset = 1;
        work_queue(WORKER_QUEU_ID, &g_indicator_info.led_blink_work,
                battery_led_blink_worker, &g_indicator_info, RESET_ON_INTERVAL);
    } else {
        gpio_direction_out(GPIO_MODS_LED_DRV_G, LED_OFF);
        gpio_direction_out(GPIO_MODS_LED_DRV_R, LED_OFF);
    }

    board_button_irq(BUTTON_POWER, battery_button_isr);

#ifdef CONFIG_PM
    if (pm_register(&pm_callback) != OK)
        dbg("Failed register to power management!\n");
#endif

    return 0;
}

int battery_indicator_open(void)
{
    int retval;

    retval = mods_attach_register(attach_state_changed_cb, &g_indicator_info);
    if (retval) {
        dbg("failed to register mods_attach callback\n");
        return retval;
    }

#ifdef CONFIG_EXT_POWER
    retval = ext_power_register_callback(ext_power_state_changed_cb, &g_indicator_info);
    if (retval) {
        dbg("failed to register ext_power_notification callback\n");
        return retval;
    }
#endif

    return OK;
}

void battery_indicator_remove(void)
{
    board_button_irq(BUTTON_POWER, NULL);

    gpio_deactivate(GPIO_MODS_LED_DRV_G);
    gpio_deactivate(GPIO_MODS_LED_DRV_R);
}
