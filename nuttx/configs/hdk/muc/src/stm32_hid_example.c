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

/* This example demonstrates a HID game controller.
 *
 * It supports 16 buttons (see HID_GAME_GPIOS for the pins to connect).
 * It supports 4 analog axis (see HID_GAME_ADC_PINS for the pins to connect).
 *
 * It enumerates as ffff:ffff which should most likely match the key layout file
 * in /system/usr/keylayout/Generic.kl (see HID_BUTTONS_VID and HID_BUTTONS_PID)
 * to change the VID:PID.
 *
 * Set DIP switches to:
 *   A: off, off on, off  (Group B disabled, MyDP Device)
 *   B: off, off, off, on (no clock, no I2S, MDK battery, debug USB powered)
 *
 * TODO:
 *   Add dead-zone.
 *   Auto-calibration (zeroing of analog sticks).
 */

#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>

#include <arch/board/mods.h>

#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/gpio.h>
#include <nuttx/usb/hid.h>

#include <nuttx/greybus/mods.h>

#include "stm32_adc.h"
#include "stm32_tim.h"

#define HID_GAME_PM_ACTIVITY    10

#define HID_BUTTONS_HID_VERSION  0x0111
#define HID_BUTTONS_VID          0xffff
#define HID_BUTTONS_PID          0xffff

static const char HID_GAME_REPORT_DESCRIPTOR[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x05, // USAGE (Game Pad)
    0xa1, 0x01, // COLLECTION (Application)
    0xa1, 0x00, // COLLECTION (Physical)

    0x05, 0x09, // USAGE_PAGE (Button)
    0x19, 0x01, // USAGE_MINIMUM (Button 1)
    0x29, 0x10, // USAGE_MAXIMUM (Button 16)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x75, 0x01, // REPORT_SIZE (1)
    0x95, 0x10, // REPORT_COUNT (16)
    0x81, 0x02, // INPUT (Data,Var,Abs)

    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x30, // USAGE (X)  [left stick's X-axis]
    0x09, 0x31, // USAGE (Y)  [left stick's Y-axis]
    0x09, 0x32, // USAGE (Z)  [right stick's X-axis]
    0x09, 0x33, // USAGE (Rx) [right stick's Y-axis]
    0x15, 0x00, // LOGICAL_MINIMUM (-127)
    0x25, 0xff, // LOGICAL_MAXIMUM (127)
    0x75, 0x08, // REPORT_SIZE (8)
    0x95, 0x04, // REPORT_COUNT (4)
    0x81, 0x02, // INPUT (Data,Var,Abs)

    0xc0, // END_COLLECTION
    0xc0 // END_COLLECTION
};

/* The order and size of these fields must match the descriptor above.
   The size of the axis must match the REPORT_SIZE * REPORT_COUNT of the
   first INPUT section above.
   The size of the buttons must match the REPORT_SIZE * REPORT_COUNT of the
   second INPUT section above.
 */
struct hid_game_report {
    uint16_t buttons;
    uint8_t axis[4];
} __attribute__((packed));

static struct hid_game {
    struct device *self;
    hid_event_callback callback;
    struct hid_game_report state;
    pthread_t thread;
    bool running;
    uint32_t sample_num;
    int adc_fd;
} g_hid_game;

/* The size of this structure must match size of hid_game_report.buttons. */
static const uint32_t HID_GAME_GPIOS[16] = {
    CALC_GPIO_NUM('A', 4),  // key 304   BUTTON_A
    CALC_GPIO_NUM('A', 5),  // key 305   BUTTON_B
    CALC_GPIO_NUM('A', 6),  // key 306   BUTTON_C
    CALC_GPIO_NUM('A', 7),  // key 307   BUTTON_X

    CALC_GPIO_NUM('H', 0),  // key 308   BUTTON_Y
    CALC_GPIO_NUM('A', 10), // key 309   BUTTON_Z
    CALC_GPIO_NUM('A', 15), // key 310   BUTTON_L1
    CALC_GPIO_NUM('B', 2),  // key 311   BUTTON_R1

    CALC_GPIO_NUM('C', 7),  // key 312   BUTTON_L2
    CALC_GPIO_NUM('C', 8),  // key 313   BUTTON_R2
    CALC_GPIO_NUM('C', 9),  // key 314   BUTTON_SELECT
    CALC_GPIO_NUM('C', 12), // key 315   BUTTON_START

    CALC_GPIO_NUM('D', 6),  // key 316   BUTTON_MODE
    CALC_GPIO_NUM('G', 9),  // key 317   BUTTON_THUMBL
    CALC_GPIO_NUM('G', 12), // key 318   BUTTON_THUMBR
    0,
};

/* Configurations of pins used by each ADC channel */
static const uint32_t HID_GAME_ADC_PINS[]  = {
    GPIO_ADC1_IN5, /* PA0 */    // axis 0x00 X
    GPIO_ADC1_IN6, /* PA1 */    // axis 0x01 Y
    GPIO_ADC1_IN7, /* PA2 */    // axis 0x02 Z
    GPIO_ADC1_IN8, /* PA3 */    // axis 0x03 RX
};

static const uint8_t HID_GAME_ADC_CHANNELS[] = { 5, 6, 7, 8 };

#define HID_GAME_ADC_DEVPATH "/dev/adc0"

static int hid_game_adc_init(struct hid_game *hid_game)
{
    struct adc_dev_s *adc;
    int ret;
    size_t i;

    vdbg("hid_game=%p\n", hid_game);

    /* Configure the pins as analog inputs for the selected channels */
    for (i = 0; i < ARRAY_SIZE(HID_GAME_ADC_PINS); i++) {
        stm32_configgpio(HID_GAME_ADC_PINS[i]);
    }

    /* Call stm32_adcinitialize() to get an instance of the ADC interface */
    adc = stm32_adcinitialize(1 /* ADC1 */, HID_GAME_ADC_CHANNELS, ARRAY_SIZE(HID_GAME_ADC_CHANNELS));
    if (!adc) {
        vdbg("ERROR: Failed to get ADC interface\n");
        return -ENODEV;
    }

    /* Register the ADC driver */
    ret = adc_register(HID_GAME_ADC_DEVPATH, adc);
    if (ret < 0)
    {
        vdbg("ERROR: adc_register %s failed: %d\n", HID_GAME_ADC_DEVPATH, ret);
        return ret;
    }

    return OK;
}

static int hid_game_adc_read(struct hid_game *hid_game, struct adc_msg_s *adc_msg, size_t num_msgs)
{
    int ret;
    size_t i;

    vdbg("hid_game=%p, msg=%p\n", hid_game, adc_msg);

    hid_game->adc_fd = open(HID_GAME_ADC_DEVPATH, O_RDONLY|O_NONBLOCK);
    if (hid_game->adc_fd < 0) {
        vdbg("ERROR: open failed: %d\n", errno);
        ret = errno;
    }

    /* Start the ADC conversion. */
    ret = ioctl(hid_game->adc_fd, ANIOC_TRIGGER, 0);
    if (ret < 0) {
        vdbg("ERROR: ioctl failed: %d\n", errno);
    }

    /* HACK: Give the ADC some time to receive all the values.
     * If we call read() too soon, it will mask the ADC interrupts and we will
     * lose interrupts. */
    up_udelay(50);

    /* The ADC device only allows a single adc_msg read at a time. */
    i = 0;
    while (i < num_msgs) {
        ret = read(hid_game->adc_fd, &adc_msg[i], sizeof(*adc_msg));
        vdbg("read: ret=%d, errno=%d\n", ret, errno);
        if (ret != sizeof(*adc_msg)) {
            if (errno == EAGAIN) {
                continue;
            }

            vdbg("ERROR: read failed: ret=%d, errno=%d\n", ret, errno);
            ret = errno;
            break;
        } else {
            ret = OK;
            vdbg("chan=%d, data=%d\n", adc_msg[i].am_channel, adc_msg[i].am_data);
            i++;
        }
    }

    if (hid_game->adc_fd >= 0) {
        close(hid_game->adc_fd);
        hid_game->adc_fd = -1;
    }

    return ret;
}

static int hid_game_poll(struct hid_game *hid_game)
{
    int ret;
    size_t i;
    size_t j;
    uint16_t new_buttons = 0;
    uint16_t bit = 1;
    struct adc_msg_s adc_msgs[ARRAY_SIZE(HID_GAME_ADC_CHANNELS)];
    uint8_t scaled;
    bool changed = false;

    vdbg("hid_game=%p\n", hid_game);

    memset(adc_msgs, 0, sizeof(adc_msgs));

    /* Read the GPIO for each button and build a bitmask of the values. */
    for (i = 0; i < ARRAY_SIZE(HID_GAME_GPIOS); i++) {
        uint32_t gpio = HID_GAME_GPIOS[i];
        uint8_t val;
        if (gpio) {
            val = gpio_get_value(gpio);
            vdbg("i=%d, gpio=%d, val=%d\n", i, gpio, val);
            if (val) {
                new_buttons |= bit;
            }
        }

        bit = bit << 1;
    }

    /* If any buttons changed, update the state and report and event. */
    if (hid_game->state.buttons != new_buttons) {
        vdbg("buttons changed: new=%04x, old=%04x\n", new_buttons, hid_game->state.buttons);
        hid_game->state.buttons = new_buttons;
        changed = true;
    }

    /* Read a single ADC every fourth loop. */
    if (hid_game->sample_num % 4 == 0) {
        ret = hid_game_adc_read(hid_game, adc_msgs, ARRAY_SIZE(adc_msgs));
        if (ret == OK) {
            for (i = 0; i < ARRAY_SIZE(adc_msgs); i++) {
                struct adc_msg_s *adc_msg = &adc_msgs[i];

                for (j = 0; j < ARRAY_SIZE(HID_GAME_ADC_CHANNELS); j++) {
                    /* Find the axis that matches this channel. */
                    if (adc_msg->am_channel == HID_GAME_ADC_CHANNELS[j]) {
                        /* Scale from 12-bit to 8-bit. */
                        scaled = (adc_msg->am_data >> 4);
                        vdbg("i=%d, axis=%d, chan=%d, data=%d, scaled=%d\n", i, j, adc_msg->am_channel, adc_msg->am_data, scaled);

                        if (hid_game->state.axis[j] != scaled) {
                            vdbg("axis-%d changed: new=%d, old=%d\n", j, scaled, hid_game->state.axis[j]);
                            hid_game->state.axis[j] = scaled;
                            changed = true;
                        }

                        break;
                    }
                }
            }
        }
    }

    if (changed && hid_game->callback) {
        hid_game->callback(hid_game->self, HID_INPUT_REPORT,
                          (uint8_t *)&hid_game->state, sizeof(hid_game->state));
    }

    hid_game->sample_num++;

    return 0;
}

static void *hid_game_thread(void *data)
{
    vdbg("data=%p\n", data);

    int ret = OK;

    struct hid_game *hid_game = (struct hid_game *)data;
    vdbg("hid_game=%p\n", hid_game);

    hid_game->running = true;

    while (hid_game->running) {
        ret = hid_game_poll(hid_game);
        if (ret != OK) {
            break;
        }

        /* 10 ms */
        usleep(1000*10);
    }

    vdbg("done: ret=%d\n", ret);
    return (void *)ret;
}

static int hid_game_thread_start(struct hid_game *hid_game)
{
    vdbg("hid_game=%p\n", hid_game);

    if (hid_game->thread) {
        vdbg("ERROR: thread already started\n");
        return -EINVAL;
    }

    int ret = pthread_create(&hid_game->thread, NULL /* attr */,
                             hid_game_thread, hid_game);
    vdbg("create: thread=%p, addr=%p, ret=%d\n", hid_game->thread, hid_game_thread, ret);
    if (ret) {
        vdbg("ERROR: Failed to create thread: %d\n", errno);
        hid_game->thread = 0;
        return -errno;
    }

    return OK;
}

static int hid_game_thread_stop(struct hid_game *hid_game)
{
    pthread_addr_t join_value;

    vdbg("hid_game=%p\n", hid_game);

    hid_game->running = false;
    if (hid_game->thread) {
        pthread_join(hid_game->thread, &join_value);
        vdbg("join=%d\n", join_value);

        if (join_value != OK) {
            vdbg("ERROR: join failed, killing\n");
            pthread_kill(hid_game->thread, 9);
        }

        hid_game->thread = 0;
    }

    return OK;
}

static int hid_game_power_on(struct device *dev)
{
    struct hid_game *hid_game;

    vdbg("dev=%p\n", dev);

    hid_game = device_get_private(dev);
    if (!hid_game) {
        return -ENOENT;
    }

    /* Enable the LED to indicate the HID device is active. */
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 1);
    gpio_set_value(GPIO_MODS_LED_DRV_3, 0);

    memset(&hid_game->state, 0, sizeof(hid_game->state));

    hid_game_thread_start(hid_game);

    return 0;
}

static int hid_game_power_off(struct device *dev)
{
    struct hid_game *hid_game;

    vdbg("dev=%p\n", dev);

    hid_game = device_get_private(dev);
    if (!hid_game) {
        return -ENOENT;
    }

    hid_game_thread_stop(hid_game);

    /* Disable the LED to indicate the HID device is inactive. */
    gpio_set_value(GPIO_MODS_LED_DRV_3, 1);
    gpio_set_value(GPIO_MODS_DEMO_ENABLE, 0);

   return 0;
}

static int hid_game_get_descriptor(struct device *dev, struct hid_descriptor *desc)
{
    vdbg("dev=%p, desc=%p\n", dev, desc);

    desc->length = sizeof(struct hid_descriptor);
    desc->report_desc_length = sizeof(HID_GAME_REPORT_DESCRIPTOR);
    desc->hid_version = HID_BUTTONS_HID_VERSION;
    desc->product_id = HID_BUTTONS_PID;
    desc->vendor_id = HID_BUTTONS_VID;
    desc->country_code = USBHID_COUNTRY_NONE;

    return 0;
}

static int hid_game_get_report_descriptor(struct device *dev, uint8_t *desc)
{
    vdbg("dev=%p, desc=%p\n", dev, desc);

    memcpy(desc, HID_GAME_REPORT_DESCRIPTOR, sizeof(HID_GAME_REPORT_DESCRIPTOR));
    return 0;
}

static int hid_game_get_report_length(struct device *dev, uint8_t report_type,
                                  uint8_t report_id)
{
    vdbg("dev=%p, report_type=%d, report_id=%d\n", dev, report_type, report_id);

    if (report_type != HID_INPUT_REPORT) {
        return -EINVAL;
    }

    return sizeof(struct hid_game_report);
}

static int hid_game_get_maximum_report_length(struct device *dev, uint8_t report_type)
{
    vdbg("dev=%p, report_type=%d\n", dev, report_type);

    if (report_type != HID_INPUT_REPORT) {
        return -EINVAL;
    }

    return sizeof(struct hid_game_report);
}

static int hid_game_get_report(struct device *dev,
                        uint8_t report_type, uint8_t report_id,
                        uint8_t *data, uint16_t len)
{
    struct hid_game *hid_game;

    vdbg("dev=%p, report_type=%d, report_id=%d, data=%p, len=%d\n",
        dev, report_type, report_id, data, len);

    if (report_type != HID_INPUT_REPORT) {
        return -EINVAL;
    }

    hid_game = device_get_private(dev);
    if (!hid_game) {
        return -ENOENT;
    }

    data = (uint8_t *)&hid_game->state;
    len = sizeof(hid_game->state);

    return 0;
}

static int hid_game_register_callback(struct device *dev, hid_event_callback callback)
{
    struct hid_game *hid_game;

    vdbg("dev=%p, callback=0x%p\n", dev, callback);

    hid_game = device_get_private(dev);
    if (!hid_game) {
        return -ENOENT;
    }

    hid_game->callback = callback;

    return 0;
}

static int hid_game_unregister_callback(struct device *dev)
{
    struct hid_game *hid_game;

    vdbg("dev=%p\n", dev);

    hid_game = device_get_private(dev);
    if (!hid_game) {
        return -ENOENT;
    }

    hid_game->callback = NULL;

    return 0;
}

static int hid_game_attach(void *arg, const void *data)
{
    if (!arg || !data) {
        return -EINVAL;
    }

    struct hid_game *hid_game = (struct hid_game *)arg;
    enum base_attached_e state = *((enum base_attached_e *)data);

    dbg("hid_game=%p, attach=%d\n", hid_game, state);

    switch (state) {
    case BASE_DETACHED:
    case BASE_ATTACHED_OFF:
        hid_game_thread_stop(hid_game);
        break;
    case BASE_ATTACHED:
    case BASE_INVALID:
        /* Ignore */
        break;
    }

    return 0;
}

static int hid_game_probe(struct device *dev)
{
    struct hid_game *hid_game;
    size_t i;

    vdbg("dev=%p\n", dev);

    hid_game = &g_hid_game;
    hid_game->self = dev;
    hid_game->callback = NULL;
    hid_game->running = false;
    hid_game->adc_fd = -1;
    memset(&hid_game->state, 0, sizeof(hid_game->state));
    device_set_private(dev, hid_game);

    for (i = 0; i < ARRAY_SIZE(HID_GAME_GPIOS); i++) {
        gpio_direction_in(HID_GAME_GPIOS[i]);
    }

    hid_game_adc_init(hid_game);

    /* Register for attach notifications. This will callback immediately. */
    if (mods_attach_register(hid_game_attach, hid_game)) {
        vdbg("ERROR: failed to register attach notifier\n");
    }

    gpio_direction_out(GPIO_MODS_LED_DRV_3, 1);
    gpio_direction_out(GPIO_MODS_DEMO_ENABLE, 0);

    return 0;
}

static struct device_hid_type_ops hid_game_type_ops = {
    .power_on = hid_game_power_on,
    .power_off = hid_game_power_off,
    .get_descriptor = hid_game_get_descriptor,
    .get_report_descriptor = hid_game_get_report_descriptor,
    .get_report_length = hid_game_get_report_length,
    .get_maximum_report_length = hid_game_get_maximum_report_length,
    .get_report = hid_game_get_report,
    .register_callback = hid_game_register_callback,
    .unregister_callback = hid_game_unregister_callback,
};

static struct device_driver_ops hid_game_driver_ops = {
    .probe = hid_game_probe,
    .type_ops = &hid_game_type_ops,
};

const struct device_driver hid_game_driver = {
    .type = DEVICE_TYPE_HID_HW,
    .name = "hid_game",
    .desc = "HID Game Controller",
    .ops = &hid_game_driver_ops,
};

/* NSH debug functions. */
#if CONFIG_NSH_CONSOLE
int _hid_example_poll(void)
{
    return hid_game_poll(&g_hid_game);
}

int _hid_example_power_on(void)
{
    return hid_game_power_on(g_hid_game.self);
}

int _hid_example_power_off(void)
{
    return hid_game_power_off(g_hid_game.self);
}
#endif
