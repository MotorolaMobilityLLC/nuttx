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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/power/pm.h>
#include <nuttx/usb/hid.h>

#define HID_BUTTONS_HID_VERSION  0x0111
#define HID_BUTTONS_PID          0xffff
#define HID_BUTTONS_VID          0x22b8

static const char gReportDescriptor[26] = {
    0x05, 0x0C,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Keypad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x0C,                    //     USAGE_PAGE (Consumer)
    0x19, 0x65,                    //     USAGE_MINIMUM
    0x29, 0x6C,                    //     USAGE_MAXIMUM
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

static struct device *gDevice;
static hid_event_callback gCallback;

int s10p_report_button(uint8_t buttons)
{
    if (gCallback)
      {
        lldbg("buttons=0x%02x\n", buttons);
        gCallback(gDevice, HID_INPUT_REPORT, &buttons, sizeof(buttons));
      }
    return OK;
}

int s10p_hid_buttons_power_on(struct device *dev)
{
    return 0;
}

int s10p_hid_buttons_power_off(struct device *dev)
{
    return 0;
}

int s10p_hid_buttons_get_descriptor(struct device *dev, struct hid_descriptor *desc)
{
    desc->length = sizeof(struct hid_descriptor);
    desc->report_desc_length = sizeof(gReportDescriptor);
    desc->hid_version = HID_BUTTONS_HID_VERSION;
    desc->product_id = HID_BUTTONS_PID;
    desc->vendor_id = HID_BUTTONS_VID;
    desc->country_code = USBHID_COUNTRY_NONE;

    return 0;
}

int s10p_hid_buttons_get_report_descriptor(struct device *dev, uint8_t *desc)
{
    memcpy(desc, gReportDescriptor, sizeof(gReportDescriptor));
    return 0;
}

int s10p_hid_buttons_get_report_length(struct device *dev, uint8_t report_type,
                                  uint8_t report_id)
{
    if (report_type != HID_INPUT_REPORT)
        return -EINVAL;

    /* As defined by the report descriptor above, the report size is one byte */
    return 1;
}

int s10p_hid_buttons_get_maximum_report_length(struct device *dev,
                                          uint8_t report_type)
{
    if (report_type != HID_INPUT_REPORT)
        return -EINVAL;

    /* As defined by the report descriptor above, the report size is one byte */
    return 1;
}

int s10p_hid_buttons_get_report(struct device *dev, uint8_t report_type,
                           uint8_t report_id, uint8_t *data, uint16_t len)
{
    if (report_type != HID_INPUT_REPORT)
        return -EINVAL;

    /* Get the current state of the buttons */
    *data = 0;
    /* As defined by the report descriptor above, the report size is one byte */
    len = 1;

    return 0;
}

int s10p_hid_buttons_register_callback(struct device *dev,
                                  hid_event_callback callback)
{
    dbg("callback=0x%p\n", callback);

    gCallback = callback;
    return 0;
}

int s10p_hid_buttons_unregister_callback(struct device *dev)
{
    gCallback = NULL;
    return 0;
}

int s10p_hid_buttons_probe(struct device *dev)
{
    gDevice = dev;

    return 0;
}

static struct device_hid_type_ops s10p_hid_buttons_type_ops = {
    .power_on = s10p_hid_buttons_power_on,
    .power_off = s10p_hid_buttons_power_off,
    .get_descriptor = s10p_hid_buttons_get_descriptor,
    .get_report_descriptor = s10p_hid_buttons_get_report_descriptor,
    .get_report_length = s10p_hid_buttons_get_report_length,
    .get_maximum_report_length = s10p_hid_buttons_get_maximum_report_length,
    .get_report = s10p_hid_buttons_get_report,
    .register_callback = s10p_hid_buttons_register_callback,
    .unregister_callback = s10p_hid_buttons_unregister_callback,
};

static struct device_driver_ops s10p_hid_buttons_driver_ops = {
    .probe = s10p_hid_buttons_probe,
    .type_ops = &s10p_hid_buttons_type_ops,
};

struct device_driver s10p_hid_buttons_driver = {
    .type = DEVICE_TYPE_HID_HW,
    .name = "s10p_hid_buttons",
    .desc = "Redcarpet HID buttons",
    .ops = &s10p_hid_buttons_driver_ops,
};
