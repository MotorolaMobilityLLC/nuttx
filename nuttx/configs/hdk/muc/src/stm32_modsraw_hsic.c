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

#include <debug.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/device_usbtun.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/fusb302.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/usb.h>

enum { DIRECT_USB, APBE_TUNNEL, HSIC_TEST };

typedef struct {
    int conn_type;
    bool attached;
    raw_send_callback raw_callback;
    struct device *hsic;
    struct device *slave_pwr_ctrl;
    struct device *usb_ext;
    usb_ext_event_callback usb_ext_callback;
    struct device *usbtun;
} common_data_t;

static common_data_t s_data;

static void _usb_turn_on(void);
static void _usb_turn_off(void);

void usb3813_set_hsic_uplink(bool uplink);

#define HSIC_DEV_RELEASE_RESET_DELAY 500000

static int _recv(struct device *dev, uint32_t len, uint8_t data[])
{
    data[len - 1] = '\0';

    if (!strcmp((const char *)data, "on")) {
        lldbg("turning on. type = %d\n", s_data.conn_type);
        _usb_turn_on();
    } else if (!strcmp((const char *)data, "off")) {
        _usb_turn_off();
        lldbg("turned off. type = %d\n", s_data.conn_type);
    } else if (!strcmp((const char *)data, "direct")) {
        usb3813_set_hsic_uplink(false);
        s_data.conn_type = DIRECT_USB;
        lldbg("set type to %d\n", s_data.conn_type);
    } else if (!strcmp((const char *)data, "tunnel")) {
        usb3813_set_hsic_uplink(true);
        s_data.conn_type = APBE_TUNNEL;
        lldbg("set type to %d\n", s_data.conn_type);
    } else if (!strcmp((const char *)data, "hsictest")) {
        usb3813_set_hsic_uplink(false);
        s_data.conn_type = HSIC_TEST;
        lldbg("HSIC test mode\n");
    } else {
        lldbg("unknown command :%s, ignored\n", data);
    }

    return 0;
}

/**
 * register the send callback function.
 */
static int _register_callback(struct device *dev,
                              raw_send_callback callback)
{
    if (!dev)
        return -ENODEV;

    s_data.raw_callback = callback;

    return 0;
}

/**
 * unregister the send callback function.
 */
static int _unregister_callback(struct device *dev)
{
    if (!dev)
        return -ENODEV;

    s_data.raw_callback = NULL;

    return 0;
}

/**
 * probe function called upon device registration.
 */
static int _probe(struct device *dev)
{
    if (!dev)
        return -EINVAL;

    s_data.conn_type = DIRECT_USB;
    s_data.raw_callback = NULL;

    return 0;
}

/**
 * remove function called when device is unregistered.
 */
static void _remove(struct device *dev)
{
    if (!dev)
        return;

    s_data.raw_callback = NULL;
}

static struct device_raw_type_ops _type_ops = {
    .recv = _recv,
    .register_callback = _register_callback,
    .unregister_callback = _unregister_callback,
};

static struct device_driver_ops _driver_ops = {
    .probe = _probe,
    .remove = _remove,
    .type_ops = &_type_ops,
};

struct device_driver mods_raw_hsic_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "raw_hsic",
    .desc = "Raw HSIC Interface",
    .ops = &_driver_ops,
};

static uint8_t _usb_ext_get_attached(void)
{
    return s_data.attached;
}

static uint8_t _usb_ext_get_protocol(void)
{
    return GB_USB_EXT_PROTOCOL_2_0;
}

static uint8_t _usb_ext_get_path(void)
{
    if (s_data.conn_type == DIRECT_USB) {
        return GB_USB_EXT_PATH_A;
    } else {
        return GB_USB_EXT_PATH_B;
    }
}

static uint8_t _usb_ext_get_type(void)
{
    return GB_USB_EXT_REMOTE_DEVICE;
}

int _usb_ext_register_callback(struct device *dev, usb_ext_event_callback callback)
{
    if (!dev)
        return -ENODEV;

    s_data.usb_ext = dev;
    s_data.usb_ext_callback = callback;

    return 0;
}

int _usb_ext_unregister_callback(struct device *dev)
{
    if (!dev)
        return -ENODEV;

    s_data.usb_ext = NULL;
    s_data.usb_ext_callback = NULL;

    return 0;
}

static int _slave_status_callback(struct device *dev, uint32_t slave_status)
{
    if (!dev)
        return -ENODEV;

    switch (slave_status) {
    case MHB_PM_STATUS_PEER_CONNECTED:
        if (s_data.hsic)
            device_hsic_release_reset(s_data.hsic);

        s_data.attached = true;
        if (s_data.usb_ext_callback)
            s_data.usb_ext_callback(s_data.usb_ext,  s_data.attached);
        break;
    }

    return 0;
}

static int _hsic_status_cb(struct device *dev, uint8_t attached) {
    lldbg("USB: %s\n", attached ? "connected" : "disconnected");

    s_data.attached = attached ? true : false;
    if (s_data.usb_ext_callback)
        s_data.usb_ext_callback(s_data.usb_ext,  s_data.attached);

    return 0;
}

static void _usb_turn_on(void) {
    int ret;

    if (s_data.conn_type == DIRECT_USB) {
        s_data.hsic = device_open(DEVICE_TYPE_HSIC_DEVICE, 0);

        if (!s_data.hsic) {
            lldbg("Failed to open HSIC devie\n");
            return;
        }

        ret = device_hsic_hold_reset(s_data.hsic);
        if (ret) {
            device_close(s_data.hsic);
            s_data.hsic = NULL;
            return;
        }

        // Turn on APBE to enable 19.2 MHx ref clock
        s_data.slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW,
                                             MHB_ADDR_HSIC);
        if (!s_data.slave_pwr_ctrl) {
            lldbg("Failed to open slave pwrctrl device\n");
            return;
        }

        ret = device_slave_pwrctrl_register_status_callback(s_data.slave_pwr_ctrl,
                                                            _slave_status_callback);
        if (ret) {
            lldbg("Failed to register slave pwr ctrl\n");
            return;
        }

        ret = device_slave_pwrctrl_send_slave_state(s_data.slave_pwr_ctrl,
                                                    SLAVE_STATE_ENABLED);
        if (ret) {
            lldbg("Failed to send state on\n");
            return;
        }
    } else {
        s_data.usbtun = device_open(DEVICE_TYPE_USBTUN_HW, 0);
        if (!s_data.usbtun) {
            lldbg("Failed to open usbtun device\n");
            return;
        }

        device_usbtun_register_callback(s_data.usbtun, &_hsic_status_cb);

        if (s_data.conn_type == HSIC_TEST) {
            s_data.attached = true;
            if (s_data.usb_ext_callback)
                s_data.usb_ext_callback(s_data.usb_ext,  s_data.attached);
        }

        ret = device_usbtun_on(s_data.usbtun);

        if (ret) {
            lldbg("Failed to turn on usb tunneling\n");
            return;
        }
    }
}

static void _usb_turn_off(void) {
    int ret;

    if (s_data.slave_pwr_ctrl) {
        ret = device_slave_pwrctrl_unregister_status_callback(
            s_data.slave_pwr_ctrl, _slave_status_callback);
        if (ret) {
            lldbg("Failed to unregister slave pwr ctrl\n");
        }

        ret = device_slave_pwrctrl_send_slave_state(s_data.slave_pwr_ctrl,
                                                    SLAVE_STATE_DISABLED);
        if (ret) {
            lldbg("Failed to send state off\n");
        }

        device_close(s_data.slave_pwr_ctrl);

        s_data.slave_pwr_ctrl = NULL;
    }

    if (s_data.usbtun) {
        ret = device_usbtun_off(s_data.usbtun);

        if (ret) {
            lldbg("Failed to turn off usb tunneling\n");
        }

        device_close(s_data.usbtun);
    }

    if (s_data.hsic) {
        device_hsic_hold_reset(s_data.hsic);
        device_close(s_data.hsic);
    }

    s_data.hsic = NULL;
    s_data.attached = false;

    if (s_data.usb_ext_callback)
        s_data.usb_ext_callback(s_data.usb_ext,  s_data.attached);
}

static int _usb_ext_open(struct device *dev)
{
    if (!dev)
        return -EINVAL;

    return 0;
}

static void _usb_ext_close(struct device *dev)
{
    if (!dev)
        return;

    if (s_data.hsic)
        device_close(s_data.hsic);

    s_data.hsic = NULL;
    s_data.attached = false;
}

static int _usb_ext_probe(struct device *dev)
{
    if (!dev)
        return -EINVAL;

    s_data.hsic = NULL;
    s_data.usb_ext = NULL;
    s_data.usb_ext_callback = NULL;
    s_data.attached = false;

    struct device_resource *res;
    uint8_t intn;
    uint8_t vena;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "c_int_n");
    if (!res) {
        lldbg("failed to get c_int_n gpio\n");
        return -ENODEV;
    }
    intn = res->start;

    res = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_GPIO, "vbus_ena");
    if (!res) {
        lldbg("failed to get vbus_ena gpio\n");
        return -ENODEV;
    }
    vena = res->start;

    fusb302_register(intn, vena);

    return 0;
}

static void _usb_ext_remove(struct device *dev)
{
    fusb302_unregister();
}

static struct device_usb_ext_type_ops _usb_ext_type_ops = {
    .get_attached         = _usb_ext_get_attached,
    .get_protocol         = _usb_ext_get_protocol,
    .get_path             = _usb_ext_get_path,
    .get_type             = _usb_ext_get_type,
    .register_callback    = _usb_ext_register_callback,
    .unregister_callback  = _usb_ext_unregister_callback,
};

static struct device_driver_ops _usb_ext_driver_ops = {
    .probe              = _usb_ext_probe,
    .remove             = _usb_ext_remove,
    .open               = _usb_ext_open,
    .close              = _usb_ext_close,
    .type_ops           = &_usb_ext_type_ops,
};

struct device_driver raw_hsic_usb_ext_driver = {
    .type       = DEVICE_TYPE_USB_EXT_HW,
    .name       = "usb_ext",
    .desc       = "USB-EXT Interface",
    .ops        = &_usb_ext_driver_ops,
};
