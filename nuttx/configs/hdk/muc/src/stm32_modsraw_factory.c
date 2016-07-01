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
#include <string.h>
#include <unistd.h>

#include <arch/board/mods.h>

#include <nuttx/device.h>
#include <nuttx/device_display.h>
#include <nuttx/device_raw.h>
#include <nuttx/greybus/debug.h>
#include <arch/byteorder.h>

#define PID                                CONFIG_ARCH_BOARDID_PID
#define VID                                CONFIG_ARCH_BOARDID_VID
#define VERSION_ID                         0x01
#define RESP_CMD_MASK                      0x80

#define SET_DISPLAY_TYPE                   0x00
#define GPIO_READ                          0x01
#define GPIO_WRITE                         0x02

static struct display_mux
{
    struct device *dev;
    struct device *display_dev;
    display_notification_cb callback;
    enum display_type type;
} g_display;

struct raw_cmd_s
{
    uint32_t     pid;
    uint32_t     vid;
    uint8_t      ver_id;
    uint8_t      cid;
    uint8_t      size;
    uint8_t      payload[];
} __packed;

static struct device *gRawDevice;
static raw_send_callback gRawCallback;

static struct device* get_display_dev(struct device *dev)
{
    struct display_mux *display = device_get_private(dev);

    if (!display) {
        lldbg("ERROR: no display mux data.\n");
        return NULL;
    }
    return display->display_dev;
}

static int set_display_type(struct device *dev, enum display_type new_type)
{
    struct display_mux *display = device_get_private(dev);

    if (!display) {
        lldbg("ERROR: no display mux data.\n");
        return -ENODEV;
    }

    if (new_type != DISPLAY_TYPE_DP && new_type != DISPLAY_TYPE_DSI) {
        lldbg("ERROR: Invalid display type.\n");
        return -EINVAL;
    }

    if (new_type != display->type) {
        // send unavailable event and close old display driver
        if (display->callback) {
            display->callback(get_display_dev(dev), DISPLAY_NOTIFICATION_EVENT_UNAVAILABLE);
        }
        device_close(get_display_dev(dev));

        display->type = new_type;

        // opened new display driver and send host ready event
        display->display_dev = device_open(DEVICE_TYPE_DISPLAY_HW, display->type);
        if (!display->display_dev) {
            lldbg("ERROR: failed to open display device - type (%d).\n", display->type);
            return -ENODEV;
        }

        // register callback for new driver if callback was already registered with old driver
        if (display->callback) {
            device_display_register_callback(display->display_dev, display->callback);
        }

        device_display_host_ready(get_display_dev(dev));
    }

    return 0;
}


/* Device operations */
static int display_mux_host_ready(struct device *dev)
{
    return device_display_host_ready(get_display_dev(dev));
}

static int display_mux_get_config(struct device *dev, uint8_t *display_type,
    uint8_t *config_type, uint32_t *size, uint8_t **config)
{
    return device_display_get_config(get_display_dev(dev), display_type,
        config_type, size, config);
}

static int display_mux_set_config(struct device *dev, uint8_t index)
{
    return device_display_set_config(get_display_dev(dev), index);
}

static int display_mux_get_state(struct device *dev, uint8_t *state)
{
    return device_display_get_state(get_display_dev(dev), state);
}

static int display_mux_set_state(struct device *dev, uint8_t state)
{
    return device_display_set_state(get_display_dev(dev), state);
}

static int display_mux_register_callback(struct device *dev,
        display_notification_cb callback)
{
    struct display_mux *display = device_get_private(dev);
    display->callback = callback;
    return device_display_register_callback(get_display_dev(dev), callback);
}

static int display_mux_unregister_callback(struct device *dev)
{
    struct display_mux *display = device_get_private(dev);
    display->callback = NULL;
    return device_display_unregister_callback(get_display_dev(dev));
}

static int display_mux_dev_open(struct device *dev)
{
    int result = 0;
    struct display_mux *display = device_get_private(dev);
    if (!display) {
        return -ENODEV;
    }

    display->display_dev = device_open(DEVICE_TYPE_DISPLAY_HW, display->type);
    if (!display->display_dev) {
        lldbg("ERROR: failed to open display device - type (%d).\n", display->type);
        result = -ENODEV;
    }

    return result;
}

static void display_mux_dev_close(struct device *dev)
{
    struct display_mux *display = device_get_private(dev);
    if (!display) {
        return;
    }

    if (display->display_dev != NULL) {
        device_close(display->display_dev);
    }
}

static int display_mux_probe(struct device *dev)
{
    struct display_mux *display = &g_display;
    device_set_private(dev, display);

    // Set the initial type to DP
    display->type = DISPLAY_TYPE_DP;
    display->dev = dev;
    display->display_dev = NULL;

    return 0;
}

static void display_mux_remove(struct device *dev)
{
    struct display_mux *display = device_get_private(dev);
    if (display) {
        device_set_private(dev, NULL);
        memset(display, 0, sizeof(*display));
    }
}

const static struct device_display_type_ops display_mux_ops = {
    .host_ready = display_mux_host_ready,
    .get_config = display_mux_get_config,
    .set_config = display_mux_set_config,
    .get_state = display_mux_get_state,
    .set_state = display_mux_set_state,
    .register_callback = display_mux_register_callback,
    .unregister_callback = display_mux_unregister_callback,
};

const static struct device_driver_ops display_mux_driver_ops = {
    .probe = display_mux_probe,
    .remove = display_mux_remove,
    .open  = display_mux_dev_open,
    .close = display_mux_dev_close,
    .type_ops = (struct device_display_type_ops *)&display_mux_ops,
};

const struct device_driver display_mux_driver = {
    .type = DEVICE_TYPE_DISPLAY_HW,
    .name = "display_mux",
    .desc = "Display Mux Driver",
    .ops = (struct device_driver_ops *)&display_mux_driver_ops,
};

static int raw_response(uint8_t cid, uint8_t size, uint8_t * data)
{
    int ret = OK;

    int response_size = sizeof(struct raw_cmd_s) + size;
    uint8_t response_data[response_size];
    struct raw_cmd_s  *response;

    response = (struct raw_cmd_s*)response_data;
    response->pid = cpu_to_be32(PID);
    response->vid = cpu_to_be32(VID);
    response->ver_id = VERSION_ID;
    response->cid = cid | RESP_CMD_MASK;
    response->size = size;
    if (size > 0)
        memcpy(response->payload, data, size);

    if (gRawCallback) {
        gRawCallback(gRawDevice, response_size, response_data);
    }
    return ret ;
}

static int handle_raw_cmd (uint8_t cid,uint8_t size, uint8_t payload[])
{
    int ret = -EINVAL;
    uint8_t res = 0;

    switch (cid) {
        case SET_DISPLAY_TYPE:
            //SET_DISPLAY_TYPE must have 1 byte of payload data
            if(size != 1) return -EINVAL;

            enum display_type type = (enum display_type) payload[0];
            dbg("RAW CMD: SET_DISPLAY_TYPE - type (%d).\n", type);
            res = (uint8_t) set_display_type((&g_display)->dev, type);
            ret = raw_response(cid,sizeof(res),&res);
            break;

        case GPIO_READ:
            //GPIO_READ must have 1 byte of payload data
            if(size != 1) return -EINVAL;

            dbg("RAW CMD: GPIO_READ\n");

            uint8_t gpioData[2];
            gpio_direction_in(payload[0]);
            gpioData[1] = gpio_get_value(payload[0]);
            gpioData[0] = 0;
            ret = raw_response(cid,sizeof(gpioData),gpioData);
            break;

        case GPIO_WRITE:
            //GPIO_WRITE must have 2 byts of payload data
            if(size !=2) return -EINVAL;

            dbg("RAW CMD: GPIO_WRITE\n");

            gpio_direction_out(payload[0], payload[1]);
            ret = raw_response(cid,sizeof(res),&res);
            break;

        default:
            dbg("Invalid command: %d\n", cid);
            break;
    }

    return ret;
}

static int mods_raw_factory_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    struct raw_cmd_s  rec_cmd;

    if (len < sizeof(struct raw_cmd_s) || data == NULL) return -EINVAL;

    memcpy(&rec_cmd,data,len);
    rec_cmd.pid = be32_to_cpu(rec_cmd.pid);
    rec_cmd.vid = be32_to_cpu(rec_cmd.vid);

    if ((rec_cmd.pid != PID) || (rec_cmd.vid != VID)|| (rec_cmd.ver_id != VERSION_ID)) return -EINVAL;
    if (len < sizeof(struct raw_cmd_s) + rec_cmd.size) return -EINVAL;

    return handle_raw_cmd(rec_cmd.cid, rec_cmd.size,rec_cmd.payload);
}

static int mods_raw_factory_register_callback(struct device *dev,
                                    raw_send_callback callback)
{
    dbg("callback=0x%p\n", callback);

    gRawCallback = callback;
    return 0;
}

static int mods_raw_factory_unregister_callback(struct device *dev)
{
    gRawCallback = NULL;
    return 0;
}

static int mods_raw_factory_probe(struct device *dev)
{
    dbg("enter\n");
    gRawDevice = dev;
    return 0;
}

static struct device_raw_type_ops mods_raw_factory_type_ops = {
    .recv = mods_raw_factory_recv,
    .register_callback = mods_raw_factory_register_callback,
    .unregister_callback = mods_raw_factory_unregister_callback,
};

static struct device_driver_ops mods_raw_factory_driver_ops = {
    .probe = mods_raw_factory_probe,
    .type_ops = &mods_raw_factory_type_ops,
};

struct device_driver mods_raw_factory_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_factory",
    .desc = "Factory Raw Interface",
    .ops = &mods_raw_factory_driver_ops,
};

