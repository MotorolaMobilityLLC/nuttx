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
 *
 */

#include <arch/byteorder.h>
#include <nuttx/clock.h>
#include <nuttx/device.h>
#include <nuttx/device_usb_ext.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/wqueue.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* greybus messages */
#define GB_USB_EXT_TYPE_PROTOCOL_VERSION  0x01
#define GB_USB_EXT_TYPE_AP_READY          0x02
#define GB_USB_EXT_TYPE_ATTACH_STATE      0x03

/* greybus protocol version defaults */
#define GB_USB_EXT_VERSION_MAJOR          0x00
#define GB_USB_EXT_VERSION_MINOR          0x01

#define GB_USB_EXT_ATTACH_DELAY_MS        1000

struct gb_usb_ext_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

struct gb_usb_ext_attach_request {
    __u8 active;
    __u8 protocol;      /* 2.0 or 3.1 */
    __u8 path;          /* A or B */
    __u8 remote_type;   /* host or device */
} __packed;

struct gb_usb_ext_info {
    struct work_s wq;
    unsigned int  cport;
    struct device *dev;
    uint8_t path;
    uint8_t protocol;
    uint8_t remote_type;
    uint8_t attached;
    bool ap_ready;
};

static struct gb_usb_ext_info usb_ext_info;

/**
 * @brief USB_EXT protocol version
 *
 */
static uint8_t gb_usb_ext_protocol_version(struct gb_operation *operation)
{
    struct gb_usb_ext_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_USB_EXT_VERSION_MAJOR;
    response->minor = GB_USB_EXT_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/**
 * @brief USB_EXT send attach message
 *
 */
static uint8_t gb_usb_ext_send_attach(unsigned int cport, uint8_t active,
        uint8_t protocol, uint8_t type, uint8_t path)
{
    struct gb_usb_ext_attach_request *request;
    struct gb_operation *operation;
    int ret;

    operation = gb_operation_create(cport, GB_USB_EXT_TYPE_ATTACH_STATE,
        sizeof(*request));
    if (!operation)
        return GB_OP_NO_MEMORY;

    request = gb_operation_get_request_payload(operation);
    if (!request) {
        gb_operation_destroy(operation);
        return GB_OP_INVALID;
    }

    request->active = active;
    request->protocol = protocol;
    request->remote_type = type;
    request->path = path;

    vdbg("active   = %d\n", request->active);
    vdbg("protocol = %s\n", (request->protocol == GB_USB_EXT_PROTOCOL_2_0) ?
            "2.0" : "3.1");
    vdbg("type     = %s\n", (request->remote_type == GB_USB_EXT_REMOTE_DEVICE) ?
            "device" : "host");
    vdbg("path     = %s\n", (request->path == GB_USB_EXT_PATH_A) ?
            "A" : "B");

    ret = gb_operation_send_request(operation, NULL, false);

    gb_operation_destroy(operation);

    return ret;
}
/**
 * @brief Gather USB state and send attach msg
 *
 */
static void gb_usb_ext_attach_worker(void *arg)
{
    struct device *dev = (struct device *)arg;
    uint8_t active;
    uint8_t protocol;
    uint8_t type;
    uint8_t path;
    int rv;

    if (dev == NULL) {
        dbg("NULL!!!!!\n");
        return;
    }

    if ((rv = device_usb_ext_get_attached(dev, &active))) {
        dbg("active error %d\n", rv);
        return;
    }
    if ((rv = device_usb_ext_get_protocol(dev, &protocol))) {
        dbg("protocol error\n");
        return;
    }
    if ((rv = device_usb_ext_get_type(dev, &type))) {
        dbg("type error\n");
        return;
    }
    if ((rv = device_usb_ext_get_path(dev, &path))) {
        dbg("path error\n");
        return;
    }

    (void)gb_usb_ext_send_attach(usb_ext_info.cport,
            active, protocol, type, path);
}
/**
 * @brief queue sending attach message
 *
 */
static void gb_usb_queue_work(struct device *dev, uint32_t delay)
{
    if (!work_available(&usb_ext_info.wq))
        work_cancel(LPWORK, &usb_ext_info.wq);

    work_queue(LPWORK, &usb_ext_info.wq, gb_usb_ext_attach_worker,
        (void *)dev, delay);
}

/**
 * @brief USB_EXT state changed callback
 *
 * This callback is called when the state (usually attached/detached) of
 * the Mod has changed.
 *
 * @return 0
 */
static int event_callback(struct device *dev, bool attached)
{
    if (usb_ext_info.ap_ready)
        gb_usb_queue_work(dev, 0);

    return 0;
}
/**
 * @brief Greybus USB ext protocol AP is ready for usb_ext msgs
 *
 * @param operation 
 * @return 0 on success, negative errno on error
 */
static uint8_t gb_usb_ext_ap_ready(struct gb_operation *operation)
{
    usb_ext_info.ap_ready = true;

    gb_usb_queue_work(usb_ext_info.dev, MSEC2TICK(GB_USB_EXT_ATTACH_DELAY_MS));

    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus USB ext protocol initialization
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_usb_ext_init(unsigned int cport)
{
    int ret;

    memset(&usb_ext_info, 0, sizeof(usb_ext_info));
    usb_ext_info.cport = cport;

    usb_ext_info.dev = device_open(DEVICE_TYPE_USB_EXT_HW, 0);
    if (!usb_ext_info.dev) {
        dbg("no device\n");
        ret = -EIO;
        goto err_out;
    }

    ret = device_usb_ext_register_callback(usb_ext_info.dev, event_callback);
    if (ret) {
        dbg("failed to register callback \n");
        goto err_close_device;
    }

    return 0;

err_close_device:
    device_close(usb_ext_info.dev);

err_out:
    memset(&usb_ext_info, 0, sizeof(usb_ext_info));

    return ret;
}

/**
 * @brief Greybus USB ext protocol deinitialize function
 *
 * @param cport CPort number
 */
static void gb_usb_ext_exit(unsigned int cport)
{
    device_usb_ext_unregister_callback(usb_ext_info.dev);

    device_close(usb_ext_info.dev);
    memset(&usb_ext_info, 0, sizeof(usb_ext_info));
}

static struct gb_operation_handler gb_usb_ext_handlers[] = {
    GB_HANDLER(GB_USB_EXT_TYPE_PROTOCOL_VERSION, gb_usb_ext_protocol_version),
    GB_HANDLER(GB_USB_EXT_TYPE_AP_READY, gb_usb_ext_ap_ready),
};

struct gb_driver usb_ext_driver = {
    .init = gb_usb_ext_init,
    .exit = gb_usb_ext_exit,
    .op_handlers = (struct gb_operation_handler*) gb_usb_ext_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_usb_ext_handlers),
};

void gb_usb_ext_register(int cport)
{
    gb_register_driver(cport, &usb_ext_driver);
}
