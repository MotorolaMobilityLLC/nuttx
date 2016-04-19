/*
 * Copyright (c) 2016 Motorola, LLC.
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

/* greybus attach definitions */
#define GB_USB_EXT_PROTOCOL_2_0           0x00
#define GB_USB_EXT_PROTOCOL_3_1           0x01

#define GB_USB_EXT_PATH_ENTERPRISE        0x00
#define GB_USB_EXT_PATH_BRIDGE            0x01

#define GB_USB_EXT_REMOTE_DEVICE          0x00
#define GB_USB_EXT_REMOTE_HOST            0x01

struct gb_usb_ext_proto_version_response {
    __u8 major;
    __u8 minor;
} __packed;

struct gb_usb_ext_attach_request {
    __u8 active;
    __u8 protocol;      /* 2.0 or 3.1 */
    __u8 path;          /* tsb bridge or shared dp/usb */
    __u8 remote_type;   /* host or device */
} __packed;

struct gb_usb_ext_info {
    struct work_s wq;
    unsigned int  cport;
    struct device *dev;
};

static struct gb_usb_ext_info usb_ext_info;

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

static uint8_t gb_usb_ext_send_attach_state(unsigned int cport, bool active)
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

    request->active = (__u8)active;

/* Rules:
 *   USB2.0 can run over either interface (bridge or enterprise)
 *   USB2.0 must run in with MOD as the device
 *   USB3.1 can only run over the enterprise bridge
 *   USB3.1 can run with MOD as either the host or device
 */
#ifdef CONFIG_GREYBUS_USB_EXT_PROTO_2_0
    request->protocol = GB_USB_EXT_PROTOCOL_2_0;
    request->remote_type = GB_USB_EXT_REMOTE_DEVICE;
# ifdef CONFIG_GREYBUS_USB_EXT_PATH_BRIDGE
    request->path = GB_USB_EXT_PATH_BRIDGE;
# elif CONFIG_GREYBUS_USB_EXT_PATH_ENTERPRISE
    request->path = GB_USB_EXT_PATH_ENTERPRISE;
# else
#  error USB_EXT Path Required
# endif

#elif CONFIG_GREYBUS_USB_EXT_PROTO_3_1
    request->protocol = GB_USB_EXT_PROTOCOL_3_1;
    request->path = GB_USB_EXT_PATH_ENTERPRISE;
# ifdef CONFIG_GREYBUS_USB_EXT_REMOTE_DEVICE
    request->remote_type = GB_USB_EXT_REMOTE_DEVICE;
# elif CONFIG_GREYBUS_USB_EXT_REMOTE_HOST
    request->remote_type = GB_USB_EXT_REMOTE_HOST;
# else
#  error USB EXT Protocol Host/Device must be defined
# endif
#else
# error USB Protocol not defined
#endif

    ret = gb_operation_send_request(operation, NULL, false);

    gb_operation_destroy(operation);

    return ret;
}

static inline uint8_t gb_usb_ext_send_attach(unsigned int cport)
{
    return gb_usb_ext_send_attach_state(cport, TRUE);
}

static inline uint8_t gb_usb_ext_send_detach(unsigned int cport)
{
    return gb_usb_ext_send_attach_state(cport, FALSE);
}

static void gb_usb_ext_attach_worker(void *arg)
{
    unsigned int cport = (unsigned int)arg;

    (void)gb_usb_ext_send_attach(cport);
}

static void gb_usb_ext_detach_worker(void *arg)
{
    unsigned int cport = (unsigned int)arg;

    (void)gb_usb_ext_send_detach(cport);
}

static int event_callback(bool attached)
{
    if (!work_available(&usb_ext_info.wq))
        work_cancel(LPWORK, &usb_ext_info.wq);

    worker_t worker = attached ? gb_usb_ext_attach_worker : gb_usb_ext_detach_worker;
    work_queue(LPWORK, &usb_ext_info.wq, worker, (void *)usb_ext_info.cport, 0);
    return 0;
}

static uint8_t gb_usb_ext_ap_ready(struct gb_operation *operation)
{
    int ret;

    memset(&usb_ext_info, 0, sizeof(usb_ext_info));

    usb_ext_info.dev = device_open(DEVICE_TYPE_USB_EXT_HW, 0);
    if (!usb_ext_info.dev) {
        return -EIO;
    }

    usb_ext_info.cport = operation->cport;

    ret = device_usb_ext_register_callback(usb_ext_info.dev, event_callback);
    if (ret) {
        device_close(usb_ext_info.dev);
        memset(&usb_ext_info, 0, sizeof(usb_ext_info));
        return ret;
    }

    return 0;
}

static struct gb_operation_handler gb_usb_ext_handlers[] = {
    GB_HANDLER(GB_USB_EXT_TYPE_PROTOCOL_VERSION, gb_usb_ext_protocol_version),
    GB_HANDLER(GB_USB_EXT_TYPE_AP_READY, gb_usb_ext_ap_ready),
};

struct gb_driver usb_ext_driver = {
    .op_handlers = (struct gb_operation_handler*) gb_usb_ext_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_usb_ext_handlers),
};

void gb_usb_ext_register(int cport)
{
    gb_register_driver(cport, &usb_ext_driver);
}
