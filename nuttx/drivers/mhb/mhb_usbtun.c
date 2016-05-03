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

#include <nuttx/device.h>
#include <nuttx/device_usbtun.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>

struct usbtun_s {
    struct device *mhb_dev;
};

static int _on(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;

    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
    hdr.result = 0;
    req.command = MHB_HSIC_COMMAND_START;
    return device_mhb_send(data->mhb_dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);
}

static int _off(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;

    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
    hdr.result = 0;
    req.command = MHB_HSIC_COMMAND_STOP;
    return device_mhb_send(data->mhb_dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);
}

static int _open(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);

    if (!data)
        return -ENODEV;

    data->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_HSIC);
    if (!data->mhb_dev) {
        lldbg("ERROR: failed to open MHB device.\n");
        return -ENOENT;
    }

    return 0;
}

static void _close(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);

    if (!data) {
        return;
    }

    if (data->mhb_dev) {
        device_close(data->mhb_dev);
        data->mhb_dev = NULL;
    }
}

static int _probe(struct device *dev) {
    struct usbtun_s *data;

    data = zalloc(sizeof(*data));
    if (!data)
        return -ENOMEM;

    device_set_private(dev, data);

    return 0;
}

static void _remove(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);

    if (data) {
        free(data);
    }
}

static struct device_usbtun_type_ops _type_ops = {
    .on  = _on,
    .off = _off,
};

static struct device_driver_ops _driver_ops = {
    .open       = _open,
    .close      = _close,
    .probe      = _probe,
    .remove     = _remove,
    .type_ops   = &_type_ops,
};

struct device_driver mhb_usbtun_driver = {
    .type   = DEVICE_TYPE_USBTUN_HW,
    .name   = "mhb_usbtun",
    .desc   = "MHB USB Tunneling Driver",
    .ops    = &_driver_ops,
};
