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
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/device.h>
#include <nuttx/device_usbtun.h>
#include <nuttx/device_slave_pwrctrl.h>
#include <nuttx/mhb/device_mhb.h>
#include <nuttx/mhb/mhb_protocol.h>
#include <nuttx/time.h>
#include <nuttx/usb.h>

struct usbtun_s {
    struct device *mhb_dev;
    struct device *slave_pwr_ctrl;
    struct device *hsic;
    usbtun_status_cb cb;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

static struct usbtun_s *s_data = NULL;

#define MHB_USBTUN_OP_TIMEOUT_NS     2000000000LL /* 2 seconds in ns */

static void _signal_response(struct usbtun_s *data) {
    pthread_mutex_lock(&data->mutex);
    pthread_cond_signal(&data->cond);
    pthread_mutex_unlock(&data->mutex);
}

static int _wait_for_response(struct usbtun_s *data) {
    int ret;
    struct timespec expires;

    if (clock_gettime(CLOCK_REALTIME, &expires)) {
        return -EBADF;
    }

    uint64_t new_ns = timespec_to_nsec(&expires);
    new_ns += MHB_USBTUN_OP_TIMEOUT_NS;
    nsec_to_timespec(new_ns, &expires);

    pthread_mutex_lock(&data->mutex);
    ret = pthread_cond_timedwait(&data->cond, &data->mutex, &expires);
    pthread_mutex_unlock(&data->mutex);

    if (ret) {
        /* timeout or other erros */
        lldbg("ERROR: wait error %d\n", -ret);
        return -ETIMEDOUT;
    }

    return 0;
}

static int _mhb_handle_msg(struct device *dev,
                           struct mhb_hdr *hdr, uint8_t *payload, size_t payload_length) {
    if (!s_data) {
        return -ENODEV;
    }

    switch(hdr->type) {
    case MHB_TYPE_HSIC_CONTROL_RSP:
        if (hdr->result != MHB_RESULT_SUCCESS) {
            lldbg("MHB HSIC Control RSP with failure\n");
        }
        _signal_response(s_data);
        break;
    case MHB_TYPE_HSIC_STATUS_NOT:
        if (payload_length && s_data->cb) {
            s_data->cb(dev, *payload);
        }
        break;
    default:
        break;
    }

    return 0;
}
static int _slave_status_callback(struct device *dev, uint32_t slave_status) {
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;

    if (!s_data)
        return -ENODEV;

    switch (slave_status) {
    case MHB_PM_STATUS_PEER_CONNECTED:
        if (s_data->hsic)
            device_hsic_release_reset(s_data->hsic);

        s_data->mhb_dev = device_open(DEVICE_TYPE_MHB, MHB_ADDR_HSIC);
        if (!s_data->mhb_dev) {
            lldbg("ERROR: failed to open MHB device.\n");
            return -ENOENT;
        }

        device_mhb_register_receiver(s_data->mhb_dev, MHB_ADDR_HSIC,
                                     _mhb_handle_msg);

        hdr.addr = MHB_ADDR_HSIC;
        hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
        hdr.result = 0;
        req.command = MHB_HSIC_COMMAND_START;

        return device_mhb_send(s_data->mhb_dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);
    }

    return 0;
}

static int _on(struct device *dev) {
    int ret;
    struct usbtun_s *data = device_get_private(dev);

    data->hsic = device_open(DEVICE_TYPE_HSIC_DEVICE, 0);

    if (!data->hsic) {
        /* It's possible HSIC device might be controlled by APBE.
           Just print warning and fail through */
        lldbg("No HSIC devie present\n");
    } else
        device_hsic_hold_reset(data->hsic);

    if (data->slave_pwr_ctrl) {
        lldbg("Slave pwr ctrl already open\n");
        return -EBUSY;
    }

    data->slave_pwr_ctrl = device_open(DEVICE_TYPE_SLAVE_PWRCTRL_HW,
                                       MHB_ADDR_HSIC);
    if (!data->slave_pwr_ctrl) {
        lldbg("Failed to open\n");
        return -ENODEV;
    }

    ret = device_slave_pwrctrl_register_status_callback(data->slave_pwr_ctrl,
                                                        _slave_status_callback);
    if (ret) {
        lldbg("Failed to register slave pwr ctrl\n");
        return ret;
    }

    ret = device_slave_pwrctrl_send_slave_state(data->slave_pwr_ctrl,
                                                SLAVE_STATE_ENABLED);
    if (ret) {
        lldbg("Failed to send state on\n");
        return ret;
    }

    return 0;
}

static void close_all(struct usbtun_s *data) {
    if (data->mhb_dev) {
        device_mhb_unregister_receiver(data->mhb_dev, MHB_ADDR_HSIC,
                                       _mhb_handle_msg);
        device_close(data->mhb_dev);
        data->mhb_dev = NULL;
    }

    if (data->slave_pwr_ctrl) {
        int ret = device_slave_pwrctrl_unregister_status_callback(
            data->slave_pwr_ctrl, _slave_status_callback);
        if (ret) {
            lldbg("Failed to unregister slave pwr ctrl\n");
        }

        ret = device_slave_pwrctrl_send_slave_state(data->slave_pwr_ctrl,
                                                SLAVE_STATE_DISABLED);
        if (ret) {
            lldbg("Failed to send state off\n");
        }

        device_close(data->slave_pwr_ctrl);
        data->slave_pwr_ctrl = NULL;
    }

    if (data->hsic) {
        device_close(data->hsic);
        data->hsic = NULL;
    }
}

static int _off(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);
    struct mhb_hdr hdr;
    struct mhb_hsic_control_req req;
    int ret;

    hdr.addr = MHB_ADDR_HSIC;
    hdr.type = MHB_TYPE_HSIC_CONTROL_REQ;
    hdr.result = 0;
    req.command = MHB_HSIC_COMMAND_STOP;
    ret = device_mhb_send(data->mhb_dev, &hdr, (const uint8_t *)&req, sizeof(req), 0);

    if (ret) {
        return ret;
    } else {
        _wait_for_response(s_data);
    }

    close_all(data);

    return 0;
}

static int _reg_cb(struct device *dev, usbtun_status_cb cb) {
    struct usbtun_s *data = device_get_private(dev);

    if (data->cb)
        return -EBUSY;

    data->cb = cb;

    return 0;
}

static int _unreg_cb(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);

    data->cb = NULL;

    return 0;
}

static void _close(struct device *dev) {
    struct usbtun_s *data = device_get_private(dev);

    if (!data) {
        return;
    }

    close_all(data);
}

static int _probe(struct device *dev) {
    struct usbtun_s *data;

    s_data = zalloc(sizeof(*data));
    if (!s_data)
        return -ENOMEM;

    pthread_mutex_init(&s_data->mutex, NULL);
    pthread_cond_init(&s_data->cond, NULL);
    device_set_private(dev, s_data);

    return 0;
}

static void _remove(struct device *dev) {
    if (s_data) {
        free(s_data);
    }
}

static struct device_usbtun_type_ops _type_ops = {
    .on  = _on,
    .off = _off,
    .register_callback = _reg_cb,
    .unregister_callback = _unreg_cb,
};

static struct device_driver_ops _driver_ops = {
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
