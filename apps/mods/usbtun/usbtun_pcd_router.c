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
#include <pthread.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/usb_device.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include "nuttx/usbtun/usbtun_pcd_router.h"

struct pcd_router_data_s {
    struct device *pcddev;
    struct usbdev_s *usbdev;
    struct usbdevclass_driver_s driver;
};

struct pcd_router_data_s s_data;

static pthread_t pcd_thread;
static pthread_mutex_t run_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t run_cond = PTHREAD_COND_INITIALIZER;
static bool do_run = false;

static int _usbclass_bind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    s_data.usbdev = dev;
    DEV_SETSELFPOWERED(dev);

    return 0;
}

static void _usbclass_unbind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    s_data.usbdev = NULL;
}

static int _usbclass_setup(struct usbdevclass_driver_s *driver, struct usbdev_s *dev,
                           const struct usb_ctrlreq_s *ctrl, uint8_t * dataout, size_t outlen) {
    /* TODO: To be filled in */
    return 0;
}
static void _usbclass_disconnect(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    /* Perform the soft connect function so that we can be re-enumerated. */
    DEV_CONNECT(dev);
}

/* USB class driver operations */
static const struct usbdevclass_driverops_s s_driverops = {
    _usbclass_bind,             /* bind */
    _usbclass_unbind,           /* unbind */
    _usbclass_setup,            /* setup */
    _usbclass_disconnect,       /* disconnect */
    NULL,                       /* suspend */
    NULL,                       /* resume */
};

static void *pcd_router_startup(void *arg) {
    int ret;

    /* TODO: Just temporary debug until HSIC become fully stable */
    struct mallinfo mem;

#ifdef CONFIG_CAN_PASS_STRUCTS
    mem = mallinfo();
#else
    (void)mallinfo(&mem);
#endif
    lldbg("total:%d, used:%d, free:%d, largest:%d\n",
          mem.arena, mem.uordblks, mem.fordblks, mem.mxordblk);

    memset(&s_data, 0, sizeof(s_data));

    s_data.pcddev = device_open(DEVICE_TYPE_USB_PCD, 0);
    if (!s_data.pcddev) {
        lldbg("Faied to open PCD device\n");
        return NULL;
    }

    s_data.driver.speed = USB_SPEED_HIGH;
    s_data.driver.ops = &s_driverops;

    ret = device_usbdev_register_gadget(s_data.pcddev, &s_data.driver);
    if (ret) {
        lldbg("Failed to register pcd_router_driver\n");
        goto devclose;
    }

    lldbg("PCD router started\n");

    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        pthread_mutex_unlock(&run_lock);
        goto unreg_gadget;
    }

    /* Wait until uninit request is received */
    pthread_cond_wait(&run_cond, &run_lock);
    pthread_mutex_unlock(&run_lock);

unreg_gadget:
    lldbg("PCD router stopped\n");
    device_usbdev_unregister_gadget(s_data.pcddev, &s_data.driver);

devclose:
    device_close(s_data.pcddev);
    s_data.pcddev = NULL;

    /* TODO: temporary debug */
#ifdef CONFIG_CAN_PASS_STRUCTS
    mem = mallinfo();
#else
    (void)mallinfo(&mem);
#endif
    lldbg("total:%d, used:%d, free:%d, largest:%d\n",
          mem.arena, mem.uordblks, mem.fordblks, mem.mxordblk);

    return NULL;
}

int usbtun_pcd_router_init(void) {
    pthread_mutex_lock(&run_lock);
    if (do_run) {
        /* already running. Just return */
        pthread_mutex_unlock(&run_lock);
        return 0;
    }
    do_run = true;
    pthread_mutex_unlock(&run_lock);

    int ret =  pthread_create(&pcd_thread, NULL, pcd_router_startup, NULL);
    if (ret) {
        lldbg("PCD router init failed\n");
        return ret;
    }

    return 0;
}

void usbtun_pcd_router_uninit(void) {
    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        /* Not running. Just return */
        pthread_mutex_unlock(&run_lock);
        return;
    }
    do_run = false;
    pthread_cond_signal(&run_cond);
    pthread_mutex_unlock(&run_lock);

    pthread_join(pcd_thread, NULL);

    return;
}

