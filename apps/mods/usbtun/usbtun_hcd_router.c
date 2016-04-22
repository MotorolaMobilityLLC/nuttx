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

#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/usb.h>
#include <nuttx/usb/usb.h>
#include <nuttx/bufram.h>

#include "nuttx/usbtun/usbtun_hcd_router.h"

/* HCD local defs */
#define SET_PORT_FEATURE 0x2303
#define GET_PORT_STATUS  0xa300
#define PORT_POWERING_DELAY_IN_MS   1000
#define PORT_RESETING_DELAY_IN_MS   1000
#define PORT_RESET      0x4
#define PORT_POWER      0x8
#define ROOT_HUB_PORT   1

#define USB_SPEED_HIGH  3


struct hcd_router_data_s {
    struct device *dev;
};

struct hcd_router_data_s s_data;

static pthread_t hcd_thread;
static pthread_mutex_t run_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t run_cond = PTHREAD_COND_INITIALIZER;
static bool do_run = false;

static void *hcd_router_startup(void *arg) {

    /* TODO: temporary memory debug - to be removed */
    struct mallinfo mem;
#ifdef CONFIG_CAN_PASS_STRUCTS
    mem = mallinfo();
#else
    (void)mallinfo(&mem);
#endif
    lldbg("total:%d, used:%d, free:%d, largest:%d\n",
          mem.arena, mem.uordblks, mem.fordblks, mem.mxordblk);

    int ret;
    uint32_t *status = bufram_alloc(sizeof(uint32_t));

    if (!status) {
        lldbg("Failed to allocate status memory\n");
        return NULL;
    }

    memset(&s_data, 0, sizeof(s_data));

    s_data.dev = device_open(DEVICE_TYPE_USB_HCD, 0);
    if (!s_data.dev) {
        lldbg("Failed to open HCD device\n");
        goto errout;
    }

    ret = device_usb_hcd_start(s_data.dev);
    if (ret) {
        lldbg("Failed to start up HCD device\n");
        goto errout;
    }
    lldbg("HCD device started\n");

    /*
     * Enable port 1 on root hub
     */
    device_usb_hcd_hub_control(s_data.dev, SET_PORT_FEATURE, PORT_POWER, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_POWERING_DELAY_IN_MS);

    device_usb_hcd_hub_control(s_data.dev, SET_PORT_FEATURE, PORT_RESET, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_RESETING_DELAY_IN_MS);

    /* Get port status of Port 1. Expecting a 4 bytes response. */
    device_usb_hcd_hub_control(s_data.dev, GET_PORT_STATUS, 0, 1, (char *)status, sizeof(*status));

    lldbg("Status on root hub port %08x\n", *status);
    lldbg("HDC Router started\n");

    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        pthread_mutex_unlock(&run_lock);
        goto stop_hcd;
    }

    /* Wait until uninit request is received */
    pthread_cond_wait(&run_cond, &run_lock);
    pthread_mutex_unlock(&run_lock);

stop_hcd:
    lldbg("HCD router stopped\n");
    device_usb_hcd_stop(s_data.dev);
errout:
    device_close(s_data.dev);
    s_data.dev = NULL;
    bufram_free(status);

    /* TODO: temporary memory debug - to be removed */
#ifdef CONFIG_CAN_PASS_STRUCTS
    mem = mallinfo();
#else
    (void)mallinfo(&mem);
#endif
    lldbg("total:%d, used:%d, free:%d, largest:%d\n",
          mem.arena, mem.uordblks, mem.fordblks, mem.mxordblk);

    return NULL;
}

int usbtun_hcd_router_init(void) {
    pthread_mutex_lock(&run_lock);
    if (do_run) {
        /* already running. Just return */
        pthread_mutex_unlock(&run_lock);
        return 0;
    }
    do_run = true;
    pthread_mutex_unlock(&run_lock);

    int ret =  pthread_create(&hcd_thread, NULL, hcd_router_startup, NULL);
    if (ret) {
        lldbg("HCD router init failed\n");
        return ret;
    }

    return 0;
}

void usbtun_hcd_router_uninit(void) {
    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        /* Not running. Just return */
        pthread_mutex_unlock(&run_lock);
        return;
    }
    do_run = false;
    pthread_cond_signal(&run_cond);
    pthread_mutex_unlock(&run_lock);

    pthread_join(hcd_thread, NULL);

    return;
}
