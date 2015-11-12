/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
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
#include <semaphore.h>
#include <stddef.h>
#include <stdlib.h>

#include <nuttx/device.h>
#include <nuttx/device_pad_det.h>
#include <nuttx/power/pad_det.h>
#include <nuttx/list.h>

static sem_t sem = SEM_INITIALIZER(1);

struct notify_node_s
{
    pad_detect_callback_t callback;
    void *arg;
    struct list_head list;
};

struct pad_det_info_s
{
    bool docked;
    struct list_head notify_list;
};

static struct pad_det_info_s *g_info;

static void pad_det_callback(void *arg, bool docked)
{
    static bool init = true; /* first callback is to init docked state */
    struct notify_node_s *node;
    struct list_head *iter;

    if (init) {
        init = false;
        g_info->docked = docked;
        return;
    }

    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            dbg("failed to acquire semaphore\n");
            return;
        }
    }

    if (g_info->docked == docked)
            goto callback_done;

    g_info->docked = docked;
    list_foreach(&g_info->notify_list, iter) {
        node = list_entry(iter, struct notify_node_s, list);
        node->callback(node->arg, docked);
    }

callback_done:
    sem_post(&sem);
}

static int pad_det_init(void)
{
    struct device *dev;
    int retval;

    g_info = zalloc(sizeof(*g_info));
    if (!g_info) {
        dbg("failed to allocate memory\n");
        return -ENOMEM;
    }

    dev = device_open(DEVICE_TYPE_PAD_DET_HW, 0);
    if (!dev) {
        dbg("failed to open pad detect device\n");
        retval = -EIO;
        goto dev_err;
    }

    retval = device_pad_det_register_callback(dev, pad_det_callback, NULL);
    if (retval) {
        dbg("failed to register pad detect callback\n");
        goto cb_err;
    }

    list_init(&g_info->notify_list);

    return 0;

cb_err:
    device_close(dev);
dev_err:
    free(g_info);
    g_info = NULL;
    return retval;
}

int pad_det_register_callback(pad_detect_callback_t callback, void *arg)
{
    struct notify_node_s *node;
    int retval = 0;

    while (sem_wait(&sem) != OK) {
        if (errno == EINVAL) {
            dbg("failed to acquire semaphore\n");
            return -EINVAL;
        }
    }

    if (!g_info)
        retval = pad_det_init();

    if (retval)
        goto register_done;

    node = zalloc(sizeof(*node));
    if (!node) {
        retval = -ENOMEM;
        goto register_done;
    }

    node->callback = callback;
    node->arg = arg;
    list_add(&g_info->notify_list, &node->list);
    callback(arg, g_info->docked);

register_done:
    sem_post(&sem);
    return retval;
}