/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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
#include <mqueue.h>
#include <fcntl.h>
#include <time.h>

#include <nuttx/greybus/mods.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/wqueue.h>
#include <nuttx/time.h>

struct raw_stub_info {
    struct device *gDevice;         /* Device handle */
    raw_send_callback gCallback;    /* Raw callback to send */
    mqd_t cmd_queue;                /* Msg queue for cmd_thread */
    pthread_t cmd_thread;           /* command thread */
};

#define MAX_MSG_SIZE 1024
#define RAW_STUB_QUEUE_SIZE 10
#define RAW_STUB_QUEUE_TIMEOUT 500000000;   /* 500ms (greybus timeout @ 1s) */
const char queue_name[] = "/rawcmd_q";

/* Structure for message queue */
struct raw_stub_msg {
    uint16_t size;
    uint8_t payload[];
} __packed;


/**
 * Call the base device to send our data
 */
static int raw_stub_send(struct device *dev, uint32_t len, uint8_t data[])
{
    struct raw_stub_info *info = NULL;

    dbg("sending %d\n", len);
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);
    if (info->gCallback) {
        info->gCallback(dev, len, data);
    }
    return OK;
}

/**
  * Blocking thread to perform work
  */
static void *raw_stub_thread(void *arg) {
    struct raw_stub_info *info = NULL;
    struct raw_stub_msg *smsg = NULL;
    int status;

    info = arg;
    if (!info || !(info->cmd_queue)) {
        return (void *)(-EINVAL);
    }
    dbg("raw_stub_thread started\n");

    /* While thread is active */
    while(1) {
        status = mq_receive(info->cmd_queue, &smsg, sizeof(void *), 0);

        if (status > 0) {
            /**
             *  Do something interesting here!
             */
            raw_stub_send(info->gDevice, smsg->size, smsg->payload);
        }
        if (smsg) {
            /* Free the message we allocated in raw_stub_recv */
            free(smsg);
        }
    }

    dbg("thread exiting\n");
    return 0;
}

/**
 * We received data from the device (phone) side.
 */
static int raw_stub_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    struct raw_stub_info *info = NULL;
    uint8_t *msg;
    struct raw_stub_msg *smsg;
    size_t msg_size;
    struct timespec tm;

    if (len == 0) {
        return -EINVAL;
    }
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);
    if (info->cmd_queue == NULL) {
        return -EPIPE;
    }

    if (len > MAX_MSG_SIZE) {
        /* Message too long, truncate */
        len = MAX_MSG_SIZE;
    }

    /* Allocate memory for the message, freed in the raw_stub_thread */
    msg_size = sizeof(struct raw_stub_msg) + len;
    msg = zalloc(msg_size);
    if (!msg) {
        dbg("Could not allocate msg memory in thread, exiting\n");
        return -ENOMEM;
    }
    smsg = (struct raw_stub_msg *)msg;  /* Cast to our msg structure */
    smsg->size = len;
    memcpy(&(smsg->payload[0]), data, len); /* Copy the payload into msg */

    /* Cannot block forever, calculate timeout */
    clock_gettime(CLOCK_REALTIME, &tm);
    tm.tv_nsec += RAW_STUB_QUEUE_TIMEOUT;
    if (tm.tv_nsec >= NSEC_PER_SEC) {
        tm.tv_sec += 1;
        tm.tv_nsec -= NSEC_PER_SEC;
    }
    
    /* Add pointer to the message to queue for handling */
    if (mq_timedsend(info->cmd_queue, (char *)&msg, sizeof(void *), 0, &tm) < 0) {
        dbg("queue full, timeout occurred\n");
        free(msg); /* Cleanup if it didn't queue! */
    } else {
        dbg("msg length %d sent to worker\n", len);
    }

    return 0;
}

int raw_stub_register_callback(struct device *dev,
        raw_send_callback callback)
{
    struct raw_stub_info *info = NULL;
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);
    info->gCallback = callback;

    dbg("callback=0x%p\n", callback);

    return 0;
}

int raw_stub_unregister_callback(struct device *dev)
{
    struct raw_stub_info *info = NULL;
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);
    info->gCallback = NULL;

    dbg("callback deregistered\n");
    return 0;
}

int raw_stub_probe(struct device *dev)
{
    struct raw_stub_info *info = NULL;
    struct mq_attr q_attr;
    int ret;

    dbg("enter\n");
    if (!dev) {
        dbg("No device, probe failed\n");
        return -EINVAL;
    }
    info = zalloc(sizeof(*info));
    if (!info) {
        dbg("info alloc failed\n");
        return -ENOMEM;
    }
    info->gDevice = dev;
    info->gCallback = NULL;

    /* Set the queue length and message size */
    q_attr.mq_maxmsg = RAW_STUB_QUEUE_SIZE;
    q_attr.mq_msgsize = sizeof(struct raw_stub_msg) + MAX_MSG_SIZE;
    info->cmd_queue = mq_open(queue_name, O_CREAT|O_RDWR, 0, &q_attr);

    /* Save the info pointer for this device */
    device_set_private(dev, info);

    /* Create the pthread to handle messages */
    ret = pthread_create(&(info->cmd_thread), NULL, raw_stub_thread, info);
    if (ret != 0) {
        dbg("ERROR: Failed to create rx thread: %s.\n", strerror(errno));
        device_set_private(dev, NULL);
        free(info);
        return ret;
    }

    dbg("raw_probe complete\n");

    return 0;
}

static struct device_raw_type_ops raw_stub_type_ops = {
    .recv = raw_stub_recv,
    .register_callback = raw_stub_register_callback,
    .unregister_callback = raw_stub_unregister_callback,
};

static struct device_driver_ops raw_stub_driver_ops = {
    .probe = raw_stub_probe,
    .type_ops = &raw_stub_type_ops,
};

struct device_driver raw_stub_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw_stub",
    .desc = "Raw Stub with Task/Queue",
    .ops = &raw_stub_driver_ops,
};
