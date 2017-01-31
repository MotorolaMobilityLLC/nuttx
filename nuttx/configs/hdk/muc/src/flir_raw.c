/*
 * Copyright (c) 2017 Motorola Mobility, LLC.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>
#include <nuttx/spi/spi.h>

#define PACKET_SIZE 164
#define PACKETS_PER_FRAME 60

#define LOCAL_GB_PAYLOAD_MAX 1024

#define LOCAL_DEBUG

typedef struct {
    struct device *raw;
    raw_send_callback raw_callback;
    bool started;
    pthread_t tid;
    struct spi_dev_s *spi;
    uint8_t result[PACKET_SIZE * PACKETS_PER_FRAME];
} common_data_t;

static common_data_t s_data;

static bool flir_start(void);
static void flir_stop(void);

static int _recv(struct device *dev, uint32_t len, uint8_t data[])
{
    data[len - 1] = '\0';

    if (!strcmp((const char *)data, "on")) {
        flir_start();
    } else if (!strcmp((const char *)data, "off")) {
        flir_stop();
    } else {
        lldbg("Unsupported command :%s\n", data);
    }
    return 0;
}

#ifdef CONFIG_PM
static int _pm_prepare(struct pm_callback_s *cb,
                       enum pm_state_e pm_state)
{
    if (s_data.started)
        return -EBUSY;
    else
        return 0;
}

static struct pm_callback_s pm_callback =
{
    .prepare = _pm_prepare,
};
#endif

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

    memset(&s_data, 0, sizeof(s_data));
    s_data.raw = dev;

#ifdef CONFIG_PM
    pm_register(&pm_callback);
#endif

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
    s_data.raw = NULL;
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

struct device_driver mods_flir_raw_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "flir_raw",
    .desc = "Raw FLIR Control Interface",
    .ops = &_driver_ops,
};

static bool flir_setup(void)
{
    if (!s_data.spi) {
        struct spi_dev_s *spi;
        spi = up_spiinitialize(1);
        if (!spi) {
            lldbg("Failed to open SPI device\n");
            return false;
        }

        SPI_SETMODE(spi, SPIDEV_MODE3);
        SPI_SETBITS(spi, 8);
        SPI_SETFREQUENCY(spi, 10000000);

        s_data.spi = spi;
    }
    return true;
}

static void flir_cleanup(void)
{
    /* There looks to be easy way to uninitialize spi device */
}

static void *flir_loop(void *arg)
{
    lldbg("In FLIR loop\n");

    if (!flir_setup()) {
        lldbg("Failed to setup SPI port\n");
        return NULL;
    }

    while(s_data.started) {
        int i;
        int offset = 0;
        SPI_LOCK(s_data.spi, true);
        SPI_SELECT(s_data.spi, 0, true);
        for (i = 0; i < PACKETS_PER_FRAME; i++) {
            SPI_RECVBLOCK(s_data.spi,
                          &s_data.result[offset], PACKET_SIZE);
            uint8_t num1 = s_data.result[offset] & 0x0F;
            uint8_t num2 = s_data.result[offset + 1];
            if (num1  == 0x0F || i != num2) {
                /* discard. wait for frame sync */
                i = -1;
                offset = 0;
                up_udelay(1000);
            } else {
#ifdef LOCAL_DEBUG
                if (i == 0) {
                    lldbg("frame sync\n");
                }
#endif
                offset += PACKET_SIZE;
            }
        }
        SPI_SELECT(s_data.spi, 0, false);
        SPI_LOCK(s_data.spi, false);

        if (i == PACKETS_PER_FRAME) {
            lldbg("got frame\n");
            if (s_data.raw_callback) {
                int sofar = 0;
                while (sofar < offset) {
                    int len = offset - sofar;
                    if (len >= LOCAL_GB_PAYLOAD_MAX) {
                        len = LOCAL_GB_PAYLOAD_MAX;
                    }
                    s_data.raw_callback(s_data.raw, len, &s_data.result[sofar]);
                    up_udelay(1000);
                    sofar += len;
                }
            }
            lldbg("done sent\n");
        }

        /* intended delay */
        usleep(70000);
    }
    flir_cleanup();

    lldbg("Quitting FLIR loop\n");

    return NULL;
}

static bool flir_start(void)
{
    if (s_data.started) {
        lldbg("FLIR already started.\n");
        return true;
    }
    s_data.started = true;

    if (pthread_create(&s_data.tid, NULL, &flir_loop, NULL) != 0) {
        lldbg("Failed to start FLIR thread.\n");
        return false;
    }

    lldbg("FLIR thread started.\n");

    return true;
}

static void flir_stop(void)
{
    if (s_data.started) {
        s_data.started = false;
        pthread_join(s_data.tid, NULL);
    }

    lldbg("FLIR thread stopped.\n");
}
