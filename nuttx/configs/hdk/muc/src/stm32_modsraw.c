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

#include <nuttx/greybus/debug.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>

static struct device *gDevice;
static raw_send_callback gCallback;

/**
 * Call the base device to send our data
 */
int mods_raw_send(uint32_t len, uint8_t data[])
{
    if (gCallback)
      {
        gCallback(gDevice, len, data);
      }
    return OK;
}

/**
 * We got data from the device (phone) side.
 */
static int mods_raw_recv(struct device *dev, uint32_t len, uint8_t data[])
{
    if (len == 0)
        return -EINVAL;

    return 0;
}

int mods_raw_register_callback(struct device *dev,
        raw_send_callback callback)
{
    dbg("callback=0x%p\n", callback);

    gCallback = callback;
    return 0;
}

int mods_raw_unregister_callback(struct device *dev)
{
    gCallback = NULL;
    return 0;
}

int mods_raw_probe(struct device *dev)
{
    dbg("enter\n");

    gDevice = dev;

    return 0;
}

static struct device_raw_type_ops mods_raw_type_ops = {
    .recv = mods_raw_recv,
    .register_callback = mods_raw_register_callback,
    .unregister_callback = mods_raw_unregister_callback,
};

static struct device_driver_ops mods_raw_driver_ops = {
    .probe = mods_raw_probe,
    .type_ops = &mods_raw_type_ops,
};

struct device_driver mods_raw_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_raw",
    .desc = "Reference Raw Interface",
    .ops = &mods_raw_driver_ops,
};
