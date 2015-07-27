/**
 * Copyright (c) 2015 Google Inc.
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
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/util.h>
#include <nuttx/usb.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio/tca64xx.h>

#include <arch/tsb/gpio.h>
#include <arch/tsb/device_table.h>
#include <arch/tsb/driver.h>

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#include <arch/board/dsi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
#include <arch/board/csi.h>
#endif

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#define TCA6408_U72             0x20
#define TCA6408_U72_INT_GPIO    0x03
#define TCA6408_U72_RST_GPIO    0x04

static int io_expander_init(struct i2c_dev_s **dev)
{
    void *driver_data;

    *dev = up_i2cinitialize(0);
    if (!*dev) {
        lowsyslog("%s(): Failed to get I/O Expander I2C bus 0\n", __func__);
        return -ENODEV;
    } else {
        if (tca64xx_init(&driver_data,
                         TCA6408_PART,
                         *dev,
                         TCA6408_U72,
                         TCA6408_U72_RST_GPIO,
                         TCA6408_U72_INT_GPIO,
                         TCA6408_GPIO_BASE) < 0) {
            lowsyslog("%s(): Failed to register I/O Expander(0x%02x)\n",
                      __func__, TCA6408_U72);
            up_i2cuninitialize(*dev);
            return -ENODEV;
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_DEVICE_CORE
static struct device devices[] = {
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB4624
    {
        .type           = DEVICE_TYPE_HSIC_DEVICE,
        .name           = "usb4624",
        .desc           = "USB4624 HSIC Hub",
        .id             = 0,
    },
#endif
};

static struct device_table bdb_device_table = {
    .device = devices,
    .device_count = ARRAY_SIZE(devices),
};

static void bdb_driver_register(void)
{
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB4624
    extern struct device_driver usb4624_driver;
    device_register_driver(&usb4624_driver);
#endif
}
#endif

static void board_display_init(void)
{
#ifdef CONFIG_BOARD_HAVE_DISPLAY
    struct i2c_dev_s *dev = NULL;

#ifndef CONFIG_APBRIDGEA
    io_expander_init(&dev);
#endif

    display_init(dev);
#endif
}

static void board_camera_init(void)
{
#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
    sleep(1);

    camera_init();
#endif
}

void board_initialize(void)
{
    tsb_gpio_register(NULL);

#ifdef CONFIG_DEVICE_CORE
    tsb_device_table_register();
    device_table_register(&bdb_device_table);

    tsb_driver_register();
    bdb_driver_register();
#endif

    board_display_init();
    board_camera_init();
}
