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

#include "tsb_scm.h"
#include "up_arch.h"

#include <arch/chip/gpio.h>

#include <arch/tsb/gpio.h>
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

static int io_expander_init(void)
{
    void *driver_data;
    struct i2c_dev_s *dev;

    dev = up_i2cinitialize(0);
    if (!dev) {
        lowsyslog("%s(): Failed to get I/O Expander I2C bus 0\n", __func__);
        return -ENODEV;
    } else {
        if (tca64xx_init(&driver_data,
                         TCA6408_PART,
                         dev,
                         TCA6408_U72,
                         TCA6408_U72_RST_GPIO,
                         TCA6408_U72_INT_GPIO,
                         TCA6408_GPIO_BASE) < 0) {
            lowsyslog("%s(): Failed to register I/O Expander(0x%02x)\n",
                      __func__, TCA6408_U72);
            up_i2cuninitialize(dev);
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
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB3813
    {
        .type           = DEVICE_TYPE_HSIC_DEVICE,
        .name           = "usb3813",
        .desc           = "USB3813 HSIC Hub",
        .id             = 1,
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
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB3813
    extern struct device_driver usb3813_driver;
    device_register_driver(&usb3813_driver);
#endif
}
#endif

static void board_display_init(void)
{
#ifdef CONFIG_BOARD_HAVE_DISPLAY
    display_init();
#endif
}

static void board_camera_init(void)
{
#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
    sleep(1);

    camera_init();
#endif
}

static void sdb_fixups(void)
{
    /**
     * DETECT_IN is not working on both GPBridges on the SDB. The workaround
     * is to pull up GPIO24.
     *
     * Documentation related to this fix (items 33 and 44)
     * https://docs.google.com/spreadsheets/d/1BBVHjFZu6GEUDCua8WlXHl9TmGYdpUwQXF82NXWEI-o/edit#gid=779323147
     *
     * This change will have no impact on BDB2{A,B} since the GPIO24 is
     * only connected to a test point.
     */
    if (tsb_get_product_id() == tsb_pid_gpbridge) {
        modifyreg32(TSB_IO_PULL_UPDOWN_ENABLE0, TSB_IO_PULL_UPDOWN_GPIO(24), 0);
        modifyreg32(TSB_IO_PULL_UPDOWN0, 0, TSB_IO_PULL_UPDOWN_GPIO(24));
    }
}

void ara_module_early_init(void)
{
    tsb_gpio_register(NULL);
#ifdef CONFIG_BOARD_HAVE_DISPLAY
#ifndef CONFIG_APBRIDGEA
    /* IO expander init is required by ps_hold */
    io_expander_init();
#endif
#endif
}

void ara_module_init(void)
{
    sdb_fixups();

#ifdef CONFIG_DEVICE_CORE
    device_table_register(&bdb_device_table);
    bdb_driver_register();
#endif

    board_display_init();
    board_camera_init();
}
