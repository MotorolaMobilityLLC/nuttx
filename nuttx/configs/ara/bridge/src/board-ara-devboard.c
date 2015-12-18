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
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/device_hid.h>
#include <nuttx/util.h>
#include <nuttx/usb.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio/tca64xx.h>
#include <nuttx/device_lights.h>

#include "tsb_scm.h"
#include "up_arch.h"

#include <arch/tsb/gpio.h>
#ifdef CONFIG_BOARD_HAVE_DISPLAY
#include <arch/board/dsi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
#include <arch/board/csi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_BATTERY
#include <nuttx/device_battery.h>
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
#include <nuttx/device_sdio_board.h>
#define SD_POWER_EN_PIN    9 /* GPIO 9 */
#define SD_CARD_DETECT_PIN 22 /* GPIO 22 */
#endif

#ifdef CONFIG_APBRIDGEA
/* must pull up or drive high on SDB APBridgeA to bring Helium out of reset */
#define HELIUM_EXT_NRST_BTN_GPIO 0
#endif

#ifdef CONFIG_BOARD_HAVE_DISPLAY
/* GPIO Chip base of the TCA6408 I/O Expander. Follows the TSB GPIOs */
#define TCA6408_GPIO_BASE           (TSB_GPIO_CHIP_BASE + tsb_nr_gpio())

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
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
static struct device_resource sdio_board_resources[] = {
    {
        .name  = "sdio_gpio_power",
        .type  = DEVICE_RESOURCE_TYPE_GPIO,
        .start = SD_POWER_EN_PIN,
        .count = 1,
    },
    {
        .name  = "sdio_gpio_cd",
        .type  = DEVICE_RESOURCE_TYPE_GPIO,
        .start = SD_CARD_DETECT_PIN,
        .count = 1,
    },
};
#endif

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
#ifdef CONFIG_ARA_BRIDGE_HAVE_BATTERY
    {
        .type           = DEVICE_TYPE_BATTERY_DEVICE,
        .name           = "ara_bridge_battery",
        .desc           = "Ara Bridge Battery Controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_HID_TOUCH
    {
        .type           = DEVICE_TYPE_HID_HW,
        .name           = "hid_touch",
        .desc           = "Multi-Touch HID Controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_LIGHTS
    {
        .type           = DEVICE_TYPE_LIGHTS_HW,
        .name           = "lights",
        .desc           = "Lights Device Controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
    {
        .type           = DEVICE_TYPE_SDIO_BOARD_HW,
        .name           = "sdio_board",
        .desc           = "SDIO Board Device Driver",
        .id             = 0,
        .resources      = sdio_board_resources,
        .resource_count = ARRAY_SIZE(sdio_board_resources),
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
#ifdef CONFIG_ARA_BRIDGE_HAVE_BATTERY
    extern struct device_driver batt_driver;
    device_register_driver(&batt_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_HID_TOUCH
    extern struct device_driver hid_touch_driver;
    device_register_driver(&hid_touch_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_LIGHTS
    extern struct device_driver lights_driver;
    device_register_driver(&lights_driver);
#endif
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
    extern struct device_driver sdio_board_driver;
    device_register_driver(&sdio_board_driver);
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

    /**
     * When attached to the 96Boards Expansion Header on the SDB, Helium is
     * held in reset unless HELIUM_EXT_NRST_BTN_GPIO is pulled high or
     * driven high on APBridgeA.
     *
     * Rob Herring indicates that this behavior is the opposite of the
     * 96Boards specification, which would suggest active low.
     *
     * We'll pull the pin high, as that's less aggressive and avoids
     * the need to enable the GPIO subsystem at this point in the boot
     * sequence.
     *
     * This change should have no impact on BDB2{A,B} since on APBridgeA,
     * HELIUM_EXT_NRST_BTN_GPIO is only connected to a test point.
     */
#ifdef CONFIG_APBRIDGEA
    if (tsb_get_product_id() == tsb_pid_apbridge) {
        modifyreg32(TSB_IO_PULL_UPDOWN_ENABLE0,
                    TSB_IO_PULL_UPDOWN_GPIO(HELIUM_EXT_NRST_BTN_GPIO),
                    0);
        modifyreg32(TSB_IO_PULL_UPDOWN0,
                    0,
                    TSB_IO_PULL_UPDOWN_GPIO(HELIUM_EXT_NRST_BTN_GPIO));
    }
#endif
}

void ara_module_early_init(void)
{
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
