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

#include <nuttx/config.h>

#include <errno.h>

#include <arch/chip/driver.h>
#include <arch/chip/gpio.h>
#include <arch/chip/device_table.h>

#include <nuttx/bufram.h>
#include <nuttx/device.h>
#include <nuttx/device_gpio_tunnel.h>
#include <nuttx/device_table.h>

#include <nuttx/mhb/device_mhb.h>
#include <nuttx/usb.h>

#include <reglog.h>
#include <tsb_fr_tmr.h>
#include <tsb_scm.h>

#ifdef CONFIG_DEVICE_CORE
static struct device_resource mhb_resources[] = {
    {
        .name   = "uart_dev_id",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 0,
        .count  = 1,
    },
    {
        .name   = "local_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 23,
        .count  = 1,
    },
    {
        .name   = "peer_wake",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 5,
        .count  = 1,
    },
};

#if CONFIG_TSB_GPIO_TUNNEL
static struct device_resource gpio_tunnel_resources[] = {
    {
        .name   = "rx_physical_0",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 8,
        .count  = 1,
    },
    {
        .name   = "rx_logical_0",
        .type   = DEVICE_RESOURCE_TYPE_GPIO,
        .start  = 0,
        .count  = 1,
    },
};
#endif

static struct device mod_devices[] = {
#if CONFIG_MHB_UART
    {
        .type = DEVICE_TYPE_MHB,
        .name = "mhb",
        .desc = "mhb",
        .id   = 0,
        .resources      = mhb_resources,
        .resource_count = ARRAY_SIZE(mhb_resources),
    },
#endif

#if CONFIG_TSB_GPIO_TUNNEL
    {
        .type = DEVICE_TYPE_GPIO_TUNNEL_HW,
        .name = "gpio_tunnel",
        .desc = "GPIO Tunnel",
        .id   = 0,
        .resources      = gpio_tunnel_resources,
        .resource_count = ARRAY_SIZE(gpio_tunnel_resources),
    },
#endif

#ifdef CONFIG_MODS_USB_HCD_ROUTER
    {
        .type = DEVICE_TYPE_HSIC_DEVICE,
        .name = "dummy_hsic",
        .desc = "Dummy HSIC Driver",
        .id   = 0,
    },
#endif

};

static struct device_table mod_device_table = {
    .device = mod_devices,
    .device_count = ARRAY_SIZE(mod_devices),
};
#endif

void tsb_boardinitialize(void) {
}

void board_initialize(void) {
    tsb_fr_tmr_init();
    /*
     * Initialize the register log.  (If not configured the function below
     * will be defined to nothing, so it will not run.)
     */
    reglog_initialize();

#if CONFIG_MM_BUFRAM_ALLOCATOR
    bufram_init();

    /*
     * Region's size must be power of 2, but the size of the full bufram is not
     * a power of 2, so split it into two regions: one of 64k and the other
     * one of 128k.
     */
    bufram_register_region(BUFRAM_BASE, 16);
    bufram_register_region(BUFRAM_BASE + (1 << 16), 17);
#endif

    tsb_gpio_register(0);
    tsb_gpio_initialize();

#if (!defined(CONFIG_ARCH_CHIP_TSB_I2S) && !defined(CONFIG_ARCH_CHIP_TSB_I2S_TUNNEL))
    int ret = tsb_request_pinshare(TSB_PIN_ETM | TSB_PIN_GPIO16 | TSB_PIN_GPIO18 |
                                   TSB_PIN_GPIO19 | TSB_PIN_GPIO20);
    if (!ret) {
        tsb_clr_pinshare(TSB_PIN_ETM);
        tsb_set_pinshare(TSB_PIN_GPIO16);
        tsb_set_pinshare(TSB_PIN_GPIO18);
        tsb_set_pinshare(TSB_PIN_GPIO19);
        tsb_set_pinshare(TSB_PIN_GPIO20);
    }
#endif

#ifdef CONFIG_DEVICE_CORE
    tsb_device_table_register();
    tsb_driver_register();

    if (mod_device_table.device_count > 0) {
        device_table_register(&mod_device_table);
    }

#if CONFIG_MHB_UART
   extern struct device_driver mhb_driver;
   device_register_driver(&mhb_driver);
#endif

#if CONFIG_TSB_GPIO_TUNNEL
   extern struct device_driver gpio_tunnel_driver;
   device_register_driver(&gpio_tunnel_driver);
#endif

#ifdef CONFIG_MODS_USB_HCD_ROUTER
   extern struct device_driver dummy_hsic_driver;
   device_register_driver(&dummy_hsic_driver);
#endif

#endif
}
