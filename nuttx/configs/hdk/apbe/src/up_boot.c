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
#include <nuttx/device_table.h>

#include <tsb_scm.h>

#ifdef CONFIG_DEVICE_CORE
static struct device mod_devices[] = {
};

static struct device_table mod_device_table = {
    .device = mod_devices,
    .device_count = ARRAY_SIZE(mod_devices),
};
#endif

void tsb_boardinitialize(void) {
}

void board_initialize(void) {
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

#ifndef CONFIG_ARCH_CHIP_TSB_I2S
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
#endif
}
