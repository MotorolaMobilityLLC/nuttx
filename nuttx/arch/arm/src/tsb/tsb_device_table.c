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
 * @author Mark Greer
 */

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/device_resource.h>
#include <nuttx/device_table.h>
#include <nuttx/device_pll.h>

#include "chip.h"

#ifdef CONFIG_ARCH_CHIP_DEVICE_PLL
#define TSB_PLLA_CG_BRIDGE_OFFSET    0x900
#define TSB_PLLA_SIZE                0x20

static struct device_resource tsb_plla_resources[] = {
    {
        .name   = "reg_base",
        .type   = DEVICE_RESOUCE_TYPE_REGS,
        .start  = SYSCTL_BASE + TSB_PLLA_CG_BRIDGE_OFFSET,
        .count  = TSB_PLLA_SIZE,
    },
};
#endif

static struct device tsb_device_table[] = {
#ifdef CONFIG_ARCH_CHIP_DEVICE_PLL
    {
        .type           = DEVICE_TYPE_PLL_HW,
        .name           = "tsb_pll",
        .desc           = "TSB PLLA Controller",
        .id             = 0,
        .resources      = tsb_plla_resources,
        .resource_count = ARRAY_SIZE(tsb_plla_resources),
    },
#endif
};

int tsb_device_table_register(void)
{
    return device_table_register(tsb_device_table,
                                 ARRAY_SIZE(tsb_device_table));
}
