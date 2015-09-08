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
#include <nuttx/device_table.h>
#include <nuttx/device_pll.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_pwm.h>
#include <nuttx/device_spi.h>
#include <nuttx/device_uart.h>
#include <nuttx/usb.h>
#include <nuttx/usb_device.h>
#ifdef CONFIG_ARCH_CHIP_DEVICE_GDMAC
#include <nuttx/device_dma.h>
#endif

#include <arch/irq.h>

#include "chip.h"
#include "tsb_pwm.h"

#ifdef CONFIG_ARCH_CHIP_DEVICE_PLL
#define TSB_PLLA_CG_BRIDGE_OFFSET    0x900
#define TSB_PLLA_SIZE                0x20

static struct device_resource tsb_plla_resources[] = {
    {
        .name   = "reg_base",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = SYSCTL_BASE + TSB_PLLA_CG_BRIDGE_OFFSET,
        .count  = TSB_PLLA_SIZE,
    },
};
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_PWM
static struct device_resource tsb_pwm_resources[] = {
    {
        .name   = "pwm_base",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = TSB_PWM_BASE,
        .count  = 1,
    },
    {
        .name   = "pwm0",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = TSB_PWM0,
        .count  = 20,
    },
    {
        .name   = "pwm1",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = TSB_PWM1,
        .count  = 20,
    },
    {
        .name   = "pwmintr",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_PWM,
        .count  = 1,
    },
};
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_I2S
static struct device_resource tsb_i2s_resources_0[] = {
    {
        .name   = "cg_bridge",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = SYSCTL_BASE,
        .count  = SYSCTL_SIZE,
    },
    {
        .name   = "i2slp_sc",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = I2SLP_SC_BASE,
        .count  = I2SLP_SC_SIZE,
    },
    {
        .name   = "i2slp_so",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = I2SLP_SO_BASE,
        .count  = I2SLP_SO_SIZE,
    },
    {
        .name   = "i2slp_si",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = I2SLP_SI_BASE,
        .count  = I2SLP_SI_SIZE,
    },
    {
        .name   = "i2soerr",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_I2SOERR,
        .count  = 1,
    },
    {
        .name   = "i2so",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_I2SO,
        .count  = 1,
    },
    {
        .name   = "i2sierr",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_I2SIERR,
        .count  = 1,
    },
    {
        .name   = "i2si",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_I2SI,
        .count  = 1,
    },
};
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_SPI
static struct device_resource tsb_spi_resources[] = {
    {
        .name   = "reg_base",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = SPI_BASE,
        .count  = SPI_SIZE,
    },
};
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_UART
static struct device_resource tsb_uart_resources[] = {
    {
        .name   = "reg_base",
        .type   = DEVICE_RESOURCE_TYPE_REGS,
        .start  = UART_BASE,
        .count  = UART_SIZE,
    },
    {
        .name   = "irq_uart",
        .type   = DEVICE_RESOURCE_TYPE_IRQ,
        .start  = TSB_IRQ_UART,
        .count  = 1,
    },
};
#endif

static struct device tsb_devices[] = {
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
#ifdef CONFIG_ARCH_CHIP_DEVICE_I2S
    {
        .type           = DEVICE_TYPE_I2S_HW,
        .name           = "tsb_i2s",
        .desc           = "TSB I2S Controller",
        .id             = 0,
        .resources      = tsb_i2s_resources_0,
        .resource_count = ARRAY_SIZE(tsb_i2s_resources_0),
    },
#endif
#ifdef CONFIG_ARCH_CHIP_USB_HCD
    {
        .type           = DEVICE_TYPE_USB_HCD,
        .name           = "dwc2_hcd",
        .desc           = "DWC2 USB Host controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARCH_CHIP_USB_PCD
    {
        .type           = DEVICE_TYPE_USB_PCD,
        .name           = "dwc2_pcd",
        .desc           = "DWC2 USB Device controller",
        .id             = 0,
    },
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_PWM
    {
        .type           = DEVICE_TYPE_PWM_HW,
        .name           = "tsb_pwm",
        .desc           = "TSB PWM Controller",
        .id             = 0,
        .resources      = tsb_pwm_resources,
        .resource_count = ARRAY_SIZE(tsb_pwm_resources),
    },
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_SPI
    {
        .type           = DEVICE_TYPE_SPI_HW,
        .name           = "tsb_spi",
        .desc           = "TSB SPI master Controller",
        .id             = 0,
        .resources      = tsb_spi_resources,
        .resource_count = ARRAY_SIZE(tsb_spi_resources),
    },
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_UART
    {
        .type           = DEVICE_TYPE_UART_HW,
        .name           = "tsb_uart",
        .desc           = "TSB UART Controller",
        .id             = 0,
        .resources      = tsb_uart_resources,
        .resource_count = ARRAY_SIZE(tsb_uart_resources),
    },
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_GDMAC
    {
        .type           = DEVICE_TYPE_DMA_HW,
        .name           = "tsb_dma",
        .desc           = "TSB DMA Device",
        .id             = 0,
    },
#endif
};

static struct device_table tsb_device_table = {
    .device = tsb_devices,
    .device_count = ARRAY_SIZE(tsb_devices),
};

int tsb_device_table_register(void)
{
    return device_table_register(&tsb_device_table);
}
