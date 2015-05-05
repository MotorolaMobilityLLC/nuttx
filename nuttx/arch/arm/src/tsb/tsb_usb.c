/*
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

#include <nuttx/arch.h>
#include <nuttx/usb.h>
#include <nuttx/mm/mm.h>

#include <errno.h>
#include <stdio.h>

#include "tsb_scm.h"
#include "up_arch.h"

#include "usb/dwc_otg_hcd_if.h"
#include "usb/dwc_otg_hcd.h"
#include "usb/dwc_otg_driver.h"

#define REG_OFFSET 0xFFFFFFFF

#define TSB_SYSCTL_USBOTG_HSIC_CONTROL  (SYSCTL_BASE + 0x500)
#define TSB_HSIC_DPPULLDOWN             (1 << 0)
#define TSB_HSIC_DMPULLDOWN             (1 << 1)

static dwc_otg_device_t *g_dev;
static struct dwc_otg_hcd_function_ops hcd_fops;
struct mm_heap_s g_usb_dma_heap;

/**
 * HSIC IRQ Handler
 *
 * @param irq IRQ number
 * @param context context of the preempted task
 * @return 0 if successful
 */
static int hsic_irq_handler(int irq, void *context)
{
    DEBUGASSERT(g_dev);
    DEBUGASSERT(g_dev->hcd);

    dwc_otg_handle_common_intr(g_dev);
    return dwc_otg_hcd_handle_intr(g_dev->hcd);
}

/**
 * Initialize the USB HCD controller
 * @return 0 if succesful
 */
static int hcd_init(void)
{
    int retval;

    DEBUGASSERT(g_dev);
    DEBUGASSERT(!g_dev->hcd);
    DEBUGASSERT(g_dev->core_if);

    g_dev->hcd = dwc_otg_hcd_alloc_hcd();
    if (!g_dev->hcd) {
        return -ENOMEM;
    }

    retval = dwc_otg_hcd_init(g_dev->hcd, g_dev->core_if);
    if (retval) {
        goto error_hcd_init;
    }

    g_dev->hcd->otg_dev = g_dev;
    dwc_otg_hcd_set_priv_data(g_dev->hcd, NULL);

    return 0;

error_hcd_init:
    dwc_otg_hcd_remove(g_dev->hcd);
    g_dev->hcd = NULL;
    return retval;
}

/**
 * Initialize the USB HCD core
 *
 * @return 0 if successful
 */
static int hcd_core_init(void)
{
    int retval;

    DEBUGASSERT(!g_dev);

    g_dev = zalloc(sizeof(*g_dev));
    if (!g_dev) {
        return -ENOMEM;
    }

    g_dev->os_dep.reg_offset = REG_OFFSET;
    g_dev->os_dep.base = (void *) HSIC_BASE;
    g_dev->common_irq_installed = 1;

    g_dev->core_if = dwc_otg_cil_init(g_dev->os_dep.base);
    if (!g_dev->core_if) {
        retval = -EIO;
        goto error_cil_init;
    }

    if (set_parameters(g_dev->core_if)) {
        retval = -EIO;
        goto error_set_parameter;
    }

    dwc_otg_disable_global_interrupts(g_dev->core_if);

    dwc_otg_core_init(g_dev->core_if);

    retval = hcd_init();
    if (retval) {
        goto error_hcd_init;
    }

    if (dwc_otg_get_param_adp_enable(g_dev->core_if)) {
        dwc_otg_adp_start(g_dev->core_if, dwc_otg_is_host_mode(g_dev->core_if));
    } else {
        dwc_otg_enable_global_interrupts(g_dev->core_if);
    }

    irq_attach(TSB_IRQ_HSIC, hsic_irq_handler);
    up_enable_irq(TSB_IRQ_HSIC);

    return 0;

error_hcd_init:
error_set_parameter:
    dwc_otg_cil_remove(g_dev->core_if);
error_cil_init:
    free(g_dev);
    g_dev = NULL;

    return retval;
}

/**
 * Initialize USB driver
 *
 * Clock the USB IP, put the usb hub under reset and initialize the driver
 *
 * @param dev: usb host device
 * @return 0 if successful
 */
static int tsb_usb_hcd_open(struct device *dev)
{
    int retval;

    if (!dev) {
        return -EINVAL;
    }

    dev->private = device_open(DEVICE_TYPE_HSIC_DEVICE, 0);
    if (!dev->private) {
        return -EINVAL;
    }

    retval = device_hsic_hold_reset(dev->private);
    if (retval) {
        goto error_hsic_hold_reset;
    }

    mm_initialize(&g_usb_dma_heap, (void *)BUFRAM2_BASE, BUFRAM_BANK_SIZE);
    mm_addregion(&g_usb_dma_heap, (void *)BUFRAM3_BASE, BUFRAM_BANK_SIZE);

    putreg32(TSB_HSIC_DPPULLDOWN | TSB_HSIC_DMPULLDOWN,
             TSB_SYSCTL_USBOTG_HSIC_CONTROL);

    tsb_clk_enable(TSB_CLK_HSIC480);
    tsb_clk_enable(TSB_CLK_HSICBUS);
    tsb_clk_enable(TSB_CLK_HSICREF);

    tsb_reset(TSB_RST_HSIC);
    tsb_reset(TSB_RST_HSICPHY);
    tsb_reset(TSB_RST_HSICPOR);

    tsb_clr_pinshare(TSB_PIN_UART_CTSRTS);

    retval = hcd_core_init();
    if (retval) {
        goto error_hcd_core_init;
    }

    return 0;

error_hcd_core_init:
    tsb_clk_disable(TSB_CLK_HSIC480);
    tsb_clk_disable(TSB_CLK_HSICBUS);
    tsb_clk_disable(TSB_CLK_HSICREF);

error_hsic_hold_reset:
    device_close(dev->private);

    return retval;
}

/**
 * Uninitialize USB driver
 *
 * Clean-up everything allocated by the driver, unclock the USB IP, and put
 * the usb hub under reset.
 *
 * @param dev: usb host device
 */
static void tsb_usb_hcd_close(struct device *dev)
{
    DEBUGASSERT(g_dev);
    DEBUGASSERT(g_dev->hcd);
    DEBUGASSERT(g_dev->core_if);

    up_disable_irq(TSB_IRQ_HSIC);
    irq_detach(TSB_IRQ_HSIC);

    dwc_otg_hcd_remove(g_dev->hcd);
    dwc_otg_cil_remove(g_dev->core_if);

    free(g_dev);
    g_dev = NULL;

    tsb_clk_disable(TSB_CLK_HSIC480);
    tsb_clk_disable(TSB_CLK_HSICBUS);
    tsb_clk_disable(TSB_CLK_HSICREF);

    if (dev && dev->private) {
        device_hsic_hold_reset(dev->private);
        device_close(dev->private);
    }
}

/**
 * Start the HCD
 *
 * @param dev: usb host device
 * @return 0 if successful
 */
static int hcd_start(struct device *dev)
{
    int retval;

    if (!dev || !dev->private) {
        return -EINVAL;
    }

    device_hsic_release_reset(dev->private);

    retval = dwc_otg_hcd_start(g_dev->hcd, &hcd_fops);
    if (retval) {
        goto error_hcd_start;
    }

    dwc_otg_set_hsic_connect(g_dev->hcd->core_if, 1);

    return 0;

error_hcd_start:
    device_hsic_hold_reset(dev->private);
    return retval;
}

/**
 * Stop the HCD
 *
 * @param dev: usb host device
 */
static void hcd_stop(struct device *dev)
{
    DEBUGASSERT(g_dev);
    DEBUGASSERT(g_dev->hcd);

    dwc_otg_hcd_stop(g_dev->hcd);

    if (dev && dev->private) {
        device_hsic_hold_reset(dev->private);
    }
}

/**
 * Send request to the root hub
 *
 * @param dev: usb host device
 * @param buf: buffer of length @a wLength where the response will get stored
 * @return 0 if successful
 *
 * @see USB Specification for the meaning of typeReq, wValue, wIndex, and
 * wLength
 */
static int hub_control(struct device *dev, uint16_t typeReq, uint16_t wValue,
                       uint16_t wIndex, char *buf, uint16_t wLength)
{
    DEBUGASSERT(g_dev);
    DEBUGASSERT(g_dev->hcd);

    return dwc_otg_hcd_hub_control(g_dev->hcd, typeReq, wValue, wIndex,
                                   (uint8_t*) buf, wLength);
}

static struct device_usb_hcd_type_ops tsb_usb_hcd_type_ops = {
    .start = hcd_start,
    .stop = hcd_stop,
    .hub_control = hub_control,
};

static struct device_driver_ops tsb_usb_hcd_driver_ops = {
    .open = tsb_usb_hcd_open,
    .close = tsb_usb_hcd_close,
    .type_ops.usb_hcd = &tsb_usb_hcd_type_ops,
};

struct device_driver tsb_usb_hcd_driver = {
    .type = DEVICE_TYPE_USB_HCD,
    .name = "dwc2_hcd",
    .desc = "DWC2 USB 2.0 Host Controller Driver",
    .ops = &tsb_usb_hcd_driver_ops,
};
