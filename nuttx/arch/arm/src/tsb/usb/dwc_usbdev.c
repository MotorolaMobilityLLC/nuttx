/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/mm/mm.h>
#include <nuttx/util.h>

#include <arch/irq.h>

#include "dwc_os.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_pcd_if.h"
#include "dwc_otg_pcd.h"

#include "tsb_scm.h"

#include <nuttx/usb_device.h>

#define SNPSID_MASK 0xFFFFF000
#define SNPSID_OTG2 0x4F542000
#define SNPSID_OTG3 0x4F543000

#define DWC_NENDPOINTS        (16)
#define EP0                   (0)

struct dwc_usbdev_ep_s {
    struct usbdev_ep_s ep;
    struct usbdev_s *usbdev;
};

/*
 * Encompasses usbdev_req_s in dwc_req_s because we may need to add
 * some private datas.
 */
struct dwc_req_s {
    struct usbdev_req_s req;    /* Standard USB request */
};

struct dwc_usbdev_s {
    struct usbdev_s usbdev;     /* Nuttx pcd */
    struct usbdevclass_driver_s *driver;        /* Gadget driver */

    /* Bitmap: each allocated ep allocated is set to 1 */
    int allocated_ep;
    dwc_otg_device_t dwc_otg_device;
    struct dwc_usbdev_ep_s eplist[DWC_NENDPOINTS];
};
static struct dwc_usbdev_s g_usbdev;
#ifdef CONFIG_USBDEV
static struct device *g_device_usbdev;
#endif

/*
 * Configure/enable and disable endpoint
 */

int
dwc_configure(struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last)
{
    int retval;
    int epno;
    struct dwc_usbdev_ep_s *privep = (struct dwc_usbdev_ep_s *)ep;
    struct dwc_usbdev_s *priv = (struct dwc_usbdev_s *)privep->usbdev;

    epno = USB_EPNO(desc->addr);
    /* ep0 is already configured by the otg driver */
    if (epno == 0) {
        DWC_WARN("%s, bad ep(0)\n", __func__);
        return -EINVAL;
    }

    /*
     * Check FIFO size?
     */
    if (!GETUINT16(desc->mxpacketsize)) {
        DWC_WARN("%s, ep%d, bad maxpacket\n", __func__, epno);
        return -ERANGE;
    }

    retval = dwc_otg_pcd_ep_enable(priv->dwc_otg_device.pcd,
                                   (const uint8_t *)desc, (void *)ep);
    if (retval) {
        DWC_WARN("%s, dwc_otg_pcd_ep_enable failed\n", __func__);
        return -EINVAL;
    }

    ep->maxpacket = GETUINT16(desc->mxpacketsize);

    return OK;
}

int dwc_disable(struct usbdev_ep_s *ep)
{
    int retval;
    struct dwc_usbdev_ep_s *privep;
    struct dwc_usbdev_s *priv;

#ifdef CONFIG_DEBUG
    if (!ep) {
        DWC_WARN("%s, bad ep(0)\n", __func__);
        return -EINVAL;
    }
#endif
    privep = (struct dwc_usbdev_ep_s *)ep;
    priv = (struct dwc_usbdev_s *)privep->usbdev;

    retval = dwc_otg_pcd_ep_disable(priv->dwc_otg_device.pcd, ep);
    if (retval) {
        DWC_WARN("%s, dwc_otg_pcd_ep_disable failed\n", __func__);
        retval = -EINVAL;
    }

    return retval;
}

/*
 * Allocate and free I/O requests
 */

struct usbdev_req_s *dwc_allocreq(struct usbdev_ep_s *ep)
{
    struct dwc_req_s *privreq;

#ifdef CONFIG_DEBUG
    if (!ep) {
        DWC_WARN("%s, ep = %p\n", __func__, ep);
        return NULL;
    }
#endif

    privreq = (struct dwc_req_s *)DWC_ALLOC(sizeof(struct dwc_req_s));
    if (!privreq) {
        DWC_ERROR("%s, Fail to allocate request for ep%d\n", __func__,
                  USB_EPNO(ep->eplog));
        return NULL;
    }
    return &privreq->req;
}

void dwc_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    struct dwc_req_s *privreq = (struct dwc_req_s *)req;

#ifdef CONFIG_DEBUG
    if (!ep || !req) {
        DWC_WARN("%s, ep = %p, req = %p\n", __func__, ep, req);
        return;
    }
#endif

    DWC_FREE(privreq);
}

/*
 * Submit and cancel I/O requests
 */

int dwc_submit(struct usbdev_ep_s *usb_ep, struct usbdev_req_s *req)
{
    int zero;
    int retval;
    dwc_dma_t dma_addr;
    dwc_otg_pcd_t *pcd;
    struct dwc_usbdev_ep_s *privep;
    struct dwc_usbdev_s *priv;

    privep = (struct dwc_usbdev_ep_s *)usb_ep;
    priv = (struct dwc_usbdev_s *)privep->usbdev;
    pcd = priv->dwc_otg_device.pcd;

    req->result = -EINPROGRESS;
    req->xfrd = 0;

    dma_addr = (dwc_dma_t) req->buf;

    zero = (req->flags & USBDEV_REQFLAGS_NULLPKT) ? 1 : 0;
    retval = dwc_otg_pcd_ep_queue(pcd, usb_ep, req->buf, dma_addr,
                                  req->len, zero, req,
                                  up_interrupt_context()? 1 : 0);
    if (retval) {
        DWC_ERROR("%s, Can't queue request: ep = %p, req = %p\n",
                  __func__, usb_ep, req);
        return -EINVAL;
    }

    return OK;
}

int dwc_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    struct dwc_usbdev_ep_s *privep;
    struct dwc_usbdev_s *priv;

    privep = (struct dwc_usbdev_ep_s *)ep;
    priv = (struct dwc_usbdev_s *)privep->usbdev;

    if (!ep || !req) {
        DWC_WARN("%s, ep = %p, req = %p\n", __func__, ep, req);
        return -EINVAL;
    }
    if (dwc_otg_pcd_ep_dequeue(priv->dwc_otg_device.pcd, ep, req)) {
        DWC_ERROR("%s, Can't dequeue request: ep = %p, req = %p\n",
                  __func__, ep, req);
        return -EINVAL;
    }

    return OK;
}

/*
 * Stall or resume an endpoint
 */

int dwc_stall(struct usbdev_ep_s *ep, bool resume)
{
    int retval = 0;
    int value = resume ? 2 : 3;

    struct dwc_usbdev_ep_s *privep;
    struct dwc_usbdev_s *priv;

    privep = (struct dwc_usbdev_ep_s *)ep;
    priv = (struct dwc_usbdev_s *)privep->usbdev;

    if (!ep) {
        DWC_WARN("%s, ep = %p\n", __func__, ep);
        return -EINVAL;
    }

    retval = dwc_otg_pcd_ep_halt(priv->dwc_otg_device.pcd, ep, value);
    if (retval == -DWC_E_AGAIN) {
        DWC_WARN("Can't stall ep%d\n", USB_EPNO(ep->eplog));
        return -EAGAIN;
    } else if (retval) {
        DWC_ERROR("Can't stall ep%d\n", USB_EPNO(ep->eplog));
        retval = -EINVAL;
    }

    return retval;
}

#ifdef CONFIG_USBDEV_DMA
static void *dwc_epallocbuffer(struct usbdev_ep_s *ep, uint16_t size)
{
    dwc_dma_t dummy;

    return DWC_DMA_ALLOC(size, &dummy);
}
static void dwc_epfreebuffer(struct usbdev_ep_s *ep, void *buf)
{
    DWC_DMA_FREE(0, buf, (dwc_dma_t) NULL);
}
#endif

static const struct usbdev_epops_s g_epops = {
    .configure = dwc_configure,
    .disable = dwc_disable,
    .allocreq = dwc_allocreq,
    .freereq = dwc_freereq,
#ifdef CONFIG_USBDEV_DMA
    .allocbuffer = dwc_epallocbuffer,
    .freebuffer  = dwc_epfreebuffer,
#endif
    .submit = dwc_submit,
    .cancel = dwc_cancel,
    .stall = dwc_stall,
};

struct usbdev_ep_s *dwc_allocep(struct usbdev_s *dev, uint8_t epno,
                                bool in, uint8_t eptype)
{
    struct dwc_usbdev_s *priv;
    struct dwc_usbdev_ep_s *privep = NULL;

#ifdef CONFIG_DEBUG
    if (!dev) {
        DWC_WARN("%s, dev = %p", __func__, dev);
        return NULL;
    }
#endif
    priv = (struct dwc_usbdev_s *)dev;
    epno = USB_EPNO(epno);

    /*
     * A logical address of 0 means that any endpoint will do
     */
    if (epno > 0) {
        /*
         * Otherwise, we will return the endpoint structure only for the
         * requested 'logical' endpoint.  All of the other checks will still be
         * performed. First, verify that the logical endpoint is in the range
         * supported by by the hardware.
         */
        if (epno >= DWC_NENDPOINTS) {
            DWC_ERROR("%s, ep(%d) is out of range\n", __func__,
                      (uint16_t) epno);
            return NULL;
        }

        /*
         * Test if the endpoint is not already allocated,
         * otherwhise allocate it.
         */
        if ((priv->allocated_ep >> epno) & 1) {
            DWC_ERROR("%s, ep(%d) is already allocated\n",
                      __func__, (uint16_t) epno);
            return NULL;
        } else {
            priv->allocated_ep |= 1 << epno;
        }
    } else {
        int i;
        int allocated;

        /*
         * Store the allocated endpoints in bitmap.
         * When requested endpoint number is 0,
         * return the first free endpoint.
         */
        for (i = 1; i < DWC_NENDPOINTS; i++) {
            allocated = (priv->allocated_ep >> i) & 1;
            if (!allocated) {
                epno = i;
                priv->allocated_ep |= 1 << i;
                break;
            }
        }

        if (!epno) {
            DWC_ERROR("%s, Can't find free ep\n", __func__);
            return NULL;
        }
    }
    privep = &priv->eplist[epno];

    return &privep->ep;
}

void dwc_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
    int epno = USB_EPNO(ep->eplog);
    struct dwc_usbdev_s *priv = (struct dwc_usbdev_s *)dev;

    priv->allocated_ep &= ~(1 << epno);
}

int dwc_getframe(struct usbdev_s *dev)
{
    int fn;
    struct dwc_usbdev_s *priv = (struct dwc_usbdev_s *)dev;

#ifdef CONFIG_DEBUG
    if (!dev) {
        DWC_WARN("%s, dev = %p\n", __func__, dev);
        return -EINVAL;
    }
#endif
    fn = dwc_otg_pcd_get_frame_number(priv->dwc_otg_device.pcd);
    return fn;
}

int dwc_wakeup(struct usbdev_s *dev)
{
    struct dwc_usbdev_s *priv = (struct dwc_usbdev_s *)dev;

#ifdef CONFIG_DEBUG
    if (!dev) {
        DWC_WARN("%s, dev = %p\n", __func__, dev);
        return -EINVAL;
    }
#endif

    dwc_otg_pcd_wakeup(priv->dwc_otg_device.pcd);
    return OK;
}

int dwc_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
    /* TODO */
    return -1;
}

int dwc_usbpullup(struct usbdev_s *dev, bool enable)
{
    struct dwc_usbdev_s *priv = (struct dwc_usbdev_s *)dev;

    /* Start HSIC connect sequence */
    dwc_otg_set_hsic_connect(priv->dwc_otg_device.core_if, 1);

    return 0;
}

static const struct usbdev_ops_s g_devops = {
    .allocep = dwc_allocep,
    .freeep = dwc_freeep,
    .getframe = dwc_getframe,
    .wakeup = dwc_wakeup,
    .selfpowered = dwc_selfpowered,
    .pullup = dwc_usbpullup,
};

static int _setup(dwc_otg_pcd_t * pcd, uint8_t * bytes)
{
    int ret = -DWC_E_NOT_SUPPORTED;
    struct dwc_usbdev_s *priv;
    dwc_otg_device_t *otg_dev = pcd->otg_dev;
    priv = CONTAINER_OF(otg_dev, struct dwc_usbdev_s, dwc_otg_device);

    if (priv && priv->driver) {
        /*
         * Forward to the control request to the class driver implementation
         */

        ret = CLASS_SETUP(priv->driver, &priv->usbdev,
                          (struct usb_ctrlreq_s *)bytes, NULL, 0);
        if (ret == -EOPNOTSUPP) {
            ret = -DWC_E_NOT_SUPPORTED;
        } else if (ret < 0) {
            ret = -DWC_E_INVALID;
        }
    }
    return ret;
}

static int _complete(dwc_otg_pcd_t * pcd, void *ep_handle,
                     void *req_handle, int32_t status, uint32_t actual)
{
    struct usbdev_req_s *req = (struct usbdev_req_s *)req_handle;

    if (req && req->callback) {
        switch (status) {
        case -DWC_E_SHUTDOWN:
            req->result = -ESHUTDOWN;
            break;
        case -DWC_E_RESTART:
            req->result = -ECONNRESET;
            break;
        case -DWC_E_INVALID:
            req->result = -EINVAL;
            break;
        case -DWC_E_TIMEOUT:
            req->result = -ETIMEDOUT;
            break;
        case -DWC_E_DISCONNECT:
            req->result = -ECONNRESET;
        default:
            req->result = status;

        }

        req->xfrd = actual;
        DWC_SPINUNLOCK(pcd->lock);
        req->callback(ep_handle, req);
        DWC_SPINLOCK(pcd->lock);
    }

    return 0;
}

static int _connect(dwc_otg_pcd_t * pcd, int speed)
{
    struct dwc_usbdev_s *priv;

    dwc_otg_device_t *otg_dev = pcd->otg_dev;
    priv = CONTAINER_OF(otg_dev, struct dwc_usbdev_s, dwc_otg_device);
    priv->usbdev.speed = speed;
    return 0;
}

static int _disconnect(dwc_otg_pcd_t * pcd)
{
    struct dwc_usbdev_s *priv;

    dwc_otg_device_t *otg_dev = pcd->otg_dev;
    priv = CONTAINER_OF(otg_dev, struct dwc_usbdev_s, dwc_otg_device);
    CLASS_DISCONNECT(priv->driver, &priv->usbdev);

    return 0;
}

static int _resume(dwc_otg_pcd_t * pcd)
{
    struct dwc_usbdev_s *priv;

    dwc_otg_device_t *otg_dev = pcd->otg_dev;
    priv = CONTAINER_OF(otg_dev, struct dwc_usbdev_s, dwc_otg_device);
    CLASS_RESUME(priv->driver, &priv->usbdev);

    return 0;
}

static int _suspend(dwc_otg_pcd_t * pcd)
{
    struct dwc_usbdev_s *priv;

    dwc_otg_device_t *otg_dev = pcd->otg_dev;
    priv = CONTAINER_OF(otg_dev, struct dwc_usbdev_s, dwc_otg_device);
    CLASS_SUSPEND(priv->driver, &priv->usbdev);

    return 0;
}

/**
 * This function updates the otg values in the gadget structure.
 */
static int _hnp_changed(dwc_otg_pcd_t * pcd)
{
    /*
     * Don't want to support hnp
     */
    return 0;
}

static int _reset(dwc_otg_pcd_t * pcd)
{
    return 0;
}

static const struct dwc_otg_pcd_function_ops fops = {
    .complete = _complete,
    .setup = _setup,
    .disconnect = _disconnect,
    .connect = _connect,
    .resume = _resume,
    .suspend = _suspend,
    .hnp_changed = _hnp_changed,
    .reset = _reset,
};

int dwc_irq_handler(int irq, void *context)
{
    struct dwc_usbdev_s *priv = &g_usbdev;
    dwc_otg_pcd_t *pcd = priv->dwc_otg_device.pcd;
    dwc_otg_device_t *dev = &priv->dwc_otg_device;

    /*
     * TODO check return code
     */
    dwc_otg_handle_common_intr(dev);
    dwc_otg_pcd_handle_intr(pcd);

    return OK;
}

int up_usbinitialize_core(struct dwc_usbdev_s *priv)
{
    int retval = 0;
    dwc_otg_device_t *dwc_otg_device = &priv->dwc_otg_device;

    memset(dwc_otg_device, 0, sizeof(*dwc_otg_device));
    dwc_otg_device->os_dep.reg_offset = 0xFFFFFFFF;

    /*
     * Enable the DWC_otg clocks
     */
    tsb_clk_enable(TSB_CLK_HSIC480);
    tsb_clk_enable(TSB_CLK_HSICREF);
    tsb_clk_enable(TSB_CLK_HSICBUS);
    tsb_reset(TSB_RST_HSIC);
    tsb_reset(TSB_RST_HSICPHY);
    tsb_reset(TSB_RST_HSICPOR);

    /*
     * Map the DWC_otg Core memory
     */
    dwc_otg_device->os_dep.base = (void *)HSIC_BASE;
    dwc_otg_device->core_if = dwc_otg_cil_init(dwc_otg_device->os_dep.base);
    if (!dwc_otg_device->core_if) {
        DWC_ERROR("CIL initialization failed!\n");
        retval = -DWC_E_NO_MEMORY;
        goto fail;
    }

    /*
     * Attempt to ensure this device is really a DWC_otg Controller.
     * Read and verify the SNPSID register contents. The value should be
     * 0x45F42XXX or 0x45F42XXX, which corresponds to either "OT2" or "OTG3",
     * as in "OTG version 2.XX" or "OTG version 3.XX".
     */
    if (((dwc_otg_get_gsnpsid(dwc_otg_device->core_if) & SNPSID_MASK) !=
         SNPSID_OTG2)
        && ((dwc_otg_get_gsnpsid(dwc_otg_device->core_if) & SNPSID_MASK) !=
            SNPSID_OTG3)) {
        DWC_ERROR("Bad value for SNPSID: 0x%08x\n",
                  dwc_otg_get_gsnpsid(dwc_otg_device->core_if));
        retval = -DWC_E_UNKNOWN;
        goto fail;
    }

    /*
     * Validate parameter values.
     */
    if (set_parameters(dwc_otg_device->core_if)) {
        retval = -DWC_E_UNKNOWN;
        goto fail;
    }

    /*
     * Disable the global interrupt until all the interrupt
     * handlers are installed.
     */
    dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);

    /*
     * Install the interrupt handler for the common interrupts before
     * enabling common interrupts in core_init below.
     */
    DWC_DEBUGPL(DBG_CIL, "registering (common) handler for irq%d\n",
                TSB_IRQ_HSIC);
    if (irq_attach(TSB_IRQ_HSIC, dwc_irq_handler) != 0) {
        DWC_ERROR("Can't register HSIC IRQ\n");
        goto fail;
    } else {
        dwc_otg_device->common_irq_installed = 1;
    }

    /*
     * Initialize the DWC_otg core.
     */
    dwc_otg_core_init(dwc_otg_device->core_if);

    /*
     * Enable the global interrupt after all the interrupt
     * handlers are installed if there is no ADP support else
     * perform initial actions required for Internal ADP logic.
     */
    if (!dwc_otg_get_param_adp_enable(dwc_otg_device->core_if))
        dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);
    else
        dwc_otg_adp_start(dwc_otg_device->core_if,
                          dwc_otg_is_host_mode(dwc_otg_device->core_if));

    return 0;
 fail:
    return retval;
}

int up_usbinitialize_device(struct dwc_usbdev_s *priv)
{
    int epno;
    dwc_otg_device_t *otg_dev = &priv->dwc_otg_device;
    int retval = 0;
    struct usbdev_ep_s *ep0;

    otg_dev->pcd = dwc_otg_pcd_init(otg_dev->core_if);

    if (!otg_dev->pcd) {
        DWC_ERROR("dwc_otg_pcd_init failed\n");
        return -ENOMEM;
    }

    otg_dev->pcd->otg_dev = otg_dev;
    priv->usbdev.ops = &g_devops;
    priv->usbdev.ep0 = &priv->eplist[EP0].ep;

    /*
     * Initialize the endpoint list
     */

    for (epno = 0; epno < DWC_NENDPOINTS; epno++) {
        /*
         * Set endpoint operations, reference to driver structure (not really
         * necessary because there is only one controller), and the (physical)
         * endpoint number which is just the index to the endpoint.
         */

        priv->eplist[epno].ep.ops = &g_epops;
        priv->eplist[epno].usbdev = (struct usbdev_s *)priv;
        priv->eplist[epno].ep.eplog = epno;

        /*
         * We will use a fixed maxpacket size for all endpoints (perhaps ISOC
         * endpoints could have larger maxpacket???).  A smaller packet size can
         * be selected when the endpoint is configured.
         */

        priv->eplist[epno].ep.maxpacket = MAX_PACKET_SIZE;
    }

    /*
     * Initialize the endpoint 0
     */
    ep0 = &priv->eplist[0].ep;
    ep0->maxpacket = MAX_EP0_SIZE;
    dwc_otg_pcd_ep_enable(otg_dev->pcd, NULL, ep0);

    dwc_otg_pcd_start(otg_dev->pcd, &fops);

    return retval;
}

#define device_to_usbdev(dev) \
    (struct dwc_usbdev_s *)(device_get_private(dev))


static int tsb_usb_pcd_register_gadget(struct device *dev,
                                struct usbdevclass_driver_s *driver)
{
    int ret;
    struct dwc_usbdev_s *priv = device_to_usbdev(dev);

#ifdef CONFIG_DEBUG
    if (!driver || !driver->ops->bind || !driver->ops->unbind ||
        !driver->ops->disconnect || !driver->ops->setup) {
        DWC_ERROR("%s, Invalid params\n", __func__);
        return -EINVAL;
    }

    if (priv->driver) {
        return -EBUSY;
    }
#endif

    /*
     * First hook up the driver
     */
    priv->driver = driver;

    /*
     * Then bind the class driver
     */
    ret = CLASS_BIND(driver, &priv->usbdev);
    if (ret) {
        DWC_ERROR("%s, Can't bind usb device driver\n", __func__);
    } else {
        /*
         * Enable USB controller interrupts at the NVIC
         */
        up_enable_irq(TSB_IRQ_HSIC);

        priv->usbdev.speed = driver->speed;
    }
    return ret;
}

static int tsb_usb_pcd_unregister_gadget(struct device *dev,
                                  struct usbdevclass_driver_s *driver)
{
    /*
     * For now there is only one USB controller, but we will always refer to it
     * using a pointer to make any future ports to multiple USB controllers
     * easier.
     */
    struct dwc_usbdev_s *priv = device_to_usbdev(dev);
    irqstate_t flags;

#ifdef CONFIG_DEBUG
    if (driver != priv->driver) {
        return -EINVAL;
    }
#endif

    /*
     * Reset the hardware and cancel all requests.  All requests must be
     * canceled while the class driver is still bound.
     */

    flags = irqsave();

    /*
     * Unbind the class driver
     */
    CLASS_UNBIND(driver, &priv->usbdev);

    /*
     * Unhook the driver
     */
    priv->driver = NULL;

    irqrestore(flags);

    return OK;
}

static void tsb_usb_pcd_close(struct device *dev);
static int tsb_usb_pcd_open(struct device *dev)
{
    struct dwc_usbdev_s *priv = &g_usbdev;

    if (!dev) {
        return -EINVAL;
    }

    device_set_private(dev, &g_usbdev);
    SET_DEBUG_LEVEL(DBG_ANY);

    if (up_usbinitialize_core(priv))
        goto fail;
    if (up_usbinitialize_device(priv))
        goto fail;
    return 0;

fail:
    tsb_usb_pcd_close(dev);
    return -1;
}

static void tsb_usb_pcd_close(struct device *dev)
{
    struct dwc_usbdev_s *priv = device_to_usbdev(dev);
    dwc_otg_device_t *otg_dev = &priv->dwc_otg_device;
    irqstate_t flags;

    flags = irqsave();

    /*
     * Disable and detach the USB IRQs
     */
    if (otg_dev->common_irq_installed) {
        up_disable_irq(TSB_IRQ_HSIC);
        irq_detach(TSB_IRQ_HSIC);
    } else {
        DWC_DEBUGPL(DBG_ANY, "%s: There is no installed irq!\n", __func__);
        goto restore_irq;
    }

    if (otg_dev->core_if) {
        dwc_otg_cil_remove(otg_dev->core_if);
    } else {
        DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->core_if NULL!\n", __func__);
        goto restore_irq;
    }

    if (priv->driver) {
        device_usbdev_unregister_gadget(dev, priv->driver);
    }

    tsb_clk_disable(TSB_CLK_HSIC480);
    tsb_clk_disable(TSB_CLK_HSICREF);
    tsb_clk_disable(TSB_CLK_HSICBUS);
 restore_irq:
    irqrestore(flags);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_USBDEV
/****************************************************************************
 * Name: up_usbinitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void up_usbinitialize(void)
{
    g_device_usbdev = device_open(DEVICE_TYPE_USB_PCD, 0);
}

/****************************************************************************
 * Name: up_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void up_usbuninitialize(void)
{
    device_close(g_device_usbdev);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will
 *   be called to bind it to a USB device driver.
 *
 ****************************************************************************/
int usbdev_register(struct usbdevclass_driver_s *driver)
{
    return device_usbdev_register_gadget(g_device_usbdev, driver);
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/
int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
    return device_usbdev_unregister_gadget(g_device_usbdev, driver);
}
#endif

static struct device_usb_pcd_type_ops tsb_usb_pcd_type_ops = {
    .register_gadget = tsb_usb_pcd_register_gadget,
    .unregister_gadget = tsb_usb_pcd_unregister_gadget,
};

static struct device_driver_ops tsb_usb_pcd_driver_ops = {
    .open = tsb_usb_pcd_open,
    .close = tsb_usb_pcd_close,
    .type_ops = &tsb_usb_pcd_type_ops,
};

struct device_driver tsb_usb_pcd_driver = {
    .type = DEVICE_TYPE_USB_PCD,
    .name = "dwc2_pcd",
    .desc = "DWC2 USB 2.0 Device Controller Driver",
    .ops = &tsb_usb_pcd_driver_ops,
};
