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
#include <debug.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/bufram.h>
#include <nuttx/device.h>
#include <nuttx/usb_device.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include "hsic_test.h"

#define HSIC_TEST_VID 0x22B8
#define HSIC_TEST_PID 0xDEAD
#define HSIC_TEST_VER 0x0001
#define HSIC_TEST_LANG 0x0409 /* en-us */
#define HSIC_TEST_MANUFACTURE "Motorola"
#define HSIC_TEST_PRODUCE     "XMCS2000"
#define HSIC_TEST_UNKNOWN     "Unknown"

#define HSIC_TEST_EP_IN  1
#define HSIC_TEST_EP_OUT 2

#define HSIC_TEST_MAX_PAYLOAD (512*32)

#define HSIC_TEST_DELAY_ON_APBE 500000

//#define DEBUG 1

void *HSIC_ALLOC(uint32_t size) {
#ifdef CONFIG_DWC_USE_BUFRAM
    void *buf = bufram_alloc(size);

    if (buf)
        memset(buf, 0, size);

    return buf;
#else
    return zalloc(size);
#endif
}

void HSIC_FREE(void *ptr) {
#ifdef CONFIG_DWC_USE_BUFRAM
    bufram_free(ptr);
#else
    free(ptr);
#endif
}

struct hsic_test_data_s {
    struct device *pcddev;
    struct usbdev_s *usbdev;
    struct usbdevclass_driver_s driver;
    struct usbdev_ep_s *ep[3];
    struct usbdev_req_s *outreq;
    bool started;
};

struct hsic_test_data_s s_data;

static pthread_t s_thread;
static pthread_mutex_t run_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t run_cond = PTHREAD_COND_INITIALIZER;
static bool do_run = false;

static int _configure_ep(struct usb_epdesc_s *desc);
static void _unconfigure_ep(uint8_t epno);
static int _prepare_out_ep(uint8_t epno);

typedef void (*usb_callback)(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

static const struct usb_devdesc_s s_devdesc = {
    USB_SIZEOF_DEVDESC,         /* len */
    USB_DESC_TYPE_DEVICE,       /* type */
    { LSBYTE(0x0200), MSBYTE(0x0200) },   /* usb */
    USB_CLASS_PER_INTERFACE,    /* classid */
    0,                          /* subclass */
    0,                          /* protocol */
    64                  ,       /* maxpacketsize */
    { LSBYTE(HSIC_TEST_VID), MSBYTE(HSIC_TEST_VID) },  /* vendor */
    { LSBYTE(HSIC_TEST_PID), MSBYTE(HSIC_TEST_PID) },  /* product */
    { LSBYTE(HSIC_TEST_VER), MSBYTE(HSIC_TEST_VER) },  /* device */
    1,      /* iManufacturer idex */
    2,      /* iproduct idx */
    0,      /* serno idx */
    1       /* nconfigs */
};

static const struct usb_cfgdesc_s s_cfgdesc = {
    USB_SIZEOF_CFGDESC,         /* len */
    USB_DESC_TYPE_CONFIG,       /* type */
    {0, 0},                     /* totallen -- to be provided */
    1,                          /* ninterfaces */
    1,                          /* cfgvalue */
    0,                          /* icfg */
    USB_CONFIG_ATTR_ONE | USB_CONFIG_ATTR_SELFPOWER , /* attr */
    0                           /* mxpower */
};

static const struct usb_ifdesc_s s_ifdesc = {
    USB_SIZEOF_IFDESC,          /* len */
    USB_DESC_TYPE_INTERFACE,    /* type */
    0,                          /* ifno */
    0,                          /* alt */
    2,                          /* neps */
    USB_CLASS_VENDOR_SPEC,      /* classid */
    0,                          /* subclass */
    0,                          /* protocol */
    0,                          /* iif */
};

static const struct usb_epdesc_s s_epbulkoutdesc = {
    USB_SIZEOF_EPDESC,          /* len */
    USB_DESC_TYPE_ENDPOINT,     /* type */
    HSIC_TEST_EP_OUT,           /* addr */
    USB_EP_ATTR_XFER_BULK,      /* attr */
    {LSBYTE(512), MSBYTE(512)}, /* maxpacket */
    0                           /* interval */
};

static const struct usb_epdesc_s s_epbulkindesc = {
    USB_SIZEOF_EPDESC,          /* len */
    USB_DESC_TYPE_ENDPOINT,     /* type */
    (USB_DIR_IN | HSIC_TEST_EP_IN), /* addr */
    USB_EP_ATTR_XFER_BULK,      /* attr */
    {LSBYTE(512), MSBYTE(512)}, /* maxpacket */
    0                           /* interval */
};

#define CONFIG_DESC_SIZE (sizeof(s_cfgdesc) + sizeof(s_ifdesc) + 2 * sizeof(struct usb_epdesc_s))
static uint8_t s_cfg[CONFIG_DESC_SIZE];

static int usb_ascii_to_utf16(uint8_t *utf16, const uint8_t *ascii, size_t len)
{
    int i, ndata;
    for (i = 0, ndata = 0; i < len; i++, ndata += 2) {
        utf16[ndata] = ascii[i];
        utf16[ndata + 1] = 0;
    }

    return ndata + 2;
}


static struct usbdev_req_s *alloc_request(struct usbdev_ep_s *ep,
                                          usb_callback callback,
                                          size_t len) {
    void *ptr = NULL;

    if (len) {
        ptr = bufram_alloc(len);
        if (!ptr) {
            lldbg("Failed to allocate EP data memory\n");
            return NULL;
        }
    }

    struct usbdev_req_s *req;

    req = EP_ALLOCREQ(ep);

    if (!req) {
        lldbg("Failed to allocate EP request\n");
        if (ptr)
            bufram_free(ptr);
        return NULL;
    }

    req->callback = callback;
    req->buf = ptr;
    req->len = len;

    return req;
}

static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req) {
    if (!req)
        return;

    if (req->buf != NULL) {
        bufram_free(req->buf);
        req->buf = NULL;
        req->len = 0;
    }
    EP_FREEREQ(ep, req);
}

static int _usbclass_bind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    lldbg("usbclass bind\n");
    s_data.usbdev = dev;
    DEV_SETSELFPOWERED(dev);

    return OK;
}

static void _usbclass_unbind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    lldbg("class unbind\n");

    _unconfigure_ep(HSIC_TEST_EP_IN);
    _unconfigure_ep(HSIC_TEST_EP_OUT);
    _unconfigure_ep(0);
}

static void req_ctrl_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req) {
    if (ep == NULL || req == NULL) {
        return;
    }
#if DEBUG
    lldbg("EP %d:  xfrd: status=%d xfrd=%d\n", ep->eplog, req->result, req->xfrd);
#endif
    free_request(ep, req);
}

static int handle_setconfig(uint16_t value) {
    if (value != 1) {
        lldbg("Invalid config %d set\n", value);
        return -EDOM;
    }

    /* Prepare one request for OUT EP */
    int ret;
    ret = _configure_ep((struct usb_epdesc_s *)&s_epbulkindesc);
    ret |= _configure_ep((struct usb_epdesc_s *)&s_epbulkoutdesc);
    ret |= _prepare_out_ep(HSIC_TEST_EP_OUT);

    return ret;
}

static int handle_get_descriptor(struct usbdev_req_s *req,
                                 uint16_t val, uint16_t idx, uint16_t len) {
    uint16_t desc_type = val >> 8 & 0xFF;
    uint16_t desc_idx = val & 0xFF;

    if (!req) {
        lldbg("No output buffer supplied\n");
        return -1;
    }

    int ret = 0;

    switch (desc_type) {
    case USB_DESC_TYPE_DEVICE:
        memcpy(req->buf, &s_devdesc, sizeof(s_devdesc));
        break;
    case USB_DESC_TYPE_CONFIG: {
        uint8_t *ptr = s_cfg;
        memcpy(ptr, &s_cfgdesc, sizeof(s_cfgdesc));
        ptr += sizeof(s_cfgdesc);
        memcpy(ptr, &s_ifdesc, sizeof(s_ifdesc));
        ptr += sizeof(s_ifdesc);
        memcpy(ptr, &s_epbulkindesc, sizeof(s_epbulkindesc));
        ptr += sizeof(s_epbulkindesc);
        memcpy(ptr, &s_epbulkoutdesc, sizeof(s_epbulkoutdesc));

        struct usb_cfgdesc_s *cfg = (struct usb_cfgdesc_s *)s_cfg;
        cfg->totallen[0] = LSBYTE(CONFIG_DESC_SIZE);
        cfg->totallen[1] = MSBYTE(CONFIG_DESC_SIZE);

        lldbg("desc size = %d\n", CONFIG_DESC_SIZE);

        memcpy(req->buf, s_cfg, len);
        break;
    }
    case USB_DESC_TYPE_STRING: {
        struct usb_strdesc_s *strdesc = (struct usb_strdesc_s *)req->buf;
        strdesc->type = USB_DESC_TYPE_STRING;
        if (desc_idx == 0) {
            /* return the list of supported LANGID */
            strdesc->len = 4;
            strdesc->data[0] = LSBYTE(0x0409);
            strdesc->data[1] = MSBYTE(0x0409);
        } else {
            const char *str;
            if (desc_idx == 1) {
                str = HSIC_TEST_MANUFACTURE;
            } else if (desc_idx == 2) {
                str = HSIC_TEST_PRODUCE;
            } else {
                str = HSIC_TEST_UNKNOWN;
            }
            strdesc->len = usb_ascii_to_utf16(strdesc->data,
                                              (uint8_t *)str,
                                              strlen(str));
        }
        req->len = strdesc->len;
        break;
    }
    default:
        lldbg("Unsupported descriptor type %d\n", desc_type);
        ret = -1;
        break;
    }

    return ret;
}

static int _usbclass_setup(struct usbdevclass_driver_s *driver, struct usbdev_s *dev,
                           const struct usb_ctrlreq_s *ctrl, uint8_t * dataout, size_t outlen) {

    uint16_t len = GETUINT16(ctrl->len);
    uint16_t val = GETUINT16(ctrl->value);
    uint16_t idx = GETUINT16(ctrl->index);

#if DEBUG
    lldbg("setup: type=%x, req=%02x, val=%04x, idx=%04x, len=%d\n",
          ctrl->type, ctrl->req, val, idx, len);
#endif

    if ((ctrl->type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD) {
        /* Do not support non-standard request */
        lldbg("Non-standard request not supported.\n");
        return -EOPNOTSUPP;
    }

    /* Allocate with potential max. */
    struct usbdev_req_s *req = NULL;

    req = alloc_request(dev->ep0, &req_ctrl_callback, len);

    int ret = 0;
    switch (ctrl->req) {
    case USB_REQ_GETDESCRIPTOR:
        ret = handle_get_descriptor(req, val, idx, len);
        break;
    case USB_REQ_SETCONFIGURATION:
        ret = handle_setconfig(val);
        break;
    case USB_REQ_GETCONFIGURATION:
        if (req) {
            *(uint8_t *) req->buf = 1;
            req->len = 1;
        }
        break;
    case USB_REQ_SETINTERFACE:
        break;
    case USB_REQ_GETINTERFACE:
        if (req) {
            if (idx != 0)
                ret = -EDOM;
            else {
                *(uint8_t *) req->buf = 0;
                req->len = 1;
            }
        }
        break;
    default:
        ret = -1;
        break;
    }

    if (ret == 0 && req) {
        req->flags = USBDEV_REQFLAGS_NULLPKT;
        ret = EP_SUBMIT(dev->ep0, req);
    }

    if (ret < 0)
        req_ctrl_callback(dev->ep0, req);

    return ret;
}

static void _usbclass_disconnect(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
}

/* USB class driver operations */
static const struct usbdevclass_driverops_s s_driverops = {
    _usbclass_bind,             /* bind */
    _usbclass_unbind,           /* unbind */
    _usbclass_setup,            /* setup */
    _usbclass_disconnect,       /* disconnect */
    NULL,                       /* suspend */
    NULL,                       /* resume */
};

static void _connect(void) {
    DEV_CONNECT(s_data.usbdev);
}

static int _configure_ep(struct usb_epdesc_s *desc) {
    int in;
    uint8_t epno;
    struct usbdev_ep_s *ep;
    int ret;

    in = desc->addr & USB_DIR_IN ? 1 : 0;
    epno = desc->addr & USB_EPNO_MASK;

    ep = DEV_ALLOCEP(s_data.usbdev, epno, in, desc->type);
    if (!ep) {
        lldbg("Failed to allocate EP %d\n", epno);
        return -ENODEV;
    }
    ep->priv = desc;

    ret = EP_CONFIGURE(ep, desc, false);
    if (ret) {
        lldbg("Failed to configure EP %d - ret %d\n", epno, ret);
        goto failout;
    }

    s_data.ep[epno] = ep;

    lldbg("EP :%d - configured - eqlog = %d\n", epno, ep->eplog);

    return 0;
failout:
    DEV_FREEEP(s_data.usbdev, ep);
    return ret;
}


static void _unconfigure_ep(uint8_t epno) {
    struct usbdev_ep_s *ep;

    if (epno == 0)
        ep = s_data.usbdev->ep0;
    else
        ep = s_data.ep[epno];

    /* Call below will force complete all queued request to
     * complete with ESHUTDOWN */
    EP_DISABLE(ep);

    EP_CANCEL(ep, s_data.outreq);
    free_request(ep, s_data.outreq);

    if (epno != 0)
        DEV_FREEEP(s_data.usbdev, ep);
}

static void req_in_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req) {
    if (ep == NULL || req == NULL) {
        return;
    }
#if DEBUG
    lldbg("EP %d xfrd: status=%d xfrd=%d\n", ep->eplog, req->result, req->xfrd);
#endif

    free_request(ep, req);
}

static void req_out_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req) {
    if (ep == NULL || req == NULL) {
        return;
    }
#if DEBUG
    lldbg("EP %d OUT status=%d xfrd=%d\n", ep->eplog, req->result, req->xfrd);
#endif

    if (!s_data.started)
        return;

    if (req->result == OK && req->xfrd) {
        struct usbdev_req_s *inreq = alloc_request(s_data.ep[HSIC_TEST_EP_IN],
                                                   &req_in_callback, req->xfrd);

        if (inreq) {
            memcpy(inreq->buf, req->buf, req->xfrd);
            int ret = EP_SUBMIT(s_data.ep[HSIC_TEST_EP_IN], inreq);
            if (ret != 0) {
                lldbg("Failed to loopback URB to EP IN\n");
            }
        } else {
            lldbg("Failed to allocate IN req\n");
        }
    } else {
        lldbg("callback fail %d, %d\n", req->result, req->xfrd);
    }

    req->xfrd = 0;
    req->result = 0;
    if (EP_SUBMIT(ep, req)) {
        lldbg("Failed to re-queue URB for EP OUT\n");
    }
}

static int _prepare_out_ep(uint8_t epno) {

    struct usbdev_req_s *req = alloc_request(s_data.ep[epno],
                                             &req_out_callback, HSIC_TEST_MAX_PAYLOAD);
    if (!req) {
        return -ENOMEM;
    }
    s_data.outreq = req;

    int ret = EP_SUBMIT(s_data.ep[epno], req);
    if (ret) {
        lldbg("Failed to submit URB for EP %d\n", epno);
        free_request(s_data.ep[epno], req);
    } else {
        lldbg("EP %d - prepared buffer\n", epno);
    }

    return ret;
}

static void pcd_start(void) {
    s_data.pcddev = device_open(DEVICE_TYPE_USB_PCD, 0);
    if (!s_data.pcddev) {
        lldbg("Faied to open PCD device\n");
        return;
    }

    s_data.driver.speed = USB_SPEED_HIGH;
    s_data.driver.ops = &s_driverops;

    int ret = device_usbdev_register_gadget(s_data.pcddev, &s_data.driver);
    if (ret) {
        lldbg("Failed to register pcd_router_driver\n");
        goto devclose;
    }

    s_data.started = true;

    return;

devclose:
    device_close(s_data.pcddev);
    s_data.pcddev = NULL;
}

static void pcd_stop(void) {
    s_data.started = false;

    device_usbdev_unregister_gadget(s_data.pcddev, &s_data.driver);
    device_close(s_data.pcddev);
    s_data.pcddev = NULL;
}

static void *hsic_test_thread(void *arg) {
    memset(&s_data, 0, sizeof(s_data));

    pcd_start();

    lldbg("HISC initialized\n");

#if CONFIG_ARCH_BOARD_HDK_APBE
    usleep(HSIC_TEST_DELAY_ON_APBE);
#endif

    _connect();

    lldbg("HISC send connect\n");

    pthread_mutex_lock(&run_lock);
    if (do_run) {
        pthread_cond_wait(&run_cond, &run_lock);
    }
    pthread_mutex_unlock(&run_lock);

    pcd_stop();

    lldbg("HSIC  stopped\n");

    return NULL;
}

int hsic_test_init(void) {
    pthread_mutex_lock(&run_lock);
    if (do_run) {
        /* already running. Just return */
        pthread_mutex_unlock(&run_lock);
        return 0;
    }
    do_run = true;
    pthread_mutex_unlock(&run_lock);

    int ret =  pthread_create(&s_thread, NULL, hsic_test_thread, NULL);
    if (ret) {
        lldbg("PCD router init failed\n");
        return ret;
    }

    return 0;
}

void hsic_test_uninit(void) {
    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        /* Not running. Just return */
        pthread_mutex_unlock(&run_lock);
        return;
    }
    do_run = false;
    pthread_cond_signal(&run_cond);
    pthread_mutex_unlock(&run_lock);

    pthread_join(s_thread, NULL);

    return;
}

