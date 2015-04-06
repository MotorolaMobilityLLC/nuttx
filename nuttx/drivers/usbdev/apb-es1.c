/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * drivers/usbdev/apb-es1.c
 *
 * Author: Alexandre Bailon <abailon@baylibre.com>
 * Based on pl2303 usb device driver
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>
#include <fcntl.h>

#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/apb_es1.h>
#include <nuttx/logbuffer.h>
#include <nuttx/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Logical endpoint numbers / max packet sizes */

#define CONFIG_APBRIDGE_EPINTIN 1

#define CONFIG_APBRIDGE_EPBULKOUT 2

#define CONFIG_APBRIDGE_EPBULKIN 3

/* Packet and request buffer sizes */

#define CONFIG_APBRIDGE_EP0MAXPACKET 64

/* Vendor and product IDs and strings */

#define CONFIG_APBRIDGE_VENDORSTR  "Toshiba"

#define CONFIG_APBRIDGE_PRODUCTSTR "APBridge"

#undef CONFIG_APBRIDGE_SERIALSTR
#define CONFIG_APBRIDGE_SERIALSTR "0"

#undef CONFIG_APBRIDGE_CONFIGSTR
#define CONFIG_APBRIDGE_CONFIGSTR "Bulk"

/* Descriptors ****************************************************************/

/* These settings are not modifiable via the NuttX configuration */

#define APBRIDGE_VERSIONNO           (0x0001)   /* Device version number */
#define APBRIDGE_CONFIGIDNONE        (0)        /* Config ID means to return to address mode */
#define APBRIDGE_CONFIGID            (1)        /* The only supported configuration ID */
#define APBRIDGE_NCONFIGS            (1)        /* Number of configurations supported */
#define APBRIDGE_INTERFACEID         (0)
#define APBRIDGE_ALTINTERFACEID      (0)
#define APBRIDGE_NINTERFACES         (1)        /* Number of interfaces in the configuration */
#define APBRIDGE_NENDPOINTS          (3)        /* Number of endpoints in the interface  */

#define APBRIDGE_NREQS               (1)
#define APBRIDGE_REQ_SIZE            (4096)

#define APBRIDGE_CONFIG_ATTR \
  USB_CONFIG_ATTR_ONE | \
  USB_CONFIG_ATTR_SELFPOWER | \
  USB_CONFIG_ATTR_WAKEUP

/* Endpoint configuration */

#define APBRIDGE_EPINTIN_ADDR        (USB_DIR_IN|CONFIG_APBRIDGE_EPINTIN)
#define APBRIDGE_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)
#define APBRIDGE_EPINTIN_MXPACKET    (1024)

#define APBRIDGE_EPOUTBULK_ADDR      (CONFIG_APBRIDGE_EPBULKOUT)
#define APBRIDGE_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define APBRIDGE_EPINBULK_ADDR       (USB_DIR_IN|CONFIG_APBRIDGE_EPBULKIN)
#define APBRIDGE_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

#define APBRIDGE_BULK_MXPACKET       (512)

/* String language */

#define APBRIDGE_STR_LANGUAGE        (0x0409)   /* en-us */

/* Descriptor strings */

#define APBRIDGE_MANUFACTURERSTRID   (1)
#define APBRIDGE_PRODUCTSTRID        (2)
#define APBRIDGE_SERIALSTRID         (3)
#define APBRIDGE_CONFIGSTRID         (4)

/* Buffer big enough for any of our descriptors */

#define APBRIDGE_MXDESCLEN           (64)

/* Vender specific control requests *******************************************/

#define APBRIDGE_RWREQUEST_SVC       (0x01)
#define APBRIDGE_RWREQUEST_LOG     (0x02)

/* Misc Macros ****************************************************************/

/* min/max macros */

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

/* Total number of endpoints (included setup endpoint) */
#define APBRIDGE_MAX_ENDPOINTS (APBRIDGE_NENDPOINTS + 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct apbridge_req_s {
    struct list_head list;
    struct usbdev_req_s *req;   /* The contained request */
    void *priv;
};

/* This structure describes the internal state of the driver */

struct apbridge_dev_s {
    struct usbdev_s *usbdev;    /* usbdev driver pointer */

    uint8_t config;             /* Configuration number */
    sem_t config_sem;

    struct usbdev_ep_s *ep[APBRIDGE_MAX_ENDPOINTS];

    struct list_head ctrlreq;       /* Control request */
    struct list_head intreq;
    struct list_head rdreq;
    struct list_head wrreq;

    struct apbridge_usb_driver *driver;
};

/* The internal version of the class driver */

struct apbridge_driver_s {
    struct usbdevclass_driver_s drvr;
    struct apbridge_dev_s *dev;
};

/* This is what is allocated */

struct apbridge_alloc_s {
    struct apbridge_dev_s dev;
    struct apbridge_driver_s drvr;
};

enum ctrlreq_state {
    USB_REQ,
    GREYBUS_SVC_REQ,
    GREYBUS_LOG,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Request helpers *********************************************************/

static int usbclass_allocreq_buf(struct usbdev_req_s *req,
                                 uint16_t len);
static void usbclass_freereq_buf(struct usbdev_req_s *req);
static struct usbdev_req_s *usbclass_allocreq(struct usbdev_ep_s *ep,
                                              uint16_t len);
static void usbclass_freereq(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static int usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);
static void usbclass_mkepdesc(const struct usb_epdesc_s *indesc,
                              uint16_t mxpacket, struct usb_epdesc_s *outdesc);
static int16_t usbclass_mkcfgdesc(uint8_t * buf, uint8_t speed, uint8_t type);
static void usbclass_resetconfig(struct apbridge_dev_s *priv);
static int usbclass_setconfig(struct apbridge_dev_s *priv, uint8_t config);

/* Completion event handlers ***********************************************/

static void usbclass_ep0incomplete(struct usbdev_ep_s *ep,
                                   struct usbdev_req_s *req);
static void usbclass_rdcomplete(struct usbdev_ep_s *ep,
                                struct usbdev_req_s *req);
static void usbclass_wrcomplete(struct usbdev_ep_s *ep,
                                struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int usbclass_bind(struct usbdevclass_driver_s *driver,
                         struct usbdev_s *dev);
static void usbclass_unbind(struct usbdevclass_driver_s *driver,
                            struct usbdev_s *dev);
static int usbclass_setup(struct usbdevclass_driver_s *driver,
                          struct usbdev_s *dev,
                          const struct usb_ctrlreq_s *ctrl,
                          uint8_t * dataout, size_t outlen);
static void usbclass_disconnect(struct usbdevclass_driver_s *driver,
                                struct usbdev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USB class device ********************************************************/

static const struct usbdevclass_driverops_s g_driverops = {
    usbclass_bind,              /* bind */
    usbclass_unbind,            /* unbind */
    usbclass_setup,             /* setup */
    usbclass_disconnect,        /* disconnect */
    NULL,                       /* suspend */
    NULL,                       /* resume */
};

/* USB descriptor templates these will be copied and modified **************/

static const struct usb_devdesc_s g_devdesc = {
    USB_SIZEOF_DEVDESC,         /* len */
    USB_DESC_TYPE_DEVICE,       /* type */
    {LSBYTE(0x0200), MSBYTE(0x0200)},   /* usb */
    USB_CLASS_PER_INTERFACE,    /* classid */
    0,                          /* subclass */
    0,                          /* protocol */
    CONFIG_APBRIDGE_EP0MAXPACKET,       /* maxpacketsize */
    {LSBYTE(CONFIG_APBRIDGE_VENDORID),  /* vendor */
     MSBYTE(CONFIG_APBRIDGE_VENDORID)},
    {LSBYTE(CONFIG_APBRIDGE_PRODUCTID), /* product */
     MSBYTE(CONFIG_APBRIDGE_PRODUCTID)},
    {LSBYTE(APBRIDGE_VERSIONNO),        /* device */
     MSBYTE(APBRIDGE_VERSIONNO)},
    APBRIDGE_MANUFACTURERSTRID, /* imfgr */
    APBRIDGE_PRODUCTSTRID,      /* iproduct */
    APBRIDGE_SERIALSTRID,       /* serno */
    APBRIDGE_NCONFIGS           /* nconfigs */
};

static const struct usb_cfgdesc_s g_cfgdesc = {
    USB_SIZEOF_CFGDESC,         /* len */
    USB_DESC_TYPE_CONFIG,       /* type */
    {0, 0},                     /* totallen -- to be provided */
    APBRIDGE_NINTERFACES,       /* ninterfaces */
    APBRIDGE_CONFIGID,          /* cfgvalue */
    APBRIDGE_CONFIGSTRID,       /* icfg */
    APBRIDGE_CONFIG_ATTR,       /* attr */
    0                           /* mxpower */
};

static const struct usb_ifdesc_s g_ifdesc = {
    USB_SIZEOF_IFDESC,          /* len */
    USB_DESC_TYPE_INTERFACE,    /* type */
    0,                          /* ifno */
    0,                          /* alt */
    APBRIDGE_NENDPOINTS,        /* neps */
    USB_CLASS_VENDOR_SPEC,      /* classid */
    0,                          /* subclass */
    0,                          /* protocol */
    APBRIDGE_CONFIGSTRID        /* iif */
};

static const struct usb_epdesc_s g_epintindesc = {
    USB_SIZEOF_EPDESC,          /* len */
    USB_DESC_TYPE_ENDPOINT,     /* type */
    APBRIDGE_EPINTIN_ADDR,      /* addr */
    APBRIDGE_EPINTIN_ATTR,      /* attr */
    {LSBYTE(APBRIDGE_EPINTIN_MXPACKET), /* maxpacket */
     MSBYTE(APBRIDGE_EPINTIN_MXPACKET)},
    1                           /* interval */
};

static const struct usb_epdesc_s g_epbulkoutdesc = {
    USB_SIZEOF_EPDESC,          /* len */
    USB_DESC_TYPE_ENDPOINT,     /* type */
    APBRIDGE_EPOUTBULK_ADDR,    /* addr */
    APBRIDGE_EPOUTBULK_ATTR,    /* attr */
    {LSBYTE(APBRIDGE_BULK_MXPACKET),
     MSBYTE(APBRIDGE_BULK_MXPACKET)},   /* maxpacket -- might change to 512 */
    0                           /* interval */
};

static const struct usb_epdesc_s g_epbulkindesc = {
    USB_SIZEOF_EPDESC,          /* len */
    USB_DESC_TYPE_ENDPOINT,     /* type */
    APBRIDGE_EPINBULK_ADDR,     /* addr */
    APBRIDGE_EPINBULK_ATTR,     /* attr */
    {LSBYTE(APBRIDGE_BULK_MXPACKET),
     MSBYTE(APBRIDGE_BULK_MXPACKET)},   /* maxpacket -- might change to 512 */
    0                           /* interval */
};

static const struct usb_epdesc_s *g_usbdesc[] = {
    NULL,               /* No descriptor for ep0 */
    &g_epintindesc,
    &g_epbulkoutdesc,
    &g_epbulkindesc,
};

static const struct usb_qualdesc_s g_qualdesc = {
    USB_SIZEOF_QUALDESC,        /* len */
    USB_DESC_TYPE_DEVICEQUALIFIER,      /* type */
    {LSBYTE(0x0200), MSBYTE(0x0200)},   /* USB */
    USB_CLASS_PER_INTERFACE,    /* classid */
    0,                          /* subclass */
    0,                          /* protocol */
    CONFIG_APBRIDGE_EP0MAXPACKET,       /* mxpacketsize */
    APBRIDGE_NCONFIGS,          /* nconfigs */
    0,                          /* reserved */
};

/**
 * @brief Wait until usb connection has been established
 * USB driver is not fully initialized until enumeration is done.
 * This method will block caller USB is working.
 * @param priv usb device.
 */

void usb_wait(struct apbridge_dev_s *priv)
{
    sem_wait(&priv->config_sem);
}

static struct list_head *epno_to_req_list(struct apbridge_dev_s *priv,
                                          uint8_t epno)
{
    switch(epno) {
    case 0:
        return &priv->ctrlreq;
    case CONFIG_APBRIDGE_EPINTIN:
        return &priv->intreq;
    case CONFIG_APBRIDGE_EPBULKIN:
        return &priv->wrreq;
    case CONFIG_APBRIDGE_EPBULKOUT:
        return &priv->rdreq;
    }
    return NULL;
}

static struct usbdev_req_s *get_request(struct list_head *list)
{
    struct usbdev_req_s *req;
    struct apbridge_req_s *reqcontainer;

    if (list_is_empty(list))
        return NULL;

    list = list->next;
    list_del(list);
    reqcontainer = list_entry(list,  struct apbridge_req_s, list);
    req = reqcontainer->req;
    return req;
}

static void put_request(struct list_head *list, struct usbdev_req_s *req)
{
    struct apbridge_req_s *reqcontainer;

    reqcontainer = (struct apbridge_req_s *)req->priv;
    list_add(list, &reqcontainer->list);
}

static int _to_usb(struct apbridge_dev_s *priv, uint8_t epno,
                   void *payload, size_t len)
{
    int ret;

    struct list_head *list;
    struct usbdev_ep_s *ep;
    struct usbdev_req_s *req;

    list = epno_to_req_list(priv, epno);
    if (!list)
        return -EINVAL;

    req = get_request(list);
    if (!req)
        return -EBUSY;
    req->len = len;
    memcpy(req->buf, payload, len);

    /* Then submit the request to the endpoint */

    ep = priv->ep[epno & USB_EPNO_MASK];
    ret = EP_SUBMIT(ep, req);
    if (ret != OK) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t) - ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Send incoming data from unipro to AP module
 * priv usb device.
 * param payload data to send from SVC
 * size of data to send on unipro
 * @return 0 in success or -EINVAL if len is too big
 */

int unipro_to_usb(struct apbridge_dev_s *priv, void *payload, size_t len)
{
    if (len > APBRIDGE_REQ_SIZE)
        return -EINVAL;

    return _to_usb(priv, CONFIG_APBRIDGE_EPBULKIN, payload, len);
}

/**
 * @brief Send data that come from SVC to AP module
 * priv usb device.
 * param payload data to send from SVC
 * size of data to send on unipro
 * @return 0 in success or -EINVAL if len is too big
 */

int svc_to_usb(struct apbridge_dev_s *priv, void *payload, size_t len)
{
    if (len > APBRIDGE_EPINTIN_MXPACKET)
        return -EINVAL;

    return _to_usb(priv, CONFIG_APBRIDGE_EPINTIN, payload, len);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_allocreq_buf
 *
 * Description:
 *   Allocate a request buffer
 *
 ****************************************************************************/

static int usbclass_allocreq_buf(struct usbdev_req_s *req, uint16_t len)
{
    req->len = len;
    req->buf = EP_ALLOCBUFFER(ep, len);
    if (!req->buf)
        return -ENOMEM;
    return 0;
}

/****************************************************************************
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *usbclass_allocreq(struct usbdev_ep_s *ep,
                                              uint16_t len)
{
    struct usbdev_req_s *req;

    req = EP_ALLOCREQ(ep);
    if (req != NULL) {
        if (!len) {
            req->len = 0;
            req->buf = NULL;
        } else {
            if (usbclass_allocreq_buf(req, len)) {
                EP_FREEREQ(ep, req);
                req = NULL;
            }
        }
    }
    return req;
}

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq_buf(struct usbdev_req_s *req)
{
    if (req->buf != NULL) {
        EP_FREEBUFFER(ep, req->buf);
        req->buf = NULL;
        req->len = 0;
    }
}


/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    if (ep != NULL && req != NULL) {
        usbclass_freereq_buf(req);
        EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
    const char *str;
    int len;
    int ndata;
    int i;

    switch (id) {
    case 0:
        {
            /* Descriptor 0 is the language id */

            strdesc->len = 4;
            strdesc->type = USB_DESC_TYPE_STRING;
            strdesc->data[0] = LSBYTE(APBRIDGE_STR_LANGUAGE);
            strdesc->data[1] = MSBYTE(APBRIDGE_STR_LANGUAGE);
            return 4;
        }

    case APBRIDGE_MANUFACTURERSTRID:
        str = CONFIG_APBRIDGE_VENDORSTR;
        break;

    case APBRIDGE_PRODUCTSTRID:
        str = CONFIG_APBRIDGE_PRODUCTSTR;
        break;

    case APBRIDGE_SERIALSTRID:
        str = CONFIG_APBRIDGE_SERIALSTR;
        break;

    case APBRIDGE_CONFIGSTRID:
        str = CONFIG_APBRIDGE_CONFIGSTR;
        break;

    default:
        return -EINVAL;
    }

    /* The string is utf16-le.  The poor man's utf-8 to utf16-le
     * conversion below will only handle 7-bit en-us ascii
     */

    len = strlen(str);
    for (i = 0, ndata = 0; i < len; i++, ndata += 2) {
        strdesc->data[ndata] = str[i];
        strdesc->data[ndata + 1] = 0;
    }

    strdesc->len = ndata + 2;
    strdesc->type = USB_DESC_TYPE_STRING;
    return strdesc->len;
}

/****************************************************************************
 * Name: usbclass_mkepdesc
 *
 * Description:
 *   Construct the endpoint descriptor
 *
 ****************************************************************************/

static inline void usbclass_mkepdesc(const struct usb_epdesc_s *indesc,
                                     uint16_t mxpacket,
                                     struct usb_epdesc_s *outdesc)
{
    /* Copy the canned descriptor */

    memcpy(outdesc, indesc, USB_SIZEOF_EPDESC);

    /* Then add the correct max packet size */

    outdesc->mxpacketsize[0] = LSBYTE(mxpacket);
    outdesc->mxpacketsize[1] = MSBYTE(mxpacket);
}

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

static int16_t usbclass_mkcfgdesc(uint8_t * buf, uint8_t speed, uint8_t type)
{
    int i;
    struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s *)buf;
    bool hispeed = (speed == USB_SPEED_HIGH);
    uint16_t mxpacket;
    uint16_t totallen;

    /* This is the total length of the configuration (not necessarily the
     * size that we will be sending now.
     */

    totallen =
        USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC +
        APBRIDGE_NENDPOINTS * USB_SIZEOF_EPDESC;

    /* Configuration descriptor -- Copy the canned descriptor and fill in the
     * type (we'll also need to update the size below
     */

    memcpy(cfgdesc, &g_cfgdesc, USB_SIZEOF_CFGDESC);
    buf += USB_SIZEOF_CFGDESC;

    /*  Copy the canned interface descriptor */

    memcpy(buf, &g_ifdesc, USB_SIZEOF_IFDESC);
    buf += USB_SIZEOF_IFDESC;

    /* Make the three endpoint configurations.  First, check for switches
     * between high and full speed
     */

    if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG) {
        hispeed = !hispeed;
    }

    for (i = 1; i < APBRIDGE_MAX_ENDPOINTS; i++) {
        if (hispeed) {
            mxpacket = GETUINT16(g_usbdesc[i]->mxpacketsize);
        } else {
            mxpacket = 64;
        }
        usbclass_mkepdesc(g_usbdesc[i],
                          mxpacket, (struct usb_epdesc_s *)buf);
        buf += USB_SIZEOF_EPDESC;
    }

    /* Finally, fill in the total size of the configuration descriptor */

    cfgdesc->totallen[0] = LSBYTE(totallen);
    cfgdesc->totallen[1] = MSBYTE(totallen);
    return totallen;
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(struct apbridge_dev_s *priv)
{
    int i;

    /* Are we configured? */

    if (priv->config != APBRIDGE_CONFIGIDNONE) {
        /* Yes.. but not anymore */

        priv->config = APBRIDGE_CONFIGIDNONE;

        /* Disable endpoints.  This should force completion of all pending
         * transfers.
         */

        for (i = 1; i < APBRIDGE_MAX_ENDPOINTS; i++)
            EP_DISABLE(priv->ep[i]);
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(struct apbridge_dev_s *priv, uint8_t config)
{
    struct list_head *iter;
    struct usbdev_req_s *req;
    struct apbridge_req_s *reqcontainer;
    struct usb_epdesc_s epdesc;
    uint16_t mxpacket;
    int i;
    int ret = 0;

#if CONFIG_DEBUG
    if (priv == NULL) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return -EIO;
    }
#endif

    if (config == priv->config) {
        /* Already configured -- Do nothing */

        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
        return 0;
    }

    /* Discard the previous configuration data */

    usbclass_resetconfig(priv);

    /* Was this a request to simply discard the current configuration? */

    if (config == APBRIDGE_CONFIGIDNONE) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
        return 0;
    }

    /* We only accept one configuration */

    if (config != APBRIDGE_CONFIGID) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
        return -EINVAL;
    }

    for (i = 1; i < APBRIDGE_MAX_ENDPOINTS; i++) {
        if (priv->usbdev->speed == USB_SPEED_HIGH) {
            mxpacket = GETUINT16(g_usbdesc[i]->mxpacketsize);
        } else {
            mxpacket = 64;
        }
        usbclass_mkepdesc(g_usbdesc[i], mxpacket, &epdesc);
        ret = EP_CONFIGURE(priv->ep[i], &epdesc, false);
        if (ret < 0) {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
            goto errout;
        }
    }

    /* Queue read requests in the bulk OUT endpoint */
    list_foreach(&priv->rdreq, iter) {
        reqcontainer = list_entry(iter, struct apbridge_req_s, list);
        req = reqcontainer->req;
        req->callback = usbclass_rdcomplete;
        ret = EP_SUBMIT(priv->ep[CONFIG_APBRIDGE_EPBULKOUT], req);

        if (ret != OK) {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t) - ret);
            goto errout;
        }
    }

    /* We are successfully configured */

    priv->config = config;
    sem_post(&priv->config_sem);

    return OK;

 errout:
    usbclass_resetconfig(priv);
    return ret;
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_ep0incomplete(struct usbdev_ep_s *ep,
                                   struct usbdev_req_s *req)
{
    struct apbridge_dev_s *priv;
    struct apbridge_usb_driver *drv;
    struct apbridge_req_s *ctrlreq;

    priv = (struct apbridge_dev_s *) ep->priv;
    drv = priv->driver;

    if (req->result || req->xfrd != req->len) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
                 (uint16_t) - req->result);
    }

    ctrlreq = (struct apbridge_req_s *)req->priv;
    if ((int) ctrlreq->priv == GREYBUS_SVC_REQ)
        drv->usb_to_svc(NULL, req->buf, req->len);
    put_request(&priv->ctrlreq, req);
}

/****************************************************************************
 * Name: usbclass_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void usbclass_rdcomplete(struct usbdev_ep_s *ep,
                                struct usbdev_req_s *req)
{
    struct apbridge_dev_s *priv;
    struct apbridge_usb_driver *drv;
    int ret;

    /* Sanity check */

#ifdef CONFIG_DEBUG
    if (!ep || !ep->priv || !req) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return;
    }
#endif

    /* Extract references to private data */

    priv = (struct apbridge_dev_s *) ep->priv;
    drv = priv->driver;

    /* Process the received data unless this is some unusual condition */

    switch (req->result) {
    case OK:                    /* Normal completion */
        usbtrace(TRACE_CLASSRDCOMPLETE, 0);
        drv->usb_to_unipro(priv, req->buf, req->xfrd);
        break;

    case -ESHUTDOWN:           /* Disconnection */
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
        return;

    default:                   /* Some other error occurred */
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
                 (uint16_t) - req->result);
        break;
    };

    ret = EP_SUBMIT(ep, req);

    if (ret != OK) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t) - ret);
    }
}

static void usbclass_wrcomplete(struct usbdev_ep_s *ep,
                              struct usbdev_req_s *req)
{
    struct list_head *list;
    struct apbridge_dev_s *priv;

    /* Sanity check */
#ifdef CONFIG_DEBUG
    if (!ep || !ep->priv || !req || !req->priv) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return;
    }
#endif

    priv = (struct apbridge_dev_s *) ep->priv;
    list = epno_to_req_list(priv, USB_EPNO(ep->eplog));
    put_request(list, req);

    switch (req->result) {
    case OK:                   /* Normal completion */
        usbtrace(TRACE_CLASSWRCOMPLETE, 0);
        break;

    case -ESHUTDOWN:           /* Disconnection */
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), 0);
        break;

    default:                   /* Some other error occurred */
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                 (uint16_t) - req->result);
        break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
****************************************************************************/
typedef void (*usb_callback)(FAR struct usbdev_ep_s *ep, FAR
                             struct usbdev_req_s *req);

static int allocep(struct apbridge_dev_s *priv,
                    const struct usb_epdesc_s *desc)
{
    int in;
    uint8_t epno;
    struct usbdev_ep_s *ep;

    if (desc) {
        in = desc->addr & USB_DIR_IN ? 1 : 0;
        epno = desc->addr & USB_EPNO_MASK;
        DEBUGASSERT(epno < APBRIDGE_MAX_ENDPOINTS);

        ep = DEV_ALLOCEP(priv->usbdev, epno, in, desc->type);
        if (!ep) {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
            return -ENODEV;
        }
        priv->ep[epno] = ep;
    } else {
        ep = priv->usbdev->ep0;
        priv->ep[0] = ep;
    }
    ep->priv = priv;

    return OK;
}

static void freeep(struct apbridge_dev_s *priv, uint8_t epno)
{
    DEBUGASSERT(epno < APBRIDGE_MAX_ENDPOINTS);
    if (epno == 0 || !priv->ep[epno])
        return;

    DEV_FREEEP(priv->usbdev, priv->ep[epno]);
    priv->ep[epno] = NULL;
}

static void prealloc_request(struct usbdev_ep_s *ep,
                             usb_callback callback, int size, int n)
{
    int i;
    struct usbdev_req_s *req;
    struct apbridge_req_s *reqcontainer;
    struct list_head *list = epno_to_req_list(ep->priv, USB_EPNO(ep->eplog));

    DEBUGASSERT(ep);
    DEBUGASSERT(list);
    DEBUGASSERT(callback);

    for (i = 0; i < n; i++) {
        reqcontainer = malloc(sizeof(struct apbridge_req_s *));
        if (!reqcontainer)
            return;
        req = usbclass_allocreq(ep, size);
        if (!req) {
            free(reqcontainer);
            return;
        }
        req->len = size;
        req->priv = reqcontainer;
        req->callback = callback;
        reqcontainer->req = req;
        list_add(list, &reqcontainer->list);
    }
}

static void free_preallocated_request(struct usbdev_ep_s *ep)
{
    struct list_head *iter, *iter_next;
    struct usbdev_req_s *req;
    struct apbridge_req_s *reqcontainer;
    struct list_head *list;

    if (!ep)
        return;
    list = epno_to_req_list(ep->priv, USB_EPNO(ep->eplog));
    DEBUGASSERT(list);

    list_foreach_safe(list, iter, iter_next) {
        reqcontainer = list_entry(iter, struct apbridge_req_s, list);
        req = reqcontainer->req;
        usbclass_freereq(ep, req);
        free(reqcontainer);
        list_del(iter);
    }
}

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(struct usbdevclass_driver_s *driver,
                         struct usbdev_s *dev)
{
    struct apbridge_dev_s *priv = ((struct apbridge_driver_s *)driver)->dev;
    struct list_head *list;
    int ret;
    int i;

    usbtrace(TRACE_CLASSBIND, 0);

    /* Bind the structures */

    priv->usbdev = dev;

    /* Allocate endpoints */

    for (i = 0; i < APBRIDGE_MAX_ENDPOINTS; i++) {
        ret = allocep(priv, g_usbdesc[i]);
        if (ret < 0)
            goto error;
    }

    /* Pre-allocate all endpoints... the endpoints will not be functional
     * until the SET CONFIGURATION request is processed in usbclass_setconfig.
     * This is done here because there may be calls to kmm_malloc and the SET
     * CONFIGURATION processing probably occurrs within interrupt handling
     * logic where kmm_malloc calls will fail.
     */

    list_init(&priv->ctrlreq);
    prealloc_request(priv->ep[0], usbclass_ep0incomplete,
                     APBRIDGE_MXDESCLEN, 1);

    list_init(&priv->intreq);
    prealloc_request(priv->ep[CONFIG_APBRIDGE_EPINTIN], usbclass_wrcomplete,
                     APBRIDGE_EPINTIN_MXPACKET, 1);

    list_init(&priv->rdreq);
    prealloc_request(priv->ep[CONFIG_APBRIDGE_EPBULKOUT],
                     usbclass_rdcomplete, APBRIDGE_REQ_SIZE, APBRIDGE_NREQS);

    list_init(&priv->wrreq);
    prealloc_request(priv->ep[CONFIG_APBRIDGE_EPBULKIN],
                     usbclass_wrcomplete, APBRIDGE_REQ_SIZE, APBRIDGE_NREQS);

    /* Report if we are selfpowered */

    DEV_SETSELFPOWERED(dev);

    for (i = 0; i < APBRIDGE_MAX_ENDPOINTS; i++) {
        list = epno_to_req_list(priv, i);
        if (list_is_empty(list)) {
            ret = -ENOMEM;
            goto error;
        }
    }

    /* And pull-up the data line for the soft connect function */

    DEV_CONNECT(dev);

    return OK;

error:
    usbclass_unbind(driver, dev);
    return ret;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(struct usbdevclass_driver_s *driver,
                            struct usbdev_s *dev)
{
    int i;
    struct apbridge_dev_s *priv;

    usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return;
    }
#endif

    /* Extract reference to private data */

    priv = ((struct apbridge_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG
    if (!priv) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
        return;
    }
#endif

    /* Make sure that we are not already unbound */

    if (priv != NULL) {
        /* Make sure that the endpoints have been unconfigured.  If
         * we were terminated gracefully, then the configuration should
         * already have been reset.  If not, then calling usbclass_resetconfig
         * should cause the endpoints to immediately terminate all
         * transfers and return the requests to us (with result == -ESHUTDOWN)
         */

        usbclass_resetconfig(priv);
        up_mdelay(50);

        /* Free the pre-allocated request */

        for (i = 0; i < APBRIDGE_MAX_ENDPOINTS; i++) {
            free_preallocated_request(priv->ep[i]);
            freeep(priv, i);
        }

    }
}

#if defined(CONFIG_APB_USB_LOG)
static struct log_buffer *g_lb;

static ssize_t usb_log_read(struct file *filep, char *buffer, size_t buflen)
{
    return 0;
}

static ssize_t usb_log_write(struct file *filep, const char *buffer,
                                 size_t buflen)
{
    return log_buffer_write(g_lb, buffer, buflen);
}

static const struct file_operations usb_log_ops = {
    .read = usb_log_read,
    .write = usb_log_write,
};

void usb_log_init(void)
{
    g_lb = log_buffer_alloc(CONFIG_USB_LOG_BUFFER_SIZE);
    if (g_lb)
        register_driver("/dev/console", &usb_log_ops,
                        CONFIG_USB_LOG_BUFFER_SIZE, NULL);
}

void usb_putc(int c)
{
    if (g_lb)
        log_buffer_write(g_lb, &c, 1);
}

int usb_get_log(void *buf, int len)
{
    return log_buffer_readlines(g_lb, buf, len);
}
#endif

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(struct usbdevclass_driver_s *driver,
                          struct usbdev_s *dev,
                          const struct usb_ctrlreq_s *ctrl,
                          uint8_t * dataout, size_t outlen)
{
    struct apbridge_dev_s *priv;
    struct usbdev_req_s *req;
    struct apbridge_req_s *ctrreq;

    uint16_t value;
    uint16_t index;
    uint16_t len;
    int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0 || !ctrl) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return -EIO;
    }
#endif

    /* Extract reference to private data */

    usbtrace(TRACE_CLASSSETUP, ctrl->req);
    priv = ((struct apbridge_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG
    if (!priv) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
        return -ENODEV;
    }
#endif
    req = get_request(&priv->ctrlreq);
    ctrreq = (struct apbridge_req_s *)req->priv;
    ctrreq->priv = (void *)USB_REQ;

    /* Extract the little-endian 16-bit values to host order */

    value = GETUINT16(ctrl->value);
    index = GETUINT16(ctrl->index);
    len = GETUINT16(ctrl->len);

    uvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrl->type, ctrl->req, value, index, len);

    switch (ctrl->type & USB_REQ_TYPE_MASK) {
     /***********************************************************************
      * Standard Requests
      ***********************************************************************/

    case USB_REQ_TYPE_STANDARD:
        {
            switch (ctrl->req) {
            case USB_REQ_GETDESCRIPTOR:
                {
                    /* The value field specifies the descriptor type in the MS byte and the
                     * descriptor index in the LS byte (order is little endian)
                     */

                    switch (ctrl->value[1]) {
                    case USB_DESC_TYPE_DEVICE:
                        {
                            ret = USB_SIZEOF_DEVDESC;
                            memcpy(req->buf, &g_devdesc, ret);
                        }
                        break;

                    case USB_DESC_TYPE_DEVICEQUALIFIER:
                        {
                            ret = USB_SIZEOF_QUALDESC;
                            memcpy(req->buf, &g_qualdesc, ret);
                        }
                        break;

                    case USB_DESC_TYPE_OTHERSPEEDCONFIG:
                    case USB_DESC_TYPE_CONFIG:
                        {
                            ret =
                                usbclass_mkcfgdesc(req->buf, dev->speed,
                                                   ctrl->req);
                        }
                        break;

                    case USB_DESC_TYPE_STRING:
                        {
                            /* index == language code. */

                            ret =
                                usbclass_mkstrdesc(ctrl->value[0],
                                                   (struct usb_strdesc_s *)
                                                   req->buf);
                        }
                        break;

                    default:
                        {
                            usbtrace(TRACE_CLSERROR
                                     (USBSER_TRACEERR_GETUNKNOWNDESC), value);
                        }
                        break;
                    }
                }
                break;

            case USB_REQ_SETCONFIGURATION:
                {
                    if (ctrl->type == 0) {
                        ret = usbclass_setconfig(priv, value);
                    }
                }
                break;

            case USB_REQ_GETCONFIGURATION:
                {
                    if (ctrl->type == USB_DIR_IN) {
                        *(uint8_t *) req->buf = priv->config;
                        ret = 1;
                    }
                }
                break;

            case USB_REQ_SETINTERFACE:
                {
                    if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE) {
                        if (priv->config == APBRIDGE_CONFIGID &&
                            index == APBRIDGE_INTERFACEID &&
                            value == APBRIDGE_ALTINTERFACEID) {
                            usbclass_resetconfig(priv);
                            usbclass_setconfig(priv, priv->config);
                            ret = 0;
                        }
                    }
                }
                break;

            case USB_REQ_GETINTERFACE:
                {
                    if (ctrl->type ==
                        (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE)
                        && priv->config == APBRIDGE_CONFIGIDNONE) {
                        if (index != APBRIDGE_INTERFACEID) {
                            ret = -EDOM;
                        } else {
                            *(uint8_t *) req->buf =
                                APBRIDGE_ALTINTERFACEID;
                            ret = 1;
                        }
                    }
                }
                break;

            default:
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                         ctrl->req);
                break;
            }
        }
        break;

        /* Put here vendor request */
    case USB_REQ_TYPE_VENDOR:
        {
            if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
                USB_REQ_RECIPIENT_INTERFACE) {
                if (ctrl->req == APBRIDGE_RWREQUEST_SVC) {
                    if ((ctrl->type & USB_DIR_IN) != 0) {
                        *(uint32_t *) req->buf = 0xdeadbeef;
                        ret = 4;
                    } else {
                        ctrreq->priv = (void *)GREYBUS_SVC_REQ;
                        ret = len;
                    }
                } else if (ctrl->req == APBRIDGE_RWREQUEST_LOG) {
                    if ((ctrl->type & USB_DIR_IN) == 0) {
                    } else {
#if defined(CONFIG_APB_USB_LOG)
                        ctrreq->priv = (void *)GREYBUS_LOG;
                        ret = usb_get_log(req->buf, len);
#else
                        ret = 0;
#endif
                    }
                } else {
                    usbtrace(TRACE_CLSERROR
                             (USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                             ctrl->type);
                }
            }
        }
        break;

    default:
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
        break;
    }

    /* Respond to the setup command if data was returned.  On an error return
     * value (ret < 0), the USB driver will stall.
     */

    if (ret >= 0) {
        req->len = min(len, ret);
        req->flags = USBDEV_REQFLAGS_NULLPKT;
        ret = EP_SUBMIT(dev->ep0, req);
        if (ret < 0) {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ),
                     (uint16_t) - ret);
            req->result = OK;
            usbclass_ep0incomplete(dev->ep0, req);
        }
    }

    return ret;
}

#ifdef CONFIG_HSIC_HUB_RESET
#define HUB_RESET_GPIO 3
static void hsic_hub_reset(void)
{
    gpio_activate(HUB_RESET_GPIO);
    gpio_direction_out(HUB_RESET_GPIO, 0);
    up_mdelay(10);
    gpio_set_value(HUB_RESET_GPIO, 1);
    gpio_deactivate(HUB_RESET_GPIO);
}
#else
#define hsic_hub_reset()
#endif

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(struct usbdevclass_driver_s *driver,
                                struct usbdev_s *dev)
{
    struct apbridge_dev_s *priv;
    irqstate_t flags;

    usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG
    if (!driver || !dev || !dev->ep0) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
        return;
    }
#endif

    /* Extract reference to private data */

    priv = ((struct apbridge_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG
    if (!priv) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
        return;
    }
#endif

    /* Inform the "upper half serial driver that we have lost the USB serial
     * connection.
     */

    flags = irqsave();

    /* Reset the configuration */

    usbclass_resetconfig(priv);

    /* Clear out all outgoing data in the circular buffer */

    irqrestore(flags);

    /* Perform the soft connect function so that we will we can be
     * re-enumerated.
     */

    DEV_CONNECT(dev);
}

int usbdev_apbinitialize(struct apbridge_usb_driver *driver)
{
    struct apbridge_alloc_s *alloc;
    struct apbridge_dev_s *priv;
    struct apbridge_driver_s *drvr;
    int ret;

    /* Reset USB HSIC HUB */
    hsic_hub_reset();

    /* Allocate the structures needed */

    alloc = (struct apbridge_alloc_s *)
        kmm_malloc(sizeof(struct apbridge_alloc_s));
    if (!alloc) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
        return -ENOMEM;
    }

    /* Convenience pointers into the allocated blob */

    priv = &alloc->dev;
    drvr = &alloc->drvr;

    /* Initialize the USB driver structure */

    memset(priv, 0, sizeof(struct apbridge_dev_s));
    priv->driver = driver;
    sem_init(&priv->config_sem, 0, 0);

    /* Initialize the USB class driver structure */

    drvr->drvr.speed = USB_SPEED_HIGH;
    drvr->drvr.ops = &g_driverops;
    drvr->dev = priv;

    /* Register the USB serial class driver */

    ret = usbdev_register(&drvr->drvr);
    if (ret) {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER),
                 (uint16_t) - ret);
        goto errout_with_alloc;
    }
    ret = priv->driver->init(priv);
    if (ret)
        goto errout_with_init;

    /* Register the single port supported by this implementation */
    return OK;

 errout_with_init:
    usbdev_unregister(&drvr->drvr);
 errout_with_alloc:
    kmm_free(alloc);
    return ret;
}
