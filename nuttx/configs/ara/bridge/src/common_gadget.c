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

#include <string.h>
#include <nuttx/util.h>
#include <nuttx/bufram.h>
#include <nuttx/list.h>
#include <arch/board/common_gadget.h>

#define REQ_SIZE_MUL (BUFRAM_PAGE_SIZE)

struct request_list {
    struct list_head list;
    struct usbdev_ep_s *ep;
    struct usbdev_req_s *req;
    size_t len;                 /* size of allocated buffer */
    void *priv;
};

struct request_pool {
    size_t len;
    struct list_head request;
    struct list_head request_in_use;
    struct list_head pool;
};
static LIST_DECLARE(request_pool);

static struct request_pool *get_request_pool(size_t len);
static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

static struct request_pool *request_pool_alloc(size_t len)
{
    struct request_pool *pool;

    pool = kmm_malloc(sizeof(*pool));
    if (!pool) {
        return NULL;
    }
    pool->len = len;
    list_init(&pool->request);
    list_init(&pool->request_in_use);

    list_add(&request_pool, &pool->pool);

    return pool;
}

static void request_pool_free(struct request_pool *pool)
{
    struct request_list *req_list;
    struct list_head *iter, *next;

    list_foreach_safe(&pool->request, iter, next) {
        req_list = list_entry(iter, struct request_list, list);
        free_request(req_list->ep, req_list->req);
        list_del(&req_list->list);
        kmm_free(req_list);
    }
    list_del(&pool->pool);
    kmm_free(pool);
}

static struct request_pool *get_request_pool(size_t len)
{
    struct list_head *iter;
    struct request_pool *pool;

    list_foreach(&request_pool, iter) {
        pool = list_entry(iter, struct request_pool, pool);
        if (pool->len == len)
            return pool;
    }
    return NULL;
}

static void init_request(struct usbdev_req_s *req,
                         usb_callback callback, size_t len, void *priv)
{
    req->len = len;
    req->priv = priv;
    req->callback = callback;
}


static struct usbdev_req_s *alloc_request(struct usbdev_ep_s *ep,
                                          usb_callback callback,
                                          size_t len, void *priv)
{
    struct usbdev_req_s *req;

    DEBUGASSERT(ep);

    req = EP_ALLOCREQ(ep);
    if (!req) {
        return NULL;
    }

    req->buf = NULL;

    /* Only allocate request buffer for OUT requests */
    if (len && ep->eplog % 2 == 0) {
        req->buf = bufram_page_alloc(bufram_size_to_page_count(len));
        if (!req->buf) {
            EP_FREEREQ(ep, req);
            return NULL;
        }
    }
    init_request(req, callback, len, priv);

    return req;
}

static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    if (!req)
        return;

    if (req->buf != NULL) {
        if (ep->eplog % 2 == 0) /* free only OUT requests */
            bufram_page_free(req->buf, bufram_size_to_page_count(req->len));
        req->buf = NULL;
        req->len = 0;
    }
    EP_FREEREQ(ep, req);
}

/*
 * Return a multiple of REQ_SIZE_MUL
 */
static size_t request_len_align(size_t len)
{
    if (!len) {
        return 0;
    }
    return ((--len / REQ_SIZE_MUL) + 1) * REQ_SIZE_MUL;
}

/*
 * Assign private data to request
 * \param req request's pointer
 * \param priv pointer to private data to assign
 */
void request_set_priv(struct usbdev_req_s *req, void *priv)
{
    struct request_list *req_list = req->priv;
    req_list->priv = priv;
}

/*
 * Get the private data from request
 * \param req request's pointer
 * \return a pointer to private data
 */
void *request_get_priv(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;
    return req_list->priv;
}

/*
 * \brief Get a request from request manager
 * Allocate a new request a return one from the request pool.
 * \param ep the pointer of endpoint that is going to use the request.
 * \param callback transfer completion callback.
 *     NULL is only accepted to preallocate request.
 * \param len Size of request.
 * \param priv pointer to private data (can be NULL).
 */
struct usbdev_req_s *get_request(struct usbdev_ep_s *ep,
                                 usb_callback callback,
                                 size_t len, void *priv)
{
    int empty;
    irqstate_t flags;
    size_t len_aligned;
    struct list_head *list;
    struct usbdev_req_s *req = NULL;
    struct request_list *req_list;
    struct request_pool *pool;

    len_aligned = request_len_align(len);
    pool = get_request_pool(len_aligned);
    if (!pool) {
        pool = request_pool_alloc(len_aligned);
        if (!pool) {
            return NULL;
        }
    }

    flags = irqsave();
    empty = list_is_empty(&pool->request);
    irqrestore(flags);
    if (empty) {
        /* Assume device driver support ep = NULL */
        req_list = kmm_malloc(sizeof(*req_list));
        if (!req_list) {
            return NULL;
        }
        req = alloc_request(ep, callback, len, req_list);
        req_list->req = req;
        req_list->len = len_aligned;
        if (!req) {
            kmm_free(req_list);
            return NULL;
        }
    } else {
        list = pool->request.next;
        req_list = list_entry(list, struct request_list, list);
        req = req_list->req;
        init_request(req, callback, len, req_list);
        flags = irqsave();
        list_del(&req_list->list);
        irqrestore(flags);
    }
    req_list->priv = priv;
    req_list->ep = ep;

    flags = irqsave();
    list_add(&pool->request_in_use, &req_list->list);
    irqrestore(flags);

    return req;
}

/*
 * Return the request to the request pool.
 * \param req request's pointer
 */
void put_request(struct usbdev_req_s *req)
{
    irqstate_t flags;
    size_t len_aligned;
    struct request_list *req_list;
    struct request_pool *pool;

    if (!req)
        return;

    req_list = req->priv;
    len_aligned = request_len_align(req_list->len);
    pool = get_request_pool(len_aligned);
    DEBUGASSERT(pool);

    flags = irqsave();
    list_del(&req_list->list);
    list_add(&pool->request, &req_list->list);
    irqrestore(flags);
}

/*
 * Look up a request in request pool using a private data
 * \param priv private data pointer to use for look up
 * \return request's pointer or NULL if request can not be found
 */
struct usbdev_req_s *find_request_by_priv(const void *priv)
{
    struct list_head *pool_iter, *req_iter;
    struct request_pool *pool;
    struct request_list *req_list;
    irqstate_t flags;

    list_foreach(&request_pool, pool_iter) {
        pool = list_entry(pool_iter, struct request_pool, pool);
        flags = irqsave();
        list_foreach(&pool->request_in_use, req_iter) {
            req_list = list_entry(req_iter, struct request_list, list);
            if (req_list->priv == priv) {
                irqrestore(flags);
                return req_list->req;
            }
        }
        irqrestore(flags);
    }
    return NULL;
}

/*
 * Get the endpoint that use the request
 * \param req request's pointer
 * \return a pointer to endpoint
 */
struct usbdev_ep_s *request_to_ep(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;

    return req_list->ep;
}

/*
 * Preallocate requests
 * \param ep endpoint that will use the request.
       Note that the requests may be available to other endpoints.
 * \param len the size of request to preallocate.
 * \param n number of request to preallocate
 */
int request_pool_prealloc(struct usbdev_ep_s *ep, size_t len, int n)
{
    int i;
    int ret = 0;
    struct usbdev_req_s *req[n];

    for (i = 0; i < n; i++) {
        req[i] = get_request(ep, NULL, len, NULL);
        if (!req[i]) {
            ret = -ENOMEM;
            break;
        }
    }
    for (i--; i >= 0; i--) {
        put_request(req[i]);
    }
    return ret;
}

/*
 * Free all requests
 */
void request_pool_freeall(void)
{
    struct request_pool *pool;
    struct list_head *iter, *next;

    list_foreach_safe(&request_pool, iter, next) {
        pool = list_entry(iter, struct request_pool, pool);
        request_pool_free(pool);
    }
}

/* Enumeration common part */

/**
 * Allocate a gadget descriptor and fill it with device descriptors
 * \param dev    pointer to the device descriptor
 * \param qual   pointer to the device qualifier descriptor
 * \return a pointer to gadget_descriptor or NULL in case of error
 */
struct gadget_descriptor *gadget_descriptor_alloc(const struct usb_devdesc_s *dev,
                                                  const struct usb_qualdesc_s *qual)
{
    int i;
    int nconfigs = dev->nconfigs;
    struct gadget_descriptor *g_desc;

    g_desc = kmm_malloc(sizeof(*g_desc));
    if (!g_desc) {
        return NULL;
    }

    g_desc->cfg = kmm_malloc(sizeof(*g_desc->cfg) * nconfigs);
    if (!g_desc->cfg) {
        kmm_free(g_desc);
        return NULL;
    }

    g_desc->dev = dev;
    g_desc->qual = qual;
    for (i = 0; i < nconfigs; i++) {
        g_desc->cfg[i].cfg = NULL;
    }

    return g_desc;
}

/**
 * Free the gadget descriptor
 * \param g_desc pointer to the gadget descriptor to free
 */
void gadget_descriptor_free(struct gadget_descriptor *g_desc)
{
    if (g_desc) {
        kmm_free(g_desc->cfg);
    }
    kmm_free(g_desc);
}

/**
 * Add one config descriptor and configs
 * \param g_desc   pointer to the gadget descriptor
 * \param desc     pointer to the config descriptor to add
 * \param usb_desc pointer to an array of config
 */
void gadget_add_cfgdesc(struct gadget_descriptor *g_desc,
                        const struct usb_cfgdesc_s *desc,
                        const struct usb_desc_s *usb_desc[])
{
    int i;
    int nconfigs = g_desc->dev->nconfigs;

    for (i = 0; i < nconfigs; i++) {
        if (!g_desc->cfg[i].cfg) {
            g_desc->cfg[i].cfg = desc;
            g_desc->cfg[i].desc = usb_desc;
            break;
        }
    }
}

/**
 * Set the list of string for the gadget
 * \param g_desc pointer to the gadget descriptor
 * \param str    pointer to the string array
 */
void gadget_set_strings(struct gadget_descriptor *g_desc,
                        const struct gadget_strings *str)
{
    g_desc->str = str;
}

static int16_t gadget_config_desc(struct gadget_descriptor *g_desc,
                                  uint8_t * buf, uint8_t speed,
                                  uint16_t id, uint8_t type)
{
    int i;
    int neps = 0;
    int ninterfaces;
    struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s *)buf;
    struct gadget_config_descriptor *g_cfgdesc;
    const struct usb_desc_s **usb_desc;
    bool hispeed = (speed == USB_SPEED_HIGH);
    uint16_t mxpacket;
    uint16_t totallen;

    g_cfgdesc = &g_desc->cfg[id];
    ninterfaces = g_cfgdesc->cfg->ninterfaces;
    /* Get the total number of endpoints */
    for (i = 0; i < ninterfaces; i++) {
        neps += ((struct usb_ifdesc_s *)g_cfgdesc->desc[i])->neps;
    }

    /* This is the total length of the configuration */
    totallen =
        USB_SIZEOF_CFGDESC +
        USB_SIZEOF_IFDESC * ninterfaces +
        USB_SIZEOF_EPDESC * neps;

    /*
     * Configuration descriptor -- Copy the canned descriptor and fill in the
     * type (we'll also need to update the size below
     */
    memcpy(cfgdesc, g_cfgdesc->cfg, USB_SIZEOF_CFGDESC);
    buf += USB_SIZEOF_CFGDESC;

    /*  Copy the canned interface descriptor */
    usb_desc = g_cfgdesc->desc;
    for (i = 0; i < ninterfaces; i++) {
        memcpy(buf, *usb_desc, USB_SIZEOF_IFDESC);
        buf += USB_SIZEOF_IFDESC;
        usb_desc++;
    }

    /*  Check for switches between high and full speed */
    if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG) {
        hispeed = !hispeed;
    }

    /* Make endpoints configurations */
    for (i = 0; i < neps; i++) {
        struct usb_epdesc_s *epdesc = (struct usb_epdesc_s *)*usb_desc;
        struct usb_epdesc_s *epdesc_buf = (struct usb_epdesc_s *)buf;
        if (hispeed) {
            mxpacket = GETUINT16(epdesc->mxpacketsize);
        } else {
            mxpacket = 64;
        }
        memcpy(epdesc_buf, epdesc, USB_SIZEOF_EPDESC);

        epdesc_buf->mxpacketsize[0] = LSBYTE(mxpacket);
        epdesc_buf->mxpacketsize[1] = MSBYTE(mxpacket);
        buf += USB_SIZEOF_EPDESC;
        usb_desc++;
    }

    /* Finally, fill in the total size of the configuration descriptor */
    cfgdesc->totallen[0] = LSBYTE(totallen);
    cfgdesc->totallen[1] = MSBYTE(totallen);
    return totallen;
}

static int usb_ascii_to_utf16(uint8_t *utf16, const uint8_t *ascii, size_t len)
{
    int i, ndata;
    for (i = 0, ndata = 0; i < len; i++, ndata += 2) {
        utf16[ndata] = ascii[i];
        utf16[ndata + 1] = 0;
    }

    return ndata + 2;
}

static int gadget_get_langs(struct gadget_descriptor *g_desc,
                            struct usb_strdesc_s *strdesc)
{
    int i = 0;
    uint16_t id = 0;

    strdesc->len = 0;
    strdesc->type = USB_DESC_TYPE_STRING;

    do {
        id = g_desc->str[i].lang;
        if (id) {
            strdesc->data[i * 2] = LSBYTE(id);
            strdesc->data[i * 2 + 1] = MSBYTE(id);
            strdesc->len += 4;
        }
        i++;
    } while (id != 0);

    return strdesc->len;
}

static int gadget_get_string(struct gadget_descriptor *g_desc,
                             int id, struct usb_strdesc_s *strdesc)
{
    int i;
    const struct gadget_string *strs;

    if (id == 0) {
        return gadget_get_langs(g_desc, strdesc);
    }

    strs = g_desc->str->strs;
    for (i = 0; &strs[i] && strs[i].str; i++) {
        if (strs[i].id == id) {
            strdesc->len = usb_ascii_to_utf16(strdesc->data,
                                             (uint8_t *)strs[i].str,
                                             strlen(strs[i].str));
            strdesc->type = USB_DESC_TYPE_STRING;
            return strdesc->len;
        }
    }

    return -EINVAL;
}

/**
 * Handle some generic control request
 * \param g_desc pointer to the gadget descriptor
 * \param dev pointer to the usb device
 * \param req pointer to the request to handle (used for data stage)
 * \param ctrl pointer to the control request to handle
 * \return the length of data to send for a data stage or < 0 in case of error
 */
int gadget_control_handler(struct gadget_descriptor *g_desc,
                           struct usbdev_s *dev,
                           struct usbdev_req_s *req,
                           const struct usb_ctrlreq_s *ctrl)
{
    uint16_t len;
    int ret = -EOPNOTSUPP;

    len = GETUINT16(ctrl->len);

    if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD) {
        if (ctrl->req == USB_REQ_GETDESCRIPTOR) {
            switch (ctrl->value[1]) {
            case USB_DESC_TYPE_DEVICE:
                ret = USB_SIZEOF_DEVDESC;
                memcpy(req->buf, g_desc->dev, ret);
                break;

            case USB_DESC_TYPE_DEVICEQUALIFIER:
                ret = USB_SIZEOF_QUALDESC;
                memcpy(req->buf, g_desc->qual, ret);
                break;

            case USB_DESC_TYPE_OTHERSPEEDCONFIG:
            case USB_DESC_TYPE_CONFIG:
                ret = gadget_config_desc(g_desc, req->buf, dev->speed,
                                         ctrl->value[0], ctrl->req);
                break;

            case USB_DESC_TYPE_STRING:
                /* index == language code. */
                ret = gadget_get_string(g_desc, ctrl->value[0],
                                        (struct usb_strdesc_s *) req->buf);
                break;
            }
        }
    }

    /* Respond to the setup command if data was returned. On an error return
     * let the gadget handle it.
     */
    if (ret >= 0) {
        req->len = MIN(len, ret);
        req->flags = USBDEV_REQFLAGS_NULLPKT;
        ret = EP_SUBMIT(dev->ep0, req);
    }

    return ret;
}

/* Vendor request API */

struct vendor_request {
    uint8_t req;
    uint8_t flags;
    control_request_callback cb;
    struct list_head list;
};

static LIST_DECLARE(vendor_requests);
static struct vendor_request *g_vendor_request = NULL;
static struct usb_ctrlreq_s g_ctrl;
static struct usbdev_s *g_dev;
static struct usbdev_req_s *g_req;

/**
 * \brief Handle the control request data stage
 * The host first send the control request and after send the data.
 * In this case, the vendor request callback is called only after
 * we receive data.
 * \param req pointer to the request to execute (used for data stage)
 */
void vendor_data_handler(struct usbdev_req_s *req)
{
    if (!g_vendor_request)
        return;

    uint16_t value;
    uint16_t index;

    value = GETUINT16(g_ctrl.value);
    index = GETUINT16(g_ctrl.index);

    g_vendor_request->cb(g_dev, g_ctrl.req, value, index, req->buf, req->xfrd);
    g_vendor_request = NULL;
}

static int vendor_request_submit(struct usbdev_s *dev,
                                 struct usbdev_req_s *req, int result)
{
    if (result >= 0) {
        result = EP_SUBMIT(dev->ep0, req);
    }

    if (result < 0) {
        req->result = result;
        req->callback(dev->ep0, req);
    }

    return result;
}

/**
 * \brief Submit a vendor request
 * Sometime, we may want to submit a request outside the
 * vendor request callback.
 * To use deferred request, you must set the VENDOR_REQUEST_DEFER flag.
 * \param dev pointer to the usb device
 */
void vendor_request_deferred_submit(struct usbdev_s *dev, int result)
{
    struct usbdev_req_s *req = g_req;

    if (!req) {
        return;
    } else {
        g_req = NULL;
    }

    vendor_request_submit(dev, req, result);
}

/**
 * \brief Handle vendor request
 * Find a request handler and execute it.
 * In case of out data stage (host to device), the callback is not called.
 * We are waiting to receive the data to call it (vendor_data_handler()).
 * If there are a data stage, the method will submit itself  the request.
 * This method is expected to work with only one control endpoint.
 * \param dev pointer to the usb device
 * \param req pointer to the request to execute (used for data stage)
 * \param ctrl pointer to the control request to execute
 * \return the size of data to send or < 0 in case of error
 */
int vendor_request_handler(struct usbdev_s *dev,
                           struct usbdev_req_s *req,
                           const struct usb_ctrlreq_s *ctrl)
{
    int in;
    uint16_t value;
    uint16_t index;
    uint16_t len;
    int ret = -EOPNOTSUPP;
    struct list_head *iter;
    struct vendor_request *vendor_request;

    value = GETUINT16(ctrl->value);
    index = GETUINT16(ctrl->index);
    len = GETUINT16(ctrl->len);

    in = (ctrl->type & USB_DIR_IN) != 0;

    if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR) {
        list_foreach(&vendor_requests, iter) {
            vendor_request = list_entry(iter, struct vendor_request, list);
            if (vendor_request->req == ctrl->req) {
                /* in transfer: nothing special to do */
                if (in && (vendor_request->flags & VENDOR_REQ_IN)) {
                    ret = vendor_request->cb(dev, ctrl->req, index, value,
                                             req->buf, len);
                } else if (!(in && vendor_request->flags & VENDOR_REQ_IN)) {
                    /* we don't expect data from host */
                    if (!(vendor_request->flags & VENDOR_REQ_DATA)) {
                        ret = vendor_request->cb(dev, ctrl->req, index, value,
                                                 req->buf, len);
                    /* wait to receive data before to call handler */
                    } else {
                        ret = len;
                        g_vendor_request = vendor_request;
                        memcpy(&g_ctrl, ctrl, sizeof(g_ctrl));
                    }
                }
            }
        }
    }

    if (ret >= 0) {
        req->len = MIN(len, ret);
        req->flags = USBDEV_REQFLAGS_NULLPKT;
        if (vendor_request->flags && VENDOR_REQ_DEFER) {
            g_req = req;
        } else {
            ret = vendor_request_submit(dev, req, ret);
        }
    }

    return ret;
}

/**
 * Register a new vendor request
 * \param req the vendor request number
 * \param flags configure how the request is handled
 * \param cb the callback to call if this request is handled
 * return 0 in sucess or < 0 in case of error
 */
int register_vendor_request(uint8_t req, uint8_t flags,
                             control_request_callback cb)
{
    irqstate_t irq_flags;
    struct vendor_request *vendor_request;

    vendor_request = kmm_malloc(sizeof(*vendor_request));
    if (!vendor_request)
        return -ENOMEM;

    irq_flags = irqsave();
    vendor_request->req = req;
    vendor_request->cb = cb;
    vendor_request->flags = flags;
    list_add(&vendor_requests, &vendor_request->list);
    irqrestore(irq_flags);

    return 0;
}

/**
 * remove a vendor request from vendor request manager
 * \param req the vendor request number of request to remove
 */
void unregister_vendor_request(uint8_t req)
{
    irqstate_t flags;
    struct list_head *iter, *next;
    struct vendor_request *vendor_request;

    flags = irqsave();
    list_foreach_safe(&vendor_requests, iter, next) {
        vendor_request = list_entry(iter, struct vendor_request, list);
        if (vendor_request->req == req) {
            list_del(iter);
            kmm_free(&vendor_request->list);
        }
    }
    irqrestore(flags);
}

/**
 * remove all vendor requests from vendor request manager
 */
void unregister_all_vendor_request(void)
{
    struct list_head *iter, *next;
    struct vendor_request *vendor_request;

    list_foreach_safe(&vendor_requests, iter, next) {
        vendor_request = list_entry(iter, struct vendor_request, list);
        list_del(iter);
        kmm_free(&vendor_request->list);
    }
}
