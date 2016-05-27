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

#include <nuttx/bufram.h>
#include <nuttx/device.h>
#include <nuttx/time.h>
#include <nuttx/usb_device.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include "usbtun.h"
#include "nuttx/usbtun/usbtun_pcd_router.h"

/* number of pre-allocate requests */
#define NUM_CTRL_REQS 5
#define NUM_IN_REQS 30
#define NUM_OUT_REQS 1

#define ROUTER_READY_WAIT_NS 200000000LL
#define PCD_RESTART_BACKOFF_US 100000

typedef struct {
    sq_entry_t entry;
    uint8_t ep;
    struct usbdev_req_s *dev_req;
    usbtun_buf_t setup;
    usbtun_buf_t data;
} pcd_req_t;

enum { USB_STATE_NONE, USB_STATE_CONNECTED, USB_STATE_DISCONNECTED };

struct pcd_router_data_s {
    struct device *pcddev;
    struct usbdev_s *usbdev;
    bool pcd_ready;
    bool hcd_ready;
    int usb_connected;
    struct usbdevclass_driver_s driver;
    struct usb_epdesc_s ep_list[MAX_ENDPOINTS];
    struct usbdev_ep_s *ep[MAX_ENDPOINTS];
};

struct pcd_router_data_s s_data;

static pthread_t pcd_thread;
static pthread_mutex_t run_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t run_cond = PTHREAD_COND_INITIALIZER;
static bool do_run = false;

static void _unconfigure_ep(uint8_t epno);
static void req_ctrl_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static void req_in_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static void req_out_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
typedef void (*usb_callback)(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

static usbtun_data_res_t handle_data_body(usbtun_buf_t *buf, uint8_t ep, uint8_t type);

static void pcd_req_mem_free(pcd_req_t *req) {
    if (req) {
        usbtun_clean_mem(&req->setup);
        usbtun_clean_mem(&req->data);
    }
}

static struct usbdev_req_s *alloc_request(struct usbdev_ep_s *ep,
                                          usb_callback callback, void *priv) {
    struct usbdev_req_s *req;

    req = EP_ALLOCREQ(ep);

    if (!req) {
        lldbg("Failed to allocate EP request\n");
        return NULL;
    }

    req->priv = priv;
    req->callback = callback;

    return req;
}

static void ep0_out_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *dev_req) {
    if (ep == NULL || dev_req == NULL) {
        return;
    }

    pcd_req_t *req = dev_req->priv;

    int ret = unipro_send_tunnel_cmd(0, PCD_SETUP, 0, req->setup.ptr, req->setup.size);

    if (ret) {
        lldbg("Failed to send ep0 out\n");

        ret = EP_STALL(s_data.usbdev->ep0);
        if (ret) {
            lldbg("Failed to stall ep0 (%d)\n", ret);
        }
    }

    pcd_req_mem_free(req);
    if (req->dev_req)
        EP_FREEREQ(ep, req->dev_req);

    USBTUN_FREE(req);
}

static pcd_req_t *alloc_setup_req(size_t dlen) {
    pcd_req_t *req = USBTUN_ALLOC(sizeof(*req));
    uint8_t *setup_ptr = NULL;

    if (req) {
        size_t buf_size;

        if (dlen) {
            /* Allocate single memory so OUT data follows on SETUP content  */
            buf_size = sizeof(struct usb_ctrlreq_s) + dlen;
            setup_ptr = bufram_alloc(buf_size);
            if (!setup_ptr)
                goto alloc_failure;

            struct usbdev_req_s *dev_req;
            dev_req = alloc_request(s_data.usbdev->ep0, &ep0_out_callback, req);
            if (!dev_req)
                goto alloc_failure;

            req->dev_req = dev_req;
            dev_req->buf = setup_ptr + sizeof(struct usb_ctrlreq_s);
            dev_req->len = dlen;
        } else {
            buf_size = sizeof(struct usb_ctrlreq_s);
            setup_ptr = bufram_alloc(buf_size);
            if (!setup_ptr)
                goto alloc_failure;
        }

        req->ep = 0;
        req->setup.type = USBTUN_MEM_BUFRAM;
        req->setup.ptr = setup_ptr;
        req->setup.size = buf_size;
    }

    return req;

alloc_failure:
    if (setup_ptr)
        bufram_free(setup_ptr);

    if (req)
        USBTUN_FREE(req);

    return NULL;
}

static int _usbclass_bind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    s_data.usbdev = dev;
    DEV_SETSELFPOWERED(dev);

    /* preallocate ep_req_s items for control endpoints */
    int i;
    for(i = 0; i < NUM_CTRL_REQS; i++) {
        pcd_req_t *req = USBTUN_ALLOC(sizeof(*req));
        if (req) {
            void *ptr = bufram_alloc(sizeof(struct usb_ctrlreq_s));
            if (!ptr)
                continue;

            req->setup.type = USBTUN_MEM_BUFRAM;
            req->setup.ptr = ptr;
            req->setup.size = sizeof(struct usb_ctrlreq_s);

            req->ep = 0;
            struct usbdev_req_s *dev_req;
            dev_req = alloc_request(s_data.usbdev->ep0, &req_ctrl_callback, req);
            if (dev_req) {
                req->dev_req = dev_req;
                usbtun_req_q(0, &req->entry);
            } else {
                pcd_req_mem_free(req);
                USBTUN_FREE(req);
            }
        }
    }

    lldbg("Prepared EP0 requests\n");

    return OK;
}

static void _usbclass_unbind(struct usbdevclass_driver_s *driver, struct usbdev_s *dev) {
    lldbg("class unbind\n");

    int i;
    for (i = 1; i < MAX_ENDPOINTS; i++) {
        if (s_data.ep_list[i].len) {
            _unconfigure_ep(i);
        }
    }

    _unconfigure_ep(0);

    memcpy(s_data.ep_list, 0, sizeof(s_data.ep_list));
}

static int _usbclass_setup(struct usbdevclass_driver_s *driver, struct usbdev_s *dev,
                           const struct usb_ctrlreq_s *ctrl, uint8_t * dataout, size_t outlen) {

    uint16_t len = GETUINT16(ctrl->len);
#ifdef USBTUN_DEBUG
    if (USBTUN_DEBUG_EP == 0) {
        uint16_t val = GETUINT16(ctrl->value);
        uint16_t idx = GETUINT16(ctrl->index);

        lldbg("setup %p(type=%x, req=%x, val=%x, idx=%x, len=%d)\n", ctrl,
              ctrl->type, ctrl->req, val, idx, len);
    }
#endif
    size_t data_size = 0;
    if (USB_REQ_ISOUT(ctrl->type) && len) {
        /* Need to wait for incoming data in data phase */
        data_size = len;
    }

    pcd_req_t *req = alloc_setup_req(data_size);

    if (!req) {
        lldbg("No memory available to send SETUP\n");
        return -ENOMEM;
    }

    memcpy(req->setup.ptr, ctrl, sizeof(*ctrl));

    int ret;

    if (data_size == 0) {
        /* No data phase. Send it right away */
        ret = unipro_send_tunnel_cmd(0, PCD_SETUP, 0, req->setup.ptr, sizeof(*ctrl));

        if (ret) {
            lldbg("Failed to send SETUP request\n");
        }
        pcd_req_mem_free(req);
    } else {
        ret = EP_SUBMIT(s_data.usbdev->ep0, req->dev_req);
        if (ret)
            lldbg("Failed sumit data phase request\n");
    }

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
    uint8_t orig;
    struct usbdev_ep_s *ep;
    int ret;

    in = desc->addr & USB_DIR_IN ? 1 : 0;
    epno = desc->addr & USB_EPNO_MASK;

    ep = DEV_ALLOCEP(s_data.usbdev, epno, in, desc->type);
    if (!ep) {
        lldbg("Failed to allocate EP %d\n", orig);
        return -ENODEV;
    }
    ep->priv = desc;

    ret = EP_CONFIGURE(ep, desc, false);
    if (ret) {
        lldbg("Failed to configure EP %d - ret %d\n", orig, ret);
        goto failout;
    }

    s_data.ep[epno] = ep;

    lldbg("EP :%d - configured - eqlog = %d\n", epno, ep->eplog);

    return 0;
failout:
    DEV_FREEEP(s_data.usbdev, ep);
    return ret;
}

static void _prepare_in_ep(uint8_t epno) {

    int i;
    pcd_req_t *req;
    struct usbdev_req_s * dev_req;

    for (i = 0; i < NUM_IN_REQS; i++) {
        req = USBTUN_ALLOC(sizeof(*req));
        if (req) {
            req->ep = epno;
            dev_req = alloc_request(s_data.ep[epno], &req_in_callback, req);

            if (dev_req) {
                req->dev_req = dev_req;
                usbtun_req_q(epno, &req->entry);
            } else {
                USBTUN_FREE(req);
            }
        }
    }
    lldbg("EP IN %d - prepared buffer\n", epno);
}


static void _prepare_out_ep(uint8_t epno) {

    int i;
    pcd_req_t *req;
    struct usbdev_req_s * dev_req;
    void *ptr;

    for (i = 0; i < NUM_OUT_REQS; i++) {
        req = USBTUN_ALLOC(sizeof(*req));
        if (req) {
            req->ep = epno;
            dev_req = alloc_request(s_data.ep[epno], &req_out_callback, req);

            if (!dev_req) {
                USBTUN_FREE(req);
                continue;
            }

            ptr = bufram_alloc(1024);
            if (!ptr) {
                lldbg("Failed to allocate bufram for EP %d\n", epno);
                EP_FREEREQ(s_data.ep[epno], dev_req);
                USBTUN_FREE(req);
                continue;
            }
            req->data.type = USBTUN_MEM_BUFRAM;
            req->data.ptr = ptr;
            req->data.size = 1024;
            dev_req->buf = ptr;
            dev_req->len = 1024;
            req->dev_req = dev_req;

            if (EP_SUBMIT(s_data.ep[epno], dev_req)) {
                lldbg("Failed to allocate URB for EP %d\n", epno);
                pcd_req_mem_free(req);
                EP_FREEREQ(s_data.ep[epno], dev_req);
                USBTUN_FREE(req);
                continue;
            }
        }
    }
    lldbg("EP OUT %d - prepared buffer\n", epno);
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

    pcd_req_t *req;
    while ((req = (pcd_req_t *)usbtun_req_dq_usb(epno)) != NULL) {
        EP_CANCEL(ep, req->dev_req);
        usbtun_req_q(epno, &req->entry);
    }
    while ((req = (pcd_req_t *)usbtun_req_dq(epno)) != NULL) {
        EP_FREEREQ(ep, req->dev_req);
        pcd_req_mem_free(req);
        USBTUN_FREE(req);
    }

    if (epno != 0)
        DEV_FREEEP(s_data.usbdev, ep);
}

static void _handle_ep_list(void *list, size_t len) {
    if (len == sizeof(s_data.ep_list)) {
        int i;
        struct usb_epdesc_s *ep_list = list;
        for (i = 0; i < MAX_ENDPOINTS; i++) {
            if (ep_list[i].len) {
                lldbg("EP %d, addr = %02x, attr=%02x, max=%d, int=%02x\n",
                      i,
                      ep_list[i].addr,
                      ep_list[i].attr,
                      GETUINT16(ep_list[i].mxpacketsize),
                      ep_list[i].interval);
            }

            if (s_data.ep_list[i].len) {
                if (ep_list[i].len)
                    continue;

                /* TODO: need to unprepare EP */
            } else {
                if (!ep_list[i].len)
                    continue;

                if (_configure_ep(&ep_list[i]) == 0) {
                    if ((ep_list[i].addr & USB_DIR_MASK) == USB_DIR_OUT)
                        _prepare_out_ep(i);
                    else
                        _prepare_in_ep(i);
                }
            }
        }
        memcpy(s_data.ep_list, ep_list, len);
    } else {
        lldbg("Received invalid EP List\n");
    }
}

static void req_ctrl_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *dev_req) {
    if (ep == NULL || dev_req == NULL) {
        return;
    }

#ifdef USBTUN_DEBUG
    if (USBTUN_DEBUG_EP == 0) {
        lldbg("EP %d xfrd: status=%d xfrd=%d, buf=%p\n",
              ep->eplog, dev_req->result, dev_req->xfrd, dev_req->buf);
    }
#endif
    pcd_req_t *req = dev_req->priv;
    /* keep setup memory untouched */
    usbtun_clean_mem(&req->data);
    usbtun_req_from_usb(ep->eplog, &req->entry);
    usbtun_req_q(ep->eplog, &req->entry);
}

static void req_in_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *dev_req) {
    if (ep == NULL || dev_req == NULL) {
        return;
    }
#ifdef USBTUN_DEBUG
    if (USBTUN_DEBUG_EP == ep->eplog) {
        if (ep->eplog != 0) {
            lldbg("EP %d xfrd: status=%d xfrd=%d, buf=%p\n",
                  ep->eplog, dev_req->result, dev_req->xfrd, dev_req->buf);
        }
    }
#endif
    pcd_req_t *req = dev_req->priv;

    pcd_req_mem_free(req);
    usbtun_req_from_usb(ep->eplog, &req->entry);
    usbtun_req_q(ep->eplog, &req->entry);
}

static void req_out_callback(struct usbdev_ep_s *ep, struct usbdev_req_s *dev_req) {
    if (ep == NULL || dev_req == NULL) {
        return;
    }

    pcd_req_t *req = dev_req->priv;

#ifdef USBTUN_DEBUG
    if (USBTUN_DEBUG_EP == ep->eplog) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        lldbg("[%u:%03ld] EP %d OUT status=%d xfrd=%d, buf=%p, ep_req=%p\n",
              ts.tv_sec, ts.tv_nsec / 1000000,
              ep->eplog, dev_req->result, dev_req->xfrd, dev_req->buf, req);
    }
#endif
    if (!s_data.pcd_ready || dev_req->result == -ESHUTDOWN) {
        usbtun_req_from_usb(req->ep, &req->entry);
        usbtun_req_q(ep->eplog, &req->entry);
        return;
    }

    if (dev_req->xfrd)
        unipro_send_tunnel_cmd(ep->eplog, 0, 0, dev_req->buf, dev_req->xfrd);

    req->dev_req->xfrd = 0;
    req->dev_req->result = 0;
    if (EP_SUBMIT(ep, dev_req)) {
        lldbg("Failed to re-queue URB for EP %d\n", ep->eplog);
        usbtun_req_from_usb(req->ep, &req->entry);
        usbtun_req_q(ep->eplog, &req->entry);
    }
}

static usbtun_hdr_res_t handle_hdr(uint8_t ep, uint8_t type, int16_t code, size_t len) {
    if (!s_data.pcd_ready)
        return USBTUN_NO_DATA;

    if (ep == 0) {
        switch (type) {
        case HCD_ROUTER_READY:
            pthread_mutex_lock(&run_lock);
            s_data.hcd_ready = true;
            pthread_cond_signal(&run_cond);
            pthread_mutex_unlock(&run_lock);
            return USBTUN_NO_DATA;
        case HCD_USB_CONNECTED:
            lldbg("hcd usb connected\n");
            pthread_mutex_lock(&run_lock);
            s_data.usb_connected = USB_STATE_CONNECTED;
            pthread_cond_signal(&run_cond);
            pthread_mutex_unlock(&run_lock);
            return USBTUN_NO_DATA;
        case HCD_USB_DISCONNECTED:
            lldbg("hcd usb_disconnected\n");
            pthread_mutex_lock(&run_lock);
            s_data.usb_connected = USB_STATE_DISCONNECTED;
            pthread_cond_signal(&run_cond);
            pthread_mutex_unlock(&run_lock);
            return USBTUN_NO_DATA;
        case HCD_SETUP_RESP:
            if (code != 0) {
                int ret = EP_STALL(s_data.usbdev->ep0);
                if (ret) {
                    lldbg("Failed to stall ep0 (%d)\n", ret);
                } else {
                    lldbg("USB request failure (%d)\n", code);
                }
                return USBTUN_NO_DATA;
            } else if (len == 0) {
                usbtun_buf_t null_buf;
                null_buf.type = USBTUN_MEM_NONE;
                null_buf.ptr = NULL;
                null_buf.size = 0;
                handle_data_body(&null_buf, 0, type);
                return USBTUN_NO_DATA;
            }
            break;
        }
    } else if (len == 0) {
        /* zero length non-control packet */
        usbtun_buf_t null_buf;
        null_buf.type = USBTUN_MEM_NONE;
        null_buf.ptr = NULL;
        null_buf.size = 0;
        handle_data_body(&null_buf, ep, 0);
        return USBTUN_NO_DATA;
    }

    return USBTUN_WAIT_DATA;
}

#ifdef USBTUN_DEBUG
static int counter = 1;
#endif

static usbtun_data_res_t handle_data_body(usbtun_buf_t *buf, uint8_t ep, uint8_t type) {
    int ret;
    pcd_req_t *req;

    if (!s_data.pcd_ready)
        return USBTUN_FREE_BUF;

    if (ep == 0) {
        switch (type) {
        case HCD_SETUP_RESP: {
#ifdef USBTUN_DEBUG
            if (USBTUN_DEBUG_EP == 0)
                lldbg("setup respose packet len = %d\n", buf->size);
#endif
            req = (pcd_req_t *)usbtun_req_dq(0);

            if (req) {
                req->data = *buf;
                req->dev_req->buf = buf->ptr;
                req->dev_req->len = buf->size;
                req->dev_req->xfrd = 0;
                req->dev_req->result = 0;
                req->dev_req->flags = USBDEV_REQFLAGS_NULLPKT;
                usbtun_req_to_usb(0, &req->entry);
                ret = EP_SUBMIT(s_data.usbdev->ep0, req->dev_req);
                if (ret) {
                    usbtun_req_from_usb(0, &req->entry);
                    usbtun_req_q(0, &req->entry);
                    lldbg("Failed to submit USB data\n");
                } else {
                    return USBTUN_KEEP_BUF;
                }
            } else {
                lldbg("Failed to allocate ep_req_s\n");
            }
            break;
        }
        case HCD_ENDPOINTS: {
            _handle_ep_list(buf->ptr, buf->size);
            break;
        }
        default:
            lldbg("Unknow data type to handle\n");
        }
    } else {
        /* must be incoming data for IN indpoint. */

        /* Only allow one request is queued at a time if the endpoint is INTERRUPT. */
        /* TODO: Need to investigate further to see if this worksaround can be removed. */
        if ((s_data.ep_list[ep].attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT &&
            !usbtun_req_is_usb_empry(ep)) {
#ifdef USBTUN_DEBUG
            if (USBTUN_DEBUG_EP == ep) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);

                lldbg("[%u:%03ld][%d] pend EP %d packet\n",
                      ts.tv_sec, ts.tv_nsec / 1000000, counter++, ep);
            }
#endif
            return USBTUN_FREE_BUF;;
        }

        req = (pcd_req_t *)usbtun_req_dq(ep);

        if (req) {
#ifdef USBTUN_DEBUG
            if (USBTUN_DEBUG_EP == ep) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);

                lldbg("[%u:%03ld][%d] EP %d packet len = %d, ep_req=%p\n",
                      ts.tv_sec, ts.tv_nsec / 1000000, counter++, ep, buf->size, req);
            }
#endif
            req->data = *buf;
            req->dev_req->buf = buf->ptr;
            req->dev_req->len = buf->size;
            req->dev_req->xfrd = 0;
            req->dev_req->result = 0;

            usbtun_req_to_usb(ep, &req->entry);
            ret = EP_SUBMIT(s_data.ep[ep], req->dev_req);
            if (ret) {
                lldbg("Failed to submit USB data\n");
                usbtun_req_from_usb(ep, &req->entry);
                usbtun_req_q(ep, &req->entry);
            } else {
                return USBTUN_KEEP_BUF;
            }
        } else {
            //lldbg("No USB request (ep %d)\n", ep);
        }
    }

    return USBTUN_FREE_BUF;
}

static void pcd_start(void) {
    if (s_data.pcd_ready)
        return;

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

    s_data.pcd_ready = true;
    lldbg("PCD router started\n");

    return;

devclose:
    device_close(s_data.pcddev);
    s_data.pcddev = NULL;
}

static void pcd_stop(void) {
    if (!s_data.pcd_ready)
        return;

    lldbg("Stopping PCD router\n");
    s_data.pcd_ready = false;

    device_usbdev_unregister_gadget(s_data.pcddev, &s_data.driver);
    device_close(s_data.pcddev);
    s_data.pcddev = NULL;
}

static void *pcd_router_startup(void *arg) {
    /* TODO: Just temporary debug until HSIC become fully stable */
    usbtun_print_mem_info();

    memset(&s_data, 0, sizeof(s_data));

    if (init_router_common(&handle_hdr, &handle_data_body))
        return NULL;

    pcd_start();

    /* Handshaking with HCD router */
    while(!s_data.hcd_ready && do_run) {
        int ret = unipro_send_tunnel_cmd(0, PCD_ROUTER_READY, 0, NULL, 0);
        if (ret) {
            lldbg("Failed to send ROUTER READY\n");
            goto common_uninit;
        }
        struct timespec expires;
        clock_gettime(CLOCK_REALTIME, &expires);
        uint64_t new_ns = timespec_to_nsec(&expires);
        new_ns += ROUTER_READY_WAIT_NS;
        nsec_to_timespec(new_ns, &expires);

        pthread_mutex_lock(&run_lock);
        pthread_cond_timedwait(&run_cond, &run_lock, &expires);
        pthread_mutex_unlock(&run_lock);
    }
    lldbg("HCD ROUTER READY\n");

    while(true) {
        pthread_mutex_lock(&run_lock);
        if (!do_run) {
            pthread_mutex_unlock(&run_lock);
            break;
        }
        if (s_data.usb_connected == USB_STATE_CONNECTED)
            _connect();
        else if (s_data.usb_connected == USB_STATE_DISCONNECTED) {
            pcd_stop();
            usleep(PCD_RESTART_BACKOFF_US);
            pcd_start();
        }

        pthread_cond_wait(&run_cond, &run_lock);
        pthread_mutex_unlock(&run_lock);
    }

    pcd_stop();

common_uninit:
    uninit_router_common();
    lldbg("PCD router stopped\n");

    /* TODO: temporary debug */
    usbtun_print_mem_info();

    return NULL;
}

int usbtun_pcd_router_init(void) {
    pthread_mutex_lock(&run_lock);
    if (do_run) {
        /* already running. Just return */
        pthread_mutex_unlock(&run_lock);
        return 0;
    }
    do_run = true;
    pthread_mutex_unlock(&run_lock);

    int ret =  pthread_create(&pcd_thread, NULL, pcd_router_startup, NULL);
    if (ret) {
        lldbg("PCD router init failed\n");
        return ret;
    }

    return 0;
}

void usbtun_pcd_router_uninit(void) {
    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        /* Not running. Just return */
        pthread_mutex_unlock(&run_lock);
        return;
    }
    do_run = false;
    pthread_cond_signal(&run_cond);
    pthread_mutex_unlock(&run_lock);

    pthread_join(pcd_thread, NULL);

    return;
}

