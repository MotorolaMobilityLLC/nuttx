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

#ifndef _COMMON_GADGET_H_
#define _COMMON_GADGET_H_

#include <errno.h>
#include <assert.h>
#include <stddef.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include <arch/irq.h>
#include <arch/byteorder.h>

/**
 * \brief USB string
 * The id must be non null and unique for a language.
 */
struct gadget_string {
    /** String id */
    uint8_t id;
    /** The string */
    const char *str;
};

/** Array of strings for a lang */
struct gadget_strings {
    /** Language id: http://www.usb.org/developers/docs/USB_LANGIDs.pdf */
    uint16_t lang;
    /** NULL terminated array of string */
    const struct gadget_string *strs;
};

struct gadget_config_descriptor {
    const struct usb_cfgdesc_s *cfg;
    const struct usb_desc_s **desc;
};

/** Contain any descriptors required by enumeration */
struct gadget_descriptor {
    /** Pointer to the device descriptor */
    const struct usb_devdesc_s *dev;
    /** Pointer to the qualifier descriptor */
    const struct usb_qualdesc_s *qual;
    /** Pointer to configuration descriptors */
    struct gadget_config_descriptor *cfg;
    /** Pointer to a NULL terminated array of string table. */
    const struct gadget_strings *str;
};

typedef void (*usb_callback)(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);

int request_pool_prealloc(struct usbdev_ep_s *ep, size_t len, int n);
void request_pool_freeall(void);
struct usbdev_req_s *get_request(struct usbdev_ep_s *ep,
                                 usb_callback callback,
                                 size_t len, void *priv);
void put_request(struct usbdev_req_s *req);
struct usbdev_req_s *find_request_by_priv(const void *priv);
struct usbdev_ep_s *request_to_ep(struct usbdev_req_s *req);
void request_set_priv(struct usbdev_req_s *req, void *priv);
void *request_get_priv(struct usbdev_req_s *req);

int gadget_control_handler(struct gadget_descriptor *g_desc,
                           struct usbdev_s *dev,
                           struct usbdev_req_s *req,
                           const struct usb_ctrlreq_s *ctrl);

struct gadget_descriptor *gadget_descriptor_alloc(const struct usb_devdesc_s *dev,
                                                  const struct usb_qualdesc_s *qual);
void gadget_descriptor_free(struct gadget_descriptor *g_desc);
void gadget_add_cfgdesc(struct gadget_descriptor *g_desc,
                        const struct usb_cfgdesc_s *desc,
                        const struct usb_desc_s *usb_desc[]);
void gadget_set_strings(struct gadget_descriptor *g_desc,
                        const struct gadget_strings *str);

/** Output request (host to device) */
#define VENDOR_REQ_OUT      (0)
/** Input request (device to host) */
#define VENDOR_REQ_IN       BIT(0)
/**
 * Output request with data stage (host to device).
 * The request callback will not be called until data stage complete.
 */
#define VENDOR_REQ_DATA     BIT(1)
 /**
  * Let the vendor request submit back the request.
  */
#define VENDOR_REQ_DEFER    BIT(2)

typedef int (*control_request_callback)(struct usbdev_s *dev, uint8_t req,
                                        uint16_t index, uint16_t value,
                                        void *buf, uint16_t len);
int vendor_request_handler(struct usbdev_s *dev,
                           struct usbdev_req_s *req,
                           const struct usb_ctrlreq_s *ctrl);
void vendor_request_complete(struct usbdev_s *dev, int16_t result);
void vendor_data_handler(struct usbdev_req_s *req);
void vendor_request_deferred_submit(struct usbdev_s *dev, int result);
int register_vendor_request(uint8_t req, uint8_t flags,
                            control_request_callback cb);
void unregister_vendor_request(uint8_t req);
void unregister_all_vendor_request(void);

#endif /* _COMMON_GADGET_H_ */

