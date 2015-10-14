/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#ifndef _APBRIDGEA_GADGET_H_
#define _APBRIDGEA_GADGET_H_

#include <sys/types.h>
#include <arch/board/common_gadget.h>

/* Vender specific control requests *******************************************/

#define APBRIDGE_RWREQUEST_LOG                  (0x02)
#define APBRIDGE_RWREQUEST_EP_MAPPING           (0x03)
#define APBRIDGE_ROREQUEST_CPORT_COUNT          (0x04)
#define APBRIDGE_WOREQUEST_CPORT_RESET          (0x05)
#define APBRIDGE_ROREQUEST_LATENCY_TAG_EN       (0x06)
#define APBRIDGE_ROREQUEST_LATENCY_TAG_DIS      (0x07)

struct apbridge_dev_s;

enum ep_mapping {
    MULTIPLEXED_EP,
    DIRECT_EP,
};

struct apbridge_usb_driver
{
    int (*usb_to_unipro)(struct apbridge_dev_s *dev, unsigned int cportid,
                         void *payload, size_t size);
    int (*init)(struct apbridge_dev_s *dev);

    void (*unipro_cport_mapping)(unsigned int cportid, enum ep_mapping mapping);
};

int unipro_to_usb(struct apbridge_dev_s *dev, unsigned cportid,
                  const void *payload, size_t size);

void usb_wait(struct apbridge_dev_s *dev);
int usbdev_apbinitialize(struct device *dev,
                         struct apbridge_usb_driver *driver);

int usb_release_buffer(struct apbridge_dev_s *priv, const void *buf);

#endif /* _APBRIDGEA_GADGET_H_ */
