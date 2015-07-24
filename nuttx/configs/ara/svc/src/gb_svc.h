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
 */

#ifndef _GB_SVC_H_
#define _GB_SVC_H_

#include <nuttx/greybus/types.h>

/* SVC IDs */
#define GB_SVC_CPORT_ID                 0x00
#define GB_SVC_DEVICE_ID                0x01

/* Version of the Greybus SVC protocol we support */
#define GB_SVC_VERSION_MAJOR            0x00
#define GB_SVC_VERSION_MINOR            0x01

/* Greybus SVC request types */
#define GB_SVC_TYPE_INVALID             0x00
#define GB_SVC_TYPE_PROTOCOL_VERSION    0x01
#define GB_SVC_TYPE_HELLO               0x02
#define GB_SVC_TYPE_INTF_DEVICE_ID      0x03
#define GB_SVC_TYPE_INTF_HOTPLUG        0x04
#define GB_SVC_TYPE_INTF_HOT_UNPLUG     0x05
#define GB_SVC_TYPE_INTF_RESET          0x06
#define GB_SVC_TYPE_CONN_CREATE         0x07
#define GB_SVC_TYPE_CONN_DESTROY        0x08
#define GB_SVC_TYPE_ROUTE_CREATE	    0x0b

struct gb_svc_protocol_version_request {
	__u8	major;
	__u8	minor;
};

struct gb_svc_protocol_version_response {
	__u8	major;
	__u8	minor;
};

struct gb_svc_route_create_request {
	__u8	intf1_id;
	__u8	dev1_id;
	__u8	intf2_id;
	__u8	dev2_id;
};

struct gb_svc_hello_request {
	__le16			endo_id;
	__u8			interface_id;
};

struct gb_svc_intf_device_id_request {
    __u8 intf_id;
    __u8 device_id;
};
/* device id response has no payload */

struct gb_svc_intf_hotplug_request {
    __u8 intf_id;
    struct {
        __le32 unipro_mfg_id;
        __le32 unipro_prod_id;
        __le32 ara_vend_id;
        __le32 ara_prod_id;
    } data;
};
/* hotplug response has no payload */

struct gb_svc_intf_hot_unplug_request {
    __u8 intf_id;
};
/* hot unplug response has no payload */

struct gb_svc_intf_reset_request {
    __u8 intf_id;
};
/* interface reset response has no payload */

struct gb_svc_conn_create_request {
    __u8   intf1_id;
    __le16 cport1_id;
    __u8   intf2_id;
    __le16 cport2_id;
    __u8   tc;
    __u8   flags;
};
/* connection create response has no payload */

struct gb_svc_conn_destroy_request {
    __u8  intf1_id;
    __le16 cport1_id;
    __u8  intf2_id;
    __le16 cport2_id;
};
/* connection destroy response has no payload */

int gb_svc_protocol_version(void);
int gb_svc_hello(uint8_t ap_intf_id);
int gb_svc_intf_hotplug(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
void gb_svc_register(int cport);

#endif
