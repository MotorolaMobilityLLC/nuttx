/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#ifndef _GREYBUS_VENDOR_MOTO_H_
#define _GREYBUS_VENDOR_MOTO_H_

#include <nuttx/config.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/types.h>

#define MIN_SZ(a,b) (a < b ? a : b)

#define GB_VENDOR_MOTO_PROTOCOL_VERSION   0x01
#define GB_VENDOR_MOTO_GET_DMESG          0x02
#define GB_VENDOR_MOTO_GET_LAST_DMESG     0x03
#define GB_VENDOR_MOTO_GET_PWR_UP_REASON  0x04
#define GB_VENDOR_MOTO_GET_DMESG_SIZE     0x05
#define GB_VENDOR_MOTO_GET_UPTIME         0x06

#define GB_VENDOR_MOTO_DMESG_SIZE \
            MIN_SZ(CONFIG_RAMLOG_BUFSIZE, GB_MAX_PAYLOAD_SIZE)

/* version request has no payload */
struct gb_vendor_moto_proto_version_response {
    __u8    major;
    __u8    minor;
} __packed;

/* power up reason request has no payload */
struct gb_vendor_moto_pwr_up_reason_response {
    __le32  reason;
} __packed;

#ifdef CONFIG_RAMLOG_SYSLOG
/* get (last) dmesg request has no payload */
struct gb_vendor_moto_get_dmesg_response {
    char    buf[GB_VENDOR_MOTO_DMESG_SIZE];
} __packed;

/* get dmesg size request has no payload */
struct gb_vendor_moto_get_dmesg_size_response {
    __le16  size;
} __packed;
#endif /* CONFIG_RAMLOG_SYSLOG */

struct gb_vendor_moto_get_uptime_response {
    __le32 secs;
} __packed;

#endif /* _GREYBUS_VENDOR_MOTO_H_ */
