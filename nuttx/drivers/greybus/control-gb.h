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

#ifndef __CONTROL_GB_H__
#define __CONTROL_GB_H__

#include <nuttx/greybus/types.h>

/* Bundle-id and cport-id for control cport */
#define GB_CONTROL_BUNDLE_ID                    0
#define GB_CONTROL_CPORT_ID                     2

/* Version of the Greybus control protocol we support */
#define GB_CONTROL_VERSION_MAJOR                0x00
#define GB_CONTROL_VERSION_MINOR                0x01

/* Greybus control request types */
#define GB_CONTROL_TYPE_INVALID                 0x00
#define GB_CONTROL_TYPE_PROTOCOL_VERSION        0x01
#define GB_CONTROL_TYPE_PROBE_AP                0x02
#define GB_CONTROL_TYPE_GET_MANIFEST_SIZE       0x03
#define GB_CONTROL_TYPE_GET_MANIFEST            0x04
#define GB_CONTROL_TYPE_CONNECTED               0x05
#define GB_CONTROL_TYPE_DISCONNECTED            0x06

/* version request has no payload */
struct gb_control_proto_version_response {
    __u8      major;
    __u8      minor;
};

/* Control protocol get manifest size request has no payload*/
struct gb_control_get_manifest_size_response {
    __le16    size;
};

/* Control protocol get manifest request has no payload */
struct gb_control_get_manifest_response {
    __u8      data[0];
};

/* Control protocol [dis]connected request */
struct gb_control_connected_request {
    __le16    cport_id;
};
/* Control protocol [dis]connected response has no payload */

#endif /* __CONTROL_GB_H__ */

