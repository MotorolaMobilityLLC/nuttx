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

#ifndef _GREYBUS_I2C_H_
#define _GREYBUS_I2C_H_

#include <nuttx/greybus/types.h>

#define GB_I2C_PROTOCOL_VERSION         0x01
#define GB_I2C_PROTOCOL_FUNCTIONALITY   0x02
#define GB_I2C_PROTOCOL_TIMEOUT         0x03
#define GB_I2C_PROTOCOL_RETRIES         0x04
#define GB_I2C_PROTOCOL_TRANSFER        0x05

/* version request has no payload */
struct gb_i2c_proto_version_response {
	__u8	major;
	__u8	minor;
};

struct gb_i2c_functionality_rsp {
	__le32	functionality;
};

struct gb_i2c_transfer_desc {
	__le16	addr;
	__le16	flags;
	__le16	size;
};

struct gb_i2c_transfer_req {
	__le16	op_count;
	struct gb_i2c_transfer_desc desc[0];
};

struct gb_i2c_transfer_rsp {
	__u8	data[0];
};

#endif /* _GREYBUS_I2C_H_ */

