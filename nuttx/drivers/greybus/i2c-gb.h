/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
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

