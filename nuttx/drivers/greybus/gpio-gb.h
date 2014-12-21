/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#ifndef __GPIO_GB_H__
#define __GPIO_GB_H__

#include <nuttx/greybus/types.h>

#define	GB_GPIO_TYPE_PROTOCOL_VERSION	0x01
#define	GB_GPIO_TYPE_LINE_COUNT         0x02
#define	GB_GPIO_TYPE_ACTIVATE           0x03
#define	GB_GPIO_TYPE_DEACTIVATE         0x04
#define	GB_GPIO_TYPE_GET_DIRECTION      0x05
#define	GB_GPIO_TYPE_DIRECTION_IN       0x06
#define	GB_GPIO_TYPE_DIRECTION_OUT      0x07
#define	GB_GPIO_TYPE_GET_VALUE          0x08
#define	GB_GPIO_TYPE_SET_VALUE          0x09
#define	GB_GPIO_TYPE_SET_DEBOUNCE       0x0a

/* version request has no payload */
struct gb_gpio_proto_version_response {
	__u8	major;
	__u8	minor;
};

/* line count request has no payload */
struct gb_gpio_line_count_response {
	__u8	count;
};

struct gb_gpio_activate_request {
	__u8	which;
};
/* activate response has no payload */

struct gb_gpio_deactivate_request {
	__u8	which;
};
/* deactivate response has no payload */

struct gb_gpio_get_direction_request {
	__u8	which;
};
struct gb_gpio_get_direction_response {
	__u8	direction;
};

struct gb_gpio_direction_in_request {
	__u8	which;
};
/* direction in response has no payload */

struct gb_gpio_direction_out_request {
	__u8	which;
	__u8	value;
};
/* direction out response has no payload */

struct gb_gpio_get_value_request {
	__u8	which;
};
struct gb_gpio_get_value_response {
	__u8	value;
};

struct gb_gpio_set_value_request {
	__u8	which;
	__u8	value;
};
/* set value response has no payload */

struct gb_gpio_set_debounce_request {
	__u8	which;
	__le16	usec;
};
/* debounce response has no payload */


#endif /* __GPIO_GB_H__ */

