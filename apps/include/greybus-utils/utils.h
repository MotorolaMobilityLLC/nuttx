/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#ifndef _GREYNUS_UTILS_UTILS_H_
#define _GREYNUS_UTILS_UTILS_H_

#include <nuttx/greybus/greybus.h>

#include <apps/greybus-utils/svc.h>
#include <apps/greybus-utils/debug.h>
#include <apps/greybus-utils/manifest.h>

static inline int gb_packet_size(char *rbuf)
{
   struct gb_operation_hdr *hdr = (struct gb_operation_hdr *)rbuf;
   return hdr->size;
}

struct cport_msg {
	__u8	cport;
	__u8	data[0];
};

#endif

