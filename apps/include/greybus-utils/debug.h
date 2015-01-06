/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#ifndef _GREYBUS_UTILS_DEBUG_H_
#define _GREYBUS_UTILS_DEBUG_H_

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/greybus/types.h>

#ifdef CONFIG_GREYBUS_DEBUG
static inline void _gb_dump(const char *func, __u8 *buf, size_t size)
{
	  int i;
    irqstate_t flags;

    flags = irqsave();
    lowsyslog("%s:\n", func);
	  for (i = 0; i < size; i++)
		  lowsyslog( "%02x ", buf[i]);
	  lowsyslog("\n");
	  irqrestore(flags);
}

#define gb_dump(buf, size) \
        _gb_dump(__func__, buf, size)
#define gb_debug(fmt, ...) \
        do { lowsyslog("[D] GB: " fmt, ##__VA_ARGS__); } while (0)
#else
#define gb_dump(buf, size) \
        do { } while (0)
#define gb_debug(fmt, ...) \
        do { } while (0)
#endif

/* debug/info/error macros */
#define gb_info(fmt, ...)                                            \
        do { lowsyslog("[I] GB: " fmt, ##__VA_ARGS__); } while (0)
#define gb_error(fmt, ...)                                           \
        do { lowsyslog("[E] GB: " fmt, ##__VA_ARGS__); } while (0)

#endif

