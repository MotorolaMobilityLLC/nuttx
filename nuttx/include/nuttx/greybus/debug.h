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

#ifndef _GREYBUS_UTILS_DEBUG_H_
#define _GREYBUS_UTILS_DEBUG_H_

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/greybus/types.h>

#include <arch/irq.h>

static inline void _gb_log(const char *fmt, ...)
{
    irqstate_t flags;
    va_list ap;

    va_start(ap, fmt);
    flags = irqsave();
    lowvsyslog(fmt, ap);
    irqrestore(flags);
    va_end(ap);
}

#ifdef CONFIG_GREYBUS_DEBUG
static inline void _gb_dump(const char *func, __u8 *buf, size_t size)
{
    int i;
    irqstate_t flags;

    flags = irqsave();
    lowsyslog("%s:\n", func);
    for (i = 0; i < size; i++) {
        lowsyslog( "%02x ", buf[i]);
    }
    lowsyslog("\n");
    irqrestore(flags);
}

#define gb_dump(buf, size) \
    _gb_dump(__func__, buf, size)
#define gb_debug(fmt, ...) \
    _gb_log("[D] GB: " fmt, ##__VA_ARGS__)
#else
#define gb_dump(buf, size) \
    do { } while (0)
#define gb_debug(fmt, ...) \
    do { } while (0)
#endif

/* debug/info/error macros */
#define gb_info(fmt, ...)                                            \
    _gb_log("[D] GB: " fmt, ##__VA_ARGS__)
#define gb_error(fmt, ...)                                           \
    _gb_log("[D] GB: " fmt, ##__VA_ARGS__)

#endif

