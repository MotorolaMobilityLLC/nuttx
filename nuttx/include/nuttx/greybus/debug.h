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
#include <sys/types.h>
#include <nuttx/config.h>
#include <nuttx/greybus/types.h>

#include <arch/irq.h>

#define GB_LOG_INFO     (0)
#define GB_LOG_ERROR    (1)
#define GB_LOG_WARNING  (2)
#define GB_LOG_DEBUG    (4)
#define GB_LOG_DUMP     (8)

#ifdef CONFIG_GREYBUS_DEBUG
#define gb_log(lvl, fmt, ...)                                       \
    do {                                                            \
        if (gb_log_level & lvl)                                     \
            _gb_log(fmt, ##__VA_ARGS__);                            \
    } while(0)

#define gb_dump(buf, size)                                          \
    do {                                                            \
        if (gb_log_level & GB_LOG_DUMP)                             \
            _gb_dump(__func__, buf, size);                          \
    } while(0);

extern int gb_log_level;
void _gb_dump(const char *func, __u8 *buf, size_t size);
void _gb_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#else
static inline void gb_dump(const char *buf, size_t size) { }
static inline __attribute__ ((format(printf, 2, 3)))
	void gb_log(int level, const char *fmt, ...) { }
#endif

#define gb_info(fmt, ...)                                           \
    gb_log(GB_LOG_INFO, "[I] GB: " fmt, ##__VA_ARGS__);
#define gb_error(fmt, ...)                                          \
    gb_log(GB_LOG_ERROR, "[D] GB: " fmt, ##__VA_ARGS__);
#define gb_warning(fmt, ...)                                        \
    gb_log(GB_LOG_WARNING, "[W] GB: " fmt, ##__VA_ARGS__);
#define gb_debug(fmt, ...)                                          \
    gb_log(GB_LOG_DEBUG, "[D] GB: " fmt, ##__VA_ARGS__);
#endif

