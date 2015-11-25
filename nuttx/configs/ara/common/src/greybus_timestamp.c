/**
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

#include <debug.h>
#include <sys/types.h>
#include <sys/time.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/loopback.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/greybus_timestamp.h>

#define USEC_PER_DAY 86400000000ULL
#define TAG_SIZE (sizeof(struct gb_operation_hdr) + \
    sizeof(struct gb_loopback_transfer_response))
static unsigned int cport_max_idx;

static useconds_t __calc_latency(useconds_t t1, useconds_t t2)
{
    if (t2 > t1)
        return t2 - t1;
    else
        return USEC_PER_DAY - t2 + t1;
}

static __le32 calc_latency(struct timeval *ts, struct timeval *te)
{
    useconds_t t1, t2;

    t1 = timeval_to_usec(ts);
    t2 = timeval_to_usec(te);
    return __calc_latency(t1, t2);
}

static inline int tag_active(struct gb_timestamp *ts, unsigned int cportid)
{
    return (cportid < cport_max_idx && ts->tag);
}

void gb_timestamp_tag_entry_time(struct gb_timestamp *ts,
                                 unsigned int cportid)
{
    if (tag_active(ts, cportid))
        gettimeofday(&ts->entry_time, NULL);
}

void gb_timestamp_tag_exit_time(struct gb_timestamp *ts,
                                unsigned int cportid)
{
    if (tag_active(ts, cportid))
        gettimeofday(&ts->exit_time, NULL);
}

void gb_timestamp_log(struct gb_timestamp *ts, unsigned int cportid,
                      void *payload, size_t len, int id)
{
    struct gb_loopback_transfer_response *resp = payload +
                                                 sizeof(struct gb_operation_hdr);
    __le32 latency;

    if (len < TAG_SIZE)
        return;

    if (tag_active(ts, cportid)) {
        latency = calc_latency(&ts->entry_time,
                               &ts->exit_time);
        if (id == GREYBUS_FW_TIMESTAMP_APBRIDGE)
            resp->reserved0 = latency;
        else
            resp->reserved1 = latency;
    }
}

void gb_timestamp_init(void)
{
    if (cport_max_idx == 0)
        cport_max_idx = unipro_cport_count();
}
