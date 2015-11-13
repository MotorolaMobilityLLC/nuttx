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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <stddef.h>
#include <errno.h>

#include "tsb_unipro.h"

#include <nuttx/bufram.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/arch.h>

int unipro_set_max_inflight_rxbuf_count(unsigned int cportid,
                                        size_t max_inflight_buf)
{
    struct cport *cport = cport_handle(cportid);
    if (!cport)
        return -EINVAL;

    cport->max_inflight_buf_count = max_inflight_buf;
    return 0;
}

void *unipro_rxbuf_alloc(unsigned int cportid)
{
    struct cport *cport = cport_handle(cportid);
    void *buf;

    if (!cport)
        return NULL;

    if (cport->max_inflight_buf_count != INFINITE_MAX_INFLIGHT_BUFCOUNT &&
        atomic_get(&cport->inflight_buf_count) >=
            cport->max_inflight_buf_count) {
        return NULL;
    }

    buf = bufram_page_alloc(bufram_size_to_page_count(CPORT_BUF_SIZE));
    if (!buf)
        return NULL;

    atomic_inc(&cport->inflight_buf_count);
    return buf;
}

void unipro_rxbuf_free(unsigned int cportid, void *ptr)
{
    struct cport *cport;
    irqstate_t flags;

    cport = cport_handle(cportid);
    if (!cport || !ptr)
        return;

    flags = irqsave();

    if (cport->switch_buf_on_free) {
        cport->switch_buf_on_free = false;
        irqrestore(flags);

        unipro_switch_rxbuf(cportid, ptr);
        unipro_unpause_rx(cportid);
        return;
    }

    irqrestore(flags);

    atomic_dec(&cport->inflight_buf_count);
    bufram_page_free(ptr, bufram_size_to_page_count(CPORT_BUF_SIZE));
}
