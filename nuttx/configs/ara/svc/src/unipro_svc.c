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

/**
 * @brief SVC implementation of unipro stack
 */

#define DBG_COMP ARADBG_SVC

#include <errno.h>
#include <stddef.h>

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <nuttx/unipro/unipro.h>

#include <ara_debug.h>
#include "tsb_switch.h"
#include "svc.h"

/* List of registered drivers */
static struct unipro_driver **g_drvs;
unsigned int unipro_cport_count(void) {
    return 5;
}

void unipro_init(void) {
    size_t size = sizeof(struct unipro_driver*) * unipro_cport_count();
    g_drvs = zalloc(size);
    if (!g_drvs) {
        return;
    }
}

void unipro_deinit(void)
{
    int i;

    for (i = 0; i < unipro_cport_count(); i++) {
        unipro_driver_unregister(i);
    }

    free(g_drvs);
    g_drvs = NULL;
}

int unipro_send(unsigned int cportid, const void *buf, size_t len) {
    struct tsb_switch *sw = svc->sw;

    if (!sw) {
        return -ENODEV;
    }

    switch_data_send(sw, (void*)buf, len);

    return 0;
}


int unipro_driver_register(struct unipro_driver *drv, unsigned int cportid) {
    lldbg("Registering driver %s on cport: %u\n", drv->name, cportid);
    /*
     * Only cports 4 and 5 are supported on ES2 silicon
     */
    switch (cportid) {
    case 4:
    case 5:
        break;
    default:
        return -EINVAL;
    }

    if (g_drvs[cportid]) {
        return -EADDRINUSE;
    }

    g_drvs[cportid] = drv;
    lldbg("Registered driver %s on cport: %u\n", drv->name, cportid);

    return 0;
}

int unipro_driver_unregister(unsigned int cportid)
{
    if (cportid >= unipro_cport_count() || !g_drvs[cportid]) {
        return -EINVAL;
    }

    g_drvs[cportid] = NULL;

    return 0;
}

/*
 * Packet entry point into SVC unipro stack. This is usually called
 * by the switch driver when a packet is received.
 */
void unipro_if_rx(unsigned int cportid, void *data, size_t len) {
    struct unipro_driver *drv;
    irqstate_t flags;

    drv = g_drvs[cportid];
    if (!drv) {
        return;
    }

    flags = irqsave();
    drv->rx_handler(cportid, data, len);
    irqrestore(flags);
}

void unipro_rxbuf_free(unsigned int cportid, void *ptr)
{
}
