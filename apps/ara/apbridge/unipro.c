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

#include <nuttx/config.h>

#include <errno.h>
#include <arch/tsb/gpio.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>
#include <arch/board/apbridgea_gadget.h>
#include <apps/greybus-utils/utils.h>

#include "apbridge_backend.h"

/*
 * TODO
 * Already defined in tsb_unipro.c
 * Move them to tsb_unipro.h
 */

#define CPORTID_CDSI0    (16)
#define CPORTID_CDSI1    (17)

static int unipro_usb_to_unipro(unsigned int cportid, void *buf, size_t len,
                                unipro_send_completion_t callback, void *priv)
{
    return unipro_send_async(cportid, buf, len, callback, priv);
}

static struct unipro_driver unipro_driver = {
    .name = "APBridge",
    .rx_handler = recv_from_unipro,
};

static void unipro_backend_init(void)
{
    int i;
    unsigned int cport_count = unipro_cport_count();

    /* unipro_init() will initialize any non-display, non-camera CPorts */
    unipro_init();

    /* Now register a driver for those CPorts */
    for (i = 0; i < cport_count; i++) {
        /* These cports are already allocated for display and camera */
        if (i == CPORTID_CDSI0 || i == CPORTID_CDSI1)
            continue;
        unipro_driver_register(&unipro_driver, i);
    }

    /*
     * Tell the SVC that the AP Module is ready
     */
    tsb_unipro_mbox_send(TSB_MAIL_READY_AP);
}

static void unipro_cport_mapping(unsigned int cportid, enum ep_mapping mapping)
{
    switch (mapping) {
    case MULTIPLEXED_EP:
        unipro_set_max_inflight_rxbuf_count(cportid, 1);
        break;

    case DIRECT_EP:
        unipro_set_max_inflight_rxbuf_count(cportid,
                                    CONFIG_TSB_UNIPRO_MAX_INFLIGHT_BUFCOUNT);
        break;
    }
}

void apbridge_backend_register(struct apbridge_backend *apbridge_backend)
{
    apbridge_backend->usb_to_unipro = unipro_usb_to_unipro;
    apbridge_backend->init = unipro_backend_init;
    apbridge_backend->unipro_cport_mapping = unipro_cport_mapping;
}
