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

#include <stdio.h>
#include <string.h>
#include <nuttx/usb/apb_es1.h>
#include <apps/greybus-utils/utils.h>

#include "apbridge_backend.h"

#define IID_LENGTH 7

static int gbsim_usb_to_unipro(unsigned int cportid, void *buf, size_t len)
{
    greybus_rx_handler(cportid, buf, len);
    return len;
}

static int gbsim_usb_to_svc(void *buf, size_t len)
{
    return svc_handle(buf, len);
}

static void init(void)
{
}

static int listen(unsigned int cport)
{
    return 0;
}

static int gbsim_recv_from_unipro(unsigned int cportid,
                                  const void *buf, size_t len)
{
    return recv_from_unipro(cportid, (void *)buf, len);
}

struct gb_transport_backend gb_unipro_backend = {
    .init = init,
    .listen = listen,
    .send = gbsim_recv_from_unipro,
};

static void manifest_enable(unsigned char *manifest_file,
                            int device_id, int manifest_number)
{
    char iid[IID_LENGTH];

    snprintf(iid, IID_LENGTH, "IID-%d", manifest_number + 1);
    enable_manifest(iid, NULL, device_id);
}


void gbsim_backend_init(void)
{
    gb_init(&gb_unipro_backend);
    foreach_manifest(manifest_enable);
    enable_cports();
}

void apbridge_backend_register(struct apbridge_backend *apbridge_backend)
{
    apbridge_backend->usb_to_unipro = gbsim_usb_to_unipro;
    apbridge_backend->usb_to_svc = gbsim_usb_to_svc;
    apbridge_backend->init = gbsim_backend_init;
}
