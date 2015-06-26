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
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <nuttx/usb/apb_es1.h>
#include <apps/greybus-utils/utils.h>
#include <apps/nsh.h>
#include <arch/tsb/gpio.h>

#ifdef CONFIG_BOARD_HAVE_DISPLAY
#include <arch/board/dsi.h>
#endif

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
#include <arch/board/csi.h>
#endif

#include "apbridge_backend.h"

#define IID_LENGTH 7

static struct apbridge_dev_s *g_usbdev = NULL;
static pthread_t g_svc_thread;
static struct apbridge_backend apbridge_backend;

static int usb_to_unipro(struct apbridge_dev_s *dev, void *buf, size_t len)
{
    struct gb_operation_hdr *hdr = buf;
    unsigned int cportid;

    gb_dump(buf, len);

    if (len < sizeof(*hdr))
        return -EPROTO;

    /*
     * Retreive and clear the cport id stored in the header pad bytes.
     */
    cportid = hdr->pad[1] << 8 | hdr->pad[0];
    hdr->pad[0] = 0;
    hdr->pad[1] = 0;

    return apbridge_backend.usb_to_unipro(cportid, buf, len);
}

static int usb_to_svc(struct apbridge_dev_s *dev, void *buf, size_t len)
{
    gb_dump(buf, len);

    return apbridge_backend.usb_to_svc(buf, len);
}

static int recv_from_svc(void *buf, size_t len)
{
    gb_dump(buf, len);

    return svc_to_usb(g_usbdev, buf, len);
}

int recv_from_unipro(unsigned int cportid, void *buf, size_t len)
{
    struct gb_operation_hdr *hdr = (void *)buf;

    /*
     * FIXME: Remove when UniPro driver provides the actual buffer length.
     */
    len = gb_packet_size(buf);

    gb_dump(buf, len);

    /* Store the cport id in the header pad bytes (if we have a header). */
    if (len >= sizeof(*hdr)) {
        hdr->pad[0] = cportid & 0xff;
        hdr->pad[1] = (cportid >> 8) & 0xff;
    }

    return unipro_to_usb(g_usbdev, buf, len);
}

static void manifest_event(unsigned char *manifest_file,
                           int device_id, int manifest_number)
{
    char iid[IID_LENGTH];

    snprintf(iid, IID_LENGTH, "IID-%d", manifest_number + 1);
    printf("send manifest %d\n", manifest_number);
    send_svc_event(0, iid, manifest_file);
}

static void *svc_sim_fn(void *p_data)
{
    struct apbridge_dev_s *priv = p_data;

    usb_wait(priv);
    apbridge_backend.init();
    send_svc_handshake();
    send_ap_id(0);

    foreach_manifest(manifest_event);

    return NULL;
}

static int svc_sim_init(struct apbridge_dev_s *priv)
{
    int ret;

    g_usbdev = priv;
    ret = pthread_create(&g_svc_thread, NULL, svc_sim_fn,
                         (pthread_addr_t)priv);
    return ret;
}

static struct apbridge_usb_driver usb_driver = {
    .usb_to_unipro = usb_to_unipro,
    .usb_to_svc = usb_to_svc,
    .init = svc_sim_init,
};

int bridge_main(int argc, char *argv[])
{
    tsb_gpio_register(NULL);

    svc_register(recv_from_svc);
    apbridge_backend_register(&apbridge_backend);
    usbdev_apbinitialize(&usb_driver);

#ifdef CONFIG_BOARD_HAVE_DISPLAY
    display_init();
#endif

    sleep(1);

#ifdef CONFIG_ARA_BRIDGE_HAVE_CAMERA
    camera_init();
#endif

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}
