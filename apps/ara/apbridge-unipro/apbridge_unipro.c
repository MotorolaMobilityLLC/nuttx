/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <nuttx/usb/apb_es1.h>
#include <apps/greybus-utils/utils.h>
#include <arch/tsb/unipro.h>

/*
 * TODO
 * Already defined in tsb_unipro.c
 * Move them to tsb_unipro.h
 */

#define CPORTID_CDSI0    (16)
#define CPORTID_CDSI1    (17)

#define MID_LENGTH 7

static struct apbridge_dev_s *g_usbdev = NULL;
static pthread_t g_svc_thread;

static int usb_to_unipro(struct apbridge_dev_s *dev,
                         void *payload, size_t size)
{
    struct cport_msg *cmsg = (struct cport_msg *)payload;

    gb_dump(cmsg->data, size - 1);

    return unipro_send(cmsg->cport, cmsg->data, size - 1);
}

static int usb_to_svc(struct apbridge_dev_s *dev, void *payload, size_t size)
{
  gb_dump(payload, size);

  return svc_handle(payload, size);
}

static int recv_from_unipro(unsigned int cportid, void *payload, size_t len)
{
    char *buf;
    int ret;

    len = gb_packet_size(payload);

    buf = malloc(len + 1);
    if (!buf)
        return -ENOMEM;

    gb_dump(payload, len);

    /*
     * TODO
     * Update UniPro driver to allocate a buffer that can contain
     * the cport number and the data in order to avoid the recopy.
     */
    memcpy(buf, &cportid, 1);
    memcpy(buf + 1, payload, len);
    ret = unipro_to_usb(g_usbdev, buf, len + 1);

    free(buf);

    return ret;
}


static int recv_from_svc(void *buf, size_t length)
{
  gb_dump(buf, length);

  return svc_to_usb(g_usbdev, buf, length);
}

static void manifest_event(unsigned char *manifest_file, int manifest_number)
{
    char mid[MID_LENGTH];

    snprintf(mid, MID_LENGTH, "MID-%d", manifest_number + 1);
    send_svc_event(0, mid, manifest_file);
}

static void *svc_sim_fn(void * p_data)
{
    struct apbridge_dev_s *priv;

    priv = (struct apbridge_dev_s *)p_data;

    usb_wait(priv);
    send_svc_handshake();
    foreach_manifest(manifest_event);
    return NULL;
}

static int svc_sim_init(struct apbridge_dev_s *priv)
{
    int ret;

    g_usbdev = priv;
    ret = pthread_create (&g_svc_thread, NULL, svc_sim_fn,
                          (pthread_addr_t)priv);
    return ret;
}

struct unipro_driver unipro_driver = {
    .name = "APBridge",
    .rx_handler = recv_from_unipro,
};

static struct apbridge_usb_driver usb_driver = {
    .usb_to_unipro = usb_to_unipro,
    .usb_to_svc = usb_to_svc,
    .init = svc_sim_init,
};

int bridge_main(int argc, char *argv[])
{
    int i;

    unipro_init();
    for (i = 0; i < CPORT_MAX; i++) {
        /* This cports are already allocated for display and camera */
        if (i == CPORTID_CDSI0 || i == CPORTID_CDSI1)
            continue;
        unipro_driver_register(&unipro_driver, i);
    }
    svc_register(recv_from_svc);
    usbdev_apbinitialize(&usb_driver);

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}
