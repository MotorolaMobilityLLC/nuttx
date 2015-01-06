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

#define MID_LENGTH 7

static struct apbridge_dev_s *g_usbdev = NULL;
static pthread_t g_svc_thread;

static int usb_to_unipro(struct apbridge_dev_s *dev, void *payload, size_t size)
{
  struct cport_msg *cmsg = (struct cport_msg *)payload;

  gb_dump(cmsg->data, size - 1);

  greybus_rx_handler(cmsg->cport, cmsg->data, size - 1);
  return size;
}

static int usb_to_svc(struct apbridge_dev_s *dev, void *payload, size_t size)
{
  gb_dump(payload, size);

  return svc_handle(payload, size);
}

static int recv_from_svc(void *buf, size_t length)
{
  gb_dump(buf, length);

  return svc_to_usb(g_usbdev, buf, length);
}

static int recv_from_unipro(unsigned int cportid, const void *buf, size_t len)
{
  int ret = -ENODEV;
  struct cport_msg *cmsg;

  cmsg = malloc(len + 1);
  if (!cmsg)
      return -ENOMEM;

  cmsg->cport = cportid;
  memcpy(cmsg->data, buf, len);

  gb_dump(cmsg->data, len);

  if (g_usbdev)
    ret = unipro_to_usb(g_usbdev, cmsg, len + 1);
  free(cmsg);
  return ret;
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
  enable_cports();

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

static struct apbridge_usb_driver usb_driver = {
  .usb_to_unipro = usb_to_unipro,
  .usb_to_svc = usb_to_svc,
  .init = svc_sim_init,
};

static void init(void)
{
}

static int listen(unsigned int cport)
{
  return 0;
}

struct gb_transport_backend gb_unipro_backend = {
    .init = init,
    .listen = listen,
    .send = recv_from_unipro,
};

int bridge_main(int argc, char *argv[])
{
    svc_register(recv_from_svc);

    gb_init(&gb_unipro_backend);

    usbdev_apbinitialize(&usb_driver);

    /* Nothing else to do since usb gadget and greybus handle everything */
#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}
