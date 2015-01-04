/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <arch/tsb/unipro.h>
#include <nuttx/greybus/greybus.h>

static struct unipro_driver greybus_driver = {
    .name = "greybus",
    .rx_handler = greybus_rx_handler,
};

static int gb_unipro_listen(unsigned int cport)
{
    return unipro_driver_register(&greybus_driver, cport);
}

struct gb_transport_backend gb_unipro_backend = {
    .init = unipro_init,
    .send = unipro_send,
    .listen = gb_unipro_listen,
};

int gb_unipro_init(void)
{
    return gb_init(&gb_unipro_backend);
}
