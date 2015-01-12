/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <nuttx/config.h>
#include <arch/tsb/unipro.h>

#include <stdio.h>
#include <string.h>

#define NUM_CPORTS   (4)

static int greybus_rx_handler(unsigned int cportid, void *data, size_t len);

static unsigned int greybus_msg[] = {
    0x00facade,
    0xbaddcafe,
    0x1badb002,
    0x1badf00d
};

static struct unipro_driver greybus_driver = {
    .name = "greybus",
    .rx_handler = greybus_rx_handler,
};


/*
 * Called in IRQ context
 */
static int greybus_rx_handler(unsigned int cportid, void *data, size_t len) {
    unsigned int *payload = (unsigned int*)data;

    /* pass it off to greybus core? */
    // gb_message_handler()
    return 0;
}

static int greybus_register(void) {
    int i = 0;
    for (i = 0; i < NUM_CPORTS; i++) {
        unipro_driver_register(&greybus_driver, i);
    }
    return 0;
}

static int attr_read(int argc, char **argv) {
    unsigned int attr;
    unsigned int val;
    int peer = 0;
    unsigned int rc;
    unsigned int selector = 0;

    if (argc < 2) {
        printf("usage: <attr> <1 for peer, 0 for local> <selector(optional)>\n");
        exit(1);
    }

    attr = strtoul(argv[0], NULL, 16);
    peer = strtoul(argv[1], NULL, 10);

    if (argc == 3) {
        selector = strtoul(argv[2], NULL, 10);
    }

    unipro_attr_read(attr, &val, selector, peer, &rc);

    printf("attr: %x peer: %u selector: %u val: %x resultcode: %x\n", attr, peer, selector, val, rc);

    return 0;
}

int unipro_main(int argc, char **argv) {
    char *op;
    int rc;

    if (argc < 2) {
        return -1;
    }

    op = argv[1];

    if (strcmp(op, "r") == 0) {
        printf("Attribute read:\n");
        return attr_read(argc - 2, &argv[2]);

    } else if (strcmp(op, "init") == 0) {
        printf("Initializing unipro.\n");
        unipro_init();
        rc = greybus_register();
        if (rc) {
            printf("Failed to register driver. rc: %d\n", rc);
            exit(1);
        }
    } else if ((strcmp(op, "send") == 0) && argc == 3) {
        rc = unipro_send(strtoul(argv[2], NULL, 10), greybus_msg, sizeof greybus_msg);
        if (rc) {
            printf("Failed to send data. rc: %d\n", rc);
            exit(1);
        }
    } else if (strcmp(op, "info") == 0) {
        unipro_info();
    }

    return 0;
}
