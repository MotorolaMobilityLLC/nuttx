/**
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
#include <nuttx/greybus/tsb_unipro.h>
#include <nuttx/unipro/unipro.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define NUM_CPORTS   (4)

#define xstr(s) str(s)
#define str(s) #s

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
    /* pass it off to greybus core? */
    // gb_message_handler()
    unipro_rxbuf_free(cportid, data);
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
    int rc;
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

    rc = unipro_attr_access(attr, &val, selector, peer, 0);

    printf("attr: %x peer: %u selector: %u val: %x resultcode: %x\n", attr, peer, selector, val, rc);

    return rc;
}

static void print_usage(char **argv) {
    printf("Usage: %s -r attr | -w attr [-s <selector] [-p]\n", argv[0]);
    printf("\tOptions:\n");
    printf("\t-r\t Read an attribute\n");
    printf("\t-w\t Write an attribute\n");
    printf("\t-s\t Selector index (optional, default 0)\n");
    printf("\t-p\t Peer access (optional)\n");
}

int unipro_main(int argc, char **argv) {
    char *op;
    int rc;
    char *attr_read_argv[2];
    int c;
    int read = 0;
    int write = 0;
    int peer = 0;
    unsigned int selector = 0;
    unsigned int attr;

    if (argc < 2) {
        return -1;
    }

    op = argv[1];

    if (!strcmp(op, "read") || !strcmp(op, "r")) {
        read = 1;
    } else if (!strcmp(op, "write") || !strcmp(op, "w")) {
        write = 1;
    } else if (strcmp(op, "init") == 0) {
        printf("Initializing unipro.\n");
        unipro_init();
        rc = greybus_register();
        if (rc) {
            printf("Failed to register driver. rc: %d\n", rc);
            exit(1);
        }
    } else if (!strcmp(op, "send") && argc == 3) {
        rc = unipro_send(strtoul(argv[2], NULL, 10), greybus_msg, sizeof greybus_msg);
        if (rc) {
            printf("Failed to send data. rc: %d\n", rc);
            exit(1);
        }
        return 0;
    } else if (!strcmp(op, "info")) {
        unipro_info();
        return 0;
    } else if (!strcmp(op, "tx")) {
        attr_read_argv[0] = (char*) xstr(TSB_DEBUGTXBYTECOUNT);
        attr_read_argv[1] = (char*) "0";
        return attr_read(2, attr_read_argv);
    } else if (strcmp(op, "rx") == 0) {
        attr_read_argv[0] = (char*) xstr(TSB_DEBUGRXBYTECOUNT);
        attr_read_argv[1] = (char*) "0";
        return attr_read(2, attr_read_argv);
    }

    if (!strcmp(op, "read") || !strcmp(op, "r")) {
        read = 1;
    } else if (!strcmp(op, "write") || !strcmp(op, "w")) {
        write = 1;
    }

    attr = strtoul(argv[2], NULL, 16);

    unsigned int val;
    optind = -1; /* Force NuttX's getopt() to re-initialize. */
    argc -= 2;   /* skip over the "unipro r/w" part */
    char **args = argv + 2;
    while (((c = getopt(argc, args, "ps:")) != -1)) {
        switch (c) {
        case 'p':
            peer = 1;
            break;
        case 's':
            selector = strtoul(optarg, NULL, 10);
            break;
        }
    }

    if (write)  {
        if (optind >= argc) {
            printf("Must specify value to write\n");
            print_usage(argv);
            return -1;
        } else {
            val = strtoul(args[optind], NULL, 16);
        }
    }

    printf("%s: %x Peer: %d Selector: %u\n", read ? "Read" : "Write", attr, peer, selector);

    rc = unipro_attr_access(attr, &val, selector, peer, !read);
    if (rc) {
        printf("Attribute access failed: %d\n", rc);
        print_usage(argv);
        return rc;
    }

    if (read) {
        printf("[%x]: %x result_code: %x\n", attr, val, rc);
    } else {
        printf("Wrote %x result_code: %x\n", val, rc);
    }

    return rc;
}
