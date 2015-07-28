/*
 * Copyright (c) 2014, 2015 Google Inc.
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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>

#include <nuttx/greybus/loopback.h>
#include <nuttx/greybus/loopback-gb.h>
#include <nuttx/util.h>

struct gb_loopback_operation {
    const char *name;
    int op;
};

static const struct gb_loopback_operation gb_loopback_ops[] = {
    {
        .name = "ping",
        .op = GB_LOOPBACK_TYPE_PING,
    },
    {
        .name = "xfer",
        .op = GB_LOOPBACK_TYPE_TRANSFER,
    },
    {
        .name = "sink",
        .op = GB_LOOPBACK_TYPE_SINK,
    }
};

static int op_type_from_str(const char *str)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(gb_loopback_ops); i++) {
        if (strcmp(str, gb_loopback_ops[i].name) == 0) {
            return gb_loopback_ops[i].op;
        }
    }

    return -1;
}

static int print_help(void)
{
    printf(
        "Greybus loopback tool\n\n"
        "Usage:\n"
        "\tgb_loopback [-c CPORT] [-s SIZE] [-t ping|xfer|sink] [-w MS]\n"
        "\t\t-c CPORT:\tcport number\n"
        "\t\t-s SIZE:\tdata size in bytes\n"
        "\t\t-t TYPE:\tloopback operation type\n"
        "\t\t-w MS:\t\twait time after operation\n"
    );

    return 1;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gb_loopback_main(int argc, char *argv[])
#endif
{
    int type = 0, size = 0, cport = -1, ms = 1000, opt;
    struct gb_loopback *loopback;

    while ((opt = getopt (argc, argv, "t:s:c:w:h")) != -1) {
        switch (opt) {
        case 'c':
            if (sscanf(optarg, "%d", &cport) != 1)
                goto help;
            break;
        case 's':
            if (sscanf(optarg, "%d", &size) != 1)
                goto help;
            break;
        case 't':
            if ((type = op_type_from_str(optarg)) < 0)
                goto help;
            break;
        case 'w':
            if (sscanf(optarg, "%d", &ms) != 1)
                goto help;
            break;
        default:
            goto help;
        }
    }

    if (cport == -1) {
        struct list_head *iter;

        list_foreach(&gb_loopback_list, iter) {
            loopback = gb_loopback_from_list(iter);
            if (gb_loopback_cport_conf(loopback, type, size, ms))
                continue;
            if (type == GB_LOOPBACK_TYPE_NONE) {
                if (gb_loopback_status(loopback)) {
                    printf("Transfer failed on cport %d: %d"
                           " packet were lost or corrupted.\n",
                           gb_loopback_to_cport(loopback),
                           gb_loopback_status(loopback));
                } else {
                    printf("transfer succeed on cport %d\n",
                           gb_loopback_to_cport(loopback));
                }
             } else {
                    printf("Start transfer on cport %d\n",
                           gb_loopback_to_cport(loopback));
             }
        }
    } else {
        loopback = gb_loopback_from_cport(cport);
        if (gb_loopback_cport_conf(loopback, type, size, ms)) {
            return -EINVAL;
        }
        if (type == GB_LOOPBACK_TYPE_NONE) {
            if (gb_loopback_status(loopback)) {
                printf("Transfer failed on cport %d: %d"
                       " packet were lost or corrupted.\n",
                       cport, gb_loopback_status(loopback));
            } else {
                printf("transfer succeed on cport %d\n", cport);
            }
        } else {
            printf("Start transfer on cport %d\n", cport);
        }

    }

    return 0;

help:
    return print_help();
}
