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

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gb_loopback_main(int argc, char *argv[])
#endif
{
    int i = 1000;
    int type = 0;
    int size = 0;
    int cport = -1;
    int monitor = 0;
    int ms = 1000;
    struct gb_loopback *gb_loopback;

    int c;
    while ((c = getopt (argc, argv, "t:s:c:mw:n:")) != -1) {
        switch (c) {
        case 'c':
            if (sscanf(optarg, "%d", &cport) != 1)
                return -EINVAL;
            break;
        case 's':
            if (sscanf(optarg, "%d", &size) != 1)
                return -EINVAL;
            break;
        case 't':
            if (sscanf(optarg, "%d", &type) != 1)
                return -EINVAL;
            break;
        case 'm':
            monitor = 1;
            break;
        case 'w':
            if (sscanf(optarg, "%d", &ms) != 1)
                return -EINVAL;
            break;
        case 'n':
            if (sscanf(optarg, "%d", &i) != 1)
                return -EINVAL;
            break;
        default:
            return -EINVAL;
        }
    }

    if (cport == -1) {
        struct list_head *iter;

        list_foreach(&gb_loopbacks, iter) {
            gb_loopback = list_to_loopback(iter);
            if (gb_loopback_cport_conf(gb_loopback, type, size, ms))
                continue;
            if (type == 0) {
                if (gb_loopback_status(gb_loopback)) {
                    printf("Transfer failed on cport %d: %d"
                           " packet were lost or corrupted.\n",
                           gb_loopback_to_cport(gb_loopback),
                           gb_loopback_status(gb_loopback));
                } else {
                    printf("transfer succeed on cport %d\n",
                           gb_loopback_to_cport(gb_loopback));
                }
             } else {
                    printf("Start transfer on cport %d\n",
                           gb_loopback_to_cport(gb_loopback));
             }
        }
    } else {
        gb_loopback = cport_to_loopback(cport);
        if (gb_loopback_cport_conf(gb_loopback, type, size, ms)) {
            return -EINVAL;
        }
        if (type == 0) {
            if (gb_loopback_status(gb_loopback)) {
                printf("Transfer failed on cport %d: %d"
                       " packet were lost or corrupted.\n",
                       cport, gb_loopback_status(gb_loopback));
            } else {
                printf("transfer succeed on cport %d\n", cport);
            }
        } else {
            printf("Start transfer on cport %d\n", cport);
        }

    }

    if (monitor == 1) {
        printf("monitor is not curretly supported!\n");
    }

    return 0;
}
