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
    int ms = -1;
    int fail = 0;

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

    printf("Start transfer\n");
    gb_loopback_reset(cport);
    while(i > 0) {
        if (type == 1) {
            if (gb_loopback_ping_host(cport)) {
                fail++;
            }
        }
        if (type == 2) {
            if (gb_loopback_ping_transfer(cport, size)) {
                fail++;
            }
        }
        i--;
    }

    printf("Wait for pending transfer\n");
    sleep(2);
    if (gb_loopback_status(cport) || fail) {
        printf("Transfer failed: %d packet were lost or corrupted.\n",
               gb_loopback_status(cport));
        printf("Transfer failed due to memory errors: %d\n",fail);
    } else {
        printf("Transfer succeed\n");
    }

    return 0;
}
