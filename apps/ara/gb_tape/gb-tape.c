/*
 * Copyright (c) 2015 Google Inc.
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <getopt.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/greybus/tape.h>

static void show_usage(const char *appname)
{
    printf("%s [-r filepath] [-p filepath]\n", appname);
    printf("\t-r: tape greybus communication into 'filepath'\n");
    printf("\t-s: stop current taping\n");
    printf("\t-p: replay greybus tape from 'filepath'\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int gb_tape_main(int argc, char *argv[])
#endif
{
    int c;
    int retval;

    if (argc < 2) {
        show_usage(argc != 1 ? "gb_tape" : argv[0]);
        return -1;
    }

    optind = -1;

    gb_tape_arm_semihosting_register();

    while ((c = getopt(argc, argv, "r:p:s")) != -1) {
        switch (c) {
        case 'r':
            retval = gb_tape_communication(optarg);
            if (retval) {
                fprintf(stderr, "gb_tape: taping error: %s\n",
                        strerror(retval));
            }
            break;

        case 's':
            retval = gb_tape_stop();
            if (retval) {
                fprintf(stderr, "gb_tape: stop taping error: %s\n",
                        strerror(retval));
            }
            break;

        case 'p':
            retval = gb_tape_replay(optarg);
            if (retval) {
                fprintf(stderr, "gb_tape: tape replay error: %s\n",
                        strerror(retval));
            }
            break;

        case '?':
        default:
            fprintf(stderr, "invalid parameter\n");
            show_usage(argv[0]);
            return -1;
        }
    }

    return 0;
}
