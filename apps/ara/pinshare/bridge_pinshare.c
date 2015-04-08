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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>

#include <up_arch.h>

static void print_usage(void) {
    printf("PINSHARE: Usage:\n");
    printf("PINSHARE:  pinshare r                         : display current value of PINSHARE register\n");
    printf("PINSHARE:  pinshare g                         : alias for \"pinshare r\"\n");
    printf("PINSHARE:  pinshare s  <hex bitmask to set>   : set specified bits of PINSHARE register\n");
    printf("PINSHARE:  pinshare c  <hex bitmask to clear> : clear specified bits of PINSHARE register\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
    int pinshare_main(int argc, char *argv[]) {
#endif
    uint32_t mask = 0;
    char cmd = '\0';
    int ret = 0;
    char *endptr = NULL;

    if (argc < 2) {
        print_usage();
        return EXIT_FAILURE;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'h':
    case '?':
        print_usage();
        break;
    case 'r':
    case 'g':
        /* read current value of PINSHARE register */
        if (argc != 2) {
            print_usage();
            ret = EXIT_FAILURE;
        } else {
            printf("Current value of PINSHARE register is: 0x%x\n",
                   tsb_get_pinshare());
        }
        break;
    default:
        /* set or clear specified bits of PINSHARE register */
        if (argc != 3) {
            print_usage();
            ret = EXIT_FAILURE;
        }
        mask = strtoul(argv[2], &endptr, 16);
        if (*endptr) {
            printf("PINSHARE: Unable to parse provided hex bitmask \"%s\"\n",
                   argv[2]);
            ret = EXIT_FAILURE;
        } else {
            switch(cmd) {
            case 's':
                tsb_set_pinshare(mask);
                printf("New value of PINSHARE register is: 0x%x\n",
                       tsb_get_pinshare());
                break;
            case 'c':
                if (mask & 0x1) {
                    printf("PINSHARE: Leaving bit 0 set, to keep UART active!\n");
                    mask &= ~1;
                }
                tsb_clr_pinshare(mask);
                printf("New value of PINSHARE register is: 0x%x\n",
                       tsb_get_pinshare());
                break;
            default:
                printf("PINSHARE: Unknown command\n");
                print_usage();
                ret = EXIT_FAILURE;
            }
        }
    }

    return ret;
}
