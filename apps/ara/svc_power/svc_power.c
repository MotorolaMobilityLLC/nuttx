/*
 * Copyright (c) 2014 Google Inc.
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

#define DBG_COMP ARADBG_POWER
#include <ara_debug.h>

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#include "up_power.h"
#include "interface.h"


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int power_main(int argc, char *argv[])
#endif
{
    uint32_t int_nr, state;
    int i;
    char cmd;
    struct interface *iface;

    if (argc < 2) {
        printf("Power: Usage:\n");
        printf("  power p [interface_nr 0/1]  : get/set interface power\n");
        printf("  power w interface_nr 0/1    : pulse/assert WAKEOUT\n");
        printf("\n interface_nr is defined as:\n");
        printf("  index\tname\n");

        interface_foreach(iface, i) {
            printf("  %02d\t%s\n", i, iface->name);
        }
        return ERROR;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'p':
        if (argc == 2) {
            /* Get the power states */
            interface_foreach(iface, i) {
                printf("Power: Interface(%02d) %s state = %d\n",
                       i,
                       iface->name,
                       interface_get_pwr_state(iface));
            }
            return 0;
        }
        if (argc > 3) {
            /* Set the power state */
            int_nr = strtol(argv[2], NULL, 10);
            state = strtol(argv[3], NULL, 10);

            iface = interface_get(int_nr);
            if (!iface)
                break;

            printf("Power: set interface(%02d) %s state to %d\n",
                   int_nr, iface->name, state);

            return state ? interface_pwr_enable(iface) :
                           interface_pwr_disable(iface);
        }

        printf("Power: wrong command\n");
        return ERROR;

    case 'w':
        if (argc == 2) {
            printf("Power: please provide an interface_nr and assert values\n");
            return ERROR;
        }
        if (argc >= 4) {
            /* Generate WAKEOUT pulse on the given interface */
            int_nr = strtol(argv[2], NULL, 10);
            state = strtol(argv[3], NULL, 10);

            iface = interface_get(int_nr);
            if (!iface)
                break;

            printf("Power: %s WAKEOUT on interface(%02d) %s\n",
                   state ? "assert" : "pulse", int_nr, iface->name);

            return interface_generate_wakeout(iface, state);
        }

        printf("Power: wrong command\n");
        return ERROR;

    default:
        printf("Power: wrong command\n");
        return ERROR;
    }

    return 0;
}
