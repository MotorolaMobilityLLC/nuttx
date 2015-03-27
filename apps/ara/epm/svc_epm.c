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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#define DBG_COMP DBG_EPM
#include "up_debug.h"
#include "up_epm.h"

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int epm_main(int argc, char *argv[])
#endif
{
    uint32_t epm_nr, state;
    int i;
    char cmd;

    if (argc < 2) {
        printk("EPM: Usage:\n");
        printk("EPM:  epm c [0/1]        : get/set 28V charge pump state\n");
        printk("EPM:  epm e [epm_nr 0/1] : get/set EPM state\n");
        return ERROR;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'c':
        if (argc == 2) {
            /* Get the 28V charge pump state */
            printk("EPM: 28V charge pump state = %d\n", epm_get_28v_state());
            return 0;
        }

        if (argc >= 3) {
            /* Set the 28V charge pump state */
            state = strtol(argv[2], NULL, 10);
            printk("EPM: set 28V charge pump state to %u\n", state);
            epm_activate_28v(state);
            return 0;
        }
        break;
    case 'e':
        if (argc == 2) {
            /* Get the EPMs states */
            for (i = 0; i < NR_EPMS; i++)
                printk("EPM %d state = %d\n", i, epm_get_state(i));
            return 0;
        }

        if (argc > 3) {
            /* Set the EPM state */
            epm_nr = strtol(argv[2], NULL, 10);
            state = strtol(argv[3], NULL, 10);
            printk("EPM: set EPM %u state to %u\n", epm_nr, state);
            epm_set_state(epm_nr, state);
            return 0;
        }
        break;
    default:
        printk("EPM: wrong command\n");
        return ERROR;
    }

    return 0;
}
