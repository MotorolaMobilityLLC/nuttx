/*
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
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
