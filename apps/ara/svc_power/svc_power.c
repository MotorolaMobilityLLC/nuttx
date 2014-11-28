/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#define DBG_COMP DBG_POWER
#include "up_debug.h"
#include "up_power.h"

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int power_main(int argc, char *argv[])
#endif
{
    uint32_t int_nr, state;
    int i;
    char cmd;

    if (argc < 2) {
        printk("Power: Usage:\n");
        printk("  power p [interface_nr 0/1] : get/set interface power\n");
        printk("  power v                    : validate Wake&Detect inputs\n");
        printk("  power w interface_mask     : generate WAKEOUT pulse\n");
        return ERROR;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'p':
        if (argc == 2) {
            /* Get the power states */
            for (i = 0; i < PWR_SPRING_NR; i++)
                printk("Power: Spring %d state = %d\n", i, power_get_power(i));
            return 0;
        }
        if (argc > 3) {
            /* Set the power state */
            int_nr = strtol(argv[2], NULL, 10);
            state = strtol(argv[3], NULL, 10);
            printk("Power: set Spring %d state to %d\n", int_nr, state);
            power_set_power(int_nr, state);
            return 0;
        }

        printk("Power: wrong command\n");
        return ERROR;

    case 'v':
        power_read_wake_detect();
        break;

    case 'w':
        if (argc == 2) {
            printk("Power: please provide a bitmask value\n");
            return ERROR;
        }
        if (argc >= 3) {
            /* Generate WAKEOUT pulse on the interface given by the bitmask */
            state = strtoul(argv[2], NULL, 16);
            for (i = 0; i < PWR_SPRING_NR; i++)
                if (state & (1 << i))
                    printk("Power: generate WAKEOUT pulse for Spring %d\n", i);
            power_set_wakeout(state, false);
            return 0;
        }

        printk("Power: wrong command\n");
        return ERROR;

    default:
        printk("Power: wrong command\n");
        return ERROR;
    }

    return 0;
}
