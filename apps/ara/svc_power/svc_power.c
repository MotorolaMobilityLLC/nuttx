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
        printk("Power: Usage:\n");
        printk("  power p [interface_nr 0/1]  : get/set interface power\n");
        printk("  power v                     : validate Wake&Detect inputs\n");
        printk("  power w interface_nr assert : assert or pulse WAKEOUT\n");
        printk("\n interface_nr is defined as:\n");
        printk("  index\tname\n");

        interface_foreach(iface, i) {
            printk("  %02d\t%s\n", i, iface->name);
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
                printk("Power: Interface(%02d) %s state = %d\n",
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

            printk("Power: set interface(%02d) %s state to %d\n",
                   int_nr, iface->name, state);

            return state ? interface_pwr_enable(iface) :
                           interface_pwr_disable(iface);
        }

        printk("Power: wrong command\n");
        return ERROR;

    case 'v':
        interface_read_wake_detect();
        break;

    case 'w':
        if (argc == 2) {
            printk("Power: please provide an interface_nr and assert values\n");
            return ERROR;
        }
        if (argc >= 4) {
            /* Generate WAKEOUT pulse on the given interface */
            int_nr = strtol(argv[2], NULL, 10);
            state = strtol(argv[3], NULL, 10);

            iface = interface_get(int_nr);
            if (!iface)
                break;

            printk("Power: %s WAKEOUT on interface(%02d) %s\n",
                   state ? "pulse" : "assert", int_nr, iface->name);

            return interface_generate_wakeout(iface, state);
        }

        printk("Power: wrong command\n");
        return ERROR;

    default:
        printk("Power: wrong command\n");
        return ERROR;
    }

    return 0;
}
