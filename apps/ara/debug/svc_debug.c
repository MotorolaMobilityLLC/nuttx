/*
 * Copyright (C) 2014 Google, Inc.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#define DBG_COMP DBG_DBG
#include "up_debug.h"

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int debug_main(int argc, char *argv[])
#endif
{
    uint32_t comp, level;

    switch (argc) {
    case 1:
        /* Get the debug parameters */
        dbg_get_config(&comp, &level);
        printk("%s(): got comp=0x%x, level=%d\n", __func__, comp, level);
        break;
    case 3:
        /* Set the debug parameters */
        comp = strtol(argv[1], NULL, 16);
        level = strtol(argv[2], NULL, 10);
        dbg_set_config(comp, level);
        printk("%s(): set comp=0x%x, level=%d\n", __func__, comp, level);
        break;
    default:
        printk("%s(): Usage: debug [comp_bit_mask level]\n", __func__);
        return ERROR;
    }

    return 0;
}
