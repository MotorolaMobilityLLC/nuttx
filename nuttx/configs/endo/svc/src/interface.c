/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 * @brief: Functions and definitions for interface block management.
 * !! Thin and temporary layer between up_power.h and interface.h       !!
 * !! To be replaced with the full-featured interface abstraction layer !!
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "up_power.h"
#include "interface.h"

static struct interface interfaces[] = {
    { "Spring A", 0 },
    { "Spring B", 1 },
    { "Spring C", 2 },
    { "Spring D", 3 },
    { "Spring E", 4 },
    { "Spring F", 5 },
    { "Spring G", 6 },
    { "Spring H", 7 },
    { "Spring I", 8 },
    { "Spring J", 9 },
    { "Spring K", 10 },
    { "Spring L", 11 },
    { "Spring M", 12 },
    { "Spring N", 13 },
};


struct interface* interface_get(uint8_t index)
{
    if (index >= PWR_SPRING_NR)
        return NULL;

    return &interfaces[index];
}

int interface_generate_wakeout(struct interface *iface, bool assert)
{
    power_set_wakeout(1 << iface->nr, assert);

    return 0;
}
