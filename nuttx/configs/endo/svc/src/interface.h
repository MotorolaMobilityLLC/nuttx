/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 *
 * @brief: Thin and temporary layer between up_power.h and interface.h.
 * !! To be replaced with the full-featured interface abstraction layer !!
 */

#ifndef  _INTERFACE_H_
#define  _INTERFACE_H_

#include "up_power.h"

struct interface {
    const char *name;
    uint32_t nr;
};

#define interface_foreach(iface, idx)                       \
        for ((idx) = 0, (iface) = interface_get(idx);       \
             (iface);                                       \
             (idx)++, (iface) = interface_get(idx))

#define interface_pwr_enable(iface)                         \
    power_set_power(iface->nr, true)

#define interface_pwr_disable(iface)                        \
    power_set_power(iface->nr, false)

#define interface_get_pwr_state(iface)                      \
    power_get_power(iface->nr)

#define interface_read_wake_detect                          \
    power_read_wake_detect

struct interface* interface_get(uint8_t index);
int interface_generate_wakeout(struct interface *iface, bool assert);

#endif
