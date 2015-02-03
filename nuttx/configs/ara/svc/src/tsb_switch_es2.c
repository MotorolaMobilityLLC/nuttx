/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Jean Pihet
 */

#define DBG_COMP    DBG_SWITCH

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <errno.h>

#include "up_debug.h"
#include "tsb_switch.h"
#include "tsb_switch_driver_es2.h"

#define SWITCH_DEVICE_ID    (0)
#define SWITCH_PORT_ID      (14)


static int es2_switch_attr_get(struct tsb_switch_driver *drv,
                               uint16_t attrid,
                               uint32_t *val)
{
    return -ENOSYS;
}

static struct tsb_switch_driver es2_drv = {
    .switch_attr_get = es2_switch_attr_get,
};

struct tsb_switch_driver *tsb_switch_es2_init(unsigned int spi_bus)
{
    return NULL;
}

void tsb_switch_es2_exit(void) {
}
