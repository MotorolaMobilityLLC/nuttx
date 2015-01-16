/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 * @brief: Functions and definitions for interface block management.
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>

#include "stm32.h"
#include "up_debug.h"
#include "interface.h"

#define POWER_OFF_TIME_IN_US (500000)

static struct interface **interfaces;
static unsigned int nr_interfaces;

/**
 * @brief Configure all the voltage regulators associated with an interface
 * to their default states.
 * @param iface interface to configure
 */
static int interface_config(struct interface *iface) {
    unsigned int i;
    int rc;

    dbg_verbose("Configuring interface %s.\n",
            iface->name ? iface->name : "unknown");

    for (i = 0; i < iface->nr_vregs; i++) {
        rc = stm32_configgpio(iface->vregs[i].gpio);
        if (rc < 0) {
            return rc;
        }
    }

    return 0;
}


/**
 * @brief Turn on the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_enable(struct interface *iface) {
    unsigned int i;

    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Enabling interface %s.\n",
                iface->name ? iface->name : "unknown");

    for (i = 0; i < iface->nr_vregs; i++) {
        stm32_gpiowrite(iface->vregs[i].gpio, iface->vregs[i].active_high);
        up_udelay(iface->vregs[i].hold_time);
    }

    return 0;
}


/**
 * @brief Turn off the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_disable(struct interface *iface) {
    unsigned int i;

    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Disabling interface %s.\n",
                iface->name ? iface->name : "unknown");


    for (i = 0; i < iface->nr_vregs; i++) {
        stm32_gpiowrite(iface->vregs[i].gpio, !iface->vregs[i].active_high);
    }

    return 0;
}


/**
 * @brief Given a table of interfaces, initialize and enable all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @returns: 0 on success, <0 on error
 * @sideeffects: leaves interfaces powered off on error.
 */
int interface_init(struct interface **ints, size_t nr_ints) {
    unsigned int i;
    int rc;
    int fail = 0;

    dbg_info("Initializing all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_config(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to configure interface %s\n", interfaces[i]->name);
            fail = 1;
            /* Continue configuring remaining interfaces */
            continue;
        }
    }

    if (fail) {
        return -1;
    }

    /* Let everything settle for a good long while.*/
    up_udelay(POWER_OFF_TIME_IN_US);

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_pwr_enable(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to enable interface %s\n", interfaces[i]->name);
            interface_exit();
            return rc;
        }
    }

    return 0;
}


/**
 * @brief Disable all associated power supplies. Must have been previously
 * configured with interface_init()
 */
void interface_exit(void) {
    unsigned int i;
    int rc;

    dbg_info("Disabling all interfaces\n");

    if (!interfaces) {
        return;
    }

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_pwr_disable(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to disable interface %s\n", interfaces[i]->name);
            /* Continue turning off the rest even if this one failed */
            continue;
        }
    }

    interfaces = NULL;
    nr_interfaces = 0;
}

