/*
 * Copyright (c) 2015 Google Inc.
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

/**
 * @brief: Functions and definitions for interface block management.
 */

#define DBG_COMP DBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>

#include "stm32.h"
#include "up_debug.h"
#include "interface.h"
#include "vreg.h"
#include "string.h"

#define POWER_OFF_TIME_IN_US            (500000)
#define WAKEOUT_PULSE_DURATION_IN_US    (100000)

static struct interface **interfaces;
static unsigned int nr_interfaces;
static unsigned int nr_spring_interfaces;


/**
 * @brief Configure all the voltage regulators associated with an interface
 * to their default states.
 * @param iface interface to configure
 */
static int interface_config(struct interface *iface)
{
    int rc = 0;

    dbg_verbose("Configuring interface %s.\n",
            iface->name ? iface->name : "unknown");

    /* Configure default state for the regulator pins */
    rc = vreg_config(iface->vreg);

    /*
     * Configure WAKEOUT as input, floating so that it does not interfere
     * with the wake and detect input pin
     */
    if (iface->wake_out) {
        if (stm32_configgpio(iface->wake_out | GPIO_INPUT) < 0) {
            dbg_error("%s: Failed to configure WAKEOUT pin for interface %s\n",
                      __func__, iface->name ? iface->name : "unknown");
            rc = -1;
        }
    }

    iface->power_state = false;

    return rc;
}


/**
 * @brief Turn on the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_enable(struct interface *iface)
{
    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Enabling interface %s.\n",
                iface->name ? iface->name : "unknown");
    vreg_get(iface->vreg);

    /* Update state */
    iface->power_state = true;

    return 0;
}


/**
 * @brief Turn off the power to this interface
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_disable(struct interface *iface)
{
    if (!iface) {
        return -ENODEV;
    }

    dbg_verbose("Disabling interface %s.\n",
                iface->name ? iface->name : "unknown");
    vreg_put(iface->vreg);

    /* Update state */
    iface->power_state = false;

    return 0;
}


/*
 * @brief Generate a WAKEOUT signal to wake-up/power-up modules.
 * If assert is true, keep the WAKEOUT lines asserted.
 *
 * The corresponding power supplies must already be enabled.
 */
int interface_generate_wakeout(struct interface *iface, bool assert)
{
    int rc;

    if (!iface) {
        return -ENODEV;
    }

   /*
    * Assert the WAKEOUT line on the interfaces in order to power up the
    * modules.
    * When the WAKEOUT signal is de-asserted the bridges have to assert
    * the PS_HOLD signal asap in order to stay powered up.
    */
    dbg_verbose("Generating WAKEOUT on interface %s.\n",
                iface->name ? iface->name : "unknown");

    if (iface->wake_out) {
        rc = stm32_configgpio(iface->wake_out | GPIO_OUTPUT | GPIO_OUTPUT_SET);
        if (rc < 0) {
            dbg_error("%s: Failed to assert WAKEOUT pin for interface %s\n",
                      __func__, iface->name ? iface->name : "unknown");
            return rc;
        }

        if (!assert) {
            /* Wait for the bridges to react */
            usleep(WAKEOUT_PULSE_DURATION_IN_US);

            /* De-assert the lines */
            rc = stm32_configgpio(iface->wake_out | GPIO_INPUT);
            if (rc < 0) {
                dbg_error("%s: Failed to de-assert WAKEOUT pin for interface %s\n",
                          __func__, iface->name ? iface->name : "unknown");
                return rc;
            }
        }
    }

    return 0;
}


/*
 * @brief Get interface power supply state, or false if no interface is supplied
 */
bool interface_get_pwr_state(struct interface *iface)
{
    if (!iface) {
        return false;
    }

    return iface->power_state;
}


/**
 * @brief           Return the name of the interface
 * @return          Interface name (string), NULL in case of error.
 * @param[in]       iface: configured interface structure
 */
const char *interface_get_name(struct interface *iface)
{
    if (!iface) {
        return NULL;
    }

    return iface->name;
}

/**
 * @brief Get the interface struct from the index, as specified in the MDK.
 *        Index 0 is for the first interface (aka 'A').
 * @returns: interface* on success, NULL on error
 */
struct interface* interface_get(uint8_t index)
{
    if ((!interfaces) || (index >= nr_interfaces))
        return NULL;

    return interfaces[index];
}


/**
 * @brief           Return the interface struct from the name
 * @returns: interface* on success, NULL on error
 */
struct interface* interface_get_by_name(const char *name)
{
    struct interface *iface;
    int i;

    interface_foreach(iface, i) {
      if (!strcmp(iface->name, name)) {
        return iface;
      }
    }

    return NULL;
}

/*
 * Interface numbering is defined as it's position in the interface table + 1.
 *
 * By convention, the AP module should be interface number 1.
 */

/**
 * @brief find an intf_id given a portid
 */
int interface_get_id_by_portid(uint8_t port_id) {
    unsigned int i;
    for (i = 0; i < nr_interfaces; i++) {
        if (interfaces[i]->switch_portid == port_id) {
            return i + 1;
        }
    }

    return -EINVAL;
}

/**
 * @brief find a port_id given an intf_id
 */
int interface_get_portid_by_id(uint8_t intf_id) {
    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }

    return interfaces[intf_id - 1]->switch_portid;
}

/**
 * @brief find a dev_id given an intf_id
 */
int interface_get_devid_by_id(uint8_t intf_id) {
    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }

    return interfaces[intf_id - 1]->dev_id;
}

/**
 * @brief set a devid for a given an intf_id
 */
int interface_set_devid_by_id(uint8_t intf_id, uint8_t dev_id) {
    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }
    interfaces[intf_id - 1]->dev_id = dev_id;

    return 0;
}

/**
 * @brief           Return the spring interface struct from the index.
 * @warning         Index 0 is for the first spring interface.
 * @return          Interface structure, NULL in case of error.
 * @param[in]       index: configured interface structure
 */
struct interface* interface_spring_get(uint8_t index)
{
    if ((!interfaces) || (index >= nr_spring_interfaces))
        return NULL;

    return interfaces[nr_interfaces - nr_spring_interfaces + index];
}


/**
 * @brief           Return the number of available interfaces.
 * @return          Number of available interfaces, 0 in case of error.
 */
uint8_t interface_get_count(void)
{
    return nr_interfaces;
}


/**
 * @brief           Return the number of available spring interfaces.
 * @return          Number of available spring interfaces, 0 in case of error.
 */
uint8_t interface_get_spring_count(void)
{
    return nr_spring_interfaces;
}


/**
 * @brief           Return the ADC instance used for this interface
 *                  current measurement.
 * @return          ADC instance, 0 in case of error
 * @param[in]       iface: configured interface structure
 */
uint8_t interface_pm_get_adc(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->adc;
}


/**
 * @brief           Return the ADC channel used for this interface
 *                  current measurement.
 * @return          ADC channel, 0 in case of error
 * @param[in]       iface: configured interface structure
 */
uint8_t interface_pm_get_chan(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->chan;
}


/**
 * @brief           Return the measurement sign pin GPIO configuration.
 * @return          Measurement sign pin GPIO configuration, 0 in case of error.
 * @param[in]       iface: configured interface structure
 */
uint32_t interface_pm_get_spin(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->spin;
}

/**
 * @brief Given a table of interfaces, power off all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @param nr_spring_ints number of spring interfaces
 * @returns: 0 on success, <0 on error
 */
int interface_early_init(struct interface **ints,
                         size_t nr_ints, size_t nr_spring_ints) {
    unsigned int i;
    int rc;
    int fail = 0;

    dbg_info("Power off all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_config(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to power interface %s\n", interfaces[i]->name);
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

    return 0;
}


/**
 * @brief Given a table of interfaces, initialize and enable all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @param nr_spring_ints number of spring interfaces
 * @returns: 0 on success, <0 on error
 * @sideeffects: leaves interfaces powered off on error.
 */
int interface_init(struct interface **ints,
                   size_t nr_ints, size_t nr_spring_ints) {
    unsigned int i;
    int rc;

    dbg_info("Initializing all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    for (i = 0; i < nr_interfaces; i++) {
        rc = interface_pwr_enable(interfaces[i]);
        if (rc < 0) {
            dbg_error("Failed to enable interface %s\n", interfaces[i]->name);
            interface_exit();
            return rc;
        }
        rc = interface_generate_wakeout(interfaces[i], true);
        if (rc < 0) {
            dbg_error("Failed to generate wakeout on interface %s\n",
                      interfaces[i]->name);
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
