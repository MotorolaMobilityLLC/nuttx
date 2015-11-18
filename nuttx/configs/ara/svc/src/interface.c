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

#define DBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>

#include <time.h>
#include <sys/time.h>
#include <errno.h>

#include "stm32.h"
#include <ara_debug.h>
#include "interface.h"
#include "vreg.h"
#include "string.h"
#include "svc.h"
#include "tsb_switch.h"

#define POWER_OFF_TIME_IN_US                        (500000)
#define WAKEOUT_PULSE_DURATION_IN_US                (100000)
#define MODULE_PORT_WAKEOUT_PULSE_DURATION_IN_US    (500000)

static struct interface **interfaces;
static unsigned int nr_interfaces;
static unsigned int nr_spring_interfaces;

static void interface_uninstall_wd_handler(struct wd_data *wd);
static int interface_install_wd_handler(struct wd_data *wd);

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

    /* Configure the WAKEOUT pin according to the interface type */
    switch (iface->if_type) {
    case ARA_IFACE_TYPE_MODULE_PORT:
        /*
         * DB3 module port:
         * The pin is configured as interrupt input at handler
         * installation time
         */
        break;
    default:
        /*
         * Pre-DB3 Interface block, spring or expansion interface:
         * Configure WAKEOUT as input, floating so that it does not interfere
         * with the wake and detect input pin
         */
        if (iface->wake_out) {
            if (stm32_configgpio(iface->wake_out | GPIO_INPUT) < 0) {
                dbg_error("Failed to configure WAKEOUT pin for interface %s\n",
                          iface->name ? iface->name : "unknown");
                rc = -1;
            }
        }
        break;
    }

    iface->power_state = rc ? ARA_IFACE_PWR_ERROR : ARA_IFACE_PWR_DOWN;

    return rc;
}


/**
 * @brief Turn on the power to this interface
 *
 * This function attempts to apply power, clock, etc. to the
 * interface, and updates the interface's power state accordingly.
 * This affects the value returned by interface_get_pwr_state(iface).
 *
 * @param iface Interface whose power to enable
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_enable(struct interface *iface)
{
    int rc;

    if (!iface) {
        dbg_error("%s: called with null interface\n", __func__);
        return -ENODEV;
    }

    rc = vreg_get(iface->vreg);
    if (rc) {
        dbg_error("Can't enable interface %s: %d\n",
                  iface->name ? iface->name : "unknown",
                  rc);
        iface->power_state = ARA_IFACE_PWR_ERROR;
    } else {
        dbg_info("Enabled interface %s.\n",
                 iface->name ? iface->name : "unknown");
        iface->power_state = ARA_IFACE_PWR_UP;
    }

    return rc;
}


/**
 * @brief Turn off the power to this interface
 *
 * This function attempts to remove power, clock, etc. from the
 * interface, and updates the interface's power state accordingly.
 * This affects the value returned by interface_get_pwr_state(iface).
 *
 * @returns: 0 on success, <0 on error
 */
int interface_pwr_disable(struct interface *iface)
{
    int rc;

    if (!iface) {
        dbg_error("%s: called with null interface\n", __func__);
        return -ENODEV;
    }

    rc = vreg_put(iface->vreg);
    if (rc) {
        dbg_error("Can't disable interface %s: %d\n",
                  iface->name ? iface->name : "unknown",
                  rc);
        iface->power_state = ARA_IFACE_PWR_ERROR;
    } else {
        dbg_info("Disabled interface %s.\n",
                 iface->name ? iface->name : "unknown");
        iface->power_state = ARA_IFACE_PWR_DOWN;
    }

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
        dbg_error("%s: called with null interface\n", __func__);
        return -ENODEV;
    }

    dbg_info("Generating WAKEOUT on interface %s\n",
             iface->name ? iface->name : "unknown");

    /* Generate a WAKEOUT pulse according to the interface type */
    switch (iface->if_type) {
    case ARA_IFACE_TYPE_MODULE_PORT:
        /*
         * DB3 module port:
         * Generate a pulse on the WD line. The polarity is reversed
         * from the DETECT_IN polarity.
         */
        if (iface->detect_in.gpio) {
            bool polarity =
                (iface->flags & ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH) ?
                true : false;
            uint32_t pulse_cfg =
                (iface->detect_in.gpio & ~GPIO_MODE_MASK) |
                GPIO_OUTPUT |
                (polarity ? GPIO_OUTPUT_CLEAR : GPIO_OUTPUT_SET);
            /* First uninstall the interrupt handler on the pin */
            interface_uninstall_wd_handler(&iface->detect_in);
            /* Then configure the pin as output and assert it */
            rc = stm32_configgpio(pulse_cfg);
            if (rc < 0) {
                dbg_error("Failed to assert WAKEOUT pin for interface %s\n",
                          iface->name ? iface->name : "unknown");
                return rc;
            }

            /* Keep the line asserted for the given duration */
            up_udelay(MODULE_PORT_WAKEOUT_PULSE_DURATION_IN_US);

            /* Finally re-install the interrupt handler on the pin */
            rc = interface_install_wd_handler(&iface->detect_in);
            if (rc) {
                return rc;
            }
        }
        break;
    default:
        /*
         * Pre-DB3 Interface block, spring or expansion interface:
         * Assert the WAKEOUT line on the interfaces in order to power up
         * the modules.
         * When the WAKEOUT signal is de-asserted the bridges have to assert
         * the PS_HOLD signal asap in order to stay powered up.
         */
        if (iface->wake_out) {
            rc = stm32_configgpio(iface->wake_out |
                                  GPIO_OUTPUT | GPIO_OUTPUT_SET);
            if (rc < 0) {
                dbg_error("Failed to assert WAKEOUT pin for interface %s\n",
                          iface->name ? iface->name : "unknown");
                return rc;
            }

            if (!assert) {
                /* Wait for the bridges to react */
                usleep(WAKEOUT_PULSE_DURATION_IN_US);

                /* De-assert the lines */
                rc = stm32_configgpio(iface->wake_out | GPIO_INPUT);
                if (rc < 0) {
                    dbg_error("Failed to de-assert WAKEOUT pin for interface %s\n",
                              iface->name ? iface->name : "unknown");
                    return rc;
                }
            }
        }
        break;
    }

    return 0;
}


/**
 * @brief Get interface power supply state
 * @param iface Interface whose power state to retrieve
 * @return iface's power state, or ARA_IFACE_PWR_ERROR if iface == NULL.
 */
enum ara_iface_pwr_state interface_get_pwr_state(struct interface *iface)
{
    if (!iface) {
        return ARA_IFACE_PWR_ERROR;
    }

    return iface->power_state;
}


/*
 * Interface power control helper, to be used by the DETECT_IN/hotplug
 * mechanism.
 *
 * Power OFF the interface
 */
static int interface_power_off(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    rc = interface_pwr_disable(iface);
    if (rc < 0) {
        dbg_error("Failed to disable interface %s\n", iface->name);
        return rc;
    }

    return 0;
}

/*
 * Interface power control helper, to be used by the DETECT_IN/hotplug
 * mechanism.
 *
 * Power ON the interface in order to cleanly reboot the interface
 * module(s). Then an initial handshake between the module(s) and the
 * interface can take place.
 */
static int interface_power_on(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    /* If powered OFF, power it ON now */
    if (!interface_get_pwr_state(iface)) {
        rc = interface_pwr_enable(iface);
        if (rc < 0) {
            dbg_error("Failed to enable interface %s\n", iface->name);
            return rc;
        }
    }

    /* Generate WAKE_OUT */
    rc = interface_generate_wakeout(iface, false);
    if (rc) {
        dbg_error("Failed to generate wakeout on interface %s\n", iface->name);
        return rc;
    }

    return 0;
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

    if (port_id == INVALID_PORT) {
        return -ENODEV;
    }

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
    int portid;

    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }

    portid = interfaces[intf_id - 1]->switch_portid;
    if (portid == INVALID_PORT) {
        return -ENODEV;
    }

    return portid;
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
 * @brief Store the hotplug/unplug state of the interface, from the
 * port ID.
 *
 * This function is used to bufferize the last state of the interface
 * while the AP is not yet ready to accept hotplug events. This prevents
 * multiple hotplug and hot-unplug events to be sent to the AP when
 * it is ready.
 */
int interface_store_hotplug_state(uint8_t port_id, enum hotplug_state hotplug)
{
    irqstate_t flags;
    int intf_id;

    intf_id = interface_get_id_by_portid(port_id);
    if (intf_id < 0) {
        dbg_error("%s: cannot get interface from portId %u\n" , __func__,
                  port_id);
        return -EINVAL;
    }

    flags = irqsave();
    interfaces[intf_id - 1]->hp_state = hotplug;
    irqrestore(flags);

    return 0;
}


/**
 * @brief Consume the hotplug/unplug state of the interface, from the
 * port ID.
 *
 * This function is used to retrieve and clear the state of the interface
 * in order to generate an event to be sent to the AP.
 */
enum hotplug_state interface_consume_hotplug_state(uint8_t port_id)
{
    enum hotplug_state hp_state;
    int intf_id;
    irqstate_t flags;

    intf_id = interface_get_id_by_portid(port_id);
    if (intf_id < 0) {
        return HOTPLUG_ST_UNKNOWN;
    }

    flags = irqsave();
    hp_state = interfaces[intf_id - 1]->hp_state;
    interfaces[intf_id - 1]->hp_state = HOTPLUG_ST_UNKNOWN;
    irqrestore(flags);

    return hp_state;
}


/**
 * @brief Get the hotplug state of an interface from the DETECT_IN signal
 */
enum hotplug_state interface_get_hotplug_state(struct interface *iface)
{
    bool polarity, active;
    enum hotplug_state hs = HOTPLUG_ST_UNKNOWN;
    irqstate_t flags;

    flags = irqsave();

    if (iface->detect_in.gpio) {
        polarity = (iface->flags & ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH) ?
                   true : false;
        active = (gpio_get_value(iface->detect_in.gpio) == polarity);
        if (active) {
            hs = HOTPLUG_ST_PLUGGED;
        } else {
            hs = HOTPLUG_ST_UNPLUGGED;
        }
    }

    irqrestore(flags);

    return hs;
}


/**
 * @brief Get the interface struct from the Wake & Detect gpio
 * @returns: interface* on success, NULL on error
 */
static struct interface* interface_get_by_wd(uint32_t gpio)
{
    int i;
    struct interface *ifc;

    interface_foreach(ifc, i) {
        if ((ifc->wake_in.gpio == gpio) ||
            (ifc->detect_in.gpio == gpio)) {
            return ifc;
        }
    }

    return NULL;
}

static int interface_wd_handler(int irq, void *context);
static void interface_wd_delayed_handler(void *data)
{
    struct wd_data *wd = (struct wd_data *) data;

    interface_wd_handler(wd->gpio, NULL);
}

/* Delayed debounce check */
static int interface_wd_delay_check(struct wd_data *wd)
{
    /*
     * If the work is already scheduled, do not schedule another one now.
     * A new one will be scheduled if more debounce is needed.
     */
    if (!work_available(&wd->work)) {
        return 0;
    }

    /* Schedule the work to run after the debounce timeout */
    return work_queue(HPWORK, &wd->work, interface_wd_delayed_handler, wd,
                      MSEC2TICK(WD_DEBOUNCE_TIME_MS));
}

/*
 * Debounce the single WD signal, as on DB3.
 * This handler is also handling the low power mode transitions and
 * wake-ups.
 */
static int interface_debounce_wd(struct interface *iface,
                                 struct wd_data *wd,
                                 bool active)
{
    struct timeval now, diff, timeout_tv = { 0, WD_DEBOUNCE_TIME_MS * 1000 };
    irqstate_t flags;

    flags = irqsave();

    /* Debounce WD signal to act as detection, which will trigger
     * the power on/off of the interface and hotplug notifications to
     * the AP.
     * Short pulses (< WD_DEBOUNCE_TIME_MS = 30ms) are filtered out.
     */
    switch (wd->db_state) {
    case WD_ST_INVALID:
    default:
        gettimeofday(&wd->debounce_tv, NULL);
        wd->db_state = active ?
                       WD_ST_ACTIVE_DEBOUNCE : WD_ST_INACTIVE_DEBOUNCE;
        interface_wd_delay_check(wd);
        break;
    case WD_ST_ACTIVE_DEBOUNCE:
        if (active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                wd->db_state = WD_ST_ACTIVE_STABLE;
                dbg_verbose("W&D: got stable %s_WD %s (gpio %d)\n",
                            iface->name,
                            active ? "Act" : "Ina",
                            wd->gpio);
                /*
                 * WD as DETECT_IN transition to active
                 *
                 * - Power ON the interface
                 *   Note: If coming back to the active stable state from
                 *         the same last stable state after an unstable
                 *         transition, power cycle (OFF/ON) the interface.
                 *         In that case consecutive hotplug events are
                 *         sent to the AP.
                 * - Signal HOTPLUG state to the higher layer
                 */
                if (wd->last_state == WD_ST_ACTIVE_STABLE) {
                    interface_power_off(iface);
                }
                interface_power_on(iface);
                if (iface->switch_portid != INVALID_PORT) {
                    svc_hot_plug(iface->switch_portid);
                }
                /* Save last stable state for power ON/OFF handling */
                wd->last_state = wd->db_state;
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_INACTIVE_DEBOUNCE:
        if (!active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                wd->db_state = WD_ST_INACTIVE_STABLE;
                dbg_verbose("W&D: got stable %s_WD %s (gpio %d)\n",
                            iface->name,
                            active ? "Act" : "Ina",
                            wd->gpio);
                /*
                 * WD as DETECT_IN transition to inactive
                 *
                 * Power OFF the interface
                 * Signal HOTPLUG state to the higher layer
                 */
                interface_power_off(iface);
                if (iface->switch_portid != INVALID_PORT) {
                    svc_hot_unplug(iface->switch_portid);
                }
                /* Save last stable state for power ON/OFF handling */
                wd->last_state = wd->db_state;
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_ACTIVE_STABLE:
        if (!active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_INACTIVE_STABLE:
        if (active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    }

    irqrestore(flags);

    return 0;
}

/* Debounce the wake_in and detect_in signals, as on SDB */
static int interface_debounce_wakein_detectin(struct interface *iface,
                                              struct wd_data *wd,
                                              bool active)
{
    struct timeval now, diff, timeout_tv = { 0, WD_DEBOUNCE_TIME_MS * 1000 };
    irqstate_t flags;

    flags = irqsave();

    /* Debounce signal */
    switch (wd->db_state) {
    case WD_ST_INVALID:
    default:
        gettimeofday(&wd->debounce_tv, NULL);
        wd->db_state = active ?
                       WD_ST_ACTIVE_DEBOUNCE : WD_ST_INACTIVE_DEBOUNCE;
        interface_wd_delay_check(wd);
        break;
    case WD_ST_ACTIVE_DEBOUNCE:
        if (active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                wd->db_state = WD_ST_ACTIVE_STABLE;
                dbg_verbose("W&D: got stable %s_%s %s (gpio %d)\n",
                            iface->name,
                            (wd == &iface->wake_in) ? "wake_in" : "detect_in",
                            active ? "Act" : "Ina",
                            wd->gpio);
                /*
                 * DETECT_IN transition to active
                 *
                 * - Power ON the interface
                 *   Note: If coming back to the active stable state from
                 *         the same last stable state after an unstable
                 *         transition, power cycle (OFF/ON) the interface.
                 *         In that case consecutive hotplug events are
                 *         sent to the AP.
                 * - Signal HOTPLUG state to the higher layer
                 */
                if (wd == &iface->detect_in) {
                    if (wd->last_state == WD_ST_ACTIVE_STABLE) {
                        interface_power_off(iface);
                    }
                    interface_power_on(iface);
                    if (iface->switch_portid != INVALID_PORT) {
                        svc_hot_plug(iface->switch_portid);
                    }
                }
                /* Save last stable state for power ON/OFF handling */
                wd->last_state = wd->db_state;
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_INACTIVE_DEBOUNCE:
        if (!active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                wd->db_state = WD_ST_INACTIVE_STABLE;
                dbg_verbose("W&D: got stable %s_%s %s (gpio %d)\n",
                            iface->name,
                            (wd == &iface->wake_in) ? "wake_in" : "detect_in",
                            active ? "Act" : "Ina",
                            wd->gpio);
                /*
                 * DETECT_IN transition to inactive
                 *
                 * Power OFF the interface
                 * Signal HOTPLUG state to the higher layer
                 */
                if (wd == &iface->detect_in) {
                    interface_power_off(iface);
                    if (iface->switch_portid != INVALID_PORT) {
                        svc_hot_unplug(iface->switch_portid);
                    }
                }
                /* Save last stable state for power ON/OFF handling */
                wd->last_state = wd->db_state;
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_ACTIVE_STABLE:
        if (!active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    case WD_ST_INACTIVE_STABLE:
        if (active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd);
        }
        break;
    }

    irqrestore(flags);

    return 0;
}

/* Interface Wake & Detect handler */
static int interface_wd_handler(int irq, void *context)
{
    struct interface *iface = NULL;
    struct wd_data *wd;
    bool polarity, active;
    int ret;

    /* Retrieve interface from the GPIO number */
    iface = interface_get_by_wd(irq);
    if (!iface) {
        dbg_error("%s: cannot get interface for pin %d\n", __func__, irq);
        return -ENODEV;
    }

    /* Get signal type, polarity, active state etc. */
    if (iface->wake_in.gpio == irq) {
        wd = &iface->wake_in;
        polarity = (iface->flags & ARA_IFACE_FLAG_WAKE_IN_ACTIVE_HIGH) ?
            true : false;
    } else {
        wd = &iface->detect_in;
        polarity = (iface->flags & ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH) ?
            true : false;
    }
    active = (gpio_get_value(irq) == polarity);

    dbg_insane("W&D: got %s_%s %s (gpio %d)\n",
               iface->name,
               (wd == &iface->wake_in) ? "wake_in" : "detect_in",
               active ? "Act" : "Ina",
               irq);

    /* Debounce and handle state changes according to the interface type */
    switch (iface->if_type) {
    case ARA_IFACE_TYPE_MODULE_PORT:
        ret = interface_debounce_wd(iface, wd, active);
        break;
    default:
        ret = interface_debounce_wakein_detectin(iface, wd, active);
        break;
    }

    return ret;
}

/*
 * Uninstall handler for Wake & Detect pin
 */
static void interface_uninstall_wd_handler(struct wd_data *wd)
{
    if (wd->gpio) {
        gpio_mask_irq(wd->gpio);
        gpio_irqattach(wd->gpio, NULL);
    }
}

/*
 * Install handler for Wake & Detect pin
 */
static int interface_install_wd_handler(struct wd_data *wd)
{
    if (wd->gpio) {
        gpio_direction_in(wd->gpio);
        if (set_gpio_triggering(wd->gpio, IRQ_TYPE_EDGE_BOTH) ||
            gpio_irqattach(wd->gpio, interface_wd_handler) ||
            gpio_unmask_irq(wd->gpio)) {
            dbg_error("Failed to attach Wake & Detect handler for pin %d\n",
                      wd->gpio);
            interface_uninstall_wd_handler(wd);
            return -EINVAL;
        }
    }

    return 0;
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
    struct interface *ifc;

    dbg_info("Power off all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    interface_foreach(ifc, i) {
        rc = interface_config(ifc);
        if (rc < 0) {
            dbg_error("Failed to configure interface %s\n", ifc->name);
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
    struct interface *ifc;

    dbg_info("Initializing all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    interface_foreach(ifc, i) {
        /* Initialize the hotplug state */
        ifc->hp_state = interface_get_hotplug_state(ifc);
        /* Power on/off the interface based on the DETECT_IN signal state */
        switch (ifc->hp_state) {
        case HOTPLUG_ST_PLUGGED:
            /* Port is plugged in, power ON the interface */
            if (interface_power_on(ifc) < 0) {
                dbg_error("Failed to power ON interface %s\n", ifc->name);
            }
            break;
        case HOTPLUG_ST_UNPLUGGED:
            /* Port unplugged, power OFF the interface */
            if (interface_power_off(ifc) < 0) {
                dbg_error("Failed to power OFF interface %s\n", ifc->name);
            }
            break;
        case HOTPLUG_ST_UNKNOWN:
        default:
            break;
        }

        /* Install handlers for WAKE_IN and DETECT_IN signals */
        ifc->wake_in.db_state = WD_ST_INVALID;
        ifc->detect_in.db_state = WD_ST_INVALID;
        ifc->wake_in.last_state = WD_ST_INVALID;
        ifc->detect_in.last_state = WD_ST_INVALID;
        rc = interface_install_wd_handler(&ifc->wake_in);
        if (rc)
            return rc;
        rc = interface_install_wd_handler(&ifc->detect_in);
        if (rc)
            return rc;
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
    struct interface *ifc;

    dbg_info("Disabling all interfaces\n");

    if (!interfaces) {
        return;
    }

    /* Uninstall handlers for WAKE_IN and DETECT_IN signals */
    interface_foreach(ifc, i) {
        interface_uninstall_wd_handler(&ifc->wake_in);
        interface_uninstall_wd_handler(&ifc->detect_in);
    }

    /* Power off */
    interface_foreach(ifc, i) {
        rc = interface_pwr_disable(ifc);
        if (rc < 0) {
            dbg_error("Failed to disable interface %s\n", ifc->name);
            /* Continue turning off the rest even if this one failed */
            continue;
        }
    }

    interfaces = NULL;
    nr_interfaces = 0;
}
