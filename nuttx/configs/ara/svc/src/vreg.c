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
 * @brief: Functions and definitions for the voltage regulator framework
 * @author: Jean Pihet
 */

#define DBG_COMP ARADBG_POWER  /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/gpio.h>

#include <errno.h>

#include <ara_debug.h>
#include "vreg.h"

/**
 * @brief Configure all the GPIOs associated with a voltage regulator
 * to their default states.
 * @param vreg regulator to configure
 */
int vreg_config(struct vreg *vreg) {
    unsigned int i;
    int rc = 0;
    const char *name;

    if (!vreg) {
        return -ENODEV;
    }

    name = vreg->name ? vreg->name : "unknown";
    dbg_verbose("%s %s\n", __func__, name);

    /*
     * If there is no vreg control, do nothing.
     *
     * This happens in the case the interface has been declared with a NULL
     * in vreg_data.
     */
    if (!vreg->vregs)
        goto out;

    /* Configure the regulator control pins */
    for (i = 0; i < vreg->nr_vregs; i++) {
        if (!&vreg->vregs[i]) {
            dbg_error("%s: invalid vreg_data for %s\n", __func__, name);
            rc = -EINVAL;
            break;
        }
        dbg_insane("%s: %s vreg, gpio %d\n", __func__,
                   vreg->name ? vreg->name : "unknown",
                   vreg->vregs[i].gpio);
        // First set default value then switch line to output mode
        gpio_set_value(vreg->vregs[i].gpio, vreg->vregs[i].def_val);
        gpio_direction_out(vreg->vregs[i].gpio, vreg->vregs[i].def_val);
    }

out:
    atomic_init(&vreg->use_count, 0);
    vreg->power_state = rc ? VREG_PWR_ERROR : VREG_PWR_DOWN;

    return rc;
}

/**
 * @brief Manage the regulator state; Turn it on when needed
 * @returns: 0 on success, <0 on error
 */
int vreg_get(struct vreg *vreg) {
    unsigned int i;
    int rc = 0;

    if (!vreg) {
        return -ENODEV;
    }

    dbg_verbose("%s %s\n", __func__, vreg->name ? vreg->name : "unknown");

    /*
     * If there is no vreg control, do nothing.
     *
     * This happens in the case the interface has been declared with a NULL
     * in vreg_data.
     */
    if (!vreg->vregs)
        return rc;

    /* Enable the regulator on the first use; Update use count */
    if (atomic_inc(&vreg->use_count) == 1) {
        for (i = 0; i < vreg->nr_vregs; i++) {
            if (!&vreg->vregs[i]) {
                rc = -EINVAL;
                break;
            }
            dbg_insane("%s: %s vreg, gpio %d to %d, hold %dus\n", __func__,
                       vreg->name ? vreg->name : "unknown",
                       vreg->vregs[i].gpio, !!vreg->vregs[i].active_high,
                       vreg->vregs[i].hold_time);
            gpio_set_value(vreg->vregs[i].gpio, vreg->vregs[i].active_high);
            up_udelay(vreg->vregs[i].hold_time);
        }
    }

    /* Update state */
    vreg->power_state = rc ? VREG_PWR_ERROR : VREG_PWR_UP;

    return rc;
}


/**
 * @brief Manage the regulator state; Turn it off when unused
 * @returns: 0 on success, <0 on error
 */
int vreg_put(struct vreg *vreg) {
    unsigned int i;
    int rc = 0;

    if (!vreg) {
        return -ENODEV;
    }

    dbg_verbose("%s %s\n", __func__, vreg->name ? vreg->name : "unknown");

    /*
     * If there is no vreg control, do nothing.
     *
     * This happens in the case the interface has been declared with a NULL
     * in vreg_data.
     */
    if (!vreg->vregs)
        return rc;

    /* If already disabled, do nothing */
    if (!atomic_get(&vreg->use_count))
        return rc;

    /* Disable the regulator on the last use; Update use count */
    if (!atomic_dec(&vreg->use_count)) {
        for (i = 0; i < vreg->nr_vregs; i++) {
            if (!&vreg->vregs[i]) {
                rc = -EINVAL;
                break;
            }
            dbg_insane("%s: %s vreg, gpio %d to %d\n", __func__,
                       vreg->name ? vreg->name : "unknown",
                       vreg->vregs[i].gpio, !vreg->vregs[i].active_high);
            gpio_set_value(vreg->vregs[i].gpio, !vreg->vregs[i].active_high);
        }

        /* Update state */
        vreg->power_state = rc ? VREG_PWR_ERROR : VREG_PWR_DOWN;
    }

    return rc;
}

/**
 * @brief Get vreg power supply state
 * @param vreg Regulator whose power state to retrieve
 * @return vreg's power state, or VREG_PWR_ERROR if vreg == NULL.
 */
enum vreg_pwr_state vreg_get_pwr_state(struct vreg *vreg) {
    if (!vreg) {
        return VREG_PWR_ERROR;
    }

    return vreg->power_state;
}
