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
 * @brief: Manages an Ara interface block
 */

#ifndef  _INTERFACE_H_
#define  _INTERFACE_H_

#include <errno.h>

#include "vreg.h"

/*
 * Structure storing information about how spring current measurement HW
 * is connected to SVC.
 */
struct pm_data {
    uint8_t adc;        /* ADC instance */
    uint8_t chan;       /* ADC channel */
    uint32_t spin;      /* ADC sign pin */
};

struct interface {
    const char *name;
    unsigned int switch_portid;
#   define ARA_IFACE_FLAG_BUILTIN (1U << 0) /* Connected to built-in UniPro peer
                                         * (like a bridge ASIC on a BDB). */
#   define ARA_IFACE_FLAG_BLOCK   (0U << 0) /* Connected to an interface block
                                         * (like on an endo, or an interface
                                         * block on a BDB). */
    unsigned int flags;
    struct vreg *vreg;
    bool power_state;
    unsigned int wake_out;
    struct pm_data *pm;
};

#define interface_foreach(iface, idx)                       \
        for ((idx) = 0, (iface) = interface_get(idx);       \
             (iface);                                       \
             (idx)++, (iface) = interface_get(idx))

int interface_init(struct interface**,
                   size_t nr_interfaces, size_t nr_spring_ints);
int interface_early_init(struct interface**,
                         size_t nr_interfaces, size_t nr_spring_ints);
void interface_exit(void);
struct interface* interface_get(uint8_t index);
struct interface* interface_get_by_name(const char *name);
struct interface* interface_spring_get(uint8_t index);
uint8_t interface_get_count(void);
uint8_t interface_get_spring_count(void);

const char *interface_get_name(struct interface *iface);
int interface_pwr_enable(struct interface*);
int interface_pwr_disable(struct interface*);
bool interface_get_pwr_state(struct interface *iface);
int interface_generate_wakeout(struct interface *, bool assert);
static inline int interface_read_wake_detect(void)
{
    return -EOPNOTSUPP;
}

/**
 * @brief Test if an interface connects to a built-in peer on the board.
 *
 * Some boards have built-in UniPro peers for some switch ports. For
 * example, BDBs have built-in bridge ASICs. This function tests if an
 * interface is to such ap eer.
 *
 * @return 1 if the interface is connected to a built-in peer, 0 otherwise.
 */
static inline int interface_is_builtin(struct interface *iface) {
    return !!(iface->flags & ARA_IFACE_FLAG_BUILTIN);
}

uint8_t interface_pm_get_adc(struct interface *iface);
uint8_t interface_pm_get_chan(struct interface *iface);
uint32_t interface_pm_get_spin(struct interface *iface);

/*
 * Macro magic.
 */
#define INIT_SPRING_PM_DATA(_adc, _chan, _spin)                \
    {                                                          \
        .adc = _adc,                                           \
        .chan = _chan,                                         \
        .spin = _spin,                                         \
    }

#define __MAKE_BB_WAKEOUT(n) WAKEOUT_SPRING ## n
#define MAKE_BB_WAKEOUT(n) __MAKE_BB_WAKEOUT(n)
#define __MAKE_BB_VREG_DATA(n) bb ## n ## _vreg_data
#define MAKE_BB_VREG_DATA(n) __MAKE_BB_VREG_DATA(n)
#define __MAKE_BB_PM(n) bb ## n ## _pm
#define MAKE_BB_PM(n) __MAKE_BB_PM(n)
#define __MAKE_BB_INTERFACE(n) bb ## n ## _interface
#define MAKE_BB_INTERFACE(n) __MAKE_BB_INTERFACE(n)

#define DECLARE_SPRING_INTERFACE(number, gpio, portid,         \
                                 pm_adc, pm_chan, pm_spin)     \
    static struct vreg_data MAKE_BB_VREG_DATA(number)[] = {    \
        INIT_ACTIVE_LOW_VREG_DATA(gpio, 0)                     \
    };                                                         \
    DECLARE_VREG(spring ## number, MAKE_BB_VREG_DATA(number))  \
                                                               \
    static struct pm_data MAKE_BB_PM(number)[] = {             \
        INIT_SPRING_PM_DATA(pm_adc, pm_chan, pm_spin)          \
    };                                                         \
                                                               \
    static struct interface MAKE_BB_INTERFACE(number) = {      \
        .name = "spring" #number,                              \
        .flags = ARA_IFACE_FLAG_BLOCK,                         \
        .vreg = &MAKE_VREG(spring ## number),                  \
        .switch_portid = portid,                               \
        .wake_out = MAKE_BB_WAKEOUT(number),                   \
        .pm = MAKE_BB_PM(number),                              \
    };

#define __MAKE_INTERFACE(n) n ## _interface
#define MAKE_INTERFACE(n) __MAKE_INTERFACE(n)
#define DECLARE_INTERFACE(_name, vreg_data, portid, _wake_out) \
    DECLARE_VREG(_name, vreg_data)                             \
    static struct interface MAKE_INTERFACE(_name) = {          \
        .name = #_name,                                        \
        .flags = ARA_IFACE_FLAG_BUILTIN,                       \
        .vreg = &MAKE_VREG(_name),                             \
        .switch_portid = portid,                               \
        .wake_out = _wake_out,                                 \
        .pm = NULL,                                            \
    };

#endif
