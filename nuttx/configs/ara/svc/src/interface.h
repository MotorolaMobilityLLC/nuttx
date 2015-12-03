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
#include <time.h>

#include <nuttx/wqueue.h>

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

/* Wake & Detect debounce state machine */
enum wd_debounce_state {
    WD_ST_INVALID,                  /* Unknown state */
    WD_ST_INACTIVE_DEBOUNCE,        /* Transition to inactive */
    WD_ST_ACTIVE_DEBOUNCE,          /* Transition to active */
    WD_ST_INACTIVE_STABLE,          /* Stable inactive */
    WD_ST_ACTIVE_STABLE,            /* Stable active */
};

/* Wake & Detect debounce time */
#define WD_DEBOUNCE_TIME_MS         300

/* Hotplug state */
enum hotplug_state {
    HOTPLUG_ST_UNKNOWN,             /* Unknown or unitialized */
    HOTPLUG_ST_PLUGGED,             /* Port is plugged in */
    HOTPLUG_ST_UNPLUGGED,           /* Nothing plugged in port  */
};

/*
 * Wake & Detect signals information
 */
struct wd_data {
    uint16_t gpio;                      /* GPIO number */
    enum wd_debounce_state db_state;    /* Debounce state */
    enum wd_debounce_state last_state;  /* Last stable debounce state */
    struct timeval debounce_tv;         /* Last time of signal debounce check */
    struct work_s work;                 /* Work queue for delayed state check */
};

#define ARA_IFACE_WD_ACTIVE_LOW     false
#define ARA_IFACE_WD_ACTIVE_HIGH    true

/* Interface types. */
enum ara_iface_type {
    /* Connected to built-in UniPro peer (like a bridge ASIC on a BDB). */
    ARA_IFACE_TYPE_BUILTIN,
    /* Connected to an interface block (like on an endo, or an
     * interface block on a BDB). */
    ARA_IFACE_TYPE_BLOCK,
    /* Expansion interface to external connectors (e.g. SMA) */
    ARA_IFACE_TYPE_EXPANSION,
    /* Module port interface, as on DB3 board */
    ARA_IFACE_TYPE_MODULE_PORT,
};

/* Interface flags */
/*  Wake In active low or high signal */
#define ARA_IFACE_FLAG_WAKE_IN_ACTIVE_LOW       (0U << 0)
#define ARA_IFACE_FLAG_WAKE_IN_ACTIVE_HIGH      (1U << 0)
/*  Detect In active low or high signal */
#define ARA_IFACE_FLAG_DETECT_IN_ACTIVE_LOW     (0U << 1)
#define ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH    (1U << 1)

/* Interface power states */
enum ara_iface_pwr_state {
    ARA_IFACE_PWR_ERROR = -1,
    ARA_IFACE_PWR_DOWN = 0,
    ARA_IFACE_PWR_UP = 1,
};

struct interface {
    const char *name;
    unsigned int switch_portid;
    uint8_t dev_id;
    enum ara_iface_type if_type;
    unsigned int flags;
    struct vreg *vreg;
    enum ara_iface_pwr_state power_state;
    struct pm_data *pm;
    unsigned int wake_out;
    struct wd_data wake_in;
    struct wd_data detect_in;
    enum hotplug_state hp_state;
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
int interface_get_id_by_portid(uint8_t portid);
int interface_get_portid_by_id(uint8_t intf_id);
int interface_get_devid_by_id(uint8_t intf_id);
int interface_set_devid_by_id(uint8_t intf_id, uint8_t dev_id);
struct interface* interface_spring_get(uint8_t index);
uint8_t interface_get_count(void);
uint8_t interface_get_spring_count(void);

const char *interface_get_name(struct interface *iface);
int interface_pwr_enable(struct interface*);
int interface_pwr_disable(struct interface*);
enum ara_iface_pwr_state interface_get_pwr_state(struct interface *iface);
int interface_generate_wakeout(struct interface *, bool assert);
int interface_store_hotplug_state(uint8_t port_id, enum hotplug_state hotplug);
enum hotplug_state interface_consume_hotplug_state(uint8_t port_id);
enum hotplug_state interface_get_hotplug_state(struct interface *iface);

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
    return !!(iface->if_type == ARA_IFACE_TYPE_BUILTIN);
}

/* @brief Test if an interface connects to a module port */
static inline int interface_is_module_port(struct interface *iface) {
    return !!(iface->if_type == ARA_IFACE_TYPE_MODULE_PORT);
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

#define INIT_WD_DATA(_gpio)                                    \
    {                                                          \
        .gpio = _gpio,                                         \
        .db_state = WD_ST_INVALID,                             \
        .debounce_tv = { 0, 0 },                               \
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
                                 pm_adc, pm_chan, pm_spin,     \
                                 wake_in_gpio, wake_in_pol,    \
                                 detect_in_gpio, detect_in_pol)\
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
        .if_type = ARA_IFACE_TYPE_BLOCK,                       \
        .flags = (wake_in_pol ?                                \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_HIGH :       \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_LOW) |       \
                 (detect_in_pol ?                              \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH :     \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_LOW),      \
        .vreg = &MAKE_VREG(spring ## number),                  \
        .switch_portid = portid,                               \
        .wake_out = MAKE_BB_WAKEOUT(number),                   \
        .pm = MAKE_BB_PM(number),                              \
        .wake_in = INIT_WD_DATA(wake_in_gpio),                 \
        .detect_in = INIT_WD_DATA(detect_in_gpio),             \
    };

#define __MAKE_INTERFACE(n) n ## _interface
#define MAKE_INTERFACE(n) __MAKE_INTERFACE(n)

#define DECLARE_EXPANSION_INTERFACE(_name, vreg_data, portid,  \
                          _wake_out, wake_in_gpio, wake_in_pol,\
                          detect_in_gpio, detect_in_pol)       \
    DECLARE_VREG(_name, vreg_data)                             \
    static struct interface MAKE_INTERFACE(_name) = {          \
        .name = #_name,                                        \
        .if_type = ARA_IFACE_TYPE_EXPANSION,                   \
        .flags = (wake_in_pol ?                                \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_HIGH :       \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_LOW) |       \
                 (detect_in_pol ?                              \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH :     \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_LOW),      \
        .vreg = &MAKE_VREG(_name),                             \
        .switch_portid = portid,                               \
        .wake_out = _wake_out,                                 \
        .pm = NULL,                                            \
        .wake_in = INIT_WD_DATA(wake_in_gpio),                 \
        .detect_in = INIT_WD_DATA(detect_in_gpio),             \
    };

#define DECLARE_INTERFACE(_name, vreg_data, portid, _wake_out, \
                          wake_in_gpio, wake_in_pol,           \
                          detect_in_gpio, detect_in_pol)       \
    DECLARE_VREG(_name, vreg_data)                             \
    static struct interface MAKE_INTERFACE(_name) = {          \
        .name = #_name,                                        \
        .if_type = ARA_IFACE_TYPE_BUILTIN,                     \
        .flags = (wake_in_pol ?                                \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_HIGH :       \
                    ARA_IFACE_FLAG_WAKE_IN_ACTIVE_LOW) |       \
                 (detect_in_pol ?                              \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH :     \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_LOW),      \
        .vreg = &MAKE_VREG(_name),                             \
        .switch_portid = portid,                               \
        .wake_out = _wake_out,                                 \
        .pm = NULL,                                            \
        .wake_in = INIT_WD_DATA(wake_in_gpio),                 \
        .detect_in = INIT_WD_DATA(detect_in_gpio),             \
    };

/*
 * Module port interface, as on DB3 board.
 *
 * There is only one pin for detect_in, wake_in and wake_out.
 * If there is no Unipro port connected to the interface, portid
 * is INVALID_PORT.
 */
#define DECLARE_MODULE_PORT_INTERFACE(_name, vreg_data,        \
                                      portid,                  \
                                      wake_detect_gpio,        \
                                      detect_in_pol)           \
    DECLARE_VREG(_name, vreg_data)                             \
    static struct interface MAKE_INTERFACE(_name) = {          \
        .name = #_name,                                        \
        .if_type = ARA_IFACE_TYPE_MODULE_PORT,                 \
        .flags = (detect_in_pol ?                              \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_HIGH :     \
                    ARA_IFACE_FLAG_DETECT_IN_ACTIVE_LOW),      \
        .vreg = &MAKE_VREG(_name),                             \
        .switch_portid = portid,                               \
        .wake_out = 0,                                         \
        .pm = NULL,                                            \
        .wake_in = INIT_WD_DATA(0),                            \
        .detect_in = INIT_WD_DATA(wake_detect_gpio),           \
    };

#endif
