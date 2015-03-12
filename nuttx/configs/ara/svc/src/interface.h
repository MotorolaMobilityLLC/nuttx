/**
 * Copyright (c) 2015 Google Inc.
 * Google Confidential/Restricted
 *
 * @brief: Manages an Ara interface block
 */

#ifndef  _INTERFACE_H_
#define  _INTERFACE_H_

struct vreg_data {
    unsigned int gpio;
    unsigned int hold_time;   // Assertion duration, in us
    unsigned int active_high; // Active-high to assert
};

struct interface {
    const char *name;
    unsigned int switch_portid;
    struct vreg_data *vregs;
    size_t nr_vregs;
};

int interface_init(struct interface**, size_t nr_interfaces);
void interface_exit(void);

int interface_pwr_enable(struct interface*);
int interface_pwr_disable(struct interface*);

/*
 * Macro magic.
 */
#define INIT_VREG_DATA(g, t) \
    { \
        .gpio = (VREG_DEFAULT_MODE | g),      \
        .hold_time = t,                       \
        .active_high = 1,                          \
    }

#define INIT_SPRING_VREG_DATA(g) \
    { \
        .gpio = (SPRING_VREG_DEFAULT_MODE | g), \
        .hold_time = 0,                         \
        .active_high = 0,                            \
    }

#define __MAKE_BB_VREG(n) bb ## n ## _vregs
#define MAKE_BB_VREG(n) __MAKE_BB_VREG(n)
#define __MAKE_BB_INTERFACE(n) bb ## n ## _interface
#define MAKE_BB_INTERFACE(n) __MAKE_BB_INTERFACE(n)
#define DECLARE_SPRING_INTERFACE(number, gpio, portid)         \
    static struct vreg_data MAKE_BB_VREG(number)[] = {         \
        INIT_SPRING_VREG_DATA(gpio)                            \
    };                                                         \
                                                               \
    static struct interface MAKE_BB_INTERFACE(number) = {      \
        .name = "spring" #number,                              \
        .vregs = MAKE_BB_VREG(number),                         \
        .nr_vregs = ARRAY_SIZE(MAKE_BB_VREG(number)),          \
        .switch_portid = portid,                               \
    };

#define __MAKE_INTERFACE(n) n ## _interface
#define MAKE_INTERFACE(n) __MAKE_INTERFACE(n)
#define DECLARE_INTERFACE(_name, gpios, portid)                \
    static struct interface MAKE_INTERFACE(_name) = {          \
        .name = #_name,                                        \
        .vregs = gpios,                                        \
        .nr_vregs = ARRAY_SIZE(gpios),                         \
        .switch_portid = portid,                               \
    };

#endif


