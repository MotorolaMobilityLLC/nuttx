/****************************************************************************
 * configs/endo/svc/src/up_power.h
 * Endo/SVC power support
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_POWER_H
#define __CONFIGS_ENDO_INCLUDE_UP_POWER_H


/* Interface block number for power and WAKEOUT control */
enum {
    PWR_SPRING_A,
    PWR_SPRING_B,
    PWR_SPRING_C,
    PWR_SPRING_D,
    PWR_SPRING_E,
    PWR_SPRING_F,
    PWR_SPRING_G,
    PWR_SPRING_H,
    PWR_SPRING_I,
    PWR_SPRING_J,
    PWR_SPRING_K,
    PWR_SPRING_L,
    PWR_SPRING_M,
    PWR_SPRING_N,
    PWR_SPRING_NR,      /* Number of interfaces */
};

#define WAKEOUT_PULSE_DURATION  100000

void power_enable_internal(void);
void power_interface_block_init(void);
int power_set_power(uint32_t int_nr, bool enable);
bool power_get_power(uint32_t int_nr);
void power_set_wakeout(uint32_t int_mask, bool assert);

void validate_wake_detect(void);
#endif // __CONFIGS_ENDO_INCLUDE_UP_POWER_H
