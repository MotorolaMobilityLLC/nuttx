/****************************************************************************
 * configs/endo/svc/src/up_ioexp.h
 * Endo/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#ifndef __CONFIGS_ENDO_INCLUDE_UP_IOEXP_H
#define __CONFIGS_ENDO_INCLUDE_UP_IOEXP_H

/*
 * IO expanders input signals. These bits reflect the position in the
 * value read in io_exp_state from the expanders.
 */
enum {
    IO_EXP_MOD_4_DETECT             = 0,     /* U601: 16 bits */
    IO_EXP_MOD_4_WAKE_IN,
    IO_EXP_MOD_5_DETECT,
    IO_EXP_MOD_5_WAKE_IN,
    IO_EXP_MOD_6_DETECT,
    IO_EXP_MOD_6_WAKE_IN,
    IO_EXP_MOD_7_DETECT,
    IO_EXP_MOD_7_WAKE_IN,
    IO_EXP_SWITCH_IRQ,                       /* Switch IRQ */
    IO_EXP_MOD_1_DETECT,
    IO_EXP_MOD_1_WAKE_IN,
    IO_EXP_MOD_2_DETECT,
    IO_EXP_MOD_2_WAKE_IN,
    IO_EXP_MOD_3_DETECT,
    IO_EXP_MOD_3_WAKE_IN,
    IO_EXP_IRQ1,                             /* IRQ from IO Expander U602 */
    IO_EXP_MOD_12_DETECT             = 16,   /* U602: 16 bits */
    IO_EXP_MOD_12_WAKE_IN,
    IO_EXP_MOD_13_DETECT,
    IO_EXP_MOD_13_WAKE_IN,
    IO_EXP_MOD_14_DETECT,
    IO_EXP_MOD_14_WAKE_IN,
    IO_EXP_NRST_WAKEUP,                      /* Reset button */
    IO_EXP_U601_NC17,
    IO_EXP_MOD_8_DETECT,
    IO_EXP_MOD_8_WAKE_IN,
    IO_EXP_MOD_9_DETECT,
    IO_EXP_MOD_9_WAKE_IN,
    IO_EXP_MOD_10_DETECT,
    IO_EXP_MOD_10_WAKE_IN,
    IO_EXP_MOD_11_DETECT,
    IO_EXP_MOD_11_WAKE_IN
};

extern void ioexp_read_iopins(void);

#endif // __CONFIGS_ENDO_INCLUDE_UP_IOEXP_H
