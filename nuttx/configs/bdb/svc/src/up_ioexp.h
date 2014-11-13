/****************************************************************************
 * configs/bdb/svc/src/up_ioexp.h
 * BDB/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
 *
 * Copyright (C) 2014 Google, Inc.
 * @author: Jean Pihet <jean.pihet@newoldbits.com>
 *
 ****************************************************************************/
#ifndef __CONFIGS_BDB_INCLUDE_UP_IOEXP_H
#define __CONFIGS_BDB_INCLUDE_UP_IOEXP_H

/*
 * IO expanders input signals. These bits reflect the position in the
 * value read in io_exp_state from the expanders.
 */
enum {
    IO_EXP_BB4_DETECT               = 0,    /* U90: 8 bits */
    IO_EXP_BB4_WAKE,
    IO_EXP_BB5_DETECT,
    IO_EXP_BB5_WAKE,
    IO_EXP_BB6_DETECT,
    IO_EXP_BB6_WAKE,
    IO_EXP_U90_NC06,
    IO_EXP_U90_NC07,
    IO_EXP_BB1_DETECT               = 8,     /* U96: 24 bits */
    IO_EXP_BB1_WAKE,
    IO_EXP_BB2_DETECT,
    IO_EXP_BB2_WAKE,
    IO_EXP_BB3_DETECT,
    IO_EXP_BB3_WAKE,
    IO_EXP_U96_NC26,
    IO_EXP_U96_NC27,
    IO_EXP_GPB1_DETECT,
    IO_EXP_GPB1_WAKE,
    IO_EXP_GPB2_DETECT,
    IO_EXP_GPB2_WAKE,
    IO_EXP_U96_NC14,
    IO_EXP_U96_NC15,
    IO_EXP_U96_NC16,
    IO_EXP_U96_NC17,
    IO_EXP_SWITCH_IRQ,
    IO_EXP_APB1_DETECT,
    IO_EXP_APB1_WAKE,
    IO_EXP_APB2_DETECT,
    IO_EXP_APB2_WAKE,
    IO_EXP_APB3_DETECT,
    IO_EXP_APB3_WAKE,
    IO_EXP_U96_NC07
};

extern void ioexp_read_iopins(void);

#endif // __CONFIGS_BDB_INCLUDE_UP_IOEXP_H
