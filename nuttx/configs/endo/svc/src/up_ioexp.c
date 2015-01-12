/****************************************************************************
 * configs/endo/svc/src/up_ioexp.c
 * Endo/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
 *
 * Copyright (C) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 ****************************************************************************/
#define DBG_COMP DBG_IOEXP
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_debug.h"
#include "up_i2c.h"
#include "up_internal.h"
#include "up_ioexp.h"
#include "up_gpio.h"
#include "up_power.h"
#include "up_switch.h"
#include "stm32.h"

/* IO expanders states */
uint32_t io_exp_state, io_exp_last_state;


/*
 * Read IO expanders (U601, U602) pins:
 * - Switch IRQ,
 * - Wake and detect signals
 *
 * Note: reading the state of the IO expanders de-asserts the #INT line
 */
void ioexp_read_iopins(void)
{
    uint8_t msg[4];
    uint32_t io_exp_chg;
    int i;

    dbg_info("%s()\n", __func__);

    /* Read 16 bits from U602 */
    msg[0] = 0x00;
    if (i2c_ioexp_read(msg, 2, I2C_ADDR_IOEXP_U602))
        return;

    /* Read 16 bits from U601 */
    msg[2] = 0x00;
    if (i2c_ioexp_read(&msg[2], 2, I2C_ADDR_IOEXP_U601))
        return;

    /* Store the state of the 32 signals */
    io_exp_state = buf_to_uint32(msg);

    /* Detect state changes */
    io_exp_chg = io_exp_state ^ io_exp_last_state;
    if (io_exp_chg) {
        dbg_info("%s(): IO EXP 0x%04x\n", __func__, io_exp_state);
        for (i = 0; i < 32; i++) {
            if (io_exp_chg & (1 << i)) {
                if (i == IO_EXP_SWITCH_IRQ)
                    dbg_info("%s(): bit%02d: Switch IRQ ***\n", __func__, i);
                if (i == IO_EXP_IRQ1)
                    dbg_info("%s(): bit %02d: IO Exp U602 IRQ ***\n",
                             __func__, i);
                dbg_info("%s(): bit%02d <- %d\n", __func__, i,
                         !!(io_exp_state & (1 << i)));
            }
        }
        io_exp_last_state = io_exp_state;
    }
}

