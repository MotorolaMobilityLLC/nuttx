/*
 * Copyright (c) 2014 Google Inc.
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

/****************************************************************************
 * configs/bdb/svc/src/up_ioexp.c
 * BDB/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
 * @author: Jean Pihet <jean.pihet@newoldbits.com>
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

#include "bdb-internal.h"

/* IO expanders states */
uint32_t io_exp_state, io_exp_last_state;

/*
 * Read IO expanders pins:
 * - Switch IRQ,
 * - Wake and detect pins from U96 and U90.
 *
 * Note: reading the state of the IO expanders de-asserts the #INT line
 */
void ioexp_read_iopins(void)
{
    uint8_t msg[4];
    uint32_t io_exp_chg;
    int i;

    dbg_info("%s()\n", __func__);

    /* Read 24 bits from U96 */
    msg[0] = 0x80;
    msg[1] = 0x00;
    msg[2] = 0x00;
    msg[3] = 0x00;
    i2c_ioexp_read(msg, 3, I2C_ADDR_IOEXP_U96);

    /* Read 8 bits from U90 */
    msg[3] = 0x00;
    i2c_ioexp_read(&msg[3], 1, I2C_ADDR_IOEXP_U90);

    /* Store the state of the 32 signals */
    io_exp_state = buf_to_uint32(msg);

    /* Detect state changes */
    io_exp_chg = io_exp_state ^ io_exp_last_state;
    if (io_exp_chg) {
        dbg_info("%s(): IO EXP 0x%04x\n", __func__, io_exp_state);
        for (i = 0; i < 32; i++) {
            if (io_exp_chg & (1 << i)) {
                if (i == IO_EXP_SWITCH_IRQ)
                    dbg_info("%s(): *** Switch IRQ ***\n", __func__);
                dbg_info("%s(): bit%02d <- %d\n", __func__, i,
                       !!(io_exp_state & (1 << i)));
            }
        }
        io_exp_last_state = io_exp_state;
    }
}

/* I2C stress test code: write to U135 and read from another I2C device */
void test_ioexp_io(void)
{
    uint8_t msg[8], x = 0;
    uint32_t y;
    uint32_t attr_value;

    dbg_info("%s()\n", __func__);

    msg[0] = 0x0C;
    i2c_ioexp_read(msg, 1, I2C_ADDR_IOEXP_U135);
    msg[0] = 0x0C;
    msg[1] = 0x7F;
    i2c_ioexp_write(msg, 2, I2C_ADDR_IOEXP_U135);
    msg[0] = 0x0C;
    i2c_ioexp_read(msg, 1, I2C_ADDR_IOEXP_U135);

    while(1) {
        y = 0;
        while(y < 23) {
            x += 0x31;
            msg[0] = 0x04;
            msg[1] = x;
            i2c_ioexp_write(msg, 2, I2C_ADDR_IOEXP_U135);
            y++;
            usleep(100000);
        }

        /* Get switch version */
        switch_get_attribute(SWVER, &attr_value);
    }
}
