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
 * configs/bdb/svc/src/up_i2c.c
 * BDB/SVC I2C support, used for communication with the switch and the
 *  IO Expanders
 *
 ****************************************************************************/
#define DBG_COMP DBG_I2C
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/i2c.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_debug.h"
#include "up_internal.h"
#include "up_i2c.h"
#include "stm32.h"

#include "bdb-internal.h"

/* Communication instance struct */
FAR struct i2c_dev_s *sw_exp_dev;


/*
 * Initialize the communication from SVC
 * Allocates internal structs etc. To be done once.
 */
inline int i2c_init_comm(uint8_t bus)
{
    /* Initialize I2C internal structs */
    sw_exp_dev = up_i2cinitialize(bus);
    if (!sw_exp_dev) {
        dbg_error("%s(): Failed to get bus %d\n", __func__, I2C_SW_BUS);
        return ERROR;
    }

    return 0;
}

/*
 * Select the destination device to communicate with.
 * I2C requires a bus address.
 */
inline void i2c_select_device(uint8_t addr)
{
    /* Write the register address followed by the data (no RESTART) */
    I2C_SETADDRESS(sw_exp_dev, addr, 7);
}

/* Send data from SVC to switch */
inline int i2c_switch_send_msg(uint8_t *buf, unsigned int size)
{
    int ret;

    dbg_verbose("%s()\n", __func__);
    dbg_print_buf(DBG_VERBOSE, buf, size);

    i2c_select_device(I2C_ADDR_SWITCH);

    ret = I2C_WRITE(sw_exp_dev, buf, size);
    if (ret) {
        dbg_error("%s(): Error %d\n", __func__, ret);
        return ERROR;
    }

    return 0;
}

/* Receive data from switch to SVC */
inline int i2c_switch_receive_msg(uint8_t *buf, unsigned int size)
{
    int ret;

    i2c_select_device(I2C_ADDR_SWITCH);

    ret = I2C_READ(sw_exp_dev, buf, size);
    if (ret) {
        dbg_error("%s(): Error %d\n", __func__, ret);
        return ERROR;
    }

    dbg_verbose("%s()\n", __func__);
    dbg_print_buf(DBG_VERBOSE, buf, size);

    return 0;
}

/* Write data to the IO Expander */
void i2c_ioexp_write(uint8_t *msg, int size, uint8_t addr)
{
    int ret;

    i2c_select_device(addr);

    ret = I2C_WRITE(sw_exp_dev, msg, size);
    if (ret) {
        dbg_error("%s(): Error %d\n", __func__, ret);
        return;
    }

    dbg_verbose("%s()\n", __func__);
    dbg_print_buf(DBG_VERBOSE, msg, size);
}

/* Read data from the IO Expander */
void i2c_ioexp_read(uint8_t *msg, int size, uint8_t addr)
{
    int ret;

    dbg_verbose("%s()\n", __func__);
    dbg_print_buf(DBG_VERBOSE, msg, size);

    i2c_select_device(addr);

    /* We need 2 messages (cf. datasheet) */
    struct i2c_msg_s msgv[2] = {
        /* Write the command byte, no restart */
        {
            .addr   = addr,
            .flags  = 0,
            .buffer = msg,
            .length = 1
        },
        /* Read the bytes */
        {
            .addr   = addr,
            .flags  = I2C_M_READ,
            .buffer = msg,
            .length = size
        }
    };

    if ((ret = I2C_TRANSFER(sw_exp_dev, msgv, 2)) != OK) {
        dbg_error("%s(): Error %d\n", __func__, ret);
        return;
    }

    dbg_verbose("%s()\n", __func__);
    dbg_print_buf(DBG_VERBOSE, msg, size);
}
