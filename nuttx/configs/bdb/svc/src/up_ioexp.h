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
 * configs/bdb/svc/src/up_ioexp.h
 * BDB/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
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
