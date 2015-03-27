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
 * configs/endo/svc/src/up_ioexp.h
 * Endo/SVC support for I2C IO Expanders, used to read the input lines status
 *  and generate an interrupt when the state changes.
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
