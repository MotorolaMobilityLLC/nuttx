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

#ifndef _TSB_PWM_H_
#define _TSB_PWM_H_

#define TSB_PWM_DIV           (16)
#define TSB_PWM_CLK           (48000000 / TSB_PWM_DIV)

#define TSB_PWM_BASE          0x40004000
#define TSB_PWM0              0x40004000
#define TSB_PWM1              0x40004020

#define TSB_PWM_CR            0x00000000
#define   PWM_CR_ENBSEL       (1 << 28)
#define   PWM_CR_DIV(div)     ((div - 1) << 16)
#define   PWM_CR_LVS          (1 << 9)
#define   PWM_CR_POL          (1 << 8)
#define   PWM_CR_UPD          (1 << 1)
#define   PWM_CR_ENB          (1 << 0)
#define TSB_PWM_FREQ          0x00000004
#define TSB_PWM_DUTY          0x00000008
#define TSB_PWM_ITERATION     0x0000000c
#define TSB_PWM_CURFREQ       0x00000010
#define TSB_PWM_CURDUTY       0x00000014
#define TSB_PWM_CURITER       0x00000018
#define TSB_PWM_COUNT         0x0000001C

#define TSB_PWM_SYSTEM        0x00000200
#define TSB_PWM_GENB          0x00000204
#define   PWM_GENB_GENB0      (1 << 0)
#define TSB_PWM_INTCONFIG     0x00000220
#define   PWM_INTMODE0        (1 << 0)
#define   PWM_INTMODE1        (1 << 1)
#define TSB_PWM_INTMASK       0x00000224
#define   PWM_INTMASK0        (1 << 0)
#define   PWM_INTMASK1        (1 << 1)
#define   PWM_ERRMASK0        (1 << 16)
#define   PWM_ERRMASK1        (1 << 17)
#define TSB_PWM_INTSTATUS     0x00000228
#define   PWM_INTSTATUS0      (1 << 0)
#define   PWM_INTSTATUS1      (1 << 1)
#define   PWM_ERRSTATUS0      (1 << 16)
#define   PWM_ERRSTATUS1      (1 << 17)

#define TSB_PWM_FLAG_OPENED     BIT(0)
#define TSB_PWM_FLAG_CONFIGURED BIT(1)
#define TSB_PWM_FLAG_ENABLED    BIT(2)
#define TSB_PWM_FLAG_ACTIVED    BIT(3)
#endif

