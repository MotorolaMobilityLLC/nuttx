/*
 * Copyright (c) 2015 Google, Inc.
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
 * may be used to endorse or promote products derived from this
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
 *
 * Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
 */

#ifndef __ARCH_ARM_SRC_TSB_TSB_TMR_H
#define __ARCH_ARM_SRC_TSB_TSB_TMR_H

#include <nuttx/irq.h>
#include <stdint.h>

/*
 * Simple API allowing access to four timers present on Toshiba bridges. As of
 * now it's only used internally by the bridge tickless OS implementation. If
 * in the future any user would like to get access to it from the outside of
 * this directory, the low-level functionality should probably be moved to
 * the device_table framework.
 */

struct tsb_tmr_ctx;

enum {
    TSB_TMR_TMR1 = 0,
    TSB_TMR_TMR2 = 1,
    TSB_TMR_TMR3 = 2,
    TSB_TMR_TMR4 = 3,
};

enum {
    TSB_TMR_MODE_PERIODIC = 0x07,
    TSB_TMR_MODE_ONESHOT = 0x05,
    TSB_TMR_MODE_FREERUN = 0x03,
};

struct tsb_tmr_ctx * tsb_tmr_get(int id);
void tsb_tmr_configure(struct tsb_tmr_ctx *tmr, int mode, xcpt_t isr);
void tsb_tmr_set_time(struct tsb_tmr_ctx *tmr, uint32_t usec);
void tsb_tmr_start(struct tsb_tmr_ctx *tmr);
uint32_t tsb_tmr_cancel(struct tsb_tmr_ctx *tmr);
uint32_t tsb_tmr_usec_left(struct tsb_tmr_ctx *tmr);
void tsb_tmr_ack_irq(struct tsb_tmr_ctx *tmr);

#endif /* __ARCH_ARM_SRC_TSB_TSB_TMR_H */
