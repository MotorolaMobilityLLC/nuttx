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

#include "tsb_tmr.h"

#include <sys/time.h>

#include <nuttx/arch.h>
#include <nuttx/time.h>
#include <nuttx/util.h>

#define FREERUN_PERIOD_SECONDS  (60)
#define FREERUN_PERIOD          (USEC_PER_SEC * FREERUN_PERIOD_SECONDS)

static struct tsb_tmr_ctx *freerun_timer;
static struct tsb_tmr_ctx *tickless_timer;

static uint32_t freerun_seconds = 0;

static int freerun_isr(int irq, void *regs)
{
    tsb_tmr_ack_irq(freerun_timer);
    freerun_seconds += FREERUN_PERIOD_SECONDS;

    return 0;
}

static int tickless_isr(int irq, void *regs)
{
    tsb_tmr_ack_irq(tickless_timer);
    tsb_tmr_cancel(tickless_timer);
    sched_timer_expiration();

    return 0;
}

void up_timer_initialize(void)
{
    freerun_timer = tsb_tmr_get(TSB_TMR_TMR1);
    tickless_timer = tsb_tmr_get(TSB_TMR_TMR2);

    if (freerun_timer == NULL || tickless_timer == NULL) {
        lldbg("error configuring timers\n");
        PANIC();
    }

    tsb_tmr_configure(freerun_timer, TSB_TMR_MODE_FREERUN, freerun_isr);
    tsb_tmr_set_time(freerun_timer, FREERUN_PERIOD);
    tsb_tmr_start(freerun_timer);

    /*
     * XXX We should be using a one-shot timer for tickless operation, but
     * Toshiba Bridge timers only allow us to read the counter register
     * in free-running mode. We need to know the counter value when the
     * scheduler calls up_timer_cancel() so we're stuck with this mode.
     */
    tsb_tmr_configure(tickless_timer, TSB_TMR_MODE_FREERUN, tickless_isr);
}

int up_timer_gettime(struct timespec *ts)
{
    uint32_t left, elapsed;
    uint64_t ns;

    left = tsb_tmr_usec_left(freerun_timer);
    /* Timer counts down, so reverse the value. */
    elapsed = FREERUN_PERIOD - left;
    ns = freerun_seconds;
    ns *= USEC_PER_SEC;
    ns += elapsed;
    ns *= NSEC_PER_USEC;
    nsec_to_timespec(ns, ts);

    return 0;
}

int up_timer_start(const struct timespec *ts)
{
    uint32_t usec;

    usec = timespec_to_usec(ts);
    tsb_tmr_set_time(tickless_timer, usec);
    tsb_tmr_start(tickless_timer);

    return 0;
}

int up_timer_cancel(struct timespec *ts)
{
    uint32_t left;

    left = tsb_tmr_cancel(tickless_timer);
    usec_to_timespec(left, ts);

    return 0;
}
