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
 * * may be used to endorse or promote products derived from this
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

#include "stm32_tim.h"

#include <nuttx/arch.h>
#include <nuttx/time.h>
#include <nuttx/util.h>

#include <stdio.h>
#include <sys/time.h>

#define TICKLESS_TIMER_ID       2
#define TICKLESS_TIMER_CLOCK    1000000 /* prescaler set to microseconds */

static volatile int tickless_timer_active = 0;
static volatile int tickless_timer_initiated = 0;
static int tickless_timer_irq = -1;

static struct stm32_tim_dev_s *tickless_timer;

#define FREERUN_TIMER_ID        5
#define FREERUN_TIMER_CLOCK     1000000
/* fire the interrupt every 60 minutes */
#define FREERUN_TIMER_PERIOD    3600000000U

static volatile uint32_t freerun_seconds = 0;
static int freerun_timer_irq = -1;

static struct stm32_tim_dev_s *freerun_timer;

static int freerun_isr_handler(int irq, void *regs)
{
    STM32_TIM_ACKINT(freerun_timer, irq);
    freerun_seconds += DIV_ROUND_CLOSEST(FREERUN_TIMER_PERIOD, FREERUN_TIMER_CLOCK);

    return 0;
}

static int tickless_isr_handler(int irq, void *regs)
{
    STM32_TIM_ACKINT(tickless_timer, irq);
    STM32_TIM_DISABLEINT(tickless_timer, 0);
    STM32_TIM_SETMODE(tickless_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(tickless_timer, 0);
    tickless_timer_active = 0;
    sched_timer_expiration();

    return 0;
}

void up_timer_initialize(void)
{
    tickless_timer = stm32_tim_init(TICKLESS_TIMER_ID);
    if (tickless_timer == NULL) {
        lldbg("error initializing tickless timer #%d\n", TICKLESS_TIMER_ID);
        PANIC();
    }

    freerun_timer = stm32_tim_init(FREERUN_TIMER_ID);
    if (freerun_timer == NULL) {
        lldbg("error initializing freerun timer #%d\n", FREERUN_TIMER_ID);
        PANIC();
    }

    STM32_TIM_DISABLEINT(freerun_timer, 0);
    STM32_TIM_SETMODE(freerun_timer, STM32_TIM_MODE_UP);
    STM32_TIM_SETPERIOD(freerun_timer, FREERUN_TIMER_PERIOD);
    STM32_TIM_SETCLOCK(freerun_timer, FREERUN_TIMER_CLOCK);
    freerun_timer_irq = STM32_TIM_SETISR(freerun_timer, freerun_isr_handler, 0);
    STM32_TIM_ACKINT(freerun_timer, freerun_timer_irq);
    STM32_TIM_ENABLEINT(freerun_timer, 0);

    /*
     * Postpone initialization of the tickless timer until the first call
     * to up_timer_start().
     */
    STM32_TIM_DISABLEINT(tickless_timer, 0);
}

int up_timer_gettime(FAR struct timespec *ts)
{
    irqstate_t flags;
    uint32_t counter;
    uint64_t ns;

    flags = irqsave();

    counter = STM32_TIM_GETCOUNTER(freerun_timer);
    ns = freerun_seconds;
    ns *= USEC_PER_SEC;
    ns += counter;
    ns *= NSEC_PER_USEC;
    nsec_to_timespec(ns, ts);

    irqrestore(flags);

    return 0;
}

int up_timer_start(FAR const struct timespec *ts)
{
    uint32_t current_period;
    uint64_t scheduled_ns;
    irqstate_t flags;

    flags = irqsave();

    scheduled_ns = timespec_to_nsec(ts);
    current_period = DIV_ROUND_CLOSEST(scheduled_ns, 1000);

    /*
     * If we're being called again without a call to up_timer_cancel() in
     * between and before the timer expires, we need to reset it.
     */
    if (tickless_timer_active) {
        STM32_TIM_DISABLEINT(tickless_timer, 0);
        STM32_TIM_SETMODE(tickless_timer, STM32_TIM_MODE_DISABLED);
        STM32_TIM_SETCLOCK(tickless_timer, 0);
    }

    STM32_TIM_SETMODE(tickless_timer, STM32_TIM_MODE_DOWN);
    STM32_TIM_SETPERIOD(tickless_timer, current_period);
    STM32_TIM_SETCLOCK(tickless_timer, TICKLESS_TIMER_CLOCK);

    /*
     * STM32_TIM_SETISR() already enables timer interrupts, so don't call
     * STM32_TIM_ENABLEINT() the first time up_timer_start() is called.
     */
    if (!tickless_timer_initiated) {
        tickless_timer_irq = STM32_TIM_SETISR(tickless_timer,
                                              tickless_isr_handler, 0);
        tickless_timer_initiated = 1;
    } else {
        /*
         * We need to clear any spurious interrupt that might have been set
         * by a call to STM32_TIM_SETCLOCK() before enabling interrupts again.
         */
        STM32_TIM_ACKINT(tickless_timer, tickless_timer_irq);
        STM32_TIM_ENABLEINT(tickless_timer, 0);
    }

    tickless_timer_active = 1;

    irqrestore(flags);

    return 0;
}

int up_timer_cancel(FAR struct timespec *ts)
{
    irqstate_t flags;
    uint32_t counter;
    uint64_t ns;

    flags = irqsave();

    if (!tickless_timer_active) {
        ts->tv_sec = 0;
        ts->tv_nsec = 0;
        irqrestore(flags);
        return 0;
    }

    counter = STM32_TIM_GETCOUNTER(tickless_timer);

    STM32_TIM_DISABLEINT(tickless_timer, 0);
    STM32_TIM_SETMODE(tickless_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(tickless_timer, 0);
    tickless_timer_active = 0;

    ns = counter * NSEC_PER_USEC;
    nsec_to_timespec(ns, ts);

    irqrestore(flags);

    return 0;
}
