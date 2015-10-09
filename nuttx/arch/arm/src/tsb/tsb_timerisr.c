/*
 * Copyright (c) 2014-2015 Google, Inc.
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
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/hires_tmr.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "up_arch.h"

/* 96 MHz */
#define         CLOCK_FREQUENCY         96000000
#define         USEC_CLOCK_COUNT        (CLOCK_FREQUENCY / 1000000)
#define         SYSTICK_USEC            10000
#define         SYSTICK_RELOAD          (USEC_CLOCK_COUNT * SYSTICK_USEC)

/*
 * Timer counts down from SYSTICK_RELOAD to 0 ,'(SYSTICK_RELOAD - clk)'
 * calculates clock count since reload, '+ (USEC_CLOCK_COUNT/2)' is used
 * for rounding.
 */
#define CLOCK_COUNT2USEC(clk) \
    (((SYSTICK_RELOAD - (clk)) + (USEC_CLOCK_COUNT / 2)) / USEC_CLOCK_COUNT)

#define SYSTICK_TILL_USEC_ROLLOVER      (0xffffffff / SYSTICK_USEC)

/*
 * Nano is special because there are 10.41667 nanoseconds per clock tick.
 * Applying the same procedure as above would lead to large rounding errors.
 * Use scaling to reduce rounding errors for CLOCK_COUNT2NSEC.
 *
 * Use the largest possible scaling value to ensure there's no overflow of
 * the 32bit value.
 */
#define MAX_NANO_SCALING ((0xffffffff / SYSTICK_RELOAD) - 1)

/* Ensure NANO_SCALING_VALUE is not greater than MAX_NANO_SCALING. */
#define UNSCALED_NANO_PER_CLOCK ((NANO_PER_SECOND / CLOCK_FREQUENCY) + 1)
#define NANO_SCALING_VALUE      (MAX_NANO_SCALING / UNSCALED_NANO_PER_CLOCK)

/*
 * Now that we have the largest possible scaling value apply it to
 * NANO_PER_CLOCK value.
 */
#define SCALED_NANO_PER_CLOCK \
            (NANO_PER_SECOND / (CLOCK_FREQUENCY/NANO_SCALING_VALUE))

#define CLOCK_COUNT2NSEC(clk) \
    (((SYSTICK_RELOAD - (clk)) * SCALED_NANO_PER_CLOCK) / NANO_SCALING_VALUE)

#define NANO_PER_SECOND         1000000000
#define MICRO_PER_SEC           1000000
#define SYSTICKS_PER_SEC        (MICRO_PER_SEC/SYSTICK_USEC)
#define NANOSEC_PER_SYSTICK \
            (SYSTICK_USEC * (NANO_PER_SECOND / MICRO_PER_SEC))

static volatile bool htr_rollover = false;

/*
 * Ensure that snapshots of timer values account for rollover of the clock.
 *
 * When not in interrupt context:
 *   If rollover occurs after reading clock_value then tick_count may or may
 *   not be valid. Reading the values again, just after the rollover, will
 *   ensure they are correct.
 *
 * When in interrupt context detecting an accurate tick_count becomes more
 * difficult:
 *   - once in interrupt context the global tickcount will not change until
 *     processed by the systick ISR
 *   - rollover can happen any time before or during the current ISR
 *   - once rollover is detected the tickcount value must be corrected
 *   - hrt_rollover must track if rollover had previously occurred
 *
 * timer_snapshot() is interrupt context safe.
 */
static inline void timer_snapshot(uint32_t *tick_count, uint32_t *clock_value)
{
    uint32_t systick_ctrl, ticks, clock;

    if (up_interrupt_context()) {
        systick_ctrl = getreg32(NVIC_SYSTICK_CTRL);
        if (systick_ctrl & NVIC_SYSTICK_CTRL_COUNTFLAG) {
            htr_rollover = true;
        }

        clock = getreg32(NVIC_SYSTICK_CURRENT);
        ticks = clock_systimer();
        if (clock < getreg32(NVIC_SYSTICK_CURRENT)) {
            clock = getreg32(NVIC_SYSTICK_CURRENT);
            ticks++;
        } else if (htr_rollover) {
            ticks++;
        }
    } else {
        clock = getreg32(NVIC_SYSTICK_CURRENT);
        ticks = clock_systimer();

        if (clock < getreg32(NVIC_SYSTICK_CURRENT)) {
            clock = getreg32(NVIC_SYSTICK_CURRENT);
            ticks = clock_systimer();
        }
    }

    *tick_count = ticks;
    *clock_value = clock;
}

int up_timerisr(int irq, uint32_t *regs)
{
    /* Process timer interrupt */
    sched_process_timer();

    return 0;
}

void up_timer_initialize(void)
{
    putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

    irq_attach(TSB_IRQ_SYSTICK, (xcpt_t)up_timerisr);

    putreg32(NVIC_SYSTICK_CTRL_CLKSOURCE |
             NVIC_SYSTICK_CTRL_TICKINT   |
             NVIC_SYSTICK_CTRL_ENABLE,
             NVIC_SYSTICK_CTRL);
}

/*
 * Ensure that the rollover indicator is reset after incrementing
 * clock_systimer otherwise other the next call to timer_snapshot
 * in IRQ state will be wrong
 */
void hrt_clear_rollover(void)
{
    getreg32(NVIC_SYSTICK_CTRL);
    htr_rollover = false;
}

/*
 * Return seconds and nanoseconds since the last reboot of the CPU. This
 * function is interrupt context safe.
 */
void hrt_gettimespec(struct timespec *ts)
{
    uint32_t clock_value, tick_count;

    timer_snapshot(&tick_count, &clock_value);

    ts->tv_sec = tick_count / SYSTICKS_PER_SEC;
    ts->tv_nsec = CLOCK_COUNT2NSEC(clock_value);
    ts->tv_nsec += (tick_count % SYSTICKS_PER_SEC) * NANOSEC_PER_SYSTICK;
}

/*
 * Return the micro seconds since the last reboot of the CPU. This function
 * is interrupt context safe.
 */
uint32_t hrt_getusec(void)
{
    uint32_t usec, clock_value, tick_count;

    timer_snapshot(&tick_count, &clock_value);

    usec = CLOCK_COUNT2USEC(clock_value);
    usec += ((tick_count % SYSTICK_TILL_USEC_ROLLOVER) * SYSTICK_USEC);

    return usec;
}
