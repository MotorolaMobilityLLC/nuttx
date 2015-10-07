/****************************************************************************
 * arch/arm/src/stm32/stm32_timerisr.c
 *
 *   Copyright (c) 2014-2015 Google, Inc.
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * The RCC feeds the Cortex System Timer (SysTick) with the AHB clock (HCLK)
 * divided by 8.  The SysTick can work either with this clock or with the
 * Cortex clock (HCLK), configurable in the SysTick Control and Status
 * register.
 */

#undef CONFIG_STM32_SYSTICK_HCLKd8 /* Power up default is HCLK, not HCLK/8 */
                                   /* And I don't know now to re-configure it yet */

#if CONFIG_STM32_SYSTICK_HCLKd8
#  define SYSTICK_RELOAD ((STM32_HCLK_FREQUENCY / 8 / CLK_TCK) - 1)
#else
#  define SYSTICK_RELOAD ((STM32_HCLK_FREQUENCY / CLK_TCK) - 1)
#endif

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

#define USEC_CLOCK_COUNT  (STM32_SYSCLK_FREQUENCY / USEC_PER_SEC)

/*
 * Timer counts down from SYSTICK_RELOAD to 0 ,'(SYSTICK_RELOAD - clk)'
 * calculates clock count since reload, '+ (USEC_CLOCK_COUNT/2)' is used
 * for rounding.
 */
#define CLOCK_COUNT2USEC(clk) \
    (((SYSTICK_RELOAD - (clk)) + (USEC_CLOCK_COUNT / 2)) / USEC_CLOCK_COUNT)

#define SYSTICK_TILL_USEC_ROLLOVER   (0xffffffff / USEC_PER_TICK)

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
#define UNSCALED_NANO_PER_CLOCK ((NSEC_PER_SEC / STM32_SYSCLK_FREQUENCY) + 1)
#define NANO_SCALING_VALUE      (MAX_NANO_SCALING / UNSCALED_NANO_PER_CLOCK)

/*
 * Now that we have the largest possible scaling value apply it to
 * NANO_PER_CLOCK value.
 */
#define SCALED_NANO_PER_CLOCK \
            (NSEC_PER_SEC / (STM32_SYSCLK_FREQUENCY/NANO_SCALING_VALUE))

#define CLOCK_COUNT2NSEC(clk) \
    (((SYSTICK_RELOAD - (clk)) * SCALED_NANO_PER_CLOCK) / NANO_SCALING_VALUE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

static volatile bool htr_rollover = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */

   sched_process_timer();

   htr_rollover = false;
   /* Clear the clock rollover bit */
   getreg32(NVIC_SYSTICK_CTRL);

   return 0;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Make sure that the SYSTICK clock source is set correctly */

#if 0 /* Does not work.  Comes up with HCLK source and I can't change it */
  regval = getreg32(NVIC_SYSTICK_CTRL);
#if CONFIG_STM32_SYSTICK_HCLKd8
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
#else
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
#endif
  putreg32(regval, NVIC_SYSTICK_CTRL);
#endif

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  (void)irq_attach(STM32_IRQ_SYSTICK, (xcpt_t)up_timerisr);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE|NVIC_SYSTICK_CTRL_TICKINT|NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(STM32_IRQ_SYSTICK);
}

/*
 * Return seconds and nanoseconds since the last reboot of the CPU. This
 * function is interrupt context safe.
 */
void hrt_gettimespec(struct timespec *ts)
{
    uint32_t clock_value, tick_count;

    timer_snapshot(&tick_count, &clock_value);

    ts->tv_sec = tick_count / CLOCKS_PER_SEC;
    ts->tv_nsec = CLOCK_COUNT2NSEC(clock_value);
    ts->tv_nsec += (tick_count % CLOCKS_PER_SEC) * NSEC_PER_TICK;
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
    usec += ((tick_count % SYSTICK_TILL_USEC_ROLLOVER) * USEC_PER_TICK);

    return usec;
}
