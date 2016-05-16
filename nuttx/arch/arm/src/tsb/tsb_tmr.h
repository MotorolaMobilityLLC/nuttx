/*
 * Copyright (c) 2016 Motorola Mobility, LLC
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

#include "chip.h"
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

#define TSB_TMR_LOAD                    (0x0000)
#define TSB_TMR_CNT                     (0x0004)
#define TSB_TMR_CTRL                    (0x0008)
#define TSB_TMR_ISR                     (0x000C)
#define TSB_TMR_WDT                     (0x0040)
#define TSB_TMR_RSR                     (0x0050)

#define TSB_TMR0_OFF                    (0x0000)
#define TSB_TMR1_OFF                    (0x0100)
#define TSB_TMR2_OFF                    (0x0200)
#define TSB_TMR3_OFF                    (0x0300)
#define TSB_TMR4_OFF                    (0x0400)

/*
 * Macro to calculate the address of the requested register for quick reads.
 * This will only work for timers which are 0x100 bytes appart in the memory
 * map.  This is the case for all versions of the TSB up to and including
 * ES3.
 */
#define TSB_TMR_GET_REG_PTR(id, reg) ((volatile uint32_t *)(TMR_BASE+(TSB_TMR1_OFF*(id))+(reg)))

/*
 * The following macro relies on the fact that the timer base frequency on the
 * TSB is 48MHz.
 */
#define TSB_TMR_RAW_TO_USEC(raw)     (uint32_t)(raw/48)
#define TSB_TMR_USEC_TO_RAW(usec)    (uint32_t)(usec*48)

enum {
    TSB_TMR_TMR0 = 0,
    TSB_TMR_TMR1 = 1,
    TSB_TMR_TMR2 = 2,
    TSB_TMR_TMR3 = 3,
    TSB_TMR_TMR4 = 4,
};

enum {
    TSB_TMR_MODE_PERIODIC = 0x07,
    TSB_TMR_MODE_ONESHOT = 0x05,
    TSB_TMR_MODE_FREERUN = 0x03,
    TSB_TMR_MODE_WATCHDOG = 0x01,
};

/**
 * @brief Read the raw value of a timer in ticks.
 *
 * Function to read the raw value of a TSB timer.  Please remember that the TSB
 * timers count down from the loaded value.  To convert the value to micro seconds
 * use TSB_TMR_RAW_TO_USEC.  If a timer enum value is used directly this code should
 * compile down to a register read.
 *
 * @param tmr Timer handle.
 *
 * @return The raw value of the timer.
 */
static inline uint32_t tsb_tmr_read_raw(int id)
{
    return *TSB_TMR_GET_REG_PTR(id, TSB_TMR_CNT);
}

/**
 * @brief Read the elapsed time of a timer in ticks.
 *
 * Function to read the time elapsed on a timer in ticks.  This is essentially
 * the value in the timer value subtracted from the load register.  It may roll
 * over and result in a shorter time that was anticipated so please keep in mind
 * the base frequency of the timer before using this.  To convert the value to
 * micro seconds use TSB_TMR_RAW_TO_USEC.
 *
 * @param tmr Timer handle.
 *
 * @return The raw value of the timer.
 */
static inline uint32_t tsb_tmr_read_elapsed_raw(int id)
{
    return *TSB_TMR_GET_REG_PTR(id, TSB_TMR_LOAD) - *TSB_TMR_GET_REG_PTR(id, TSB_TMR_CNT);
}

struct tsb_tmr_ctx * tsb_tmr_get(int id);
void tsb_tmr_configure(struct tsb_tmr_ctx *tmr, int mode, xcpt_t isr);
void tsb_tmr_set_time(struct tsb_tmr_ctx *tmr, uint32_t usec);
void tsb_tmr_start(struct tsb_tmr_ctx *tmr);
uint32_t tsb_tmr_cancel(struct tsb_tmr_ctx *tmr);
uint32_t tsb_tmr_usec_left(struct tsb_tmr_ctx *tmr);
void tsb_tmr_ack_irq(struct tsb_tmr_ctx *tmr);

#endif /* __ARCH_ARM_SRC_TSB_TSB_TMR_H */
