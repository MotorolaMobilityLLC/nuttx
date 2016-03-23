/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
 * This code uses TSB timer 4 as a free running timer for any system which
 * needs it.  To use this code a driver simply needs to call tsb_fr_timer_reserve()
 * then tsb_fr_timer_start() or tsb_fr_timer_stop().  When the timer is no longer
 * needed call tsb_fr_timer_release().  The code is in place so the timer can
 * be started and stopped to save power.
 *
 * At some point it would make sense to integrate this with the tickless kernel.
 * In that case the timer would not need to be shutdown.  However, this is not yet
 * needed, so it has not been done.
 */

#include "tsb_fr_tmr.h"
#include "tsb_tmr.h"

#include <errno.h>
#include <string.h>

/* The ID of the timer to use for the free running timer. */
#define TSB_FR_TMR_ID TSB_TMR_TMR4

/* The max number of users supported. */
#define TSB_FR_TMR_MAX_USERS (sizeof(g_tsb_fr_tmr.assigned_mask)*8)

struct tsb_fr_tmr_s
{
    struct tsb_tmr_ctx *tmr;
    uint32_t assigned_mask;
    uint32_t enable_mask;
    uint32_t upper32;
};

struct tsb_fr_tmr_s g_tsb_fr_tmr;

/*
 * ISR called when the timer reaches 0.
 */
static int tsb_fr_tmr_overflow_isr(int irq, void *regs)
{
    tsb_tmr_ack_irq(g_tsb_fr_tmr.tmr);
    g_tsb_fr_tmr.upper32++;
    return 0;
}

/**
 * @brief Get the 32 bit value of the free running timer.
 *
 * The timer should be reserved and started before calling this function or it
 * may not return a valid value.
 */
uint32_t tsb_fr_tmr_get(void)
{
    return (tsb_tmr_read_elapsed_raw(TSB_FR_TMR_ID));
}

/**
 * @brief Get the 64 bit value of the free running timer.
 *
 * The timer should be reserved and started before calling this function or it
 * may not return a valid value.
 */
uint64_t tsb_fr_tmr_get64(void)
{
    return (((uint64_t)g_tsb_fr_tmr.upper32 << 32) | tsb_tmr_read_elapsed_raw(TSB_FR_TMR_ID));
}

/**
 * @brief Request the use of a timer.
 *
 * @return The ID of the timer of use or -1 if no timers are available.
 */
int tsb_fr_tmr_reserve(void)
{
    int tmr_id = 0;

    while (((g_tsb_fr_tmr.assigned_mask & (1 << tmr_id)) != 0) &&
           (tmr_id < TSB_FR_TMR_MAX_USERS))
    {
        tmr_id++;
    }
    if (tmr_id < TSB_FR_TMR_MAX_USERS)
    {
        g_tsb_fr_tmr.assigned_mask |= (1 << tmr_id);
    }
    else
    {
        tmr_id = -1;
    }
    return tmr_id;
}

/**
 * @brief Release the timer so it can be used by another driver.
 *
 * @param tmr_id The timer to release.
 */
void tsb_fr_tmr_release(int tmr_id)
{
    if (tmr_id < TSB_FR_TMR_MAX_USERS)
    {
        /* Stop the timer if it is running, just in case. */
        if (tsb_fr_tmr_stop(tmr_id) == OK)
        {
            g_tsb_fr_tmr.assigned_mask &= ~(1 << tmr_id);
        }
    }
}

/**
 * @brief Start the free running timer, if it is not already running.
 *
 * @param tmr_id The timer to start.
 *
 * @return The OK when all is good or -EINVAL if tmr_id is invalid or not
 *         assigned.
 */
int tsb_fr_tmr_start(int tmr_id)
{
    uint32_t tmr_msk = 1 << tmr_id;

    if ((tmr_id >= 0) && (g_tsb_fr_tmr.assigned_mask & tmr_msk))
    {
        if (!(g_tsb_fr_tmr.enable_mask & tmr_msk))
        {
            if (g_tsb_fr_tmr.enable_mask == 0)
            {
                g_tsb_fr_tmr.upper32 = 0;
                tsb_tmr_configure(g_tsb_fr_tmr.tmr, TSB_TMR_MODE_FREERUN, tsb_fr_tmr_overflow_isr);
                tsb_tmr_start(g_tsb_fr_tmr.tmr);
            }
            g_tsb_fr_tmr.enable_mask |= tmr_msk;
        }
        return OK;
    }
    return -EINVAL;
}

/**
 * @brief Stop the free running timer, if it is not already stopped.
 *
 * @param tmr_id The timer to stop.
 *
 * @return The OK when all is good or -EINVAL if tmr_id is invalid or not
 *         assigned.
 */
int tsb_fr_tmr_stop(int tmr_id)
{
    uint32_t tmr_msk = 1 << tmr_id;

    if ((tmr_id >= 0) && (g_tsb_fr_tmr.assigned_mask & tmr_msk))
    {
        if (g_tsb_fr_tmr.enable_mask & tmr_msk)
        {
            g_tsb_fr_tmr.enable_mask &= ~tmr_msk;
        }
        if (g_tsb_fr_tmr.enable_mask == 0)
        {
            tsb_tmr_cancel(g_tsb_fr_tmr.tmr);
        }
        return OK;
    }
    return -EINVAL;
}

/**
 * @brief Initialize the free running timer system.
 */
void tsb_fr_tmr_init(void)
{
    memset(&g_tsb_fr_tmr, 0, sizeof(g_tsb_fr_tmr));
    g_tsb_fr_tmr.tmr = tsb_tmr_get(TSB_FR_TMR_ID);
}
