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


/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/hires_tmr.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "sched.h"

#ifdef CONFIG_USEC_MEASURE_PERF

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/
/* Software switch to enable/disable Performance Tracking */
static bool perf_active;

/* Keep track of the perf time start */
static uint32_t perf_start;

/* Keep track of the perf time stop */
static uint32_t perf_stop;

/* Keep track of the point time was recorded.
 * Its intended this variable is updated only when get_perf_diff_from_last
 * is called.
 */
static uint32_t last_perf_time;

/* Keep track of the current tcb being tracked.
 * Its intended this variable is updated only in sched_track_switch
 */
static int curr_hash_index;

/* Keep track of time spent in interrupts */
static uint32_t irq_times[NR_IRQS];

/* Keep track of the current irq in interrupt context */
static bool curr_irq;

/* No interrupt to track */
#define NO_IRQ (NR_IRQS +1)


/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: get_perf_time
 *
 * Description:
 *   Get the number of uSec since performance tracking was started
 *   function will turn off interrupts before taking reading
 *
 * Inputs:
 *   none
 *
 * Return Value:
 *   uSec since last perftracking session was started
 *
 * Assumptions/Limitations:
 *    timer rolls over every (1.19 hours)
 *
 ************************************************************************/
inline uint32_t get_perf_time(void)
{
    uint32_t usec = hrt_getusec();

    if(usec > perf_start) {
        usec -= perf_start;
    }
    else {
        usec += (0xffffffff - perf_start);
    }

    return usec;
}


/************************************************************************
 * Name: get_perf_diff_from_last
 *
 * Description:
 *   calculate the difference from the last time this function was called.
 *
 * Inputs:
 *   result from call to get_perf_time
 *
 * Return Value:
 *   Difference in perf_time from the last time this function was called
 *
 * Assumptions/Limitations:
 *   Other than initializing this is the only function to manipulate
 *   last_perf_time.
 *
 ************************************************************************/
inline uint32_t get_perf_diff_from_last(uint32_t current_time)
{
    uint32_t diff_time;

    /* For troubleshooting timing
    if(current_time < last_perf_time)
    {
        lowsyslog("Error Performance Timer ran backward!!! \r\n");
    }
    */

    if(last_perf_time == 0) {
        diff_time = 0;
    }
    else {
        diff_time = current_time - last_perf_time;
    }

    last_perf_time = current_time;

    return diff_time;
}


/************************************************************************
 * Name: sched_process_perf_track
 *
 * Description:
 *   Initialize all variables for performance tracking.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called at OS initialization time.
 *
 ************************************************************************/
void init_perf_track(void)
{
    perf_active = false;
    perf_start = 0;
    perf_stop = 0;
    last_perf_time = 0;
    curr_irq = NO_IRQ;
}


/************************************************************************
 * Name: start_perf_track
 *
 * Description:
 *   Start or reset performance tracking.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   None
 *
 ************************************************************************/
void start_perf_track(void)
{
    irqstate_t flags;
    int i;

    /* Momentarily disable interrupts.  We need (1) the task to stay valid
     * while we are doing these operations and (2) the tick counts to be
     * synchronized when read.
     */
    flags = irqsave();

    /* clear all timers */
    for (i = 0; i < CONFIG_MAX_TASKS; i++) {
      g_pidhash[i].thread_time = 0;
    }
    for (i = 0; i < NR_IRQS; i++) {
        irq_times[i] = 0;
    }

    last_perf_time = 0;

    perf_stop = 0;

    perf_start = hrt_getusec();

    curr_hash_index = PIDHASH(((struct tcb_s*)g_readytorun.head)->pid);

    perf_active = true;

    irqrestore(flags);
}

/************************************************************************
 * Name: stop_perf_track
 *
 * Description:
 *   Stop performance tracking and gather performance data.
 *
 * Inputs:
 *   ToDo
 *
 * Return Value:
 *   OK (0) on success
 *   -EPERM_STR
 *
 ************************************************************************/
int stop_perf_track(void)
{
    irqstate_t flags;
    int ret_val = OK;

    /* Momentarily disable interrupts.  We need (1) the task to stay valid
     * while we are doing these operations and (2) the tick counts to be
     * synchronized when read.
     */
    flags = irqsave();

    if(!perf_active) {
        ret_val = -EPERM;
    }
    else {

    	perf_stop = hrt_getusec();

        perf_active = false;
    }

    irqrestore(flags);

    return ret_val;
}

/************************************************************************
 * Name: get_total_perf_time
 *
 * Description:
 *   Get the time performance tracking is or was active
 *
 * Inputs:
 *   none
 *
 * Return Value:
 *   Time from when performance tracking started to when it stopped.
 *   or
 *   If performance tracking has not stopped return time up till now.
 *
 ************************************************************************/
uint32_t get_total_perf_time(void)
{
    uint32_t total_time = 0;

    if(!perf_active) {
	    if(perf_stop > perf_start) {
	    	total_time = perf_stop - perf_start;
	    }
	    else {
	    	total_time = (0xffffffff - perf_start) + perf_stop;
	    }
    }
    else {
    	total_time = get_perf_time();
    }

    return total_time;
}

/************************************************************************
 * Name: irq_perf_foreach
 *
 * Description:
 *   Every interrupt is enumerated and passed to handler of
 *   "irq_perf_foreach_t" type.
 *
 * Inputs:
 *   handler - irq_perf_foreach_t
 *   arg - passed from original caller
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void irq_perf_foreach(irq_perf_foreach_t handler, FAR void *arg)
{
    int irq;

    if(handler) {
        for ( irq = 0; irq < NR_IRQS; irq++) {
            handler(irq, tsb_irq_name(irq), irq_times[irq], arg);
        }
    }
}

/************************************************************************
 * Name: sched_perf_foreach
 *
 * Description:
 *   Every tcb is enumerated and passed to handler of
 *   "sched_perf_foreach_t" type.
 *
 *   Each terminated process also tracked
 *
 * Inputs:
 *   handler - sched_perf_foreach_t
 *   arg - passed from original caller
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void sched_perf_foreach(sched_perf_foreach_t handler, FAR void *arg)
{
    irqstate_t flags = irqsave();
    int ndx;
    struct tcb_s *tcb;

    if(handler) {
        for (ndx = 0; ndx < CONFIG_MAX_TASKS; ndx++) {
            if (g_pidhash[ndx].tcb) {
                tcb = g_pidhash[ndx].tcb;

    #if CONFIG_TASK_NAME_SIZE > 0
                handler(tcb->pid, tcb->name, g_pidhash[ndx].thread_time, arg);
    #else
                handler(tcb->pid, "<noname>", g_pidhash[ndx].thread_time, arg);
    #endif
            }
        }
    }

    irqrestore(flags);
}

/************************************************************************
 * Name: sched_track_switch
 *
 * Description:
 *   Record timing of context switch.
 *
 *   Only call when not in IRQ context.
 *
 * Inputs:
 *   new_tcb - tcb being switched to.
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void sched_track_switch (struct tcb_s* new_tcb)
{
    uint32_t usec;
    int hash_index;
    irqstate_t flags;

    if (perf_active) {
        /* only called when in non irq context state */
        flags = irqsave();

        /* get the time diff from the last sample */
        usec = get_perf_diff_from_last(get_perf_time());

        hash_index = PIDHASH(new_tcb->pid);

        /* add the time diff to the old sample */
        g_pidhash[curr_hash_index].thread_time += usec;

        /* keep track of who we are tracking now */
        curr_hash_index = hash_index;

        irqrestore(flags);
    }
}

/************************************************************************
 * Name: sched_track_irq_stop
 *
 * Description:
 *   Stop the tracking of irq.
 *
 * Inputs:
 *   none
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
inline void sched_track_irq_stop(void)
{
    if (perf_active) {
        /* guard against more more one call per irq */
        if ( curr_irq != NO_IRQ) {
            /* stop tracking interrupt */
            irq_times[curr_irq] += get_perf_diff_from_last(get_perf_time());

            curr_irq = NO_IRQ;

            /* update to the tcb now being tracked
             * after possible context switch in IRQ
             */
            curr_hash_index = PIDHASH(((struct tcb_s*)g_readytorun.head)->pid);
        }
    }
}

/************************************************************************
 * Name: sched_track_irq_start
 *
 * Description:
 *   Stop tracking task and start tracking interrupt
 *
 * Inputs:
 *   irq      - the irq being handled
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
inline void sched_track_irq_start (int irq)
{
    if (perf_active) {
        /* stop tracking tcb */
        g_pidhash[curr_hash_index].thread_time += get_perf_diff_from_last(get_perf_time());

        /* track the current irq */
        curr_irq = irq;

    }
}


/************************************************************************
 * Name: sched_track_pre_exit
 *
 * Description:
 *   tcb is about to be removed, tack it.
 *
 * Inputs:
 *   dead_tcb - tcb about to be removed.
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void sched_track_pre_exit(struct tcb_s* dead_tcb)
{
    if (perf_active) {
        //ToDo
        lldbg("Hit sched_track_pre_exit\n");
    }
}

/************************************************************************
 * Name: sched_track_post_exit
 *
 * Description:
 *   tcb was just removed and this is the new one.
 *
 * Inputs:
 *   new_tcb - just switched to.
 *
 * Return Value:
 *   void
 *
 ************************************************************************/
void sched_track_post_exit(struct tcb_s* new_tcb)
{
    if (perf_active) {
        lldbg("Hit sched_track_post_exit\n");
    }
}

#endif
