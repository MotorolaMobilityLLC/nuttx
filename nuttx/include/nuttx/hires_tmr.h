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

#ifndef __ARCH_ARM_INCLUDE_HIRES_TMR_H
#define __ARCH_ARM_INCLUDE_HIRES_TMR_H

#include <time.h>

/*
 * This is a simple framework that is meant to supplement the basic system
 * tick based timer used by nuttx. It's not meant to replace the rtc
 * functionality nor is it connected to it in any way. If the board is
 * equipped with a real RTC then the rtc framework should be used instead.
 */

#ifdef CONFIG_ARCH_HAVE_HIRES_TIMER
extern void hrt_gettimespec(struct timespec *ts);
extern uint32_t hrt_getusec(void);
extern void hrt_clear_rollover(void);
#else
static inline void hrt_gettimespec(struct timespec *ts)
{
    ts->tv_sec = 0;
    ts->tv_nsec = 0;
}
static inline uint32_t hrt_getusec(void)
{
    return 0;
}
#endif /* CONFIG_HAVE_HIRES_TIMER */

#endif /* __ARCH_ARM_INCLUDE_HIRES_TMR_H */
