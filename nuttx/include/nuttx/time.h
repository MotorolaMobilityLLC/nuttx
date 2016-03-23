/****************************************************************************
 * include/nuttx/time.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_TIME_H
#define __INCLUDE_NUTTX_TIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/util.h>

#include <stdbool.h>
#include <time.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* If Gregorian time is not supported, then neither is Julian */

#ifndef CONFIG_GREGORIAN_TIME
#  undef CONFIG_JULIAN_TIME
#else
#  define JD_OF_EPOCH           2440588    /* Julian Date of noon, J1970 */

#  ifdef CONFIG_JULIAN_TIME
#    define GREG_DUTC           -141427    /* Default is October 15, 1582 */
#    define GREG_YEAR            1582
#    define GREG_MONTH           10
#    define GREG_DAY             15
#  endif /* CONFIG_JULIAN_TIME */
#endif /* !CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function:  clock_isleapyear
 *
 * Description:
 *    Return true if the specified year is a leap year
 *
 ****************************************************************************/

EXTERN int clock_isleapyear(int year);

/****************************************************************************
 * Function:  clock_daysbeforemonth
 *
 * Description:
 *    Get the number of days that occurred before the beginning of the month.
 *
 ****************************************************************************/

EXTERN int clock_daysbeforemonth(int month, bool leapyear);

/****************************************************************************
 * Function:  clock_calendar2utc
 *
 * Description:
 *    Calendar/UTC conversion based on algorithms from p. 604
 *    of Seidelman, P. K. 1992.  Explanatory Supplement to
 *    the Astronomical Almanac.  University Science Books,
 *    Mill Valley.
 *
 ****************************************************************************/

EXTERN time_t clock_calendar2utc(int year, int month, int day);

#undef EXTERN
#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Function:  msec_to_usec
 *
 * Description:
 *    Convert milliseconds to microseconds.
 *
 ****************************************************************************/
static inline useconds_t msec_to_usec(unsigned ms)
{
    return ms * USEC_PER_MSEC;
}

/****************************************************************************
 * Function:  timespec_to_usec
 *
 * Description:
 *    Convert struct timespec to microseconds.
 *
 ****************************************************************************/
static inline useconds_t timespec_to_usec(const struct timespec *ts)
{
    return (ts->tv_sec * USEC_PER_SEC) + DIV_ROUND_CLOSEST(ts->tv_nsec, NSEC_PER_USEC);
}

/****************************************************************************
 * Function:  timespec_to_nsec
 *
 * Description:
 *    Convert struct timespec to nanoseconds.
 *
 ****************************************************************************/
static inline uint64_t timespec_to_nsec(const struct timespec *ts)
{
	return ((uint64_t)ts->tv_sec) * NSEC_PER_SEC + ts->tv_nsec;
}

/****************************************************************************
 * Function:  timeval_to_usec
 *
 * Description:
 *    Convert struct timeval to microseconds.
 *
 ****************************************************************************/
static inline useconds_t timeval_to_usec(const struct timeval *tv)
{
    return (tv->tv_sec * USEC_PER_SEC) + tv->tv_usec;
}

/****************************************************************************
 * Function:  usec_to_timespec
 *
 * Description:
 *    Convert microseconds to struct timespec.
 *
 ****************************************************************************/
static inline void usec_to_timespec(uint32_t usec, struct timespec *ts)
{
    ts->tv_sec = usec / USEC_PER_SEC;
    ts->tv_nsec = (usec % USEC_PER_SEC) * NSEC_PER_USEC;
}

/****************************************************************************
 * Function:  nsec_to_timespec
 *
 * Description:
 *    Convert nanoseconds to struct timespec.
 *
 ****************************************************************************/
static inline void nsec_to_timespec(uint64_t ns, struct timespec *ts)
{
    ts->tv_sec = ns / NSEC_PER_SEC;
    ts->tv_nsec = ns % NSEC_PER_SEC;
}

#endif /* __INCLUDE_NUTTX_TIME_H */
