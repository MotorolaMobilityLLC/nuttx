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
 */
#ifndef __INCLUDE_REGLOG_H
# define __INCLUDE_REGLOG_H

#include <sys/types.h>

/**
 * @brief The number of log entries to keep before the buffer wraps around.
 *
 * This value must be a power of 2 or the code will not function as expected.
 */
# define REGLOG_DEPTH 1024

/**
 * @brief Structure to hold a register address or generic id and value along
 *        with the time the value was logged.
 *
 * The time logged is kept in system timer ticks unless it has been overloaded
 * by a lower level driver.
 */
struct reglog_value_s
{
    uint32_t time;
    uint32_t addr;
    uint32_t val;
};

typedef enum
{
    /** @brief The default mode is to wrap arround the end of the buffer and log all entries. */
    REGLOG_MODE_FIFO,
    /**
     * @brief In stack mode no further entries are added once the end of the buffer is reached until
     *         the buffer is read.
     */
    REGLOG_MODE_STACK,
    REGLOG_MODE_END
} REGLOG_MODE_T;
/**
 * @brief Structure to hold an array of log values as well as an index to the
 *        current value.
 *
 * The indicies continues to increment past the end of the array.  This must be taken
 * into account when indexing the array.  This is done to the log can include the
 * number of samples which was not included in the buffer.
 */
struct reglog_s
{
    uint32_t (*get_time_fcn)(void);
    REGLOG_MODE_T mode;
    unsigned int head;
    unsigned int tail;
    struct reglog_value_s log[REGLOG_DEPTH];
};

# if defined(CONFIG_REGLOG)
extern struct reglog_s g_reglog;

/**
 * @brief Overload the default log function.
 *
 * This function can be used to overload the default get time function in order
 * to provide a higher resolution timer.  A default function will be installed
 * upon init of this driver, so it is not necessary to call this.
 *
 * @param reglog_get_time_fcn The time function to register.
 */
static inline void reglog_register(uint32_t (*reglog_get_time_fcn)(void))
{
    g_reglog.get_time_fcn = reglog_get_time_fcn;
}

/**
 * @brief Function provided by the low level driver to initialize itself.
 */
extern void up_reglog_init(void);
extern void reglog_log(uint32_t reg, uint32_t val);
extern unsigned int reglog_advance_tail(void);
extern size_t reglog_get_entries(struct reglog_value_s *entries, size_t n);
extern void reglog_set_mode(REGLOG_MODE_T mode);
extern void reglog_initialize(void);
# else
/*
 * If not enabled make sure the functions do nothing.   This allows reglog_log
 * lines to remain in the code even when it is not configured.
 */
# define reglog_log(reg, val)
# define reglog_advance_tail() 0
# define reglog_get_entries(p, n) 0
# define reglog_set_mode() 0
# define reglog_initialize()
# endif
#endif
