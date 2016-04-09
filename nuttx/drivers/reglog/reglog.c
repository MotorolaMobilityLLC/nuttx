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
 * This module is used to dump a log of time, address and value pairs.  This
 * buffer is commonly used to log register or data writes with little impact
 * to real time.  The data can then be dumped through the /dev file system
 * or viewed on a debugger.  No semaphores are used in the dumping of the log
 * so that the write code can operate within interrupts without any issue.  This
 * means that is is possible for the output to be corrupted if the head pointer
 * is updated before the tail can advance.  This is deemeed acceptable, since it
 * should be rare and real time performance is more important than data
 * output.
 */
#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <nuttx/arch.h>
#include <reglog.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/compiler.h>

/*
 * Define the length of a reglog output line.  Currently it has the form
 * tttttttt:  aaaaaaaa vvvvvvvv
 *
 * Where:
 *   t is the time the log entry was added (8 hex digits)
 *   a is the address of the log entry (8 hex digits)
 *   v is the value of the log entry (8 hex digits)
 *
 * This results in 8*3 (digits) + 2 (spaces) + 1 (colon) + 1 (newline)
 * + 1 (null).
 */
#define REGLOG_LINE_LEN  29

struct reglog_s g_reglog;

static ssize_t reglog_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
    struct inode *inode_p  = filep->f_inode;
    struct reglog_s *reglog_p;
    struct reglog_value_s *value_p;
    irqstate_t flags;
    ssize_t i;
    ssize_t nout;

    /* Some sanity checking */
    DEBUGASSERT(inode_p && inode_p->i_private);
    reglog_p = inode_p->i_private;

    nout = 0;
    /* Warn that the data has overflowed. */
    if (reglog_p->tail+REGLOG_DEPTH < reglog_p->head)
    {
        i = sprintf(buffer, "Reglog has overflowed by %u entries since the last dump.\n",
                    reglog_p->head-reglog_p->tail-REGLOG_DEPTH);
        nout = i;
        len -= i;
        buffer += i;
        reglog_p->tail = reglog_p->head-REGLOG_DEPTH;
    }
    /* Output until data is gone. */
    while ((reglog_p->head != reglog_p->tail) && (len > REGLOG_LINE_LEN))
    {
        value_p = &reglog_p->log[reglog_p->tail++&(REGLOG_DEPTH-1)];
        /* If this format is updated, REGLOG_LINE_LEN must also be updated. */
        i = snprintf(buffer, REGLOG_LINE_LEN, "%08x: %08x %08x\n",
                     value_p->time, value_p->addr, value_p->val);
        len -= i;
        buffer += i;
        nout += i;
    }
    /* If all was output and in stack mode, reset the head pointer. */
    if ((g_reglog.mode == REGLOG_MODE_STACK) &&
        (nout == 0))
    {
        flags = irqsave();
        reglog_p->head = 0;
        reglog_p->tail = 0;
        irqrestore(flags);
    }
    return nout;
}

static ssize_t reglog_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
    if (len > 0)
    {
        switch (buffer[0])
        {
            case 'F':
            case 'f':
                /* Swtich to fifo mode. */
                reglog_set_mode(REGLOG_MODE_FIFO);
                break;

            case 'S':
            case 's':
                /* Switch to statck mode. */
                reglog_set_mode(REGLOG_MODE_STACK);
                break;

            default:
                break;
        }
    }
    return len;
}

static const struct file_operations g_reglogfops =
{
    NULL,             /* open */
    NULL,             /* close */
    reglog_read,      /* read */
    reglog_write,     /* write */
    NULL,             /* seek */
    NULL              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
    , NULL            /* poll */
#endif
};

/**
 * @brief Provide a pointer to the next location to place log data.
 *
 * The pointer provided can be directly updated to add the values into the log.
 * The head will be advanced when the pointer is provided, unless in stack
 * mode and at the end of the buffer.  In this case the last entry will be
 * returned and no advance will be done.
 *
 * @return A pointer to the log entry to be updated.
 */
static inline struct reglog_value_s *reglog_get_next_entry(void)
{
    if ((g_reglog.mode == REGLOG_MODE_STACK) &&
        (g_reglog.head >= REGLOG_DEPTH))
    {
        return (&g_reglog.log[REGLOG_DEPTH-1]);
    }
    return (&g_reglog.log[(g_reglog.head++)&(REGLOG_DEPTH-1)]);
}

/**
 * @brief Function to handle logging.
 *
 * This function handles placing values into the log array.  The schedule clock
 * is used as the timer, unless it was overloaded by a lower lever driver.  This
 * can be done via a call to the reglog_register() function.
 *
 * @param reg The address to place in the log.
 * @param val The value to place in the log.
 */
void reglog_log(uint32_t reg, uint32_t val)
{
    struct reglog_value_s *reglog_p;

    reglog_p = reglog_get_next_entry();
    reglog_p->time = g_reglog.get_time_fcn();
    reglog_p->addr = reg;
    reglog_p->val = val;
}

/**
 * @brief Advance the tail past the duplicate data.
 *
 * Since the head and tail continue past the end of the buffer it may be necessary
 * to advance the tail to the latest data.  This may be desirable when reading
 * the data over a low speed interface.
 *
 * @return The number of bytes which were missed due to buffer overflow.
 */
unsigned int reglog_advance_tail(void)
{
    unsigned int old_tail;

    old_tail = g_reglog.tail;
    if (g_reglog.tail+REGLOG_DEPTH < g_reglog.head)
    {
        g_reglog.tail = g_reglog.head-REGLOG_DEPTH;
    }
    return g_reglog.tail-old_tail;
}

/**
 * @brief Read a number of entries from the register log and return them.
 *
 * Copies log entries into a buffer provided.  The copy is necessary to allow the
 * entries to be reused once the tail is advanced.
 *
 * @param entries Pointer to the memory in which to place the log entries.
 * @param n       The maximum number of log entries to copy.
 *
 * @return The number of entries read.
 */
size_t reglog_get_entries(struct reglog_value_s *entries, size_t n)
{
    size_t i = 0;

    while ((g_reglog.head != g_reglog.tail) && (n > i))
    {
        memcpy(entries++, &g_reglog.log[(g_reglog.tail++)&(REGLOG_DEPTH-1)], sizeof(g_reglog.log[0]));
        i++;
    }
    return i;
}

/**
 * @brief Set the mode of logging to stack or FIFO.
 *
 * @param mode The mode to set the log to.  See REGLOG_MODE_T for more details.
 */
void reglog_set_mode(REGLOG_MODE_T mode)
{
    irqstate_t flags;

    if (g_reglog.mode != mode)
    {
        flags = irqsave();
        g_reglog.head = 0;
        g_reglog.tail = 0;
        g_reglog.mode = mode;
        irqrestore(flags);
    }
}

/**
 * @brief Provide a default clock function.
 *
 * This is necessary since clock_systimer is a macro.
 */
uint32_t reglog_get_time_default(void)
{
    return clock_systimer();
}

/**
 * @brief Initlaize the reglog structures and low lever driver if necessary.
 */
void reglog_initialize(void)
{
    memset(&g_reglog, 0, sizeof(g_reglog));
    g_reglog.get_time_fcn = reglog_get_time_default;
    up_reglog_init();
    register_driver("/dev/reglog", &g_reglogfops, 0666, &g_reglog);
}
