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

#include <nuttx/fs/fs.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include "up_arch.h"
#include "debug.h"

#if defined(CONFIG_ARCH_CORTEXM0) || defined(CONFIG_ARCH_CORTEXM1) || \
    defined(CONFIG_ARCH_CORTEXM3) || defined(CONFIG_ARCH_CORTEXM4)
#define SEMIHOSTING_SVC "bkpt 0xAB;"
#else
#define SEMIHOSTING_SVC "svc 0xAB;"
#endif

#define NVIC_DHCSR              0xe000edf0
#define NVIC_DHCSR_C_DEBUGEN    (1 << 0)

enum open_mode {
    READ_MODE = 0,
    WRITE_MODE = 4,
};

enum semihosting_stream {
    SEMIHOSTING_READ_STREAM = 0,
    SEMIHOSTING_WRITE_STREAM = 1,
};

enum semihosting_syscall {
    SYSCALL_OPEN = 0x1,
    SYSCALL_CLOSE = 0x2,
    SYSCALL_WRITEC = 0x3,
    SYSCALL_WRITE = 0x5,
    SYSCALL_READ = 0x6,
};

struct semihosting_priv {
    int fd[2];
};

static struct semihosting_priv semihosting_console;

static uint32_t semihosting_syscall(int syscall, uint32_t *params)
{
    uint32_t result;

    asm volatile(
        "mov r0, %2;"
        "mov r1, %1;"
        SEMIHOSTING_SVC
        "mov %0, r0;"
        :"=r"(result)
        :"r"(params), "r"(syscall)
        :"r0", "r1", "memory"
    );

    return result;
}

int semihosting_open(const char *const filename, int mode)
{
    uint32_t params[] = {
        (uint32_t) filename,
        mode,
        (uint32_t) strlen(filename)
    };

    if (!(getreg32(NVIC_DHCSR) & NVIC_DHCSR_C_DEBUGEN)) {
        lowsyslog("arm semihosting: no debugger attached\n");
        return -ENODEV;
    }

    return semihosting_syscall(SYSCALL_OPEN, &params[0]);
}

int semihosting_close(int fd)
{
    uint32_t param = fd;
    return semihosting_syscall(SYSCALL_CLOSE, &param);
}

ssize_t semihosting_read(int fd, const char *buffer, size_t buflen)
{
    ssize_t nread;
    uint32_t params[3];

    params[0] = (uint32_t) fd;
    params[1] = (uint32_t) buffer;
    params[2] = (uint32_t) buflen;

    nread = semihosting_syscall(SYSCALL_READ, &params[0]);

    return buflen - nread;
}

ssize_t semihosting_write(int fd, const char *buffer, size_t buflen)
{
    size_t not_written = 0;
    uint32_t params[3];

    params[0] = (uint32_t) fd;
    params[1] = (uint32_t) buffer;
    params[2] = (uint32_t) buflen;

    not_written = semihosting_syscall(SYSCALL_WRITE, &params[0]);

    return buflen - not_written;
}

static ssize_t semihosting_consoleread(struct file *filep, char *buffer,
                                       size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct semihosting_priv *priv = inode->i_private;

    DEBUGASSERT(priv);

    return semihosting_read(priv->fd[SEMIHOSTING_READ_STREAM], buffer, buflen);
}

static ssize_t semihosting_consolewrite(struct file *filep, const char *buffer,
                                 size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct semihosting_priv *priv = inode->i_private;

    DEBUGASSERT(priv);

    return semihosting_write(priv->fd[SEMIHOSTING_READ_STREAM], buffer, buflen);
}

void semihosting_putc(char c)
{
    uint32_t c32 = c;

    if (!(getreg32(NVIC_DHCSR) & NVIC_DHCSR_C_DEBUGEN))
        return;

    semihosting_syscall(SYSCALL_WRITEC, (uint32_t*) &c32);
}

static const struct file_operations semihosting_ops = {
    .read = semihosting_consoleread,
    .write = semihosting_consolewrite,
};

void semihosting_consoleinit(void)
{
    if (!(getreg32(NVIC_DHCSR) & NVIC_DHCSR_C_DEBUGEN)) {
        lowsyslog("arm semihosting: no debugger attached\n");
        return;
    }

    semihosting_console.fd[SEMIHOSTING_READ_STREAM] =
        semihosting_open(":tt", READ_MODE);
    if (semihosting_console.fd[SEMIHOSTING_READ_STREAM] == -1)
        return;

    semihosting_console.fd[SEMIHOSTING_WRITE_STREAM] =
        semihosting_open(":tt", WRITE_MODE);
    if (semihosting_console.fd[SEMIHOSTING_WRITE_STREAM] == -1)
        return;

    register_driver("/dev/console", &semihosting_ops, 0666,
                    &semihosting_console);
}
