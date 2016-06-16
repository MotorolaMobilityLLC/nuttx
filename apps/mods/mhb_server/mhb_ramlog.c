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

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <unistd.h>

#include <nuttx/syslog/ramlog.h>

#include "mhb_server.h"

#define MHB_SIG_STOP (9)

#define MHB_RAMLOG_NOT_SIZE (64)

#define MHB_RAMLOG_TIMEOUT_MS (1000)

struct mhb_ramlog {
    pthread_t thread;
    bool is_running;
};

static struct mhb_ramlog g_mhb_ramlog;

static void *mhb_ramlog_thread(void *data)
{
    vdbg("data=%p\n", data);

#if CONFIG_DISABLE_POLL
    dbg("ERROR: polling disabled\n");
    return (void *)-EINVAL;
#else
    int ret = 0;
    struct pollfd pollfd;

    struct mhb_ramlog *mhb_ramlog = (struct mhb_ramlog *)data;
    if (!mhb_ramlog) {
        return (void *)-EINVAL;
    }

    pollfd.events = POLLIN;
    pollfd.fd = open(CONFIG_SYSLOG_DEVPATH, O_RDONLY);
    if (pollfd.fd == -1) {
        dbg("opening FIFO for reading failed: %d\n", errno);
        goto err;
    }

    mhb_ramlog->is_running = true;
    while (mhb_ramlog->is_running) {
        char buffer[MHB_RAMLOG_NOT_SIZE];
        size_t used = 0;

        ret = poll(&pollfd, 1 /* num */, MHB_RAMLOG_TIMEOUT_MS);
        if (ret == 0) {
            vdbg("timeout\n");
        } else if (ret < 0) {
            if (errno == EINTR) {
                vdbg("poll interrupted\n");
                continue;
            }

            dbg("ERROR: ramlog poll failed\n", errno);
            ret = -errno;
            goto err;
        }

        /* Read loop */
        ssize_t nbytes;
        do {
            nbytes = read(pollfd.fd, buffer + used, sizeof(buffer) - used);
            if (nbytes < 0) {
                vdbg("read: %d\n", ret);
                /* Error */
                if (errno == EINTR) {
                    vdbg("read interrupted\n");
                    continue;
                }

                /* Fail */
                dbg("ERROR: ramlog read failed\n", errno);
                ret = -errno;
                goto err;
            } else {
                used += nbytes;

                if (used == sizeof(buffer) || (!nbytes && used)) {
                    /* Buffer full or poll timeout, send log notification. */
                    mhb_send_diag_log_not(buffer, used);
                    used = 0;
                }
            }
        } while (nbytes);
    }

err:
    close(pollfd.fd);

    vdbg("done: ret=%d\n", ret);
    return (void *)ret;
#endif
}

int mhb_ramlog_enable(void)
{
    vdbg("\n");

    int ret = pthread_create(&g_mhb_ramlog.thread, NULL /* attr */,
                             mhb_ramlog_thread, &g_mhb_ramlog);
    vdbg("create: thread=%p, addr=%p, ret=%d\n",
         g_mhb_ramlog.thread, mhb_ramlog_thread, ret);
    if (ret) {
        dbg("ERROR: Failed to create rx thread: %d\n", errno);
        g_mhb_ramlog.thread = 0;
        return -errno;
    }

    return 0;
}

int mhb_ramlog_disable(bool force)
{
    vdbg("force=%d\n", force);

    int ret = 0;

    if (!g_mhb_ramlog.thread) {
        return ret;
    }

    if (!force) {
        /* If not forcing, give the thread time to send any queued logs. */
        vdbg("before\n");
        usleep(2 * MHB_RAMLOG_TIMEOUT_MS * 1000);
        vdbg("after\n");
    }

    /* Signal to the thread to stop. */
    g_mhb_ramlog.is_running = false;

    pthread_addr_t join_value = 0;
    ret = pthread_join(g_mhb_ramlog.thread, &join_value);
    vdbg("join: thread=%p, addr=%p, ret=%d\n",
         g_mhb_ramlog.thread, join_value, ret);

    g_mhb_ramlog.thread = 0;

    return ret;
}