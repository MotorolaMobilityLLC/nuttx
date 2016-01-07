/*
 * Copyright (C) 2015 Motorola Mobility, LLC.
 * Copyright (C) 2012 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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
 */

#include <nuttx/config.h>
#include <nuttx/watchdog.h>

#include <sys/ioctl.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define WDOG_TIMEOUT_MS  15000
#define WDOG_PING_MS      5000

#define WDOG_STACK_SIZE    640

#ifdef CONFIG_WATCHDOG
int wdog_fd;

static void *wdog_ping_thread(void *v)
{
  int ret;

  dbg("Watchdog started\n");

  while(1)
    {
      usleep(WDOG_PING_MS * 1000);

      ret = ioctl(wdog_fd, WDIOC_KEEPALIVE, 0);
      if (ret < 0)
          dbg("Failed to pet the watchdog!\n");
    }

  return NULL;
}
#endif

int wdog_init(void)
{
#ifdef CONFIG_WATCHDOG
  int ret;
  pthread_attr_t thread_attr;
  pthread_t wdog_thread;

  ret = up_wdginitialize();
  if (ret != OK)
    {
      dbg("up_wdginitialize failed: %d\n", ret);
      return ret;
    }

  /* Open the watchdog device for reading */

  wdog_fd = open(CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (wdog_fd < 0)
    {
      dbg("open %s failed: %d\n", CONFIG_WATCHDOG_DEVPATH, errno);
      return wdog_fd;
    }

  /* Set the watchdog timeout */

  ret = ioctl(wdog_fd, WDIOC_SETTIMEOUT, WDOG_TIMEOUT_MS);
  if (ret < 0)
    {
      dbg("ioctl(WDIOC_SETTIMEOUT) failed: %d\n", errno);
      return ret;
    }

  /* Start the watchdog timer. */

  ret = ioctl(wdog_fd, WDIOC_START, 0);
  if (ret < 0)
    {
      dbg("ioctl(WDIOC_START) failed: %d\n", errno);
      return ret;
    }

  pthread_attr_init(&thread_attr);
  pthread_attr_setstacksize(&thread_attr, WDOG_STACK_SIZE);
  pthread_create(&wdog_thread, &thread_attr, wdog_ping_thread, NULL);
  pthread_detach(wdog_thread);
  pthread_attr_destroy(&thread_attr);
#endif

  return OK;
}
