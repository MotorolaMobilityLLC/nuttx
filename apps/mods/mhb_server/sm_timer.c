/*
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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

#include <debug.h>
#include <errno.h>
#include <signal.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/list.h>

#include "sm_timer.h"

#define SVC_TIMER_SIGNAL 16
#define MAX_SVC_TIMER 8

struct svc_timer_item {
    struct list_head node;
    timer_t timer;
    svc_timer_handler handler;
    long timeout;
};

static struct svc_timer_item s_timers[MAX_SVC_TIMER];

static struct list_head s_timer_list;

static pthread_mutex_t s_mutex;

static struct svc_timer_item *get_item(void) {
    struct svc_timer_item* item = NULL;

    if (!list_is_empty(&s_timer_list)) {
        item = list_entry(s_timer_list.next,
                          struct svc_timer_item, node);
        list_del(&item->node);
    }

    return item;
}

static void put_item(struct svc_timer_item *item) {
    struct list_head *iter;

    /* duplicate check */
    list_foreach(&s_timer_list, iter) {
        if (item == list_entry(iter, struct svc_timer_item, node)) {
            return;
        }
    }
    item->timer = NULL;
    item->handler = NULL;
    item->timeout = 0;
    list_add(&s_timer_list, &item->node);
}

static bool is_item_valid_in_use(struct svc_timer_item *item) {
    int i;
    bool valid = false;

    if (!item) {
        return valid;
    }

    for (i = 0; i < MAX_SVC_TIMER; i++) {
        if (&s_timers[i] == item &&
            item->timer != NULL &&
            item->handler != NULL) {
            valid = true;
            break;
        }
    }

    return valid;
}

static void timer_handler(int signo, siginfo_t *info, void *ucontext)
{
    if (signo != SVC_TIMER_SIGNAL) {
        return;
    }

    /* cross check */
    if (!info || info->si_code != SI_TIMER ||
        info->si_signo != SVC_TIMER_SIGNAL) {
        return;
    }

    struct svc_timer_item *item =
        (struct svc_timer_item *)info->si_value.sival_ptr;

    pthread_mutex_lock(&s_mutex);
    if (is_item_valid_in_use(item)) {
        item->handler((svc_timer_t)item);
    }
    pthread_mutex_unlock(&s_mutex);
}

static bool start_timer(timer_t timerid, long timeout)
{
    struct itimerspec timer;

    memset(&timer, 0, sizeof(timer));
    /* timeout is passed in ms order*/
    timer.it_value.tv_sec     = timeout / 1000;
    timer.it_value.tv_nsec    = (timeout % 1000) * 1000;

    if (timer_settime(timerid, 0, &timer, NULL) != OK) {
        lldbg("ERROR: timer_settime failed, errno=%d\n", errno);
        return false;
    }
    return true;
}

void svc_timer_init(void)
{
    sigset_t sigset;
    struct sigaction act;
    int status;
    int i;

    (void)sigemptyset(&sigset);
    (void)sigaddset(&sigset, SIGALRM);
    status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
    if (status != OK) {
        lldbg("ERROR: sigprocmask failed, status=%d\n", status);
    }

    act.sa_handler = NULL;
    act.sa_sigaction = timer_handler;
    act.sa_flags  = SA_SIGINFO;

    (void)sigfillset(&act.sa_mask);
    (void)sigdelset(&act.sa_mask, SVC_TIMER_SIGNAL);

    status = sigaction(SVC_TIMER_SIGNAL, &act, NULL);
    if (status != OK) {
        lldbg("ERROR: sigaction failed, status=%d\n" , status);
    }

    pthread_mutex_init(&s_mutex, NULL);

    list_init(&s_timer_list);
    for (i = 0; i < MAX_SVC_TIMER; i++) {
        put_item(&s_timers[i]);
    }
}

svc_timer_t svc_set_timer(svc_timer_handler handler, long timeout)
{
    struct sigevent notify;
    timer_t timerid;
    struct svc_timer_item *item;
    int status;

    pthread_mutex_lock(&s_mutex);

    item = get_item();
    if (!item) {
        lldbg("ERROR: No more svc timer can be allocated\n");
        pthread_mutex_unlock(&s_mutex);
        return NULL;
    }

    notify.sigev_notify          = SIGEV_SIGNAL;
    notify.sigev_signo           = SVC_TIMER_SIGNAL;
    notify.sigev_value.sival_ptr = item;

    status = timer_create(CLOCK_REALTIME, &notify, &timerid);
    if (status != OK) {
        lldbg("ERROR: timer_create failed, errno=%d\n", errno);
        goto deleteitem;
    }

    if (!start_timer(timerid, timeout)) {
        goto deletetimer;
    }

    item->timer = timerid;
    item->handler = handler;
    item->timeout = timeout;

    pthread_mutex_unlock(&s_mutex);

    return (svc_timer_t)item;

deletetimer:
    timer_delete(timerid);
deleteitem:
    put_item(item);
    pthread_mutex_unlock(&s_mutex);
    return SVC_TIMER_INVALID;
}

bool svc_restart_timer(svc_timer_t timerid) {
    bool ret;
    struct svc_timer_item *item = (struct svc_timer_item *)timerid;

    pthread_mutex_lock(&s_mutex);
    if (!is_item_valid_in_use(item)) {
        lldbg("ERROR: try to restart invalid timer item. (0x%p)\n", item);
        pthread_mutex_unlock(&s_mutex);
        return false;
    }
    ret = start_timer(item->timer, item->timeout);
    pthread_mutex_unlock(&s_mutex);

    return ret;
}


void svc_delete_timer(svc_timer_t timerid) {

    struct svc_timer_item *item = (struct svc_timer_item *)timerid;

    pthread_mutex_lock(&s_mutex);

    if (!is_item_valid_in_use(item)) {
        lldbg("ERROR: try to delete invalid timer item. (0x%p)\n", item);
        pthread_mutex_unlock(&s_mutex);
        return;
    }
    timer_delete(item->timer);
    put_item(item);

    pthread_mutex_unlock(&s_mutex);
}
