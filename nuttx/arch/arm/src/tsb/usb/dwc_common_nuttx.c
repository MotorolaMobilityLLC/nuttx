/*
 * Copyright (c) 2014-2015 Google Inc.
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 *
 * Based on dwc_common_linux.c
 */

#include <stdint.h>
#include <stdarg.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/bufram.h>
#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <arch/byteorder.h>

#ifdef DWC_CCLIB
# include "dwc_cc.h"
#endif

#ifdef DWC_NOTIFYLIB
# include "dwc_notifier.h"
#endif

/* OS-Level Implementations */

#include "dwc_os.h"
#include "dwc_list.h"

#ifdef CONFIG_DWC_QUIET
#undef __DWC_WARN
#undef __DWC_ERROR
#undef DWC_PRINTF
#endif

/* MISC */

static inline uint32_t msecs_to_ticks(uint32_t time)
{
    return time * (CLOCKS_PER_SEC / 1000);
}

void *DWC_MEMSET(void *dest, uint8_t byte, uint32_t size)
{
    return memset(dest, byte, size);
}

void *DWC_MEMCPY(void *dest, void const *src, uint32_t size)
{
    return memcpy(dest, src, size);
}

void *DWC_MEMMOVE(void *dest, void *src, uint32_t size)
{
    return memmove(dest, src, size);
}

int DWC_MEMCMP(void *m1, void *m2, uint32_t size)
{
    return memcmp(m1, m2, size);
}

int DWC_STRNCMP(void *s1, void *s2, uint32_t size)
{
    return strncmp(s1, s2, size);
}

int DWC_STRCMP(void *s1, void *s2)
{
    return strcmp(s1, s2);
}

int DWC_STRLEN(char const *str)
{
    return strlen(str);
}

char *DWC_STRCPY(char *to, char const *from)
{
    return strcpy(to, from);
}

char *DWC_STRDUP(char const *str)
{
    return strdup(str);
}

int DWC_ATOI(const char *str, int32_t *value)
{
    char *end = NULL;

    *value = strtol(str, &end, 0);
    if (*end == '\0') {
        return 0;
    }

    return -1;
}

int DWC_ATOUI(const char *str, uint32_t *value)
{
    char *end = NULL;

    *value = strtoul(str, &end, 0);
    if (*end == '\0') {
        return 0;
    }

    return -1;
}

/* dwc_debug.h */

dwc_bool_t DWC_IN_IRQ(void)
{
    uint32_t ipsr;
    asm volatile("mrs %0, epsr" : "=r"(ipsr));
    return (ipsr & 0x1ff) != 0;
}

dwc_bool_t DWC_IN_BH(void)
{
    return false;
}

void DWC_VPRINTF(char *format, va_list args)
{
    lowvsyslog(format, args);
}

int DWC_VSNPRINTF(char *str, int size, char *format, va_list args)
{
    return vsnprintf(str, size, format, args);
}

void DWC_PRINTF(char *format, ...)
{
    va_list args;

    va_start(args, format);
    DWC_VPRINTF(format, args);
    va_end(args);
}

int DWC_SPRINTF(char *buffer, char *format, ...)
{
    int retval;
    va_list args;

    va_start(args, format);
    retval = vsprintf(buffer, format, args);
    va_end(args);
    return retval;
}

int DWC_SNPRINTF(char *buffer, int size, char *format, ...)
{
    int retval;
    va_list args;

    va_start(args, format);
    retval = vsnprintf(buffer, size, format, args);
    va_end(args);
    return retval;
}

void __DWC_WARN(char *format, ...)
{
    va_list args;

    va_start(args, format);
    DWC_PRINTF("Warning: ");
    DWC_VPRINTF(format, args);
    va_end(args);
}

void __DWC_ERROR(char *format, ...)
{
    va_list args;

    va_start(args, format);
    DWC_PRINTF("Error: ");
    DWC_VPRINTF(format, args);
    va_end(args);
}

void DWC_EXCEPTION(char *format, ...)
{
    va_list args;

    va_start(args, format);
    DWC_PRINTF("Error: ");
    DWC_VPRINTF(format, args);
    va_end(args);
    asm volatile("b .");
}

#ifdef DEBUG
void __DWC_DEBUG(char *format, ...)
{
    va_list args;

    va_start(args, format);
    DWC_VPRINTF(format, args);
    va_end(args);
}
#endif

/* dwc_mem.h */

void *__DWC_DMA_ALLOC(void *dma_ctx, uint32_t size, dwc_dma_t *dma_addr)
{
    void *buf = bufram_alloc(size);
    memset(buf, 0, size); /* TODO check if it is necessary */
    *dma_addr = (dwc_dma_t) buf;
    return buf;
}

void __DWC_DMA_FREE(void *dma_ctx, uint32_t size, void *virt_addr,
                    dwc_dma_t dma_addr)
{
    bufram_free(virt_addr);
}

void *__DWC_ALLOC(void *mem_ctx, uint32_t size)
{
#ifdef CONFIG_DWC_USE_BUFRAM
    void *buf = bufram_alloc(size);

    if (buf)
        memset(buf, 0, size);

    return buf;
#else
    return zalloc(size);
#endif
}

void __DWC_FREE(void *mem_ctx, void *addr)
{
#ifdef CONFIG_DWC_USE_BUFRAM
    bufram_free(addr);
#else
    free(addr);
#endif
}

/* Byte Ordering Conversions */

uint32_t DWC_CPU_TO_LE32(uint32_t *p)
{
    return cpu_to_le32(*p);
}

uint32_t DWC_CPU_TO_BE32(uint32_t *p)
{
    return cpu_to_be32(*p);
}

uint32_t DWC_LE32_TO_CPU(uint32_t *p)
{
    return le32_to_cpu(*p);
}

uint32_t DWC_BE32_TO_CPU(uint32_t *p)
{
    return be32_to_cpu(*p);
}

uint16_t DWC_CPU_TO_LE16(uint16_t *p)
{
    return cpu_to_le16(*p);
}

uint16_t DWC_CPU_TO_BE16(uint16_t *p)
{
    return cpu_to_be16(*p);
}

uint16_t DWC_LE16_TO_CPU(uint16_t *p)
{
    return le16_to_cpu(*p);
}

uint16_t DWC_BE16_TO_CPU(uint16_t *p)
{
    return be16_to_cpu(*p);
}


/* Registers */

uint32_t DWC_READ_REG32(uint32_t volatile *reg)
{
    return *reg;
}

void DWC_WRITE_REG32(uint32_t volatile *reg, uint32_t value)
{
    *reg = value;
}

void DWC_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask,
                      uint32_t set_mask)
{
    DWC_WRITE_REG32(reg, (DWC_READ_REG32(reg) & ~clear_mask) | set_mask);
}

/* Locking */

dwc_spinlock_t *DWC_SPINLOCK_ALLOC(void)
{
    return (dwc_spinlock_t *)~0u;
}

void DWC_SPINLOCK_FREE(dwc_spinlock_t *lock)
{
}

void DWC_SPINLOCK(dwc_spinlock_t *lock)
{
    sched_lock();
}

void DWC_SPINUNLOCK(dwc_spinlock_t *lock)
{
    sched_unlock();
}

void DWC_SPINLOCK_IRQSAVE(dwc_spinlock_t *lock, dwc_irqflags_t *flags)
{
    *flags = (dwc_irqflags_t) irqsave();
}

void DWC_SPINUNLOCK_IRQRESTORE(dwc_spinlock_t *lock, dwc_irqflags_t flags)
{
    irqrestore((irqstate_t) flags);
}

/* Timing */

void DWC_UDELAY(uint32_t usecs)
{
    up_udelay(usecs);
}

void DWC_MDELAY(uint32_t msecs)
{
    up_mdelay(msecs);
}

void DWC_MSLEEP(uint32_t msecs)
{
    while (msecs--)
        usleep(1000);
}

uint32_t DWC_TIME(void)
{
    uint32_t r;
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    r = t.tv_sec * 1000 + t.tv_nsec / 1000000;
    return r;
}

/* Timers */

struct dwc_timer {
    timer_t t;
    struct sigevent evp;
    struct sigaction sa;
    char *name;
    dwc_timer_callback_t cb;
    void *data;
    uint8_t scheduled;
    dwc_spinlock_t *lock;
};

static void timer_callback(int signo, siginfo_t *si, void *ptr)
{
    dwc_timer_t *timer = (dwc_timer_t *)si->si_value.sival_ptr;
    dwc_irqflags_t flags;

    DWC_SPINLOCK_IRQSAVE(timer->lock, &flags);
    timer->scheduled = 0;
    DWC_SPINUNLOCK_IRQRESTORE(timer->lock, flags);
    DWC_DEBUG("Timer %s callback", timer->name);
    if (!timer->cb) {
        DWC_ERROR("BUG: Timer doesn't have callback");
        return;
    }
    timer->cb(timer->data);
}

dwc_timer_t *DWC_TIMER_ALLOC(char *name, dwc_timer_callback_t cb, void *data)
{
    dwc_timer_t *t = DWC_ALLOC(sizeof(*t));
    memset(t, 0, sizeof(*t));

    if (!t) {
        DWC_ERROR("Cannot allocate memory for timer");
        return NULL;
    }

    t->name = DWC_STRDUP(name);
    if (!t->name) {
        DWC_ERROR("Cannot allocate memory for timer->name");
        goto no_name;
    }

    t->lock = DWC_SPINLOCK_ALLOC();
    if (!t->lock) {
        DWC_ERROR("Cannot allocate memory for lock");
        goto no_lock;
    }

    t->scheduled = 0;
    t->cb = cb;
    t->data = data;

    t->sa.sa_flags = SA_SIGINFO;
    t->sa.sa_sigaction = timer_callback;
    sigemptyset(&t->sa.sa_mask);
    if (sigaction(SIGRTMAX, &t->sa, NULL) == -1) {
        DWC_ERROR("Cannot enable signal");
        goto no_sig;
    }

    memset(&t->evp, 0, sizeof(t->evp));
    t->evp.sigev_notify = SIGEV_SIGNAL;
    t->evp.sigev_signo = SIGRTMAX;
    t->evp.sigev_value.sival_ptr = t;

    if (timer_create(CLOCK_REALTIME, &t->evp, &t->t) < 0)
         DWC_ERROR("Cannot create timer");

    return t;

 no_sig:
    DWC_SPINLOCK_FREE(t->lock);
 no_lock:
    DWC_FREE(t->name);
 no_name:
    DWC_FREE(t);
    return NULL;
}

void DWC_TIMER_FREE(dwc_timer_t *timer)
{
    dwc_irqflags_t flags;

    DWC_SPINLOCK_IRQSAVE(timer->lock, &flags);

    if (timer->scheduled) {
        timer->scheduled = 0;
        DWC_TIMER_CANCEL(timer);
    }

    timer_delete(timer->t);
    DWC_SPINUNLOCK_IRQRESTORE(timer->lock, flags);
    DWC_SPINLOCK_FREE(timer->lock);
    DWC_FREE(timer->name);
    DWC_FREE(timer);
}

void _DWC_TIMER_SCHEDULE(dwc_timer_t *timer, uint32_t time, int periodic)
{
    uint32_t ms;
    struct itimerspec its;
    dwc_irqflags_t flags;

    DWC_SPINLOCK_IRQSAVE(timer->lock, &flags);

    ms = time % 1000;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec = time / 1000;
    its.it_value.tv_nsec = ms * 1000000;
    if (periodic) {
        its.it_interval.tv_sec = time / 1000;
        its.it_interval.tv_nsec = ms * 1000000;
    }

    if (!timer->scheduled) {
        timer->scheduled = 1;
        DWC_DEBUG("Scheduling timer %s to expire in +%d msec", timer->name,
                  time);
    } else {
        DWC_DEBUG("Modifying timer %s to expire in +%d msec", timer->name,
                  time);
    }
    if (timer_settime(timer->t, 0, &its, NULL) < 0)
        perror("Can't start timer");

    DWC_SPINUNLOCK_IRQRESTORE(timer->lock, flags);
}

void DWC_TIMER_SCHEDULE(dwc_timer_t *timer, uint32_t time)
{
    _DWC_TIMER_SCHEDULE(timer, time, 0);
}

void DWC_TIMER_SCHEDULE_PERIODIC(dwc_timer_t *timer, uint32_t time)
{
    _DWC_TIMER_SCHEDULE(timer, time, 1);
}

void DWC_TIMER_CANCEL(dwc_timer_t *timer)
{
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    if (timer_settime(timer->t, 0, &its, NULL) < 0)
        perror("Can't stop timer!");
}

/* Wait Queues */

struct dwc_waitq {
    int abort;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
};

dwc_waitq_t *DWC_WAITQ_ALLOC(void)
{
    dwc_waitq_t *wq = DWC_ALLOC(sizeof(*wq));

    if (!wq) {
        DWC_ERROR("Cannot allocate memory for waitqueue\n");
        return NULL;
    }

    pthread_cond_init(&wq->cond, NULL);
    wq->abort = 0;
    return wq;
}

void DWC_WAITQ_FREE(dwc_waitq_t *wq)
{
    pthread_cond_destroy(&wq->cond);
    DWC_FREE(wq);
}

int32_t DWC_WAITQ_WAIT(dwc_waitq_t *wq, dwc_waitq_condition_t cond, void *data)
{
    int result = 0;

    while (!(cond(data) || wq->abort) && result == 0)
        result = pthread_cond_wait(&wq->cond, &wq->mutex);

    if (wq->abort == 1) {
        wq->abort = 0;
        return -DWC_E_ABORT;
    }

    wq->abort = 0;

    if (result == 0) {
        return 0;
    }

    return -DWC_E_UNKNOWN;
}

int32_t DWC_WAITQ_WAIT_TIMEOUT(dwc_waitq_t *wq, dwc_waitq_condition_t cond,
                   void *data, int32_t msecs)
{
    struct timespec abstime = {.tv_nsec = msecs * 1000000 };
    int result = 0;

    while (!(cond(data) || wq->abort) && result == 0)
        result = pthread_cond_timedwait(&wq->cond, &wq->mutex, &abstime);

    if (wq->abort == 1) {
        wq->abort = 0;
        return -DWC_E_ABORT;
    }

    wq->abort = 0;

    if (result == 0) {
        return -DWC_E_TIMEOUT;
    }

    return -DWC_E_UNKNOWN;
}

void DWC_WAITQ_TRIGGER(dwc_waitq_t *wq)
{
    wq->abort = 0;
    pthread_cond_broadcast(&wq->cond);
}

void DWC_WAITQ_ABORT(dwc_waitq_t *wq)
{
    wq->abort = 1;
    pthread_cond_broadcast(&wq->cond);
}

/* tasklets
 - run in interrupt context (cannot sleep)
 - each tasklet runs on a single CPU
 - different tasklets can be running simultaneously on different CPUs
 */
struct dwc_tasklet {
    struct work_s work;
    dwc_tasklet_callback_t cb;
    void *data;
};

static void tasklet_callback(void *data)
{
    dwc_tasklet_t *task = (dwc_tasklet_t *)data;
    task->cb(task->data);
}

dwc_tasklet_t *DWC_TASK_ALLOC(char *name, dwc_tasklet_callback_t cb, void *data)
{
    dwc_tasklet_t *t = DWC_ALLOC(sizeof(*t));

    if (t) {
        t->cb = cb;
        t->data = data;
    } else {
        DWC_ERROR("Cannot allocate memory for tasklet\n");
    }

    return t;
}

void DWC_TASK_FREE(dwc_tasklet_t *task)
{
    work_cancel(HPWORK, &task->work);
    DWC_FREE(task);
}

void DWC_TASK_SCHEDULE(dwc_tasklet_t *task)
{
    work_queue(HPWORK, &task->work, tasklet_callback, task, 0);
}

/* workqueues
 - run in process context (can sleep)
 */
typedef struct work_container {
    dwc_work_callback_t cb;
    void *data;
    dwc_workq_t *wq;
    char *name;

#ifdef DEBUG
    DWC_CIRCLEQ_ENTRY(work_container) entry;
#endif
} work_container_t;

#ifdef DEBUG
DWC_CIRCLEQ_HEAD(work_container_queue, work_container);
#endif

struct dwc_workq {
    struct work_s *wq;
    dwc_spinlock_t *lock;
    dwc_waitq_t *waitq;
    int pending;

#ifdef DEBUG
    struct work_container_queue entries;
#endif
};

static void do_work(void *data)
{
    dwc_irqflags_t flags;
    work_container_t *container = data;
    dwc_workq_t *wq = container->wq;

    container->cb(container->data);

#ifdef DEBUG
    DWC_CIRCLEQ_REMOVE(&wq->entries, container, entry);
#endif
    DWC_DEBUG("Work done: %s, container=%p", container->name, container);
    if (container->name) {
        DWC_FREE(container->name);
    }
    DWC_FREE(container);

    DWC_SPINLOCK_IRQSAVE(wq->lock, &flags);
    wq->pending--;
    DWC_SPINUNLOCK_IRQRESTORE(wq->lock, flags);
    DWC_WAITQ_TRIGGER(wq->waitq);
}

static int work_done(void *data)
{
    dwc_workq_t *workq = (dwc_workq_t *)data;
    return workq->pending == 0;
}

int DWC_WORKQ_WAIT_WORK_DONE(dwc_workq_t *workq, int timeout)
{
    return DWC_WAITQ_WAIT_TIMEOUT(workq->waitq, work_done, workq, timeout);
}

dwc_workq_t *DWC_WORKQ_ALLOC(char *name)
{
    dwc_workq_t *wq = DWC_ALLOC(sizeof(*wq));

    if (!wq) {
        return NULL;
    }

    wq->wq = malloc(sizeof(*wq->wq));
    memset(wq->wq, 0, sizeof(*wq->wq));
    if (!wq->wq) {
        goto no_wq;
    }

    wq->pending = 0;

    wq->lock = DWC_SPINLOCK_ALLOC();
    if (!wq->lock) {
        goto no_lock;
    }

    wq->waitq = DWC_WAITQ_ALLOC();
    if (!wq->waitq) {
        goto no_waitq;
    }

#ifdef DEBUG
    DWC_CIRCLEQ_INIT(&wq->entries);
#endif
    return wq;

 no_waitq:
    DWC_SPINLOCK_FREE(wq->lock);
 no_lock:
    free(wq->wq);
 no_wq:
    DWC_FREE(wq);

    return NULL;
}

void DWC_WORKQ_FREE(dwc_workq_t *wq)
{
#ifdef DEBUG
    if (wq->pending != 0) {
        struct work_container *wc;
        DWC_ERROR("Destroying work queue with pending work");
        DWC_CIRCLEQ_FOREACH(wc, &wq->entries, entry) {
            DWC_ERROR("Work %s still pending", wc->name);
        }
    }
#endif
    free(wq->wq);
    DWC_SPINLOCK_FREE(wq->lock);
    DWC_WAITQ_FREE(wq->waitq);
    DWC_FREE(wq);
}

void DWC_WORKQ_SCHEDULE(dwc_workq_t *wq, dwc_work_callback_t cb, void *data,
                        char *format, ...)
{
    dwc_irqflags_t flags;
    work_container_t *container;
    static char name[128];
    va_list args;

    va_start(args, format);
    DWC_VSNPRINTF(name, 128, format, args);
    va_end(args);

    DWC_SPINLOCK_IRQSAVE(wq->lock, &flags);
    wq->pending++;
    DWC_SPINUNLOCK_IRQRESTORE(wq->lock, flags);
    DWC_WAITQ_TRIGGER(wq->waitq);

    container = DWC_ALLOC_ATOMIC(sizeof(*container));
    if (!container) {
        DWC_ERROR("Cannot allocate memory for container\n");
        return;
    }

    container->name = DWC_STRDUP(name);
    if (!container->name) {
        DWC_ERROR("Cannot allocate memory for container->name\n");
        DWC_FREE(container);
        return;
    }

    container->cb = cb;
    container->data = data;
    container->wq = wq;
    DWC_DEBUG("Queueing work: %s, container=%p", container->name, container);

#ifdef DEBUG
    DWC_CIRCLEQ_INSERT_TAIL(&wq->entries, container, entry);
#endif
    work_queue(LPWORK, wq->wq, do_work, container, 0);
}

void DWC_WORKQ_SCHEDULE_DELAYED(dwc_workq_t *wq, dwc_work_callback_t cb,
                void *data, uint32_t time, char *format, ...)
{
    dwc_irqflags_t flags;
    work_container_t *container;
    static char name[128];
    va_list args;

    va_start(args, format);
    DWC_VSNPRINTF(name, 128, format, args);
    va_end(args);

    DWC_SPINLOCK_IRQSAVE(wq->lock, &flags);
    wq->pending++;
    DWC_SPINUNLOCK_IRQRESTORE(wq->lock, flags);
    DWC_WAITQ_TRIGGER(wq->waitq);

    container = DWC_ALLOC_ATOMIC(sizeof(*container));
    if (!container) {
        DWC_ERROR("Cannot allocate memory for container\n");
        return;
    }

    container->name = DWC_STRDUP(name);
    if (!container->name) {
        DWC_ERROR("Cannot allocate memory for container->name\n");
        DWC_FREE(container);
        return;
    }

    container->cb = cb;
    container->data = data;
    container->wq = wq;
    DWC_DEBUG("Queueing work: %s, container=%p", container->name, container);

#ifdef DEBUG
    DWC_CIRCLEQ_INSERT_TAIL(&wq->entries, container, entry);
#endif
    work_queue(LPWORK, wq->wq, do_work, container, msecs_to_ticks(time));
}

int DWC_WORKQ_PENDING(dwc_workq_t *wq)
{
    return wq->pending;
}

void *phys_to_virt(unsigned long address)
{
    return (void *)address;
}
