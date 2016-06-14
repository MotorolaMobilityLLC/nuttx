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
 */

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/tape.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/greybus/mods-ctrl.h>
#include <nuttx/wdog.h>
#include <loopback-gb.h>

#include <arch/atomic.h>
#include <arch/byteorder.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "rtr.h"

#define DEFAULT_STACK_SIZE      CONFIG_PTHREAD_STACK_DEFAULT
#define TIMEOUT_IN_MS           1000
#define GB_INVALID_TYPE         0

#define ONE_SEC_IN_MSEC         1000
#define ONE_MSEC_IN_NSEC        1000000

#define TIMEOUT_WD_DELAY    (TIMEOUT_IN_MS * CLOCKS_PER_SEC) / ONE_SEC_IN_MSEC

struct gb_cport_driver {
    struct gb_driver *driver;
    struct list_head tx_fifo;
    struct list_head rx_fifo;
    sem_t rx_fifo_lock;
    pthread_t thread;
    volatile bool exit_worker;
    struct wdog_s timeout_wd;
    struct gb_operation timedout_operation;
    uint16_t cport;
};

struct gb_tape_record_header {
    uint16_t size;
    uint16_t cport;
};

static atomic_t request_id;

static void **cport_tbl;

static struct gb_cport_driver *_g_cport(unsigned int cport)
{
    return rtr_get_value(cport_tbl, (uint16_t) (cport & 0xffff));
}
#define g_cport(c) (*(_g_cport(c)))

bool gb_is_valid_cport(unsigned int cport)
{
    static unsigned int _cport_count = -1;

    if (_cport_count < 0)
       _cport_count =  unipro_cport_count();

    return (mods_cport_valid(cport) || (cport < _cport_count));
}

static struct gb_transport_backend *transport_backend;
static struct gb_tape_mechanism *gb_tape;
static int gb_tape_fd = -EBADFD;
static struct gb_operation_hdr *timedout_hdr;
static struct gb_operation_hdr *oom_hdr;

static void g_cport_add_entry(uint16_t cport, struct gb_driver *driver)
{
    struct gb_cport_driver *entry = zalloc(sizeof(struct gb_cport_driver));

    if (entry) {
        sem_init(&entry->rx_fifo_lock, 0, 0);
        list_init(&entry->rx_fifo);
        list_init(&entry->tx_fifo);
        wd_static(&entry->timeout_wd);
        entry->timedout_operation.request_buffer = timedout_hdr;
        list_init(&entry->timedout_operation.list);
        entry->driver = driver;
        entry->cport = cport;

        rtr_add_value(cport_tbl, cport, (void *)entry);
    }
}

static inline void g_cport_remove_entry(uint16_t cport)
{
	struct gb_cport_driver *entry = _g_cport(cport);

	if (entry) {
		rtr_remove_value(cport_tbl, cport);
	    free(entry);
	}
}

static void gb_operation_timeout(int argc, uint32_t cport, ...);
static struct gb_operation *_gb_operation_create(unsigned int cport);

uint8_t gb_errno_to_op_result(int err)
{
    switch (err) {
    case 0:
        return GB_OP_SUCCESS;

    case ENOMEM:
    case -ENOMEM:
        return GB_OP_NO_MEMORY;

    case EINTR:
    case -EINTR:
        return GB_OP_INTERRUPTED;

    case ETIMEDOUT:
    case -ETIMEDOUT:
        return GB_OP_TIMEOUT;

    case EPROTO:
    case -EPROTO:
    case ENOSYS:
    case -ENOSYS:
        return GB_OP_PROTOCOL_BAD;

    case EINVAL:
    case -EINVAL:
        return GB_OP_INVALID;

    case EOVERFLOW:
    case -EOVERFLOW:
        return GB_OP_OVERFLOW;

    case ENODEV:
    case -ENODEV:
    case ENXIO:
    case -ENXIO:
        return GB_OP_NONEXISTENT;

    case EBUSY:
    case -EBUSY:
        return GB_OP_RETRY;

    default:
        return GB_OP_UNKNOWN_ERROR;
    }
}

#ifdef CONFIG_GREYBUS_FEATURE_HAVE_TIMESTAMPS
static void op_mark_send_time(struct gb_operation *operation)
{
    clock_gettime(CLOCK_REALTIME, &operation->send_ts);
}

static void op_mark_recv_time(struct gb_operation *operation)
{
    clock_gettime(CLOCK_REALTIME, &operation->recv_ts);
}
#else
static void op_mark_send_time(struct gb_operation *operation) { }
static void op_mark_recv_time(struct gb_operation *operation) { }
#endif

static int gb_compare_handlers(const void *data1, const void *data2)
{
    const struct gb_operation_handler *handler1 = data1;
    const struct gb_operation_handler *handler2 = data2;
    return (int)handler1->type - (int)handler2->type;
}

static struct gb_operation_handler *find_operation_handler(uint8_t type,
                                                           unsigned int cport)
{
    struct gb_driver *driver = g_cport(cport).driver;
    int l,r;

    if (type == GB_INVALID_TYPE || !driver->op_handlers) {
        return NULL;
    }

    /*
     * This function is performance sensitive, so let's use an inline binary
     * search algorithm. The libc version takes pointer to the comparison
     * function as argument which is then called via full-blown function
     * calls. The below version doesn't require calling any other function.
     */
    l = 0;
    r = driver->op_handlers_count - 1;
    while (l <= r) {
        int m = (l + r) / 2;
        if (driver->op_handlers[m].type < type)
            l = m + 1;
        else if (driver->op_handlers[m].type > type)
            r = m - 1;
        else
            return &driver->op_handlers[m];
    }

    return NULL;
}

static void gb_process_request(struct gb_operation_hdr *hdr,
                               struct gb_operation *operation)
{
    struct gb_operation_handler *op_handler;
    uint8_t result;

    op_handler = find_operation_handler(hdr->type, operation->cport);
    if (!op_handler) {
        gb_error("Cport %u: Invalid operation type %u\n",
                 operation->cport, hdr->type);
        gb_operation_send_response(operation, GB_OP_INVALID);
        return;
    }

    result = op_handler->handler(operation);
    gb_debug("%s: %u\n", gb_handler_name(op_handler), result);

    if (hdr->id)
        gb_operation_send_response(operation, result);
    op_mark_send_time(operation);
}

static bool gb_operation_has_timedout(struct gb_operation *operation)
{
    struct timespec current_time;
    struct timespec timeout_time;

    timeout_time.tv_sec = operation->time.tv_sec +
                          TIMEOUT_IN_MS / ONE_SEC_IN_MSEC;
    timeout_time.tv_nsec = operation->time.tv_nsec +
                          (TIMEOUT_IN_MS % ONE_SEC_IN_MSEC) * ONE_MSEC_IN_NSEC;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    if (current_time.tv_sec > timeout_time.tv_sec)
        return true;

    if (current_time.tv_sec < timeout_time.tv_sec)
        return false;

    return current_time.tv_nsec > timeout_time.tv_nsec;
}

/**
 * Update watchdog state
 *
 * Cancel cport watchdog if there is no outgoing message waiting for a response,
 * or update the watchdog if there is still outgoing messages.
 *
 * TODO use fine-grain timeout delay when doing the update
 *
 * @note This function should be called from an atomic context
 */
static void gb_watchdog_update(unsigned int cport)
{
    irqstate_t flags;

    flags = irqsave();

    if (list_is_empty(&g_cport(cport).tx_fifo)) {
        wd_cancel(&g_cport(cport).timeout_wd);
    } else {
        wd_start(&g_cport(cport).timeout_wd, TIMEOUT_WD_DELAY,
                 gb_operation_timeout, 1, cport);
    }

    irqrestore(flags);
}

static void gb_clean_timedout_operation(unsigned int cport)
{
    irqstate_t flags;
    struct list_head *iter, *iter_next;
    struct gb_operation *op;

    list_foreach_safe(&g_cport(cport).tx_fifo, iter, iter_next) {
        op = list_entry(iter, struct gb_operation, list);

        if (!gb_operation_has_timedout(op)) {
            continue;
        }

        flags = irqsave();
        list_del(iter);
        irqrestore(flags);

        if (op->callback) {
            op->callback(op);
        }
        gb_operation_unref(op);
    }

    gb_watchdog_update(cport);
}

static void gb_process_response(struct gb_operation_hdr *hdr,
                                struct gb_operation *operation)
{
    irqstate_t flags;
    struct list_head *iter, *iter_next;
    struct gb_operation *op;
    struct gb_operation_hdr *op_hdr;

    list_foreach_safe(&g_cport(operation->cport).tx_fifo, iter, iter_next) {
        op = list_entry(iter, struct gb_operation, list);
        op_hdr = op->request_buffer;

        if (hdr->id != op_hdr->id)
            continue;

        flags = irqsave();
        list_del(iter);
        gb_watchdog_update(operation->cport);
        irqrestore(flags);

        /* attach this response with the original request */
        gb_operation_ref(operation);
        op->response = operation;
        op_mark_recv_time(op);
        if (op->callback)
            op->callback(op);
        gb_operation_unref(op);
        return;
    }

    gb_error("CPort %u: cannot find matching request for response %hu. Dropping message.\n",
             operation->cport, le16_to_cpu(hdr->id));
}

static void *gb_pending_message_worker(void *data)
{
    const int cportid = (int) data;
    irqstate_t flags;
    struct gb_operation *operation;
    struct list_head *head;
    struct gb_operation_hdr *hdr;
    int retval;

    while (1) {
        retval = sem_wait(&g_cport(cportid).rx_fifo_lock);
        if (retval < 0)
            continue;

        if (g_cport(cportid).exit_worker &&
            list_is_empty(&g_cport(cportid).rx_fifo)) {
            break;
        }

        flags = irqsave();
        head = g_cport(cportid).rx_fifo.next;
        list_del(g_cport(cportid).rx_fifo.next);
        irqrestore(flags);

        operation = list_entry(head, struct gb_operation, list);
        hdr = operation->request_buffer;

        if (hdr == timedout_hdr) {
            gb_clean_timedout_operation(cportid);
            continue;
        }

        if (hdr->type & GB_TYPE_RESPONSE_FLAG)
            gb_process_response(hdr, operation);
        else
            gb_process_request(hdr, operation);
        gb_operation_destroy(operation);
    }

    return NULL;
}

#if defined(CONFIG_UNIPRO_ZERO_COPY)
static struct gb_operation *gb_rx_create_operation(unsigned cport, void *data,
                                                   size_t size)
{
    struct gb_operation *op;

    if (transport_backend->headroom) {
        gb_error("headroom allocation cannot be used with zero copy\n");
        return NULL;
    }

    op = _gb_operation_create(cport);
    if (!op)
        return NULL;

    op->is_unipro_rx_buf = true;
    op->request_headroom = data;
    op->request_buffer = data;

    return op;
}
#else
static struct gb_operation *gb_rx_create_operation(unsigned cport, void *data,
                                                   size_t size)
{
    struct gb_operation *op;

    op = gb_operation_create(cport, 0, size - sizeof(struct gb_operation_hdr));
    if (!op)
        return NULL;

    memcpy(op->request_buffer, data, size);

    return op;
}
#endif

int greybus_rx_handler(unsigned int cport, void *data, size_t size)
{
    irqstate_t flags;
    struct gb_operation *op;
    struct gb_operation_hdr *hdr = data;
    struct gb_operation_handler *op_handler;
    size_t hdr_size;

    gb_loopback_log_entry(cport);
    if ((!gb_is_valid_cport(cport) || (!_g_cport(cport)) || !data)) {
        gb_error("Invalid cport number: %u\n", cport);
        return -EINVAL;
    }

    if (!g_cport(cport).driver || !g_cport(cport).driver->op_handlers) {
        gb_error("Cport %u does not have a valid driver registered\n", cport);
        return 0;
    }

    if (sizeof(*hdr) > size) {
        gb_error("Dropping garbage request\n");
        return -EINVAL; /* Dropping garbage request */
    }

    hdr_size = le16_to_cpu(hdr->size);

    if (hdr_size > size || sizeof(*hdr) > hdr_size) {
        gb_error("Dropping garbage request\n");
        return -EINVAL; /* Dropping garbage request */
    }

    gb_dump(data, size);

    if (gb_tape && gb_tape_fd >= 0) {
        struct gb_tape_record_header record_hdr = {
            .size = size,
            .cport = cport,
        };

        gb_tape->write(gb_tape_fd, &record_hdr, sizeof(record_hdr));
        gb_tape->write(gb_tape_fd, data, size);
    }

    op_handler = find_operation_handler(hdr->type, cport);
    if (op_handler && op_handler->fast_handler) {
        gb_debug("%s\n", gb_handler_name(op_handler));
        op_handler->fast_handler(cport, data);
        return 0;
    }

    op = gb_rx_create_operation(cport, data, hdr_size);
    if (!op)
        return -ENOMEM;

    op_mark_recv_time(op);

    flags = irqsave();
    list_add(&g_cport(cport).rx_fifo, &op->list);
    sem_post(&g_cport(cport).rx_fifo_lock);
    irqrestore(flags);

    return 0;
}

static void gb_flush_tx_fifo(unsigned int cport)
{
    struct list_head *iter, *iter_next;

    list_foreach_safe(&_g_cport(cport)->tx_fifo, iter, iter_next) {
        struct gb_operation *op = list_entry(iter, struct gb_operation, list);

        list_del(iter);
        gb_operation_unref(op);
    }
}

int gb_unregister_driver(unsigned int cport)
{
    if (!gb_is_valid_cport(cport) || !_g_cport(cport) ||
		!_g_cport(cport)->driver || !transport_backend)
        return -EINVAL;

    if (transport_backend->stop_listening)
        transport_backend->stop_listening(cport);

    wd_cancel(&g_cport(cport).timeout_wd);

    g_cport(cport).exit_worker = true;
    sem_post(&g_cport(cport).rx_fifo_lock);
    pthread_join(g_cport(cport).thread, NULL);

    gb_flush_tx_fifo(cport);

    if (g_cport(cport).driver->exit)
        g_cport(cport).driver->exit(cport);
    _g_cport(cport)->driver = NULL;

    return 0;
}

int _gb_register_driver(unsigned int cport, struct gb_driver *driver)
{
    pthread_attr_t thread_attr;
    pthread_attr_t *thread_attr_ptr = &thread_attr;
    struct gb_cport_driver *cport_entry;
    int retval;

    gb_debug("Registering Greybus driver on CP%u\n", cport);

    if (!gb_is_valid_cport(cport)) {
        gb_error("Invalid cport number %u\n", cport);
        return -EINVAL;
    }

    if (!driver) {
        gb_error("No driver to register\n");
        return -EINVAL;
    }

    /* check if we have already registered this cport */
    cport_entry = _g_cport(cport);
    if (cport_entry != NULL) {
        if (cport_entry->driver != NULL) {
            gb_error("driver is already registered for CP%u\n", cport);
            return -EEXIST;
        }
    }

    if (!driver->op_handlers && driver->op_handlers_count > 0) {
        gb_error("Invalid driver\n");
        return -EINVAL;
    }

    if (driver->init) {
        retval = driver->init(cport);
        if (retval) {
            gb_error("Can not init %s\n", gb_driver_name(driver));
            return retval;
        }
    }

    if (driver->op_handlers) {
        qsort(driver->op_handlers, driver->op_handlers_count,
              sizeof(*driver->op_handlers), gb_compare_handlers);
    }

    g_cport(cport).exit_worker = false;

    if (!driver->stack_size)
        driver->stack_size = DEFAULT_STACK_SIZE;

    retval = pthread_attr_init(&thread_attr);
    if (retval)
        goto pthread_attr_init_error;

    retval = pthread_attr_setstacksize(&thread_attr, driver->stack_size);
    if (retval)
        goto pthread_attr_setstacksize_error;

    gb_debug("add cport %d\n", cport);
    g_cport_add_entry(cport, NULL);

    retval = pthread_create(&g_cport(cport).thread, &thread_attr,
                            gb_pending_message_worker, (unsigned*) cport);
    if (retval)
        goto pthread_create_error;

    pthread_attr_destroy(&thread_attr);
    thread_attr_ptr = NULL;

    _g_cport(cport)->driver = driver;

    return 0;

pthread_create_error:
pthread_attr_setstacksize_error:
    if (thread_attr_ptr != NULL)
        pthread_attr_destroy(&thread_attr);
pthread_attr_init_error:
    gb_error("Can not create thread for %s\n: ", gb_driver_name(driver));
    if (driver->exit)
        driver->exit(cport);
    return retval;
}

int gb_listen(unsigned int cport)
{
    DEBUGASSERT(transport_backend);
    DEBUGASSERT(transport_backend->listen);

    if (!gb_is_valid_cport(cport)) {
        gb_error("Invalid cport number %u\n", cport);
        return -EINVAL;
    }

    if (!g_cport(cport).driver) {
        gb_error("No driver registered! Can not connect CP%u.\n", cport);
        return -EINVAL;
    }

    return transport_backend->listen(cport);
}

int gb_stop_listening(unsigned int cport)
{
    DEBUGASSERT(transport_backend);
    DEBUGASSERT(transport_backend->stop_listening);

    if (!gb_is_valid_cport(cport)) {
        gb_error("Invalid cport number %u\n", cport);
        return -EINVAL;
    }

    if (!g_cport(cport).driver) {
        gb_error("No driver registered! Can not disconnect CP%u.\n",
                 cport);
        return -EINVAL;
    }

    return transport_backend->stop_listening(cport);
}

static void gb_operation_timeout(int argc, uint32_t cport, ...)
{
    irqstate_t flags;

    flags = irqsave();

    /* timedout operation could potentially already been queued */
    if (!list_is_empty(&g_cport(cport).timedout_operation.list)) {
        irqrestore(flags);
        return;
    }

    list_add(&g_cport(cport).rx_fifo, &g_cport(cport).timedout_operation.list);
    sem_post(&g_cport(cport).rx_fifo_lock);
    irqrestore(flags);
}

int gb_operation_send_request(struct gb_operation *operation,
                              gb_operation_callback callback,
                              bool need_response)
{
    struct gb_operation_hdr *hdr = operation->request_buffer;
    int retval = 0;
    irqstate_t flags;

    DEBUGASSERT(operation);
    DEBUGASSERT(transport_backend);
    DEBUGASSERT(transport_backend->send);

    if (g_cport(operation->cport).exit_worker)
        return -ENETDOWN;

    hdr->id = 0;

    flags = irqsave();

    if (need_response) {
        hdr->id = cpu_to_le16(atomic_inc(&request_id));
        if (hdr->id == 0) /* ID 0 is for request with no response */
            hdr->id = cpu_to_le16(atomic_inc(&request_id));
        clock_gettime(CLOCK_MONOTONIC, &operation->time);
        operation->callback = callback;
        gb_operation_ref(operation);
        list_add(&g_cport(operation->cport).tx_fifo, &operation->list);
        if (!WDOG_ISACTIVE(&g_cport(operation->cport).timeout_wd)) {
            wd_start(&g_cport(operation->cport).timeout_wd, TIMEOUT_WD_DELAY,
                     gb_operation_timeout, 1, operation->cport);
        }
    }

    gb_dump(operation->request_buffer, hdr->size);
    retval = transport_backend->send(operation->cport,
                                     operation->request_buffer,
                                     le16_to_cpu(hdr->size));
    op_mark_send_time(operation);
    if (need_response && retval) {
        list_del(&operation->list);
        gb_watchdog_update(operation->cport);
        gb_operation_unref(operation);
    }

    irqrestore(flags);

    return retval;
}

static void gb_operation_callback_sync(struct gb_operation *operation)
{
    sem_post(&operation->sync_sem);
}

int gb_operation_send_request_sync(struct gb_operation *operation)
{
    int retval;

    sem_init(&operation->sync_sem, 0, 0);

    retval =
        gb_operation_send_request(operation, gb_operation_callback_sync, true);
    if (retval)
        return retval;

    do {
        retval = sem_wait(&operation->sync_sem);
    } while (retval < 0 && errno == EINTR);

    return retval;
}

static int gb_operation_send_oom_response(struct gb_operation *operation)
{
    int retval;
    irqstate_t flags;
    struct gb_operation_hdr *req_hdr = operation->request_buffer;

    if (g_cport(operation->cport).exit_worker)
        return -ENETDOWN;

    flags = irqsave();

    oom_hdr->id = req_hdr->id;
    oom_hdr->type = GB_TYPE_RESPONSE_FLAG | req_hdr->type;

    retval = transport_backend->send(operation->cport, oom_hdr,
                                     sizeof(*oom_hdr));

    irqrestore(flags);

    return retval;
}

int gb_operation_send_response(struct gb_operation *operation, uint8_t result)
{
    struct gb_operation_hdr *resp_hdr;
    int retval;
    bool has_allocated_response = false;

    DEBUGASSERT(operation);
    DEBUGASSERT(transport_backend);
    DEBUGASSERT(transport_backend->send);

    if (g_cport(operation->cport).exit_worker)
        return -ENETDOWN;

    if (operation->has_responded)
        return -EINVAL;

    if (!operation->response_buffer) {
        gb_operation_alloc_response(operation, 0);
        if (!operation->response_buffer)
            return gb_operation_send_oom_response(operation);

        has_allocated_response = true;
    }

    resp_hdr = operation->response_buffer;
    resp_hdr->result = result;

    gb_dump(operation->response_buffer, resp_hdr->size);
    gb_loopback_log_exit(operation->cport, operation, resp_hdr->size);
    retval = transport_backend->send(operation->cport,
                                     operation->response_buffer,
                                     le16_to_cpu(resp_hdr->size));
    if (retval) {
        gb_error("Greybus backend failed to send: error %d\n", retval);
        if (has_allocated_response) {
            gb_debug("Free the response buffer\n");
            transport_backend->free_buf(operation->response_headroom);
            operation->response_headroom = NULL;
            operation->response_buffer = NULL;
        }
        return retval;
    }

    operation->has_responded = true;
    return retval;
}

void *gb_operation_alloc_response(struct gb_operation *operation, size_t size)
{
    struct gb_operation_hdr *req_hdr;
    struct gb_operation_hdr *resp_hdr;

    DEBUGASSERT(operation);

    operation->response_headroom =
        transport_backend->alloc_buf(
                size + sizeof(*resp_hdr) + transport_backend->headroom);
    if (!operation->response_headroom) {
        gb_error("Can not allocate a response_buffer\n");
        return NULL;
    }

    operation->response_buffer =
        (char *)operation->response_headroom + transport_backend->headroom;

    req_hdr = operation->request_buffer;
    resp_hdr = operation->response_buffer;

    resp_hdr->size = cpu_to_le16(size + sizeof(*resp_hdr));
    resp_hdr->id = req_hdr->id;
    resp_hdr->type = GB_TYPE_RESPONSE_FLAG | req_hdr->type;
    return gb_operation_get_response_payload(operation);
}

void gb_operation_destroy(struct gb_operation *operation)
{
    DEBUGASSERT(operation);
    gb_operation_unref(operation);
}

void gb_operation_ref(struct gb_operation *operation)
{
    DEBUGASSERT(operation);
    DEBUGASSERT(atomic_get(&operation->ref_count) > 0);
    atomic_inc(&operation->ref_count);
}

void gb_operation_unref(struct gb_operation *operation)
{
    DEBUGASSERT(operation);
    DEBUGASSERT(atomic_get(&operation->ref_count) > 0);

    uint32_t ref_count = atomic_dec(&operation->ref_count);
    if (ref_count != 0) {
        return;
    }

    if (operation->is_unipro_rx_buf) {
        unipro_rxbuf_free(operation->cport, operation->request_headroom);
    } else {
        transport_backend->free_buf(operation->request_headroom);
    }

    transport_backend->free_buf(operation->response_headroom);
    if (operation->response) {
        gb_operation_unref(operation->response);
    }
    free(operation);
}

static struct gb_operation *_gb_operation_create(unsigned int cport)
{
    struct gb_operation *operation;

    if (!gb_is_valid_cport(cport))
        return NULL;

    operation = malloc(sizeof(*operation));
    if (!operation)
        return NULL;

    memset(operation, 0, sizeof(*operation));
    operation->cport = cport;

    list_init(&operation->list);
    atomic_init(&operation->ref_count, 1);

    return operation;
}

struct gb_operation *gb_operation_create(unsigned int cport, uint8_t type,
                                         uint32_t req_size)
{
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;

    if (!gb_is_valid_cport(cport))
        return NULL;

    operation = _gb_operation_create(cport);
    if (!operation) {
        return NULL;
    }

    operation->request_headroom =
        transport_backend->alloc_buf(
                req_size + sizeof(*hdr) + transport_backend->headroom);
    if (!operation->request_headroom)
        goto malloc_error;

    operation->request_buffer =
        (char *)operation->request_headroom + transport_backend->headroom;
    hdr = operation->request_buffer;
    hdr->size = cpu_to_le16(req_size + sizeof(*hdr));
    hdr->type = type;

    return operation;
malloc_error:
    free(operation);
    return NULL;
}

size_t gb_operation_get_request_payload_size(struct gb_operation *operation)
{
    struct gb_operation_hdr *hdr;

    if (!operation || !operation->request_buffer) {
        return 0;
    }

    hdr = operation->request_buffer;
    if (le16_to_cpu(hdr->size) < sizeof(*hdr)) {
        return 0;
    }

    return le16_to_cpu(hdr->size) - sizeof(*hdr);
}

uint8_t gb_operation_get_request_result(struct gb_operation *operation)
{
    struct gb_operation_hdr *hdr;

    if (!operation) {
        return GB_OP_INTERNAL;
    }

    if (!operation->response) {
        return GB_OP_TIMEOUT;
    }

    hdr = operation->response->request_buffer;
    if (!hdr || hdr->size < sizeof(*hdr)) {
        return GB_OP_INTERNAL;
    }

    return hdr->result;
}

int gb_init(struct gb_transport_backend *transport)
{
    if (!transport)
        return -EINVAL;

    if (transport->headroom & 0x0003) {
        dbg("Headroom (%d) not aligned\n", transport->headroom);
        return -EINVAL;
    }

    timedout_hdr = zalloc(sizeof(struct gb_operation_hdr) + transport->headroom);
    if (!timedout_hdr)
        return -ENOMEM;

    oom_hdr = zalloc(sizeof(struct gb_operation_hdr) + transport->headroom);
    if (!oom_hdr) {
        free(timedout_hdr);
        return -ENOMEM;
    }

    timedout_hdr =
        (struct gb_operation_hdr *)((char *)timedout_hdr + transport->headroom);
    timedout_hdr->size = sizeof(struct gb_operation_hdr);
    timedout_hdr->result = GB_OP_TIMEOUT;
    timedout_hdr->type = GB_TYPE_RESPONSE_FLAG;

    oom_hdr =
        (struct gb_operation_hdr *)((char *)oom_hdr + transport->headroom);
    oom_hdr->size = sizeof(struct gb_operation_hdr),
    oom_hdr->result = GB_OP_NO_MEMORY;
    oom_hdr->type = GB_TYPE_RESPONSE_FLAG;

    cport_tbl = rtr_alloc_table();

    atomic_init(&request_id, (uint32_t) 0);

    transport_backend = transport;
    transport_backend->init();

    return 0;
}

void gb_deinit(void)
{
    int i;
    struct gb_cport_driver *drv;
    void *v;

    if (!transport_backend)
        return; /* gb not initialized */

    for (v = rtr_get_first_value(cport_tbl);
         v != NULL;
         v = rtr_get_next_value(cport_tbl, i)) {

        drv = (struct gb_cport_driver *) v;
        i = drv->cport;

        gb_unregister_driver(i);

        wd_delete(&drv->timeout_wd);
        sem_destroy(&drv->rx_fifo_lock);

        g_cport_remove_entry(i);
    }

    rtr_free_table(cport_tbl);
    cport_tbl = NULL;

    if (transport_backend->exit)
        transport_backend->exit();
    transport_backend = NULL;
}

int gb_tape_register_mechanism(struct gb_tape_mechanism *mechanism)
{
    if (!mechanism || !mechanism->open || !mechanism->close ||
        !mechanism->read || !mechanism->write)
        return -EINVAL;

    if (gb_tape)
        return -EBUSY;

    gb_tape = mechanism;

    return 0;
}

int gb_tape_communication(const char *pathname)
{
    if (!gb_tape)
        return -EINVAL;

    if (gb_tape_fd >= 0)
        return -EBUSY;

    gb_tape_fd = gb_tape->open(pathname, GB_TAPE_WRONLY);
    if (gb_tape_fd < 0)
        return gb_tape_fd;

    return 0;
}

int gb_tape_stop(void)
{
    if (!gb_tape || gb_tape_fd < 0)
        return -EINVAL;

    gb_tape->close(gb_tape_fd);
    gb_tape_fd = -EBADFD;

    return 0;
}

int gb_tape_replay(const char *pathname)
{
    struct gb_tape_record_header hdr;
    char *buffer;
    ssize_t nread;
    int retval = 0;
    int fd;

    if (!pathname || !gb_tape)
        return -EINVAL;

    lowsyslog("greybus: replaying '%s'...\n", pathname);

    fd = gb_tape->open(pathname, GB_TAPE_RDONLY);
    if (fd < 0)
        return fd;

    buffer = malloc(CPORT_BUF_SIZE);
    if (!buffer) {
        retval = -ENOMEM;
        goto error_buffer_alloc;
    }

    while (1) {
        nread = gb_tape->read(fd, &hdr, sizeof(hdr));
        if (!nread)
            break;

        if (nread != sizeof(hdr)) {
            gb_error("gb-tape: invalid byte count read, aborting...\n");
            retval = -EIO;
            break;
        }

        nread = gb_tape->read(fd, buffer, hdr.size);
        if (hdr.size != nread) {
            gb_error("gb-tape: invalid byte count read, aborting...\n");
            retval = -EIO;
            break;
        }

        greybus_rx_handler(hdr.cport, buffer, nread);
    }

    free(buffer);

error_buffer_alloc:
    gb_tape->close(fd);

    return retval;
}

int gb_notify(unsigned cport, enum gb_event event)
{
    if (!gb_is_valid_cport(cport))
        return -EINVAL;

    if (!_g_cport(cport) || !_g_cport(cport)->driver)
        return -ENOTCONN;

    switch (event) {
    case GB_EVT_CONNECTED:
        if (g_cport(cport).driver->connected)
            g_cport(cport).driver->connected(cport);
        break;

    case GB_EVT_DISCONNECTED:
        if (g_cport(cport).driver->disconnected)
            g_cport(cport).driver->disconnected(cport);
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

int gb_notify_all(enum gb_event event)
{
    struct gb_cport_driver *drv;
    void *v;
    int ret;

    if (!cport_tbl)
        return -ENODEV;

    for (v = rtr_get_first_value(cport_tbl);
         v != NULL;
         v = rtr_get_next_value(cport_tbl, drv->cport)) {

        drv = (struct gb_cport_driver *) v;
        ret = gb_notify(drv->cport, event);
        if (ret)
            return ret;
    }

    return 0;
}
