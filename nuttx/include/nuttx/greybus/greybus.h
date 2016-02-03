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

#ifndef _GREYBUS_H_
#define _GREYBUS_H_

#include <stddef.h>
#include <pthread.h>

#include <arch/atomic.h>
#include <nuttx/list.h>
#include <nuttx/util.h>
#include <nuttx/greybus/types.h>

#define GB_MTU                  2048
#define GB_MAX_PAYLOAD_SIZE     (GB_MTU - sizeof(struct gb_operation_hdr))

enum gb_event {
    GB_EVT_CONNECTED,
    GB_EVT_DISCONNECTED,
};

struct gb_operation;

typedef void (*gb_operation_callback)(struct gb_operation *operation);
typedef uint8_t (*gb_operation_handler_t)(struct gb_operation *operation);
typedef void (*gb_operation_fast_handler_t)(unsigned int cport, void *data);

#if !defined(CONFIG_GREYBUS_DEBUG)
#define GB_HANDLER(t, h) \
    { \
        .type = t, \
        .handler = h, \
    }

#define GB_FAST_HANDLER(t, h) \
    { \
        .type = t, \
        .fast_handler = h, \
    }
#else
#define GB_HANDLER(t, h) \
    { \
        .type = t, \
        .handler = h, \
        .name = #h, \
    }

#define GB_FAST_HANDLER(t, h) \
    { \
        .type = t, \
        .fast_handler = h, \
        .name = #h, \
    }
#endif

struct gb_operation_handler {
    uint8_t type;
    gb_operation_handler_t handler;
    gb_operation_fast_handler_t fast_handler;
#ifdef CONFIG_GREYBUS_DEBUG
    const char *name;
#endif
};

struct gb_transport_backend {
    int headroom;

    void (*init)(void);
    void (*exit)(void);
    int (*listen)(unsigned int cport);
    int (*stop_listening)(unsigned int cport);
    int (*send)(unsigned int cport, const void *buf, size_t len);
    void *(*alloc_buf)(size_t size);
    void (*free_buf)(void *ptr);
};

struct gb_operation {
    unsigned int cport;
    bool has_responded;
    atomic_t ref_count;
    struct timespec time;

    void *request_headroom;
    void *response_headroom;

    void *request_buffer;
    void *response_buffer;
    bool is_unipro_rx_buf;

    gb_operation_callback callback;
    sem_t sync_sem;

    void *priv_data;
    struct list_head list;

    struct gb_operation *response;

#ifdef CONFIG_GREYBUS_FEATURE_HAVE_TIMESTAMPS
    struct timespec send_ts;
    struct timespec recv_ts;
#endif
};

struct gb_driver {
    int (*init)(unsigned int cport);
    void (*exit)(unsigned int cport);
    void (*connected)(unsigned int cport);
    void (*disconnected)(unsigned int cport);

    struct gb_operation_handler *op_handlers;

    size_t stack_size;
    size_t op_handlers_count;
    const char *name;
};

struct gb_operation_hdr {
    __le16 size;
    __le16 id;
    __u8 type;
    __u8 result; /* present in response only */
    __u8 pad[2];
};

enum gb_operation_type {
    GB_TYPE_RESPONSE_FLAG       = 0x80,
};

enum gb_operation_result {
    GB_OP_SUCCESS               = 0x00,
    GB_OP_INTERRUPTED           = 0x01,
    GB_OP_TIMEOUT               = 0x02,
    GB_OP_NO_MEMORY             = 0x03,
    GB_OP_PROTOCOL_BAD          = 0x04,
    GB_OP_OVERFLOW              = 0x05,
    GB_OP_INVALID               = 0x06,
    GB_OP_RETRY                 = 0x07,
    GB_OP_NONEXISTENT           = 0x08,
    GB_OP_UNKNOWN_ERROR         = 0xfe,
    GB_OP_INTERNAL              = 0xff,
};

static inline void*
gb_operation_get_response_payload(struct gb_operation *operation)
{
    return (char*)operation->response_buffer + sizeof(struct gb_operation_hdr);
}

static inline void*
gb_operation_get_request_payload(struct gb_operation *operation)
{
    return (char*)operation->request_buffer + sizeof(struct gb_operation_hdr);
}

static inline struct gb_operation *gb_operation_get_response_op(struct gb_operation *op) {
    return op->response;
}

static inline uint8_t
gb_operation_get_response_result(struct gb_operation *operation)
{
    uint8_t ret = GB_OP_INTERNAL;

    if (operation->response) {
        struct gb_operation_hdr *hdr =
            (struct gb_operation_hdr *)(operation->response)->request_buffer;
        ret = hdr->result;
    }
    return ret;
}

static inline const char *gb_driver_name(struct gb_driver *driver)
{
    return driver->name;
}

static inline const char *gb_handler_name(struct gb_operation_handler *handler)
{
#ifdef CONFIG_GREYBUS_DEBUG
    return handler->name;
#else
    return "unknown";
#endif
}

int gb_init(struct gb_transport_backend *transport);
void gb_deinit(void);
int gb_unipro_init(void);
int _gb_register_driver(unsigned int cport, struct gb_driver *driver);
int gb_unregister_driver(unsigned int cport);

static inline int gb_register_named_driver(unsigned int cport,
                                           struct gb_driver *driver,
                                           const char *name)
{
    driver->name = name;
    return _gb_register_driver(cport, driver);
}

#define gb_register_driver(cport, driver) \
    gb_register_named_driver(cport, driver, __FILE__)
int gb_listen(unsigned int cport);
int gb_stop_listening(unsigned int cport);
int gb_notify(unsigned cport, enum gb_event event);
int gb_notify_all(enum gb_event event);

void gb_operation_destroy(struct gb_operation *operation);
void *gb_operation_alloc_response(struct gb_operation *operation, size_t size);
int gb_operation_send_response(struct gb_operation *operation, uint8_t result);
int gb_operation_send_request_sync(struct gb_operation *operation);
int gb_operation_send_request(struct gb_operation *operation,
                              gb_operation_callback callback,
                              bool need_response);
struct gb_operation *gb_operation_create(unsigned int cport, uint8_t type,
                                         uint32_t req_size);
void gb_operation_ref(struct gb_operation *operation);
void gb_operation_unref(struct gb_operation *operation);
size_t gb_operation_get_request_payload_size(struct gb_operation *operation);
uint8_t gb_operation_get_request_result(struct gb_operation *operation);
int greybus_rx_handler(unsigned int, void*, size_t);

void gb_control_register(int cport);
void gb_gpio_register(int cport);
void gb_i2c_register(int cport);
void gb_loopback_register(int cport);
void gb_pwm_register(int cport);
void gb_uart_register(int cport);
void gb_nsh_uart_register(int cport);
void gb_vendor_register(int cport);
int gb_i2c_set_dev(struct i2c_dev_s *dev);
struct  i2c_dev_s *gb_i2c_get_dev(void);

uint8_t gb_errno_to_op_result(int err);

bool gb_is_valid_cport(unsigned int cport);

#endif /* _GREYBUS_H_ */
