/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#ifndef _GREYBUS_H_
#define _GREYBUS_H_

#include <stddef.h>
#include <pthread.h>

#include <arch/atomic.h>
#include <nuttx/list.h>

struct gb_operation;

typedef void (*gb_operation_callback)(struct gb_operation *operation);
typedef uint8_t (*gb_operation_handler_t)(struct gb_operation *operation);
typedef void (*gb_operation_fast_handler_t)(unsigned int cport, void *data);

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

struct gb_operation_handler {
    uint8_t type;
    gb_operation_handler_t handler;
    gb_operation_fast_handler_t fast_handler;
};

struct gb_transport_backend {
    void (*init)(void);
    int (*listen)(unsigned int cport);
    int (*send)(unsigned int cport, const void *buf, size_t len);
};

struct gb_operation {
    unsigned int cport;
    bool has_responded;
    atomic_t ref_count;
    struct timespec time;

    void *request_buffer;
    void *response_buffer;

    gb_operation_callback callback;
    sem_t sync_sem;

    void *priv_data;
    struct list_head list;
};

struct gb_driver {
    int (*init)(unsigned int cport);
    void (*exit)(unsigned int cport);
    struct gb_operation_handler *op_handlers;

    size_t stack_size;
    size_t op_handlers_count;
};

struct gb_operation_hdr {
    uint16_t size;
    uint16_t id;
    uint8_t type;
    uint8_t result; /* present in response only */
    uint8_t pad[2];
};

enum gb_operation_result {
    GB_OP_SUCCESS       = 0x00,
    GB_OP_INTERRUPTED   = 0x01,
    GB_OP_TIMEOUT       = 0x02,
    GB_OP_NO_MEMORY     = 0x03,
    GB_OP_PROTOCOL_BAD  = 0x04,
    GB_OP_OVERFLOW      = 0x05,
    GB_OP_INVALID       = 0x06,
    GB_OP_RETRY         = 0x07,
    GB_OP_NONEXISTENT   = 0x08,
    GB_OP_UNKNOWN_ERROR = 0xfe,
    GB_OP_MALFUNCTION   = 0xff,
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

int gb_init(struct gb_transport_backend *transport);
int gb_unipro_init(void);
int gb_register_driver(unsigned int cport, struct gb_driver *driver);

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
int greybus_rx_handler(unsigned int, void*, size_t);

void gb_gpio_register(int cport);

#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#endif /* _GREYBUS_H_ */
