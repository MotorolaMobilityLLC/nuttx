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

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/util.h>
#include <nuttx/arch.h>
#include <nuttx/bufram.h>

#include <arch/chip/chip.h>

#define MM_BUCKET_MAX           31
#define MM_CANARY               0xfab0fab0

#ifdef CONFIG_MM_BUFRAM_DEBUG
#define mm_warn(message...) lowsyslog(message)
#else
#define mm_warn(message...)
#endif

static struct list_head mm_bucket[MM_BUCKET_MAX + 1];

#ifdef CONFIG_MM_BUFRAM_DEBUG
static size_t g_bufram_allocs;
static size_t g_bufram_frees;
#endif

struct mm_buffer {
    uint32_t canary;
    uint32_t bucket;
    struct list_head list;
} __attribute__((packed)); // MUST be a multiple of 8 bytes

size_t bufram_size_to_page_count(size_t size)
{
    int page_count = size / BUFRAM_PAGE_SIZE;
    return size % BUFRAM_PAGE_SIZE ? page_count + 1 : page_count;
}

static int size_to_order(size_t size)
{
    int order = -1;
    size_t bit_set_count = 0;

    for (; size; size >>= 1) {
        order++;

        if (size & 1)
            bit_set_count++;
    }

    return bit_set_count == 1 ? order : order + 1;
}

static size_t order_to_size(int order)
{
    return 1 << order;
}

void bufram_register_region(uintptr_t base, unsigned order)
{
    struct mm_buffer *buffer;

    buffer = (struct mm_buffer*) base;
    buffer->bucket = order;
#if defined(CONFIG_MM_BUFRAM_CANARY)
    buffer->canary = MM_CANARY;
#endif
    list_init(&buffer->list);

    list_add(&mm_bucket[buffer->bucket], &buffer->list);
}

void bufram_init(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mm_bucket); i++)
        list_init(&mm_bucket[i]);
}

static inline void *get_buffer_payload(struct mm_buffer *buffer)
{
    return buffer + 1; // payload immediately follows the control header
}

static inline struct mm_buffer *get_buffer_control_data(void *payload)
{
    return (struct mm_buffer*) payload - 1;
}

static struct mm_buffer *find_buffer_in_bucket(int order)
{
    if (order > MM_BUCKET_MAX)
        return NULL;

    if (list_is_empty(&mm_bucket[order]))
        return NULL;

    return list_entry(mm_bucket[order].next, struct mm_buffer, list);
}

static int fill_bucket(int order)
{
    struct mm_buffer *buffer1;
    struct mm_buffer *buffer2;
    int retval;

    if (order > MM_BUCKET_MAX)
        return -ENOMEM;

    buffer1 = find_buffer_in_bucket(order + 1);
    if (!buffer1) {
        retval = fill_bucket(order + 1);
        if (retval)
            return retval;

        buffer1 = find_buffer_in_bucket(order + 1);
        DEBUGASSERT(buffer1);
    }

    list_del(&buffer1->list);

    buffer2 = (struct mm_buffer*) ((char*) buffer1 + order_to_size(order));
    list_init(&buffer2->list);

    buffer1->bucket = order;
    buffer2->bucket = order;

#if defined(CONFIG_MM_BUFRAM_CANARY)
    buffer2->canary = buffer1->canary = MM_CANARY;
#endif

    list_add(&mm_bucket[order], &buffer1->list);
    list_add(&mm_bucket[order], &buffer2->list);

    return 0;
}

static void defragment(struct mm_buffer *buffer)
{
    struct list_head *iter;
    struct mm_buffer *buffer_low;
    struct mm_buffer *buffer_high;
    struct mm_buffer *buffer2;
    struct mm_buffer *buffer3;

    if (list_is_empty(&mm_bucket[buffer->bucket])) {
        mm_warn("mm: trying to defragment empty bucket");
        return;
    }

    if (((unsigned long) buffer) & (1 << buffer->bucket)) {
        buffer_low = buffer2 = (struct mm_buffer*)
            ((char*) buffer - order_to_size(buffer->bucket));
        buffer_high = buffer;
    } else {
        buffer_low = buffer;
        buffer_high = buffer2 = (struct mm_buffer*)
            ((char*) buffer + order_to_size(buffer->bucket));
    }

    list_foreach(&mm_bucket[buffer->bucket], iter) {
        buffer3 = list_entry(iter, struct mm_buffer, list);
        if (buffer3 != buffer2)
            continue;

        list_del(&buffer_low->list);
        list_del(&buffer_high->list);

        buffer_low->bucket++;
        list_add(&mm_bucket[buffer_low->bucket], &buffer_low->list);

        defragment(buffer_low);

        return;
    }
}

static struct mm_buffer *get_buffer(int order)
{
    struct mm_buffer *buffer;
    int retval;

    buffer = find_buffer_in_bucket(order);
    if (buffer)
        return buffer;

    retval = fill_bucket(order);
    if (!retval) {
        buffer = find_buffer_in_bucket(order);
        DEBUGASSERT(buffer);
        return buffer;
    }

    return NULL;
}

void *bufram_alloc(size_t size)
{
    int order;
    struct mm_buffer *buffer;
    irqstate_t flags;

    if (!size)
        return NULL;

    size += sizeof(*buffer);
    order = size_to_order(size);
    if (order < 0)
        return NULL;

    if (order > MM_BUCKET_MAX)
        return NULL;

    flags = irqsave();

    buffer = get_buffer(order);
    if (!buffer)
        goto error;

#ifdef CONFIG_MM_BUFRAM_DEBUG
    g_bufram_allocs++;
#endif

    list_del(&buffer->list);
    irqrestore(flags);

    return get_buffer_payload(buffer);

error:
    irqrestore(flags);
    return NULL;
}

void bufram_free(void *ptr)
{
    struct mm_buffer *buffer;
    irqstate_t flags;

    if (!ptr)
        return;

    buffer = get_buffer_control_data(ptr);
    if (buffer->list.prev != buffer->list.next) {
        mm_warn("mm: trying to free invalid pointer: %p\n", ptr);
        return;
    }

#if defined(CONFIG_MM_BUFRAM_CANARY)
    if (buffer->canary != MM_CANARY) {
        lowsyslog("mm: memory corruption detected. canary: %x != %x\n",
                  buffer->canary, MM_CANARY);
    }
#endif

    flags = irqsave();

#ifdef CONFIG_MM_BUFRAM_DEBUG
    g_bufram_frees++;
#endif

    list_add(&mm_bucket[buffer->bucket], &buffer->list);
    defragment(buffer);

    irqrestore(flags);
}

void *bufram_page_alloc(size_t page_count)
{
    char *buffer;
    size_t size = page_count * BUFRAM_PAGE_SIZE - sizeof(struct mm_buffer);

    buffer = bufram_alloc(size);
    if (!buffer)
        return NULL;

    return get_buffer_control_data(buffer);
}

void bufram_page_free(void *ptr, size_t page_count)
{
    struct mm_buffer *buffer = ptr;
    uintptr_t ptraddr = (uintptr_t) ptr;
    size_t size = page_count * BUFRAM_PAGE_SIZE;

    if (ptraddr < BUFRAM_BASE || ptraddr + size >= BUFRAM_BASE + BUFRAM_SIZE) {
        mm_warn("mm: trying to free invalid pointer: %p\n", ptr);
        return;
    }

    list_init(&buffer->list);
    buffer->bucket = size_to_order(size);
#if defined(CONFIG_MM_BUFRAM_CANARY)
    buffer->canary = MM_CANARY;
#endif

    bufram_free(get_buffer_payload(buffer));
}

#ifdef CONFIG_MM_BUFRAM_DEBUG
void bufram_dump(void)
{
    irqstate_t flags;
    flags = irqsave();

    lldbg("allocs=%d, frees=%d, outstanding=%d\n",
          g_bufram_allocs, g_bufram_frees, (g_bufram_allocs - g_bufram_frees));

    size_t i;
    for (i = 0; i < ARRAY_SIZE(mm_bucket); i++) {
        size_t count = 0;
        size_t errors = 0;

        if (!list_is_empty(&mm_bucket[i])) {
            struct list_head *iter;
            list_foreach(&mm_bucket[i], iter) {
                struct mm_buffer *buffer;
                buffer = list_entry(iter, struct mm_buffer, list);

#if defined(CONFIG_MM_BUFRAM_CANARY)
                if (buffer->canary != MM_CANARY) {
                    errors++;
                }
#endif

                if (buffer->bucket != i) {
                    errors++;
                }

                count++;
            }
        }

        lldbg("[% 2d] count=%zd, errors=%zd\n", i, count, errors);
    }

    irqrestore(flags);
}
#endif