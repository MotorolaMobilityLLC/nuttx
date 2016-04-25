/**
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
 *
 * @author Mark Greer
 * @brief Ring Buffer Package
 */

#include <nuttx/kmalloc.h>
#include <nuttx/ring_buf.h>

#if defined(CONFIG_MHB) && defined(CONFIG_MM_BUFRAM_ALLOCATOR)
#include <string.h>
#include <nuttx/bufram.h>
#endif

/**
 * Initialize the data pointers of a ring buffer entry.
 * It is assumed that the 'rb' was allocated using ring_buf_alloc() or
 * ring_buf_alloc_ring().
 *
 * @brief Initialize ring buffer entry's data pointers
 * @param rb Address of the ring buffer entry whose data pointers will
 *           be initialized
 * @param buf Address of the buffer including headroom and tailroom
 * @param headroom Number of bytes in 'buf' reserved before the data area
 * @param data_len Number of bytes of data (between headroom and tailroom)
 */
void ring_buf_init(struct ring_buf *rb, void *buf, unsigned int headroom,
                   unsigned int data_len)
{
    rb->headroom = buf;
    rb->data = buf + headroom;
    rb->tailroom = rb->data + data_len;

    ring_buf_reset(rb);
}

/**
 * @brief Allocate a ring buffer entry (will be owned by producer)
 * @param headroom Number of bytes to reserve before the data area
 * @param data_len Number of bytes of the data area
 * @param tailroom Number of bytes to reserve after the data area
 * @return Address of the allocated ring buffer entry or NULL on failure
 */
struct ring_buf *ring_buf_alloc(unsigned int headroom, unsigned int data_len,
                                unsigned int tailroom)
{
    struct ring_buf *rb;
    unsigned int buf_len;
    void *chunk;

    buf_len = headroom + data_len + tailroom;

#if defined(CONFIG_MHB) && defined(CONFIG_MM_BUFRAM_ALLOCATOR)
    chunk = bufram_alloc(sizeof(*rb) + buf_len);
#else
    chunk = zalloc(sizeof(*rb) + buf_len);
#endif
    if (!chunk)
        return NULL;

#if defined(CONFIG_MHB) && defined(CONFIG_MM_BUFRAM_ALLOCATOR)
    memset(chunk, 0, sizeof(*rb) + buf_len);
#endif

    rb = chunk;

    ring_buf_set_owner(rb, RING_BUF_OWNER_PRODUCER);

    if (buf_len)
        ring_buf_init(rb, chunk + sizeof(*rb), headroom, data_len);

    return rb;
}

/**
 * @brief Free a ring buffer entry allocated by ring_buf_alloc()
 * @param Address of the ring buffer entry
 */
void ring_buf_free(struct ring_buf *rb)
{
#if defined(CONFIG_MHB) && defined(CONFIG_MM_BUFRAM_ALLOCATOR)
    bufram_free(rb);
#else
    free(rb);
#endif
}

/**
 * Allocate a set of ring buffer entries and arrange them into a ring.
 * There is no head or tail for the ring but each ring buffer entry has
 * a data area associated with it which has a head and tail.
 *
 * @brief Allocate a ring of ring buffer entries
 * @param entries Number of ring buffer entries in the ring
 * @param headroom Number of bytes to reserve before the data area in each
 *                 ring buffer entry
 * @param data_len Number of bytes of of data in each ring buffer entry
 * @param tailroom Number of bytes to reserve after the data area in each
 *                 ring buffer entry
 * @param alloc_callback Callback routine called after each ring buffer entry
 *                       is allocated
 * @param free_callback Callback routine called before each ring buffer entry
 *                      is freed (on error)
 * @param arg Argument to pass to alloc_callback() and free_callback()
 * @return Address of a ring buffer entry in the ring or NULL on failure
 */
struct ring_buf *ring_buf_alloc_ring(unsigned int entries,
                                     unsigned int headroom,
                                     unsigned int data_len,
                                     unsigned int tailroom,
                                     int (*alloc_callback)(struct ring_buf *rb,
                                                           void *arg),
                                     void (*free_callback)(struct ring_buf *rb,
                                                           void *arg),
                                     void *arg)
{
    struct ring_buf *rb, *prev_rb, *first_rb = NULL;
    unsigned int i;
    int ret;

    if (!entries)
        return NULL;

    for (i = 0; i < entries; i++) {
        rb = ring_buf_alloc(headroom, data_len, tailroom);
        if (!rb)
            break;

        if (alloc_callback) {
            ret = alloc_callback(rb, arg);
            if (ret) {
                ring_buf_free(rb);
                break;
            }
        }

        if (!first_rb)
            first_rb = rb;
        else
            prev_rb->next = rb;

        prev_rb = rb;
    }

    if (i < entries) {
        for (rb = first_rb; i > 0; i--) {
            prev_rb = rb;
            rb = ring_buf_get_next(prev_rb);

            if (free_callback)
                free_callback(prev_rb, arg);

            ring_buf_free(prev_rb);
        }

        return NULL;
    }

    rb->next = first_rb;

    return first_rb;
}

/**
 * @brief Free a ring buffer ring allocated by ring_buf_alloc_ring()
 * @param rb Address of a ring buffer entry in the ring being freed
 * @param free_callback Callback routine called before each ring buffer entry
 *                      is freed
 * @param arg Argument to pass to free_callback()
 */
void ring_buf_free_ring(struct ring_buf *rb,
                        void (*free_callback)(struct ring_buf *rb, void *arg),
                        void *arg)
{
    struct ring_buf *first_rb, *next_rb;

    if (!rb)
        return;

    first_rb = rb;

    do {
        next_rb = ring_buf_get_next(rb);

        if (free_callback)
            free_callback(rb, arg);

        ring_buf_free(rb);
        rb = next_rb;
    } while (rb != first_rb);
}
