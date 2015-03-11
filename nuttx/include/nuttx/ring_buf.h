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
 */
/**
 * @file ring_buf.h
 * @brief Ring buffer support library
 * Ring buffer package used to create, destroy, and manipulate a set of
 * ring buffer entries arranged in a ring.  This is modeled after the
 * ring buffer interfaces that many network controllers have.
 *
 * Each ring buffer entry has a buffer hanging off it.  The buffer may
 * be allocated by the ring buffer package or independently and attached
 * to a ring buffer entry using ring_buf_init().  The buffer is broken
 * into a head area, data area, and tail area.  The head area or headroom
 * is reserved and not touched by the ring buffer package.  It may be used
 * to add headers to the data, for example.  The tail area or tailroom
 * is similar to the head area but is placed after the data area.  This
 * is similar to the Linux kernel's sk_buff.  The data area has a head
 * and tail pointer where data may be added to (pushed) or removed from
 * (pulled).  The data are is *not* circular so once there tail reaches
 * the end of the data area, there is no more room unless the head and
 * tail are reset using ring_buf_reset().  Each ring buffer entry belongs
 * to either the producer or consumer.  Only the entity that currently
 * owns the ring buffer entry may examine the ring buffer entry.  Since
 * there can be only one accessor at a time, no locking is required to
 * access a ring buffer entry's data area (again, just like a network
 * controller's ring buffer interface).  ring_buf_pass() change ownership.
 */

#ifndef __INCLUDE_NUTTX_RING_BUF_H
#define __INCLUDE_NUTTX_RING_BUF_H

enum ring_buf_owner {
    RING_BUF_OWNER_INVALID,
    RING_BUF_OWNER_PRODUCER,
    RING_BUF_OWNER_CONSUMER,
};

struct ring_buf {
    struct ring_buf     *next;
    enum ring_buf_owner owner;
    void                *priv;
    void                *headroom;
    void                *data;
    void                *tailroom;
    void                *head;
    void                *tail;
};

/**
 * @brief Get the next ring buffer entry
 * @param rb Address of the current ring buffer entry
 * @return Address of the next ring buffer entry
 */
static inline void *ring_buf_get_next(struct ring_buf *rb)
{
    return rb->next;
}

/**
 * @brief Set owner of a ring buffer entry
 * @param rb Address of the ring buffer entry whose owner is set
 * @param owner The new owner
 */
static inline void ring_buf_set_owner(struct ring_buf *rb,
                                      enum ring_buf_owner owner)
{
    rb->owner = owner;
}

/**
 * @brief Test if the ring buffer entry is owned by the producer
 * @param rb Address of the ring buffer entry whose ownership is tested
 * @return 0: ring buffer entry is not owned by the producer
 *         1: ring buffer entry is owned by the producer
 */
static inline int ring_buf_is_producers(struct ring_buf *rb)
{
    return rb->owner == RING_BUF_OWNER_PRODUCER;
}

/**
 * @brief Test if the ring buffer entry is owned by the consumer
 * @param rb Address of the ring buffer entry whose ownership is tested
 * @return 0: ring buffer entry is not owned by the consumer
 *         1: ring buffer entry is owned by the consumer
 */
static inline int ring_buf_is_consumers(struct ring_buf *rb)
{
    return rb->owner == RING_BUF_OWNER_CONSUMER;
}

/**
 * @brief Switch ownership of the ring buffer entry
 * @param rb Address of the ring buffer entry whose owner is switched
 */
static inline void ring_buf_pass(struct ring_buf *rb)
{
    if (ring_buf_is_producers(rb))
        ring_buf_set_owner(rb, RING_BUF_OWNER_CONSUMER);
    else
        ring_buf_set_owner(rb, RING_BUF_OWNER_PRODUCER);
}

/**
 * @brief Get the ring buffer entry's private data
 * @param rb Address of the ring buffer entry whose private data is returned
 * @return Ring buffer entry's private data
 */
static inline void *ring_buf_get_priv(struct ring_buf *rb)
{
    return rb->priv;
}

/**
 * @brief Set the ring buffer entry's private data
 * @param rb Address of the ring buffer entry whose private data is set
 * @param priv Ring buffer entry's private data
 */
static inline void *ring_buf_set_priv(struct ring_buf *rb, void *priv)
{
    return rb->priv = priv;
}

/**
 * @brief Get the ring buffer entry's buffer address
 * @param rb Address of the ring buffer entry whose buffer address
 *           is returned
 * @return Ring buffer entry's buffer address
 */
static inline void *ring_buf_get_buf(struct ring_buf *rb)
{
    return rb->headroom;
}

/**
 * @brief Get the ring buffer entry's data area address
 * @param rb Address of the ring buffer entry whose data area address
 *           is returned
 * @return Ring buffer entry's data area address
 */
static inline void *ring_buf_get_data(struct ring_buf *rb)
{
    return rb->data;
}

/**
 * @brief Get the ring buffer entry's head address
 * @param rb Address of the ring buffer entry whose head address
 *           is returned
 * @return Ring buffer entry's head address
 */
static inline void *ring_buf_get_head(struct ring_buf *rb)
{
    return rb->head;
}

/**
 * @brief Get the ring buffer entry's tail address
 * @param rb Address of the ring buffer entry whose tail address
 *           is returned
 * @return Ring buffer entry's tail address
 */
static inline void *ring_buf_get_tail(struct ring_buf *rb)
{
    return rb->tail;
}

/**
 * @brief Reset the ring buffer entry's head and tail address
 * @param rb Address of the ring buffer entry whose head and tail address
 *           is reset
 */
static inline void ring_buf_reset(struct ring_buf *rb)
{
    rb->head = ring_buf_get_data(rb);
    rb->tail = rb->head;
}

/**
 * @brief Get the ring buffer entry's current data length
 * @param rb Address of the ring buffer entry whose data length
 *           is returned
 * @return Ring buffer entry's data length
 */
static inline unsigned int ring_buf_len(struct ring_buf *rb)
{
    return rb->tail - rb->head;
}

/**
 * @brief Get the ring buffer entry's remaining data space
 * @param rb Address of the ring buffer entry whose remaining data space
 *           is returned
 * @return Ring buffer entry's remaining data space
 */
static inline int ring_buf_space(struct ring_buf *rb)
{
    return rb->tailroom - rb->tail;
}

/**
 * @brief Test if the ring buffer entry is empty (has no data)
 * @param rb Address of the ring buffer entry being tested
 * @return 0: ring buffer entry is not empty
 *         1: ring buffer entry is empty
 */
static inline int ring_buf_is_empty(struct ring_buf *rb)
{
    return ring_buf_len(rb) == 0;
}

/**
 * @brief Test if the ring buffer entry is full (has no more space)
 * @param rb Address of the ring buffer entry being tested
 * @return 0: ring buffer entry is not full
 *         1: ring buffer entry is full
 */
static inline int ring_buf_is_full(struct ring_buf *rb)
{
    return ring_buf_space(rb) == 0;
}

/**
 * @brief Add data to a ring buffer entry
 * @param rb Address of the ring buffer entry getting more data
 * @return Address of the tail before data was added
 */
static inline void *ring_buf_put(struct ring_buf *rb, unsigned int len)
{
    void *orig_tail = rb->tail;

    rb->tail += len;

    return orig_tail;
}

/**
 * @brief Remove data from a ring buffer entry
 * @param rb Address of the ring buffer entry losing the data
 * @return The new head address
 */
static inline void *ring_buf_pull(struct ring_buf *rb, unsigned int len)
{
    rb->head += len;

    return rb->head;
}

void ring_buf_init(struct ring_buf *rb, void *buf, unsigned int headroom,
                   unsigned int data_len);
struct ring_buf *ring_buf_alloc(unsigned int headroom, unsigned int data_len,
                                unsigned int tailroom);
void ring_buf_free(struct ring_buf *rb);
struct ring_buf *ring_buf_alloc_ring(unsigned int entries,
                                     unsigned int headroom,
                                     unsigned int data_len,
                                     unsigned int tailroom,
                                     int (*alloc_callback)(struct ring_buf *rb,
                                                           void *arg),
                                     void (*free_callback)(struct ring_buf *rb,
                                                           void *arg),
                                     void *arg);
void ring_buf_free_ring(struct ring_buf *first_rb,
                        void (*free_callback)(struct ring_buf *rb, void *arg),
                                              void *arg);

#endif /* __INCLUDE_NUTTX_RING_BUF_H */
