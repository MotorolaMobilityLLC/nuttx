/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/buf_con.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ring_buf.h>
#include <nuttx/util.h>

/*

                               ---- IN --->

------- buf_con_write_in  ----------------------- buf_con_read_in    -------
|     | ----------------> | in->tail   in->head | -----------------> |     |
|     |                   |                     |                    | e.g.|
| APP |                   |    BUFFER CONSOLE   |                    | NSH |
|     |                   |                     |                    |     |
|     | <---------------- | out->head out->tail | <----------------- |     |
-------  buf_con_read_out -----------------------  buf_con_write_out -------

                               <--- OUT ----

*/

struct ring;

typedef void (*ring_cb)(struct ring *ring, ssize_t n);

struct ring {
    struct ring_buf *first;
    struct ring_buf *head;
    struct ring_buf *tail;
    ring_cb read_done_cb;
    ring_cb write_done_cb;
};

static struct ring *ring_create(unsigned int entries, unsigned int data_len);
static void ring_destroy(struct ring *c);
static void ring_set_read_done_callback(struct ring *c, ring_cb cb);
static void ring_set_write_done_callback(struct ring *c, ring_cb cb);
static ssize_t ring_read(struct ring *c, uint8_t *buffer, size_t buflen);
static ssize_t ring_write(struct ring *c, const uint8_t *buffer, size_t buflen);

static struct ring *ring_create(unsigned int entries, unsigned int data_len) {
    struct ring *c = kmm_zalloc(sizeof(*c));
    if (!c) {
        return NULL;
    }

    c->first = c->head = c->tail =
        ring_buf_alloc_ring(CONFIG_BUF_CON_IN_ENTRIES,
        0 /* headroom */, CONFIG_BUF_CON_IN_BUFSIZE, 0 /* tailroom */,
        NULL /* alloc */, NULL /* free */, NULL /* arg */ );
    if (!c->first) {
        kmm_free(c);
        return NULL;
    }

    return c;
}

static void ring_destroy(struct ring *c) {
    ring_buf_free_ring(c->first, NULL /* free */, NULL /* arg */);
    kmm_free(c);
}

static void ring_set_read_done_callback(struct ring *c, ring_cb cb) {
    c->read_done_cb = cb;
}

static void ring_set_write_done_callback(struct ring *c, ring_cb cb) {
    c->write_done_cb = cb;
}

static ssize_t ring_read(struct ring *c, uint8_t *buffer, size_t buflen) {
    ssize_t n = 0;
    while (n < buflen) {
        if (!ring_buf_is_consumers(c->head)) {
            /* No data, don't block */
            break;
        }

        void *src = ring_buf_get_head(c->head);
        size_t used = MIN(ring_buf_len(c->head), buflen - n);

        memcpy(&buffer[n], src, used);

        ring_buf_pull(c->head, used);

        if (ring_buf_is_empty(c->head)) {
            ring_buf_reset(c->head);
            ring_buf_pass(c->head);

            c->head = ring_buf_get_next(c->head);
        }

        n += used;
    }

    if (n) {
        /* Notify the app that there is more space to be filled. */
        if (c->read_done_cb) {
            c->read_done_cb(c, n);
        }
    }

    return n;
}

static ssize_t ring_write(struct ring *c, const uint8_t *buffer, size_t buflen) {
    ssize_t n = 0;
    while (n < buflen) {
        if (!ring_buf_is_producers(c->tail)) {
            /* Out of space, don't block */
            break;
        }

        void *dst = ring_buf_get_tail(c->tail);
        size_t space = MIN(ring_buf_space(c->tail), buflen - n);

        memcpy(dst, &buffer[n], space);

        ring_buf_put(c->tail, space);

        if (ring_buf_is_full(c->tail)) {
            ring_buf_pass(c->tail);

            c->tail = ring_buf_get_next(c->tail);
        }

        n += space;
    }

    if (n) {
        /* Pass a partial buffer to be consumed. */
        if (!ring_buf_is_empty(c->tail)) {
            ring_buf_pass(c->tail);

            c->tail = ring_buf_get_next(c->tail);
        }

        /* Notify the app that there is more to be consumed. */
        if (c->write_done_cb) {
            c->write_done_cb(c, n);
        }
    }

    return n;
}

struct buf_con {
    struct ring *in;
    struct ring *out;
    sem_t in_sem;
    sem_t out_sem;
    buf_con_cb read_ready_cb;
    void *read_ready_arg;
    buf_con_cb write_ready_cb;
    void *write_ready_arg;
};

static struct buf_con *g_buf_con;

static int buf_con_open(FAR struct file *filep) {
    if (!g_buf_con) {
        return -ENOMEM;
    }

    struct inode *inode = filep->f_inode;
    inode->i_private = g_buf_con;

    return 0;
}

static int buf_con_close(FAR struct file *filep) {
    return 0;
}

static ssize_t buf_con_in_read(struct file *filep, char *buffer, size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct buf_con *c = inode->i_private;

    ssize_t n = 0;
    while (n < buflen) {
        ssize_t read = ring_read(c->in, (uint8_t *)(buffer + n), buflen - n);
        if (!read) {
            if (n) {
                /* Return what data we have */
                break;
            }

            if (filep->f_oflags & O_NONBLOCK) {
                /* Don't block */
                return -EAGAIN;
            }

            /* Block, then try again. */
            sem_wait(&c->in_sem);
            continue;
        }

        n += read;
    }

    if (n) {
        /* Notify the app that there is more space to be filled. */
        if (c->write_ready_cb) {
            c->write_ready_cb(n, c->write_ready_arg);
        }
    }

    return n;
}

static ssize_t buf_con_out_write(struct file *filep, const char *buffer, size_t buflen)
{
    struct inode *inode = filep->f_inode;
    struct buf_con *c = inode->i_private;

    ssize_t n = 0;
    while (n < buflen) {
        ssize_t written = ring_write(c->out, (const uint8_t *)(buffer + n), buflen - n);
        if (!written) {
            if (filep->f_oflags & O_NONBLOCK) {
                /* Don't block */
                return -EAGAIN;
            }

            /* Block, then try again. */
            sem_wait(&c->out_sem);
            continue;
        }

        n += written;
    }

    if (n) {
        /* Notify the app that there is more to be consumed. */
        if (c->read_ready_cb) {
            c->read_ready_cb(n, c->read_ready_arg);
        }
    }

    return n;
}

static const struct file_operations g_buf_con_ops =
{
  buf_con_open,
  buf_con_close,
  buf_con_in_read,
  buf_con_out_write,
  NULL, /* close */
  NULL /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL /* poll */
#endif
};

void buf_con_set_write_ready_callback(buf_con_cb cb, void *arg) {
    struct buf_con *c = g_buf_con;
    c->write_ready_cb = cb;
    c->write_ready_arg = arg;
}

void buf_con_set_read_ready_callback(buf_con_cb cb, void *arg) {
    struct buf_con *c = g_buf_con;
    c->read_ready_cb = cb;
    c->read_ready_arg = arg;
}

ssize_t buf_con_out_read(uint8_t *buffer, size_t buflen) {
    struct buf_con *c = g_buf_con;

    ssize_t n = ring_read(c->out, buffer, buflen);
    if (n) {
        /* Notify that there is more space for the producer. */
        sem_post(&c->out_sem);
    }

    return n;
}

ssize_t buf_con_in_write(const uint8_t *buffer, size_t buflen) {
    struct buf_con *c = g_buf_con;

    ssize_t n = ring_write(c->in, buffer, buflen);
    if (n) {
        /* Notify that there is more space for the producer. */
        sem_post(&c->in_sem);
    }

    return n;
}

int buf_con_init(void) {
    if (g_buf_con) {
        return -EINVAL;
    }

    struct buf_con *c = kmm_zalloc(sizeof(struct buf_con));
    if (!c) {
        return -ENOMEM;
    }

    sem_init(&c->out_sem, 0, 0);
    sem_init(&c->in_sem, 0, 0);

    c->in = ring_create(CONFIG_BUF_CON_IN_ENTRIES, CONFIG_BUF_CON_IN_BUFSIZE);
    if (!c->in) {
        goto in_failed;
    }

    c->out = ring_create(CONFIG_BUF_CON_OUT_ENTRIES, CONFIG_BUF_CON_OUT_BUFSIZE);
    if (!c->out) {
        goto out_failed;
    }

    g_buf_con = c;

    register_driver(CONFIG_BUF_CONDEV, &g_buf_con_ops, 0666, NULL);

    return 0;

out_failed:
    ring_destroy(c->in);
in_failed:
    sem_destroy(&c->out_sem);
    sem_destroy(&c->in_sem);
    kmm_free(c);
    return -ENOMEM;

}
