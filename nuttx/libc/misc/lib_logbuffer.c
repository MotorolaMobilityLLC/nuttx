/*
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
 */

#include <stdlib.h>
#include <string.h>

#include <nuttx/logbuffer.h>

#define LOG_BUFFER_SIZE(s) \
    (sizeof(struct log_buffer) + s)

/**
 @brief Allocate a log_buffer
 @param size The of buffer to allocate
 @return log_buffer pointer or NULL in case of faillure
*/
struct log_buffer *log_buffer_alloc(size_t size)
{
    struct log_buffer *lb;

    lb = malloc(sizeof(struct log_buffer) + size);
    if (!lb)
        return NULL;
    lb->data = ((char *)lb) + sizeof(struct log_buffer);
    lb->in = lb->data;
    lb->out = lb->data;
    lb->end = lb->data + size;
    return lb;
}

#define min(x,y) ({ \
        (x) < (y) ? (x) : (y); })

static void *log_buffer_memrchr(char *start, char *end, size_t len)
{
    int i = 0;
    char *p = start;
    char *c = start;
    while(c < end && i < len) {
        if (*c == '\n')
            p = c + 1;
        c++;
        i++;
    }
    if (i == len && p == start)
        return p + len;
    return p;
}

static size_t log_buffer_get_lines_len(struct log_buffer *lb, size_t size)
{
    char *p, *p1;

    if (lb->out < lb->in) {
        p = log_buffer_memrchr(lb->out, lb->in, size);
        return p - lb->out;
    } else if (lb->out > lb->in) {
        p = log_buffer_memrchr(lb->out, lb->end, size);
        size -= lb->end - lb->out;
        p1 = log_buffer_memrchr(lb->data, lb->in, size);
        if (p1 != lb->data)
            return lb->end - lb->out + p1 - lb->data;
        else
            return p - lb->out;
    }
    return 0;
}

/**
 @brief Write data in log buffer
 @param lb The log buffer to write to
 @param data Data to write
 @param len number of bytes to write
 @return number of bytes writtem
 */
size_t log_buffer_write(struct log_buffer *lb, const void *data, size_t len)
{
    size_t remain = 0;

    if ((lb->in + len) > lb->end) {
        remain = len - (lb->end - lb->in);
        len -= remain;
    }
    memcpy(lb->in, data, len);
    lb->in += len;

    if (lb->in == lb->end)
        lb->in = lb->data;

    memcpy(lb->in, ((char *)data) + len, remain);
    lb->in += remain;

    return len;
}

/**
 @brief Read lines from log buffer
 This function can return one or more lines (ie string terminated \n).
 The only execption is when a lines is longer than buffer size.
 The function doesn't append \0 at then end strings.
 @param lb The log buffer to read from
 @param data buffer to receive data from log buffer
 @param len number of bytes to read
 @return number of bytes read
 */
size_t log_buffer_readlines(struct log_buffer *lb, void *data, size_t len)
{
    size_t remain = 0;
    if (len > log_buffer_get_lines_len(lb, len))
        len = log_buffer_get_lines_len(lb, len);
    if ((lb->out + len) > lb->end) {
        remain = len - (lb->end - lb->out);
        len = lb->end - lb->out;
    }
    memcpy(data, lb->out, len);
    lb->out += len;

    if (lb->out == lb->end)
        lb->out = lb->data;

    if (remain) {
        memcpy((char *)data + len, lb->out, remain);
        lb->out += remain;
        len += remain;
    }

    return len;
}

