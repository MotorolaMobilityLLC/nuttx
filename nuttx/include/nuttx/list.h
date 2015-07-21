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

#ifndef __LIST_H__
#define __LIST_H__

#include <stdbool.h>
#include <stdint.h>

struct list_head {
    struct list_head *prev;
    struct list_head *next;
};

void list_init(struct list_head *head);
void list_add(struct list_head *head, struct list_head *node);
void list_del(struct list_head *head);
bool list_is_empty(struct list_head *head);
int list_count(struct list_head *head);

#define list_entry(n, s, f) ((void*) (((uint8_t*) (n)) - offsetof(s, f)))

#define list_foreach(head, iter) \
    for ((iter) = (head)->next; (iter) != (head); (iter) = (iter)->next)

#define list_reverse_foreach(head, iter) \
    for ((iter) = (head)->prev; (iter) != (head); (iter) = (iter)->prev)

#define list_foreach_safe(head, iter, niter) \
    for ((iter) = (head)->next, (niter) = (iter)->next; \
         (iter) != (head); \
         (iter) = (niter), (niter) = (niter)->next)

#define LIST_INIT(head) { .prev = &head, .next = &head }
#define LIST_DECLARE(name) struct list_head name = { .prev = &name, \
                                                     .next = &name }

#endif /* __LIST_H__ */

