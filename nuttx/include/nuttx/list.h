/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
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

#define list_entry(n, s, f) ((void*) (((uint8_t*) (n)) - offsetof(s, f)))

#define list_foreach(head, iter) \
    for ((iter) = (head)->next; (iter) != (head); (iter) = (iter)->next)

#define list_foreach_safe(head, iter, niter) \
    for ((iter) = (head)->next, (niter) = (iter)->next; \
         (iter) != (head); \
         (iter) = (niter), (niter) = (niter)->next)

#endif /* __LIST_H__ */

