/*
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <nuttx/list.h>

void list_init(struct list_head *head)
{
    head->prev = head->next = head;
}

bool list_is_empty(struct list_head *head)
{
    return head->prev == head->next;
}

void list_add(struct list_head *head, struct list_head *node)
{
    node->next = head;
    node->prev = head->prev;

    head->prev->next = node;
    head->prev = node;
}

void list_del(struct list_head *head)
{
    head->prev->next = head->next;
    head->next->prev = head->prev;
    head->prev = head->next = head;
}
