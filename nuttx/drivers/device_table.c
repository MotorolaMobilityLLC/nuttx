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
 * @author Fabien Parent
 */

#include <errno.h>

#include <nuttx/list.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/kmalloc.h>

static LIST_DECLARE(device_table_list);

struct device *device_table_iter_next(struct device_table_iter *iter)
{
    if (!iter) {
        return NULL;
    }

    /* first iteration */
    if (!iter->table) {
        if (list_is_empty(&device_table_list)) {
            return NULL;
        }

        iter->item = 0;
        iter->table = list_entry(device_table_list.next,
                                 struct device_table, list);
    } else {
        iter->item++;
    }

    /* we are done with the current device table, switch to the next one */
    if (iter->item >= iter->table->device_count) {
        /* last device table entry, no device left to process */
        if (iter->table->list.next == &device_table_list) {
            return NULL;
        }

        /* go to first elem of next device table */
        iter->item = 0;
        iter->table = list_entry(iter->table->list.next,
                                 struct device_table, list);
    }

    return &iter->table->device[iter->item];
}

/**
 * @brief Register a device table
 * @param table Device table to be registered
 * @return 0: Device table registered
 *         -EINVAL: if invalid paramter
 *         -ENOMEM: Out of Memory
 */
int device_table_register(struct device_table *table)
{
    if (!table || !table->device || !table->device_count) {
        return -EINVAL;
    }

    list_init(&table->list);
    list_add(&device_table_list, &table->list);

    return 0;
}
