/*
 * Copyright (C) 2016 Motorola Mobility, LLC.
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

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>

#include <arch/irq.h>

#include <nuttx/list.h>
#include <nuttx/notifier.h>

int notifier_register(notifier_head_t *head, notifier_fn_t fn, void *arg)
{
  irqstate_t flags;
  struct notifier_node_s *node;

  DEBUGASSERT(head);
  DEBUGASSERT(fn);

  node = malloc(sizeof(struct notifier_node_s));
  if (!node)
      return -ENOMEM;

  node->fn = fn;
  node->arg = arg;

  flags = irqsave();
  list_add(head, &node->list);
  irqrestore(flags);

  return OK;
}

int notifier_call_chain(notifier_head_t *head, const void *data)
{
  struct list_head *iter;
  struct list_head *iter_next;
  struct notifier_node_s *node;
  int err_cnt = 0;

  DEBUGASSERT(head);

  list_foreach_safe(head, iter, iter_next)
    {
      node = list_entry(iter, struct notifier_node_s, list);
      if (node->fn(node->arg, data) != OK)
          err_cnt++;
    }

  return err_cnt ? -EIO : OK;
}
